#include "pt.h"
#include "deca_device_api.h"
#include "durin.h"
#include "deca_regs.h"
#include "hardware.h"

#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>

#define RANGING_DELAY 1000000 //10ms
#define RANGING_RANDOM_DELAY 2000.0 //2ms

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)
#define RX_TX_DELAY 2600 //more than 2ms because of free rtos...

#define UUS_TO_DWT_TIME 63898

#define PRINT_REG(reg) {\
    for (uint8_t i = 0; i < 32; i++) {\
        printf("%d:%d,", i, (reg >> i) & 1);\
    } printf("\n");\
}\

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING,
    UWB_MSG_RESPONSE_RANGING
};


// i don't use these but they are my source of truth so to say
#pragma pack(1)
struct uwb_poll_msg {
    char header[3]; //{'d','r','n'}
    uint8_t msg_type;
    uint8_t sender;
    uint8_t receiver;
};

#pragma pack(1)
struct uwb_response_msg {
    char header[3];
    uint8_t msg_type;
    uint8_t sender;
    uint8_t receiver;
    uint64_t rx_timestamp;
    uint64_t tx_timestamp;
};

static void uwb_poll_node(uint8_t node);
static void uwb_parse_message();
static void uwb_task();
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

uint64_t polled_node_at[NUM_NODES];

void init_uwb() {
    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }
    
    dwt_config_t config = {
        9,               /* Channel number. */
        DWT_PLEN_128,    /* Preamble length. Used in TX only. */
        DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
        DWT_BR_6M8,      /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        DWT_PHRRATE_STD, /* PHY header rate. */
        (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
        DWT_STS_MODE_OFF, /* STS disabled */
        DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
        DWT_PDOA_M0      /* PDOA mode off */
    };

    dwt_txconfig_t txconfig_options = {
        0x34,       /* PG delay. */
        0xfdfdfdfd, /* TX power. */
        0x0         /*PG count*/
    };

    if (dwt_configure(&config)) {
        printf("uwb error2\n");
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    // dwt_setcallbacks(uwb_tx_done_callback, uwb_rx_ok_callback, uwb_rx_to_callback, uwb_rx_err_callback, uwb_spi_err_callback, uwb_spi_rdy_callback);

    xTaskCreatePinnedToCore(uwb_task, "uwb_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL, TRASH_CORE);
}

void uwb_task() {
    uint64_t last_poll = esp_timer_get_time();
    uint8_t poll_node = 0;
    uint64_t random_delay = 0;
    while (1) {
        if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_RXFCG_BIT_MASK) {
            printf("got msg222");
            uwb_parse_message(); // this function can take up to 2ms because of delayed tx
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        if (esp_timer_get_time() - last_poll > RANGING_DELAY + random_delay) {
            last_poll = esp_timer_get_time();
            poll_node = (poll_node + 1) % NUM_NODES;
            dwt_forcetrxoff();
            uwb_poll_node(poll_node);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            random_delay = (RANGING_RANDOM_DELAY * esp_random()) / UINT32_MAX;
        }
        vTaskDelay(1);
    }
}

static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void uwb_poll_node(uint8_t node) {
    printf("poll node %d\n", node);
    uint8_t tx_buf[32];
    uint32_t reg;
    durin.info.failed_node_polls[node] += 1;
    memcpy(tx_buf, "drn", 3);
    tx_buf[3] = UWB_MSG_POLL_RANGING;
    tx_buf[4] = durin_persistent.node_id;
    tx_buf[5] = node;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(6, tx_buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(6 + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    //wait untill done
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        vTaskDelay(1);
    }
    polled_node_at[node] = dwt_readtxtimestamplo32();
}

static void uwb_parse_message() {
    printf("got msg\n");
    uint8_t rx_buf[32];
    uint8_t tx_buf[32];

    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RX_BUFFER_MAX_LEN;
    printf("length %d\n", length);
    if (length > sizeof(rx_buf)) {
        printf("too long\n");
        return;
    }

    dwt_readrxdata(rx_buf, length, 0);
    printf("begin\n");
    for (uint8_t i = 0; i < length; i++) {
        printf("lol %d\n", rx_buf[i]);
    }
    printf("end\n");


    if (memcmp(rx_buf, "drn", 3) != 0) {
        printf("invalid header\n");
        return;
    }
    uint8_t msg_type = rx_buf[3];
    uint8_t sender = rx_buf[4];
    uint8_t receiver = rx_buf[5];
    if (sender >= NUM_NODES || receiver >= NUM_NODES) {
        printf("invalid sender or receiver %d %d\n", sender, receiver);
        return;
    }
    
    if (receiver != durin_persistent.node_id) {
        // printf("Not for me but for %d i am %d\n", receiver, durin_persistent.node_id);
        return;
    }

    if (msg_type == UWB_MSG_POLL_RANGING) {
        // printf("got poll\n");
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t resp_tx_time = (poll_rx_ts + (RX_TX_DELAY * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        memcpy(tx_buf, "drn", 3);
        tx_buf[3] = UWB_MSG_RESPONSE_RANGING;
        tx_buf[4] = durin_persistent.node_id;
        tx_buf[5] = sender;
        memcpy(tx_buf + 6, &poll_rx_ts, 4);
        memcpy(tx_buf + 10, &resp_tx_ts, 4);

        dwt_writetxdata(14, tx_buf, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(14 + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
        dwt_setdelayedtrxtime(resp_tx_time);

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        uint8_t ret = dwt_starttx(DWT_START_TX_DELAYED);
        
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
            vTaskDelay(1);
        }
    }
    else if (msg_type == UWB_MSG_RESPONSE_RANGING) {
        printf("got response\n");
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

        /* Get timestamps embedded in response message. */
        memcpy(&poll_rx_ts, rx_buf + 6, 4);
        memcpy(&resp_tx_ts, rx_buf + 10, 4);
    
        // poll_tx_ts = dwt_readtxtimestamplo32();
        poll_tx_ts = polled_node_at[sender]; // use saved timestamp because another message could have come before this
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        // not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        durin.telemetry.distance_to_node[sender] = 1000 * tof * SPEED_OF_LIGHT;
        printf("distance %f\n", tof * SPEED_OF_LIGHT);
        //printf("ts %d %d %d %d\n", poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts);
    }  
    else {

    }
}