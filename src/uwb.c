#include "pt.h"
#include "deca_device_api.h"
#include "durin.h"
#include "deca_regs.h"

#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>

#define RANGING_POLL_EVERY 100

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458*1000)
#define RX_TX_DELAY 2600 //more than 2ms because of free rtos...
#define RX_TIMEOUT 3000

#define UUS_TO_DWT_TIME 63898

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING,
    UWB_MSG_RESPONSE_RANGING
};

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

static void uwb_rx_ok_callback(const dwt_cb_data_t *data);
static void uwb_poll_node(uint8_t node);
static void uwb_task();

void init_uwb() {
    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }
    
    dwt_config_t config = {
        5,               /* Channel number. */
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

    dwt_setrxtimeout(RX_TIMEOUT);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
    dwt_setcallbacks(uwb_rx_ok_callback, uwb_rx_ok_callback, uwb_rx_ok_callback, uwb_rx_ok_callback, uwb_rx_ok_callback, uwb_rx_ok_callback);

    xTaskCreatePinnedToCore(uwb_task, "uwb_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}

void uwb_task() {
    uint32_t last_poll = esp_timer_get_time();
    while (1) {
        vTaskDelay(1);
        dwt_isr();
        if (esp_timer_get_time() - last_poll > 1000000) {
            last_poll = esp_timer_get_time();
            uwb_poll_node(0);
        }
    }
}

uint64_t get_tx_timestamp_u64(void)
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

uint64_t get_rx_timestamp_u64(void)
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

static void uwb_rx_ok_callback(const dwt_cb_data_t *data) {
    printf("got msg\n");
    uint8_t rx_buf[32];
    if (data->datalength > sizeof(rx_buf)) {
        return;
    }
    dwt_readrxdata(rx_buf, data->datalength, 0);
    if (memcmp(rx_buf, "drn", 3) != 0) {
        return;
    }
    uint8_t msg_type = rx_buf[3];
    uint8_t sender = rx_buf[4];
    uint8_t receiver = rx_buf[5];
    if (receiver != durin.info.node_id) {
        return;
    }
    if (msg_type == UWB_MSG_POLL_RANGING) {
        printf("got poll\n");
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t resp_tx_time = (poll_rx_ts + (RX_TX_DELAY * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        struct uwb_response_msg resp_msg;
        memcpy(resp_msg.header, "drn", 3);
        resp_msg.msg_type = UWB_MSG_RESPONSE_RANGING;
        resp_msg.receiver = sender;
        resp_msg.sender = durin.info.node_id;
        resp_msg.rx_timestamp = poll_rx_ts;
        resp_msg.tx_timestamp = resp_tx_ts;
        dwt_writetxdata(sizeof(resp_msg), &resp_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        uint8_t ret = dwt_starttx(DWT_START_TX_DELAYED);
        if (ret == DWT_SUCCESS) {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
                vTaskDelay(1);
            }
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        }
    }
    else if (msg_type == UWB_MSG_RESPONSE_RANGING) {
        printf("got response\n");
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Retrieve poll transmission and response reception timestamps. See NOTE 9 below. */
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Read carrier integrator value and calculate clock offset ratio. See NOTE 11 below. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

        /* Get timestamps embedded in response message. */
        struct uwb_response_msg *msg;
        msg = rx_buf;
        poll_rx_ts = msg->rx_timestamp;
        resp_tx_ts = msg->tx_timestamp;

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        //not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        durin.telemetry.distance_to_node[sender] = tof * SPEED_OF_LIGHT;
    }  
    else {

    }       
}

static void uwb_poll_node(uint8_t node) {
    printf("poll\n");
    struct uwb_poll_msg msg;
    msg.sender = durin.info.node_id;
    msg.receiver = node;
    memcpy(msg.header, "drn", 3);
    msg.msg_type = UWB_MSG_POLL_RANGING;

    dwt_setrxaftertxdelay(RX_TX_DELAY);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    dwt_writetxdata(sizeof(msg), &msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(msg), 0, 1); /* Zero offset in TX buffer, ranging. */
    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
        * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
        vTaskDelay(1);
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    durin.info.polled_node_at[node] = dwt_readtxtimestamplo32();
}