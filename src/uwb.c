#include "pt.h"
#include "deca_device_api.h"
#include "durin.h"
#include "deca_regs.h"
#include "hardware.h"
#include "uwb_definitions.h"

#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>

#define RANGING_DELAY 1000000 //10ms
#define RANGING_RANDOM_DELAY 1000.0 //2ms

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)
#define RX_TX_DELAY 2600 //more than 2ms because of free rtos...

#define RANDOM_MESSAGE_SEND_PERIOD 100 //us

#define UUS_TO_DWT_TIME 63898

static void send_message(uint8_t* buf, uint8_t len, uint64_t ts);
static void uwb_parse_message();
static void uwb_task();
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

uint64_t polled_everyone_at;
uint64_t last_poll_received_at;

uint8_t sum_alive_nodes = 1; //I am always alive
uint8_t alive_nodes[NUM_NODES] = {0};

void init_uwb() {
    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }

    dwt_config_t config = UWB_CONFIG;

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
    uint64_t random_delay = 0;
    while (1) {
        uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
        if (sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
            uwb_parse_message(); // this function can take up to 2ms because of delayed tx
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        if (sys_status & SYS_STATUS_RXFCE_BIT_MASK) {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        uint64_t cycle_time = sum_alive_nodes * RANDOM_MESSAGE_SEND_PERIOD + RX_TX_DELAY;
        if (esp_timer_get_time() - last_poll > sum_alive_nodes * cycle_time + random_delay
        && esp_timer_get_time() - last_poll_received_at > cycle_time) {
            last_poll = esp_timer_get_time();
            struct uwb_poll_msg msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_RANGING,
                .header.sender = durin_persistent.node_id,
                .header.receiver = UWB_BROADCAST_ID,
            };
            send_message(&msg, sizeof(msg), 0);
            polled_everyone_at = dwt_readtxtimestamplo32();
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

static void send_message(uint8_t* buf, uint8_t len, uint64_t ts) {
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    if (ts == 0) {
        ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    } else {
        dwt_setdelayedtrxtime(ts);
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    }
    
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
}

static void uwb_parse_message() {
    uint8_t rx_buf[64];

    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RX_BUFFER_MAX_LEN;
    if (length > sizeof(rx_buf)) {
        printf("too long\n");
        return;
    }
    dwt_readrxdata(rx_buf, length, 0);
    struct uwb_header *header = (struct uwb_header *) rx_buf; 
    if (memcmp(header->magic, UWB_MAGIC_WORD, sizeof(header->magic)) != 0) {
        printf("invalid header %.*s\n", sizeof(header->magic), header->magic);
        return;
    }
    if (alive_nodes[header->sender] == false) {
        alive_nodes[header->sender] = true;
        sum_alive_nodes++;
    }
    durin.info.uwb_messages_received++;
    if (header->receiver != durin_persistent.node_id && header->receiver != 0xff) {
        return;
    }
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        last_poll_received_at = esp_timer_get_time();
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t extra_time = sum_alive_nodes * RANDOM_MESSAGE_SEND_PERIOD;
        uint64_t random_delay = extra_time * esp_random() / UINT32_MAX;
        uint64_t resp_tx_time = (poll_rx_ts + ((random_delay + RX_TX_DELAY) * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        struct uwb_response_msg tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_RESPONSE_RANGING,
            .header.sender = durin_persistent.node_id,
            .header.receiver = header->sender,
            .rx_timestamp = poll_rx_ts,
            .tx_timestamp = resp_tx_ts
        };
        send_message(&tx_msg, sizeof(tx_msg), resp_tx_time);
    }
    else if (header->msg_type == UWB_MSG_RESPONSE_RANGING) {
        struct uwb_response_msg *rx_msg = (struct uwb_header *) rx_buf;
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Read carrier integrator value and calculate clock offset ratio. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

        /* Get timestamps embedded in response message. */
        poll_rx_ts = rx_msg->rx_timestamp;
        resp_tx_ts = rx_msg->tx_timestamp;
    
        // poll_tx_ts = dwt_readtxtimestamplo32();
        poll_tx_ts = polled_everyone_at; // use saved timestamp because another message could have come before this
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        // not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        durin.telemetry.distance_to_node[header->sender] = 1000 * tof * SPEED_OF_LIGHT;
        printf("d %d %f\n", header->sender, tof * SPEED_OF_LIGHT);
        //printf("ts %d %d %d %d\n", poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts);
    }  
    else {

    }
}