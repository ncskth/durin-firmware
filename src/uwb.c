#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <nvs.h>

#include "pt.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "hardware.h"
#include "uwb_definitions.h"
#include "position_solver.h"
#include "durin.h"
#include "protocol.h"
#include "tof_and_expander.h"

#define USER_UART

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)

#define UUS_TO_DWT_TIME 63898

#define MIN_TRX_DELAY 700

#define TIME_TO_INACTIVE_US 10000000
#define INACTIVE_POLL_PERIOD_MS 10000

static void send_message_delayed(uint8_t* buf, uint8_t len, uint64_t ts);
static void send_message_global(uint8_t* buf, uint8_t len);
static void send_message_instantly(uint8_t* buf, uint8_t len);
static void uwb_parse_message();
static void uwb_misc_task();
static void uwb_reading_task();
static void uwb_send_task();

static void send_position_telemetry();
static void send_uwb_nodes_telemetry();

static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

SemaphoreHandle_t uwb_mutex;

TaskHandle_t uwb_misc_task_handle;
TaskHandle_t uwb_send_task_handle;
volatile uint8_t* uwb_send_task_buf; // i really can't be bothered to set up a RTOS queue for this...
volatile uint8_t uwb_send_task_len;

uint8_t all_beacon_ids[32];
uint8_t all_beacon_ids_index = 0;
uint8_t user_ids[32];
uint8_t user_ids_index = 0;

uint64_t last_poll_received_at = 0;
uint64_t quiet_until = 0;
uint64_t last_message_received_at = 0;

IRAM_ATTR void uwb_irq(TaskHandle_t *uwb_task) {
    int has_awoken;
    vTaskNotifyGiveFromISR(*uwb_task, &has_awoken);
    portYIELD_FROM_ISR(has_awoken);
}

void init_uwb() {
    // uwb spi
    spi_bus_config_t spi_config = {
        .sclk_io_num = PIN_SPI_UWB_SCK,
        .miso_io_num = PIN_SPI_UWB_MISO,
        .mosi_io_num = PIN_SPI_UWB_MOSI,
    };
    spi_bus_initialize(SPI_UWB_HOST, &spi_config, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t uwb_conf = {
        .address_bits = 0,
        .clock_speed_hz = SPI_UWB_FREQ,
        .command_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .input_delay_ns = 0,
        .spics_io_num = PIN_SPI_UWB_CS,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI_UWB_HOST, &uwb_conf, &deca_spi_device);

    configure_expander_pin(EX_PIN_UWB_RST, 0);
    write_expander_pin(EX_PIN_UWB_RST, 1);
    vTaskDelay(1);
    write_expander_pin(EX_PIN_UWB_RST, 0);
    vTaskDelay(1);
    write_expander_pin(EX_PIN_UWB_RST, 1);
    vTaskDelay(1);

    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }

    dwt_config_t config = UWB_CONFIG;

    dwt_txconfig_t txconfig_options = {
        0x34,       /* PG delay. */
        0xFEFEFEFE, /* TX power. */
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
    dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);

    uwb_mutex = xSemaphoreCreateMutex();
    static TaskHandle_t uwb_reading_task_handle;
    xTaskCreatePinnedToCore(uwb_reading_task, "uwb_reading_task", 4096, NULL, configMAX_PRIORITIES - 1, &uwb_reading_task_handle, TRASH_CORE);
    xTaskCreatePinnedToCore(uwb_misc_task, "uwb_misc_task", 8096, NULL, 2, &uwb_misc_task_handle, TRASH_CORE);
    xTaskCreatePinnedToCore(uwb_send_task, "uwb_send_task", 4096, NULL, 2, &uwb_send_task_handle, TRASH_CORE);

    gpio_set_direction(PIN_IRQ_UWB, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_IRQ_UWB, GPIO_INTR_POSEDGE);
    gpio_intr_enable(PIN_IRQ_UWB);
    gpio_isr_handler_add(PIN_IRQ_UWB, uwb_irq, &uwb_reading_task_handle);
}

void uwb_reading_task() {
    uint64_t last_poll = esp_timer_get_time();
    uint64_t random_delay = 0;
    
    while (1) {
        uint32_t notification_value;
        xTaskNotifyWait(0, 0, &notification_value, 100);
        
        xSemaphoreTake(uwb_mutex, portMAX_DELAY);
        uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
        xSemaphoreGive(uwb_mutex);

        if (sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
            uwb_parse_message();
            xSemaphoreTake(uwb_mutex, portMAX_DELAY);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            xSemaphoreGive(uwb_mutex);
        }
        else if (sys_status & SYS_STATUS_ALL_RX_ERR) {
            xSemaphoreTake(uwb_mutex, portMAX_DELAY);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            xSemaphoreGive(uwb_mutex);
        } else {

        }
        xTaskNotify(uwb_misc_task_handle, 0, 0);
    }
}

void uwb_send_task() {
    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        vTaskDelay(GET_BROADCAST_WAIT_MS(durin_persistent.node_id));
        send_message_instantly(uwb_send_task_buf, uwb_send_task_len);
    }
}

void uwb_misc_task() {
    uint64_t last_poll_sent_at = 0;
    uint8_t next_poll_index = 0;
    uint64_t last_position_telemetry = 0;
    uint64_t last_uwb_nodes_telemetry = 0;

    // wait for everyone to quiet down
    vTaskDelay(500);
    while (1) {
        struct uwb_quiet poll_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_QUIET,
            .header.receiver = UWB_BROADCAST_ID,
            .header.sender = durin_persistent.node_id,
            .header.purpose = UWB_PURPOSE_USER,
            .duration = BROADCAST_BUSY_FOR_MS + 500,
        };
        send_message_instantly(&poll_msg, sizeof(poll_msg));
        vTaskDelay(100);
        if (esp_timer_get_time() - last_message_received_at > 90 * 1000) {
            break;
        }
    }

    user_ids_index = 0;
    quiet_until = 0;
    next_poll_index = 0;
    struct uwb_poll_alive poll_msg = {
        .header.magic = UWB_MAGIC_WORD,
        .header.msg_type = UWB_MSG_POLL_ALIVE,
        .header.receiver = UWB_BROADCAST_ID,
        .header.sender = durin_persistent.node_id,
        .header.purpose = UWB_PURPOSE_USER,
    };
    send_message_instantly(&poll_msg, sizeof(poll_msg));

    struct uwb_is_alive self_response_msg = {
        .header.magic = UWB_MAGIC_WORD,
        .header.msg_type = UWB_MSG_IS_ALIVE,
        .header.sender = durin_persistent.node_id,
        .header.receiver = UWB_BROADCAST_ID,
        .header.purpose = UWB_PURPOSE_USER,
    };
    send_message_global(&self_response_msg, sizeof(self_response_msg));
    vTaskDelay(BROADCAST_BUSY_FOR_MS);

    while (true) {
        if (!durin.info.active) {
            vTaskDelay(100);
            continue;
        }
        uint64_t current_time = esp_timer_get_time();
        if (
            current_time - last_poll_sent_at > user_ids_index * SINGLE_BUSY_FOR_MS * 1000 &&
            current_time - last_poll_received_at > SINGLE_BUSY_FOR_MS * 1000 &&
            current_time > quiet_until &&
            all_beacon_ids_index > 0
        ) {

            struct uwb_poll_ranging msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_RANGING,
                .header.receiver = all_beacon_ids[next_poll_index],
                .header.sender = durin_persistent.node_id,
                .header.purpose = UWB_PURPOSE_USER,
            };
            send_message_instantly(&msg, sizeof(msg));
            last_poll_sent_at = esp_timer_get_time();
            vTaskDelay(1);

            next_poll_index = (next_poll_index + 1) % all_beacon_ids_index;
            if (next_poll_index == 0) {
                struct pos_solver_position pos = {
                    durin.telemetry.pos_x, 
                    durin.telemetry.pos_y, 
                    durin.telemetry.pos_z
                };  

                memcpy(durin.telemetry.distance_data_old_chache, durin.telemetry.distance_data, sizeof(durin.telemetry.distance_data)); //store an old cache for polling
                durin.telemetry.distance_old_cache_index = durin.telemetry.distance_index;
                solve_for_position(durin.telemetry.distance_data, durin.telemetry.distance_index, &pos, &durin.telemetry.fix_type);
                durin.telemetry.distance_index = 0;
                durin.telemetry.pos_x = pos.x;
                durin.telemetry.pos_y = pos.y;
                durin.telemetry.pos_z = pos.z;
                if (current_time - last_position_telemetry > durin.info.position_stream_period * 1000) {
                    send_position_telemetry();
                    last_position_telemetry = current_time;
                }
                if (current_time - last_uwb_nodes_telemetry > durin.info.position_stream_period * 1000) {
                    send_uwb_nodes_telemetry();
                    last_uwb_nodes_telemetry = current_time;
                }
            }
        } else {
            vTaskDelay(1);
            durin.telemetry.pos_x = 0;
            durin.telemetry.pos_y = 0;
            durin.telemetry.pos_z = 0;
            durin.telemetry.fix_type = 0;
            if (current_time - last_position_telemetry > (1000 + durin.info.position_stream_period) * 1000) {
                send_position_telemetry();
                last_position_telemetry = current_time;
            }
            if (current_time - last_uwb_nodes_telemetry > (1000 + durin.info.position_stream_period) * 1000) {
                send_uwb_nodes_telemetry();
                last_uwb_nodes_telemetry = current_time;
            }
        }
    }
    
}

void build_uwb_nodes_message(UwbNodes_ptr *ptr, struct capn_segment *cs) {
    struct UwbNodes msg;
    msg.nodes = new_UwbNode_list(cs, durin.telemetry.distance_old_cache_index);

    for (uint8_t i = 0; i < durin.telemetry.distance_old_cache_index; i++) {
        struct distance_measurement node = durin.telemetry.distance_data_old_chache[i];
        struct UwbNode capn_node;
        capn_node.nodeId = node.id;
        capn_node.distanceMm = node.distance;
        switch (node.purpose) {
            case UWB_PURPOSE_BEACON_ORIGIN:
                capn_node.purpose = UwbNodePurpose_origin;
                break;
            case UWB_PURPOSE_BEACON_X:
                capn_node.purpose = UwbNodePurpose_x;
                break;
            case UWB_PURPOSE_BEACON_Y:
                capn_node.purpose = UwbNodePurpose_y;
                break;
            case UWB_PURPOSE_BEACON_Z:
                capn_node.purpose = UwbNodePurpose_z;
                break;
            case UWB_PURPOSE_BEACON_REPEATER:
                capn_node.purpose = UwbNodePurpose_repeater;
                break;
            case UWB_PURPOSE_BEACON_PASSIVE:
                capn_node.purpose = UwbNodePurpose_passive;
                break;
            case UWB_PURPOSE_USER:
                capn_node.purpose = UwbNodePurpose_user;
                break;
        }
        if (node.flags & UWB_HAS_3D_POSITION_BITMASK) {
            capn_node.position_which = UwbNode_position_vectorMm;
            capn_node.position.vectorMm.x = node.position_x;
            capn_node.position.vectorMm.y = node.position_y;
            capn_node.position.vectorMm.z = node.position_z;
        } else {
            capn_node.position_which = UwbNode_position_unknown;
        }
        set_UwbNode(&capn_node, msg.nodes, i);
    }
    *ptr = new_UwbNodes(cs);
    write_UwbNodes(&msg, *ptr);
}

static void send_position_telemetry() {
    struct capn c;
    struct capn_segment *cs;
    struct DurinBase msg;
    init_durinbase(&c, &cs, &msg);
    struct Position position;    
    if (durin.telemetry.fix_type == 3) {
        position.which = Position_vectorMm;
        position.vectorMm.x = durin.telemetry.pos_x * 1000;
        position.vectorMm.y = durin.telemetry.pos_y * 1000;
        position.vectorMm.z = durin.telemetry.pos_z * 1000;
    } else {
        position.which = Position_unknown;
    }
    msg.position = new_Position(cs);
    write_Position(&position, msg.position);
    msg.which = DurinBase_position;

    uint8_t buf[128];
    uint16_t len = sizeof(buf);
    finish_durinbase(&c, &cs, &msg, buf, &len);
    send_telemetry(buf, len);
}

static void send_uwb_nodes_telemetry() {
    struct capn c;
    struct capn_segment *cs;
    struct DurinBase msg;
    init_durinbase(&c, &cs, &msg);

    struct UwbNodes uwb_nodes;
    build_uwb_nodes_message(&msg.uwbNodes, cs);
    msg.which = DurinBase_uwbNodes;
    uint8_t buf[1024];
    uint16_t len = sizeof(buf);
    finish_durinbase(&c, &cs, &msg, buf, &len);

    send_telemetry(buf, len);
}

static uint64_t get_tx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64_t get_rx_timestamp_u64(void) {
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void send_message_instantly(uint8_t* buf, uint8_t len) {
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
    xSemaphoreGive(uwb_mutex);
}

static void send_message_delayed(uint8_t* buf, uint8_t len, uint64_t ts) {
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    dwt_setdelayedtrxtime(ts);
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
    xSemaphoreGive(uwb_mutex);
}

static void send_message_global(uint8_t* buf, uint8_t len) {
    static uint8_t static_buf[64];
    memcpy(static_buf, buf, len);
    uwb_send_task_buf = static_buf;
    uwb_send_task_len = len;
    xTaskNotifyGive(uwb_send_task_handle);
}

static void uwb_parse_message() {
    uint8_t rx_buf[64];
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RX_BUFFER_MAX_LEN;
    xSemaphoreGive(uwb_mutex);
    if (length > sizeof(rx_buf)) {
        printf("too long\n");
        return;
    }
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_readrxdata(rx_buf, length, 0);
    xSemaphoreGive(uwb_mutex);

    struct uwb_header *header = (struct uwb_header *) rx_buf; 
    if (memcmp(header->magic, UWB_MAGIC_WORD, sizeof(header->magic)) != 0) {
        printf("invalid header %.*s\n", sizeof(header->magic), header->magic);
        return;
    }
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        last_poll_received_at = esp_timer_get_time();
    }
    last_message_received_at = esp_timer_get_time();
    if (header->receiver != durin_persistent.node_id && header->receiver != UWB_BROADCAST_ID) {
        return;
    }

    //Update active nodes
    if (header->purpose == UWB_PURPOSE_USER) {
        uint8_t exists = 0;
        for (uint8_t i = 0; i < user_ids_index; i++) {
            if (user_ids[i] == header->sender) {
                exists = 1;
                break;
            }
        }
        if (exists == 0) {
            user_ids[user_ids_index] = header->sender;
            user_ids_index++;
            printf("registered uwb node %d as user\n", header->sender);
        }
    }
    
    if (
        header->purpose == UWB_PURPOSE_BEACON_REPEATER ||
        header->purpose == UWB_PURPOSE_BEACON_X ||
        header->purpose == UWB_PURPOSE_BEACON_Y ||
        header->purpose == UWB_PURPOSE_BEACON_Z ||
        header->purpose == UWB_PURPOSE_BEACON_ORIGIN ||
        header->purpose == UWB_PURPOSE_BEACON_PASSIVE
    ) {
        uint8_t exists = 0;
        for (uint8_t i = 0; i < all_beacon_ids_index; i++) {
            if (all_beacon_ids[i] == header->sender) {
                exists = 1;
                break;
            }
        }
        if (exists == 0) {
            all_beacon_ids[all_beacon_ids_index] = header->sender;
            all_beacon_ids_index++;
            printf("registered uwb node %d as beacon %d\n", header->sender, header->purpose);
        }
    }
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        last_poll_received_at = esp_timer_get_time();
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t resp_tx_time = (poll_rx_ts + ((MIN_TRX_DELAY) * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        struct uwb_response_ranging tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_RESPONSE_RANGING,
            .header.sender = durin_persistent.node_id,
            .header.receiver = header->sender,
            .header.purpose = UWB_PURPOSE_USER,
            .rx_timestamp = poll_rx_ts,
            .tx_timestamp = resp_tx_ts
        };
        send_message_delayed(&tx_msg, sizeof(tx_msg), resp_tx_time);
    } else
    if (header->msg_type == UWB_MSG_RESPONSE_RANGING) {
        struct uwb_response_ranging *rx_msg = (struct uwb_response_ranging*) rx_buf;
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;
        /* Read carrier integrator value and calculate clock offset ratio. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
        /* Get timestamps embedded in response message. */
        poll_rx_ts = rx_msg->rx_timestamp;
        resp_tx_ts = rx_msg->tx_timestamp;
        // poll_tx_ts = dwt_readtxtimestamplo32();
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();
        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;
        // not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        if (tof < 0) {
            tof = 0;
        }
        struct distance_measurement new_measurement;
        new_measurement.id = header->sender;
        new_measurement.distance = 1000 * tof * SPEED_OF_LIGHT;
        new_measurement.error = rx_msg->error;
        new_measurement.position_x = rx_msg->position_x;
        new_measurement.position_y = rx_msg->position_y;
        new_measurement.position_z = rx_msg->position_z;
        new_measurement.purpose = rx_msg->header.purpose;
        new_measurement.flags = rx_msg->flags,
        durin.telemetry.distance_data[durin.telemetry.distance_index] = new_measurement;
        durin.telemetry.distance_index++;
    } else 
    if (header->msg_type == UWB_MSG_IS_ALIVE) {

    } else
    if (header->msg_type == UWB_MSG_POLL_ALIVE) {
        struct uwb_is_alive tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_IS_ALIVE,
            .header.sender = durin_persistent.node_id,
            .header.receiver = UWB_BROADCAST_ID,
            .header.purpose = UWB_PURPOSE_USER,
        };
        send_message_global(&tx_msg, sizeof(tx_msg));
    }
}