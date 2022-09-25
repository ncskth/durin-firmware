#include <driver/uart.h>

#include "pt.h"
#include "durin.h"
#include "hardware.h"
#include "protocol.h"

// uint16_t get_tx_buffer_remaining(uint8_t uart_num) {
//     *size = p_uart_obj[uart_num]->tx_buf_size - p_uart_obj[uart_num]->tx_len_tot;
// }

void send_uart(uint8_t *buf, uint16_t len) {
    uart_write_bytes(UART_USER, buf, len);
}

void init_user_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_USER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        //.rx_flow_ctrl_thresh = UART_USER_RTS_THRESHOLD
    };
    ESP_ERROR_CHECK(uart_param_config(UART_USER, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_USER, PIN_UART_USER_TX, PIN_UART_USER_RX, PIN_UART_USER_RTS, PIN_UART_USER_CTS));
    ESP_ERROR_CHECK(uart_driver_install(UART_USER, 512, 6096, 0, NULL, 0));
}

void update_user_uart(struct pt *pt) {
    PT_BEGIN(pt);
    static struct protocol_state prot_state = {0};
    prot_state.channel = CHANNEL_UART;
    while (1) {
        uint8_t buf[512];
        uint16_t len;
        uart_get_buffered_data_len(UART_USER, &len);
        len = len > 512 ? 512 : len;
        uart_read_bytes(UART_USER, buf, len, 0);

        for (uint8_t i = 0; i < len; i++) {
            protocol_parse_byte(&prot_state, buf[i]);
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}