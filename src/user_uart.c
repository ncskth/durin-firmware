#include <driver/uart.h>

#include "pt.h"
#include "durin.h"
#include "hardware.h"
#include "protocol.h"

void init_user_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_USER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = UART_USER_RTS_THRESHOLD
    };

    uart_param_config(UART_USER, &uart_config);
    uart_set_pin(UART_USER, PIN_UART_USER_TX, PIN_UART_USER_RX, PIN_UART_USER_RTS, PIN_UART_USER_CTS);
    uart_driver_install(UART_USER, 512, 2048, 20, NULL, 0);
}

void update_user_uart(struct pt *pt) {
    PT_BEGIN(pt);

    static struct protocol_state prot_state;
    while (1) {
        uint16_t len;
        uart_get_buffered_data_len(UART_USER, &len);
        while(len--) {
            uint8_t byte;
            uart_read_bytes(UART_USER, &byte, 1, 0);
            protocol_parse_byte(&prot_state, byte);
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}