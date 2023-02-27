#include <driver/uart.h>

#include "pt.h"
#include "durin.h"
#include "hardware.h"
#include "protocol.h"

void send_uart(uint8_t *buf, uint16_t len) {
    #ifdef CONSOLE_ENABLED
    return;
    #endif
    if (durin.info.ota_in_progress) {
        return;
    }
    size_t remaining;
    uart_get_tx_buffer_free_size(UART_USER, &remaining);

    if (remaining < len) {
        printf("UART send buffer full\n");
        return;
    }
    // This is not optimal but uart_write_bytes becomes blocking for some reason...
    if (gpio_get_level(PIN_UART_USER_CTS) == 1) {
        printf("UART cts high\n");
        return;
    }
    uart_write_bytes(UART_USER, buf, len);
}

void init_user_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_USER_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    #ifndef CONSOLE_ENABLED
    ESP_ERROR_CHECK(uart_set_pin(UART_USER, PIN_UART_USER_TX, PIN_UART_USER_RX, PIN_UART_USER_RTS, PIN_UART_USER_CTS));
    ESP_ERROR_CHECK(uart_param_config(UART_USER, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_USER, 2048, 4096, 0, NULL, 0));
    #endif
}

void update_user_uart(struct pt *pt) {
    #ifdef CONSOLE_ENABLED
    return;
    #endif
    PT_BEGIN(pt);
    static struct protocol_state prot_state = {0};
    prot_state.channel = CHANNEL_UART;
    while (1) {
        uint8_t buf[1024];
        size_t len;
        uart_get_buffered_data_len(UART_USER, &len);
        len = len > sizeof(buf) ? sizeof(buf) : len;
        int read_len = uart_read_bytes(UART_USER, buf, len, 0);
        if (read_len == -1) {
            printf("UART ERROR\n");
            PT_YIELD(pt);
            continue;
        }
        for (size_t i = 0; i < read_len; i++) {
            protocol_parse_byte(&prot_state, buf[i]);
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}