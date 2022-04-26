#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <hal/i2c_hal.h>
#include <driver/gpio.h>
#include <hal/gpio_hal.h>
#include <stdint.h>

typedef struct nbe_i2c {
    volatile uint8_t *tx_buf;
    volatile uint16_t should_write;
    volatile uint16_t has_written;
    volatile uint8_t *rx_buf;
    volatile uint16_t should_read;
    volatile uint16_t has_read;
    volatile i2c_hal_context_t hi2c;
    volatile i2c_hw_cmd_t cmd;
    volatile uint8_t cmd_index;
    volatile uint8_t i2c_num;
    volatile uint8_t busy;
    volatile uint8_t preamble_size; //up to 32
} nbe_i2c_t;

uint8_t i2c_first_byte_read(uint8_t address);
uint8_t i2c_first_byte_write(uint8_t address);

uint8_t nbe_i2c_is_busy(nbe_i2c_t *nbe_i2c);
void nbe_i2c_reset(nbe_i2c_t *nbe_i2c);
void nbe_i2c_init(nbe_i2c_t *nbe_i2c, uint8_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t frequency);
void nbe_i2c_set_rx_buf(nbe_i2c_t *nbe_i2c, uint8_t *buf);
void nbe_i2c_set_tx_buf(nbe_i2c_t *nbe_i2c, uint8_t *buf);

void nbe_i2c_start(nbe_i2c_t *nbe_i2c);
void nbe_i2c_start_read(nbe_i2c_t *nbe_i2c, uint8_t address, uint8_t *tx_buf, uint8_t *rx_buf);
void nbe_i2c_start_write(nbe_i2c_t *nbe_i2c, uint8_t address, uint8_t *tx_buf, uint8_t *rx_buf);

void nbe_i2c_write_preamble(nbe_i2c_t *nbe_i2c, uint8_t *buf, uint8_t len); //max 32 bytes IN TOTAL

void nbe_i2c_write(nbe_i2c_t *nbe_i2c, uint8_t amount); // the library doesn't support a write after a read (maybe) :(
void nbe_i2c_read(nbe_i2c_t *nbe_i2c, uint8_t amount);
void nbe_i2c_read_nak(nbe_i2c_t *nbe_i2c, uint8_t amount);
void nbe_i2c_read_ack(nbe_i2c_t *nbe_i2c, uint8_t amount);

void nbe_i2c_stop(nbe_i2c_t *nbe_i2c);

void nbe_i2c_commit(nbe_i2c_t *nbe_i2c);

#ifdef __cplusplus
}
#endif