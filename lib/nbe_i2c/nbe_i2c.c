#ifdef __cplusplus
extern "C" {
#endif

#include <hal/i2c_hal.h>
#include <driver/gpio.h>
#include <hal/gpio_hal.h>
#include <soc/i2c_periph.h>
#include <driver/periph_ctrl.h>
#include <esp_rom_gpio.h>
#include <esp_rom_sys.h>
#include <esp_intr_alloc.h>
#include <stdint.h>

#include "nbe_i2c.h"

#define NBE_I2C_FILTER_CYC_NUM_DEF 7
#define NBE_I2C_IO_INIT_LEVEL 1

#define NBE_I2C_FIFO_FULL_THRESH_VAL 22
#define NBE_I2C_FIFO_EMPTY_THRESH_VAL 10

#define NBE_I2C_FIFO_SIZE 32

static void nbe_i2c_end(nbe_i2c_t *nbe_i2c);
static void nbe_i2c_set_pin(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num, bool sda_pullup_en, bool scl_pullup_en);
static void IRAM_ATTR nbe_i2c_isr(void *arg);

static void nbe_i2c_set_pin(i2c_port_t i2c_num, gpio_num_t sda_io_num, gpio_num_t scl_io_num, bool sda_pullup_en, bool scl_pullup_en) {
    int sda_in_sig, sda_out_sig, scl_in_sig, scl_out_sig;
    sda_out_sig = i2c_periph_signal[i2c_num].sda_out_sig;
    sda_in_sig = i2c_periph_signal[i2c_num].sda_in_sig;
    scl_out_sig = i2c_periph_signal[i2c_num].scl_out_sig;
    scl_in_sig = i2c_periph_signal[i2c_num].scl_in_sig;

    gpio_set_level(sda_io_num, NBE_I2C_IO_INIT_LEVEL);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[sda_io_num], PIN_FUNC_GPIO);
    gpio_set_direction(sda_io_num, GPIO_MODE_INPUT_OUTPUT_OD);
    esp_rom_gpio_connect_out_signal(sda_io_num, sda_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(sda_io_num, sda_in_sig, 0);
    if (sda_pullup_en == GPIO_PULLUP_ENABLE) {
        gpio_set_pull_mode(sda_io_num, GPIO_PULLUP_ONLY);
    } else {
        gpio_set_pull_mode(sda_io_num, GPIO_FLOATING);
    }

    gpio_set_level(scl_io_num, NBE_I2C_IO_INIT_LEVEL);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[scl_io_num], PIN_FUNC_GPIO);
    gpio_set_direction(scl_io_num, GPIO_MODE_INPUT_OUTPUT_OD);
    esp_rom_gpio_connect_out_signal(scl_io_num, scl_out_sig, 0, 0);
    esp_rom_gpio_connect_in_signal(scl_io_num, scl_in_sig, 0);
    if (scl_pullup_en == GPIO_PULLUP_ENABLE) {
        gpio_set_pull_mode(scl_io_num, GPIO_PULLUP_ONLY);
    } else {
        gpio_set_pull_mode(scl_io_num, GPIO_FLOATING);
    }
}

#pragma GCC optimize ("O3")
static void IRAM_ATTR nbe_i2c_isr(void *arg) {
    nbe_i2c_t *nbe_i2c = (nbe_i2c_t*) arg;
    i2c_dev_t *dev = nbe_i2c->hi2c.dev;
    if (dev->int_status.tx_send_empty) {
        uint8_t *tx_start = nbe_i2c->tx_buf + nbe_i2c->has_written;
        uint8_t max_len = NBE_I2C_FIFO_SIZE - NBE_I2C_FIFO_EMPTY_THRESH_VAL;
        uint16_t to_send = nbe_i2c->should_write - nbe_i2c->has_written;
        uint8_t len = to_send < max_len ? to_send : max_len;
        nbe_i2c->has_written += len;
        i2c_ll_write_txfifo(dev, tx_start, len);
    }
    else if (dev->int_status.rx_rec_full) {
        uint8_t *rx_start = nbe_i2c->rx_buf + nbe_i2c->has_read;
        i2c_ll_read_rxfifo(dev, rx_start, NBE_I2C_FIFO_FULL_THRESH_VAL);
        nbe_i2c->has_read += NBE_I2C_FIFO_FULL_THRESH_VAL;
    }
    else if (dev->int_status.trans_complete) {
        uint8_t *rx_start = nbe_i2c->rx_buf + nbe_i2c->has_read;
        uint8_t len = i2c_ll_get_rxfifo_cnt(dev);
        i2c_ll_read_rxfifo(dev, rx_start, len);
        nbe_i2c->busy = 0;
    } 
    else if (dev->int_status.end_detect) {
        uint8_t *rx_start = nbe_i2c->rx_buf + nbe_i2c->has_read;
        uint8_t len = i2c_ll_get_rxfifo_cnt(dev);
        i2c_ll_read_rxfifo(dev, rx_start, len);
        nbe_i2c->busy = 0;
    }
    else if (dev->int_status.time_out) {
        nbe_i2c->busy = 0;
    } 
    else if (dev->int_status.arbitration_lost) {
        nbe_i2c->busy = 0;
    }

    dev->int_clr.val = (uint32_t) 0xffff;
    return;
}

uint8_t i2c_first_byte_read(uint8_t address) {
    return address << 1 | 1;
}

uint8_t i2c_first_byte_write(uint8_t address) {
    return address << 1 | 0;
}

void nbe_i2c_init(nbe_i2c_t *nbe_i2c, uint8_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t frequency) {
    nbe_i2c->i2c_num = i2c_num;
    nbe_i2c->hi2c.dev = I2C_LL_GET_HW(nbe_i2c->i2c_num);
    nbe_i2c->cmd.ack_en = 0; //dont care about ack
    nbe_i2c->cmd.ack_exp = 0; //expect a 0
    nbe_i2c->cmd.ack_val = 1; //what the master will send as ACK
    
    nbe_i2c_set_pin(nbe_i2c->i2c_num, sda, scl, 1, 1); // always pullup
    periph_module_enable(i2c_periph_signal[nbe_i2c->i2c_num].module);
    i2c_hal_master_init(&nbe_i2c->hi2c, nbe_i2c->i2c_num);
    i2c_hal_set_filter(&nbe_i2c->hi2c, NBE_I2C_FILTER_CYC_NUM_DEF);
    i2c_hal_set_bus_timing(&nbe_i2c->hi2c, frequency, I2C_SCLK_APB);

    nbe_i2c->hi2c.dev->int_ena.val = 0; //disable all
    nbe_i2c->hi2c.dev->int_ena.rx_rec_full = 1;
    nbe_i2c->hi2c.dev->int_ena.tx_send_empty = 1;
    nbe_i2c->hi2c.dev->int_ena.trans_complete = 1;
    nbe_i2c->hi2c.dev->int_ena.end_detect = 1;
    nbe_i2c->hi2c.dev->int_ena.arbitration_lost = 1;
    nbe_i2c->hi2c.dev->int_ena.time_out = 1;

    i2c_hal_set_rxfifo_full_thr(&nbe_i2c->hi2c, NBE_I2C_FIFO_FULL_THRESH_VAL);
    i2c_hal_set_txfifo_empty_thr(&nbe_i2c->hi2c, NBE_I2C_FIFO_EMPTY_THRESH_VAL);
    esp_intr_alloc(i2c_periph_signal[nbe_i2c->i2c_num].irq, ESP_INTR_FLAG_IRAM, nbe_i2c_isr, nbe_i2c, NULL);
    nbe_i2c_reset(nbe_i2c);
}

uint8_t nbe_i2c_is_busy(nbe_i2c_t *nbe_i2c) {
    return nbe_i2c->busy;
}

void nbe_i2c_set_rx_buf(nbe_i2c_t *nbe_i2c, uint8_t *buf) {
    nbe_i2c->rx_buf = buf;
}

void nbe_i2c_set_tx_buf(nbe_i2c_t *nbe_i2c, uint8_t *buf) {
    nbe_i2c->tx_buf = buf;
}

void nbe_i2c_reset(nbe_i2c_t *nbe_i2c) {
    i2c_hal_rxfifo_rst(&nbe_i2c->hi2c);
    i2c_hal_txfifo_rst(&nbe_i2c->hi2c);
    nbe_i2c->should_read = 0;
    nbe_i2c->has_read = 0;
    nbe_i2c->should_write = 0;
    nbe_i2c->has_written = 0;
    nbe_i2c->cmd_index = 0;
    nbe_i2c->preamble_size = 0;
}

void nbe_i2c_start(nbe_i2c_t *nbe_i2c) {
    nbe_i2c->cmd.op_code = I2C_LL_CMD_RESTART;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
}

void nbe_i2c_start_write(nbe_i2c_t *nbe_i2c, uint8_t address, uint8_t *tx_buf, uint8_t *rx_buf) {
    nbe_i2c_reset(nbe_i2c);
    nbe_i2c_set_tx_buf(nbe_i2c, tx_buf);
    nbe_i2c_set_rx_buf(nbe_i2c, rx_buf);
    uint8_t byte = i2c_first_byte_write(address);
    nbe_i2c_start(nbe_i2c);
    nbe_i2c_write_preamble(nbe_i2c, &byte, 1);

}

void nbe_i2c_start_read(nbe_i2c_t *nbe_i2c, uint8_t address, uint8_t *tx_buf, uint8_t *rx_buf) {
    nbe_i2c_reset(nbe_i2c);
    nbe_i2c_set_tx_buf(nbe_i2c, tx_buf);
    nbe_i2c_set_rx_buf(nbe_i2c, rx_buf);
    uint8_t byte = i2c_first_byte_read(address);
    nbe_i2c_start(nbe_i2c);
    nbe_i2c_write_preamble(nbe_i2c, &byte, 1);
}

void nbe_i2c_write_preamble(nbe_i2c_t *nbe_i2c, uint8_t *buf, uint8_t len) {
    i2c_hal_write_txfifo(&nbe_i2c->hi2c, buf, len);
    nbe_i2c->cmd.op_code = I2C_LL_CMD_WRITE;
    nbe_i2c->cmd.byte_num = len;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
    nbe_i2c->preamble_size += len;
}

void nbe_i2c_write(nbe_i2c_t *nbe_i2c, uint8_t amount) {
    nbe_i2c->cmd.op_code = I2C_LL_CMD_WRITE;
    nbe_i2c->cmd.byte_num = amount;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
    nbe_i2c->should_write += amount; 
}

void nbe_i2c_read(nbe_i2c_t *nbe_i2c, uint8_t amount) {
    nbe_i2c->cmd.op_code = I2C_LL_CMD_READ;
    nbe_i2c->cmd.byte_num = amount;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
    nbe_i2c->should_read += amount;
}

static void nbe_i2c_end(nbe_i2c_t *nbe_i2c) {
    nbe_i2c->cmd.op_code = I2C_LL_CMD_END;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
}

void nbe_i2c_stop(nbe_i2c_t *nbe_i2c) {
    nbe_i2c->cmd.op_code = I2C_LL_CMD_STOP;
    i2c_hal_write_cmd_reg(&nbe_i2c->hi2c, nbe_i2c->cmd, nbe_i2c->cmd_index);
    nbe_i2c->cmd_index++;
}

void nbe_i2c_commit(nbe_i2c_t *nbe_i2c) {
    nbe_i2c_end(nbe_i2c);
    nbe_i2c->has_written = nbe_i2c->should_write < (NBE_I2C_FIFO_SIZE - nbe_i2c->preamble_size) ? nbe_i2c->should_write : (NBE_I2C_FIFO_SIZE - nbe_i2c->preamble_size); 
    nbe_i2c->busy = 1;
    i2c_hal_write_txfifo(&nbe_i2c->hi2c, nbe_i2c->tx_buf, nbe_i2c->has_written);
    i2c_hal_trans_start(&nbe_i2c->hi2c);
}

#ifdef __cplusplus
}
#endif