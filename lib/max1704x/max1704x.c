#pragma once

#include <stdint.h>
#include <stdbool.h>

#include <max1704x.h>

enum max1704x_reg {
    MAX1704X_REG_VCELL = 0x02,
    MAX1704X_REG_SOC = 0x04,
    MAX1704X_REG_MODE = 0x06,
    MAX1704X_REG_VERSION = 0x08,
    MAX1704X_REG_HIBRT = 0x0A,
    MAX1704X_REG_CONFIG = 0x0C,
    MAX1704X_REG_VALRT = 0x14,
    MAX1704X_REG_CRATE = 0x16,
    MAX1704X_REG_VRESET_ID = 0x18,
    MAX1704X_REG_STATUS = 0x1A,
    MAX1704X_REG_TABLE = 0x40,
    MAX1704X_REG_CMD = 0xFE
};

static void write(max1704x_t *dev, uint8_t reg, uint16_t data);
static uint16_t read(max1704x_t *dev, uint8_t reg);
static uint16_t buf_to_uint16(uint8_t *buf);
static void read_non_blocking(max1704x_t *dev, uint8_t reg, uint8_t len);

void max1704x_begin(max1704x_t *dev, uint8_t address, max1704x_platform_t platform, bool is_max17049) {
    dev->address = address;
    dev->platform = platform;
    dev->is_max17049 = is_max17049;
}

float max1704x_get_adc_volt(max1704x_t *dev) {
    uint16_t raw_adc = read(dev, MAX1704X_REG_VCELL);
    float cells = dev->is_max17049 ? 2.0 : 1.0;
    return (float) raw_adc * cells * 78.125f / 1000000.f;
}

float max1704x_get_soc_percent(max1704x_t *dev) {
    return read(dev, MAX1704X_REG_SOC) / 256.0f;
}

float max1704x_get_discharge_rate_percent_hour(max1704x_t *dev) {
    return (int16_t) read(dev, MAX1704X_REG_CRATE) * 0.208f;
}

void max1704x_get_interesting_async_start(max1704x_t *dev) {
    // lmao i don't know why it works when i double all reads. The datasheet specifically says to read ONLY 16 bits but hey if it works it works
    read_non_blocking(dev, MAX1704X_REG_VCELL, (MAX1704X_REG_CRATE - MAX1704X_REG_VCELL + 2) * 2);
}
void max1704x_get_interesting_async_end(max1704x_t *dev, float *volt, float *percent, float *discharge_rate) {
    uint16_t raw_adc = buf_to_uint16(dev->async_data);
    float cells = dev->is_max17049 ? 2.0f : 1.0f;
    *volt = raw_adc * cells * 78.125f / 1000000.f;
    *percent =  buf_to_uint16(dev->async_data + 4) / 256.0f;
    *discharge_rate = (int16_t) buf_to_uint16(dev->async_data + (MAX1704X_REG_CRATE - MAX1704X_REG_VCELL) * 2) * 0.208f;
}

void max1704x_quickstart(max1704x_t *dev) {
    uint16_t v = read(dev, MAX1704X_REG_MODE);
    v |= 1 << 14;
    write(dev, MAX1704X_REG_MODE, v);
}

void max1704x_enable_sleep(max1704x_t *dev, bool enabled) {
    uint16_t v = read(dev, MAX1704X_REG_MODE);
    v |= 1 << 13;
    write(dev, MAX1704X_REG_MODE, v);
}

bool max1704x_can_sleep(max1704x_t *dev) {
    return read(dev, MAX1704X_REG_MODE) & (1 << 13);
}

bool max1704x_is_hibernating(max1704x_t *dev) {
    return read(dev, MAX1704X_REG_MODE) & (1 << 12);
}

void max1704x_set_hibernate_threshold(max1704x_t *dev, float threshold) {
    uint16_t v = read(dev, MAX1704X_REG_HIBRT) & 0x00FF;
    if (threshold > 0.0)
    {
        if (threshold < 53.04){
            v |= (uint16_t)(threshold / 0.208) << 8;
        } else{
            v |= 0xFF00;
        }
    }
    write(dev, MAX1704X_REG_HIBRT, v);
}

void max1704x_set_hibernate_reactivate_threshold(max1704x_t *dev, float threshold) {
    uint16_t v = read(dev, MAX1704X_REG_HIBRT) & 0xFF00;
    if (threshold > 0.0)
    {
        if (threshold < 0.31875) {
            v |= (uint16_t)(threshold / 0.00125) & 0x00FF;
        } else {
            v |= 0x00FF;
        }
    }
    write(dev, MAX1704X_REG_HIBRT, v);
}

void max1704x_get_version(max1704x_t *dev) {
    return read(dev, MAX1704X_REG_VERSION);
}

void max1704x_set_rcomp_temperature(max1704x_t *dev, float temp) {
    uint8_t v = 0;
    if (temp > 20.0) v = 0x97 + (temp - 20.0) * -0.5;
    else             v = 0x97 + (temp - 20.0) * -5.0;
    max1704x_set_rcomp(dev, v);
}

void max1704x_set_rcomp(max1704x_t *dev, uint8_t rcomp) {
    uint16_t v = (read(dev, MAX1704X_REG_CONFIG) & 0x00FF) | (rcomp << 8);
    write(dev, MAX1704X_REG_CONFIG, v);
}

void max1704x_force_sleep(max1704x_t *dev, bool b) {
    uint16_t v = read(dev, MAX1704X_REG_CONFIG);
    if (b) {
        v |= 1 << 7;
    } else {
        v &= ~(1 << 7);
    }
    write(dev, MAX1704X_REG_CONFIG, v);
}


// IMPLEMENTATION

static void write(max1704x_t *dev, uint8_t reg, uint16_t value) {
    uint8_t data[2];
    data[0] = (value & 0xff00) >> 8; // big endian
    data[1] = value & 0x00ff;
    dev->platform.write(&data, 2, dev->address, reg, dev->platform.ctx);
}

static uint16_t read(max1704x_t *dev, uint8_t reg) {
    uint8_t data[2]; // big endian
    dev->platform.read(&data, 2, dev->address, reg, dev->platform.ctx);
    return data[1] + (data[0] << 8);
}

static uint16_t buf_to_uint16(uint8_t *buf) {
    return buf[1] + (buf[0] << 8);
}

static void read_non_blocking(max1704x_t *dev, uint8_t reg, uint8_t len) {
    dev->platform.read_non_blocking(dev->async_data, len, dev->address, reg, dev->platform.ctx);
}