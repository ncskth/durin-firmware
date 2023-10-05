#pragma once
#ifndef MAX1704X_H
#define MAX1704X_H

#include <stdint.h>
#include <stdbool.h>

#define MAX1704X_ADDR 54

enum max1704x_alert {
    MAX1704X_ALERT_RESET = (1 << 0),  // Reset indicator
    MAX1704X_ALERT_VOLTAGE_HIGH = (1 << 1),  // Voltage high alert
    MAX1704X_ALERT_VOLTAGE_LOW = (1 << 2),  // Voltage low alert
    MAX1704X_ALERT_VOLTAGE_RESET = (1 << 3),  // Voltage reset alert
    MAX1704X_ALERT_SOC_LOW = (1 << 4),  // SOC low alert
    MAX1704X_ALERT_SOC_CHANGE = (1 << 5)   // SOC change alert
};

typedef struct max1704x_platform {
    void* ctx;
    void (*write)(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx);
    void (*read)(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx);
    void (*read_non_blocking)(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg, void* ctx);
} max1704x_platform_t;

typedef struct max1704x_dev {
    uint8_t address;
    bool is_max17049;
    max1704x_platform_t platform;
    uint8_t async_data[64];
} max1704x_t;

void max1704x_begin(max1704x_t *dev, uint8_t address, max1704x_platform_t platform, bool is_max_17049);

float max1704x_get_adc_volt(max1704x_t *dev);
float max1704x_get_soc_percent(max1704x_t *dev);
float max1704x_get_discharge_rate_percent_hour(max1704x_t *dev);

void max1704x_get_interesting_async_start(max1704x_t *dev);
void max1704x_get_interesting_async_end(max1704x_t *dev, float *volt, float *percent, float *discharge_rate);

void max1704x_quickstart(max1704x_t *dev);
void max1704x_enable_sleep(max1704x_t *dev, bool enabled);
bool max1704x_can_sleep(max1704x_t *dev);
bool max1704x_is_hibernating(max1704x_t *dev);
void max1704x_set_hibernate_threshold(max1704x_t *dev, float threshold);
void max1704x_set_hibernate_reactivate_threshold(max1704x_t *dev, float threshold);
void max1704x_get_version(max1704x_t *dev);
void max1704x_set_rcomp_temperature(max1704x_t *dev, float temp);
void max1704x_set_rcomp(max1704x_t *dev, uint8_t rcomp);
void max1704x_force_sleep(max1704x_t *dev, bool b);

#endif