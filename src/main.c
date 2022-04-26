#include <driver/uart.h>
#include <hal/i2c_hal.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <string.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/gpio_hal.h>
#include <nvs_flash.h>
#include <mdns.h>
#include <sys/socket.h>
#include <netdb.h>
#include <esp_netif.h>
#include <driver/ledc.h>
#include <driver/adc.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>

#include "hardware.h"
#include "nbe_i2c.h"
#include "dynamixel.h"
#include "durin.h"
#include "pt.h"
#include "icm20948.h"
#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_async.h"

#include "tof_and_expander.h"
#include "wifi.h"
#include "servo.h"
#include "misc.h"
#include "imu.h"

#define MAIN_LOOP_PERIOD 1000 // us

void init_led();

void core0_task(void* arg) {
    struct pt servo_pt;
    struct pt tof_and_expander_pt;
    struct pt misc_pt;
    struct pt imu_pt;
    struct pt uwb_pt;
    struct pt wifi_pt;

    PT_INIT(&servo_pt);
    PT_INIT(&tof_and_expander_pt);
    PT_INIT(&misc_pt);
    PT_INIT(&imu_pt);
    PT_INIT(&uwb_pt);
    PT_INIT(&wifi_pt);

    while (!durin.info.init_finished) {vTaskDelay(1000 / portTICK_PERIOD_MS);}

    durin.info.cycle_count = 0;
    uint64_t start_time = 0;
    uint64_t end_time = esp_timer_get_time();

    while (1) {
        uint64_t start_time = end_time;
        durin.info.cycle_count += 1;
        update_tof_and_expander(&tof_and_expander_pt);
        update_servo(&servo_pt);
        update_misc(&misc_pt);
        update_imu(&imu_pt);
        update_wifi(&wifi_pt);

        while (end_time < start_time + MAIN_LOOP_PERIOD) {
            end_time = esp_timer_get_time();
            // vTaskDelay(0);
        }
        // vTaskDelay(0);
    }
    vTaskDelete(NULL);
}

//run all drivers on core1
//run main loop on core0
void core1_task(void* arg) {
    // misc
    init_misc();

    // timer
    esp_timer_early_init();

    // uwb spi
    spi_bus_config_t spi_config = {
        .sclk_io_num = PIN_SPI_UWB_SCK,
        .miso_io_num = PIN_SPI_UWB_MISO,
        .mosi_io_num = PIN_SPI_UWB_MOSI,
    };
    spi_bus_initialize(SPI2_HOST, &spi_config, SPI_DMA_CH_AUTO);

    // i2c
    nbe_i2c_init(&durin.hw.i2c_tof, I2C_NUM_TOF, PIN_TOF_SDA, PIN_TOF_SCL, I2C_TOF_HZ);
    nbe_i2c_init(&durin.hw.i2c_imu, I2C_NUM_IMU, PIN_IMU_SDA, PIN_IMU_SCL, I2C_IMU_HZ);

    // uart2
    uart_config_t uart_servo = {
        .baud_rate = UART_SERVO_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_SERVO, &uart_servo);
    uart_set_pin(UART_NUM_2, PIN_UART_SERVO_TX, PIN_UART_SERVO_RX, GPIO_NUM_NC, GPIO_NUM_NC);
    uart_driver_install(UART_NUM_2, 256, 256, 20, NULL, 0);
    
    // servo
    init_servo();

    init_imu();

    // init TOF
    init_tof_and_expander();

    // NVS
    nvs_flash_init();

    // wifi
    init_wifi();    

    durin.info.init_finished = 1;
    printf("init done\n");

    while (1) {

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main() {
    durin.info.init_finished = 0;
    // enable the voltage regulator really fast
    // set level first to prevent it from going to ground (and killing itself)
    gpio_set_level(PIN_3V3_EN, 1);
    gpio_set_direction(PIN_3V3_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_3V3_EN, 1);
    esp_event_loop_create_default();
    xTaskCreatePinnedToCore(core0_task, "core_0", 4086, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(core1_task, "core_1", 4086 * 2, NULL, 5, NULL, 1);
    return;
}