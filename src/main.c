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
#include "deca_device_api.h"

#include "tof_and_expander.h"
#include "wifi.h"
#include "servo.h"
#include "misc.h"
#include "imu.h"
#include "uwb.h"

#define MAIN_LOOP_PERIOD 1000 // us

void init_led();

extern icm20948_t icm;

void rt_loop(void* arg) {
    struct pt servo_pt;
    struct pt tof_and_expander_pt;
    struct pt misc_pt;
    struct pt imu_pt;
    struct pt uwb_pt;
    struct pt wifi_pt;
    struct pt tcp_server_pt;

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
    uint64_t average_time = 0;
    while (1) {
        start_time = end_time;
        durin.info.cycle_count += 1;
        update_tcp_server(&tcp_server_pt);
        update_tof_and_expander(&tof_and_expander_pt);
        update_servo(&servo_pt);
        update_misc(&misc_pt);
        update_wifi(&wifi_pt);
        update_imu(&imu_pt);

        if (durin.info.cycle_count % 1000 == 0) {
                // float ax, ay, az, gx, gy, gz, mx, my, mz;
                // icm20948_parseAllMetric(&icm, &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
                // printf("accelerometer metric: %f %f %f %f %f %f %f %f %f\n", ax, ay, az, gx, gy, gz, mx, my, mz);
                printf("uwb messages %d\n", durin.info.uwb_messages_received);
                durin.info.uwb_messages_received = 0;
        }

        while (end_time < start_time + MAIN_LOOP_PERIOD) {
            end_time = esp_timer_get_time();
        }
        average_time = 0.99 * average_time + (end_time - start_time) * 0.01;
        if (average_time > MAIN_LOOP_PERIOD * 1.05 && durin.info.cycle_count % 1000 == 0) {
            printf("loop time %lld\n", average_time);
        }
    }
    vTaskDelete(NULL);
}

//run all drivers on core1
//run main loop on core0
void setup(void* arg) {
    printf("booting\n");

    // misc
    init_misc();
    set_led(BLUE);

    // timer
    esp_timer_early_init();

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

    // i2c
    nbe_i2c_init(&durin.hw.i2c_tof, I2C_NUM_TOF, PIN_TOF_SDA, PIN_TOF_SCL, I2C_TOF_HZ);
    nbe_i2c_init(&durin.hw.i2c_imu, I2C_NUM_IMU, PIN_IMU_SDA, PIN_IMU_SCL, I2C_IMU_HZ);

    // servo
    init_servo();

    // init TOF
    init_tof_and_expander();

    // wifi
    init_wifi();

    init_imu();

    // uwb
    init_uwb();

    uint8_t working_tof = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (durin.info.tof_sensor_alive[i]) {
            working_tof += 1;
        }
    }

    if (working_tof == 8) {
        set_led(YELLOW);
    }
    else if (working_tof > 0) {
        set_led(PINK);
    } else {
        set_led(RED);
    }
    durin.info.init_finished = 1;
    printf("init done\n");
    printf("version 10\n");
    // while (1) {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    vTaskDelete(NULL);
}

void app_main() {
    durin.info.init_finished = 0;
    esp_event_loop_create_default();

    xTaskCreatePinnedToCore(rt_loop, "rt_loop", 4086, NULL, 5, NULL, RT_CORE);
    xTaskCreatePinnedToCore(setup, "setup", 4086 * 2, NULL, 5, NULL, TRASH_CORE);
    return;
}