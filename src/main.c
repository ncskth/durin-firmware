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
#include <stdio.h>
#include <esp_ipc.h>

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
#include "user_uart.h"

#include "capnp_c.h"
#include "schema.capnp.h"

#define MAIN_LOOP_PERIOD 1000 // us

void init_led();
int durin_writefn(void* cookie, const char* data, int size);
int IRAM_ATTR void_writefn(void*cookie, const char* data, int size);

extern icm20948_t icm;

void rt_loop(void* arg) {
    struct pt servo_pt;
    struct pt tof_and_expander_pt;
    struct pt misc_pt;
    struct pt imu_pt;
    struct pt uwb_pt;
    struct pt wifi_pt;
    struct pt tcp_server_pt;
    struct pt uart_pt;

    PT_INIT(&servo_pt);
    PT_INIT(&tof_and_expander_pt);
    PT_INIT(&misc_pt);
    PT_INIT(&imu_pt);
    PT_INIT(&uwb_pt);
    PT_INIT(&wifi_pt);
    PT_INIT(&uart_pt);

    while (!durin.info.init_finished) {vTaskDelay(1000 / portTICK_PERIOD_MS);}
    durin.info.cycle_count = 0;
    uint64_t start_time = 0;
    uint64_t end_time = esp_timer_get_time();
    uint64_t average_time = 0;
    uint64_t worst_time = 0;
    uint64_t best_time = 874983247;
    uint64_t last_check = 0;
    while (1) {
        start_time = end_time;
        durin.info.cycle_count += 1;
        update_user_uart(&uart_pt);
        update_tcp_server(&tcp_server_pt);
        update_tof_and_expander(&tof_and_expander_pt);
        update_servo(&servo_pt);
        update_misc(&misc_pt);
        update_wifi(&wifi_pt);
        update_imu(&imu_pt);

        end_time = esp_timer_get_time();
        uint64_t loop_time = end_time - start_time;
        average_time = 0.99 * average_time + (loop_time) * 0.01;
        if (loop_time > worst_time) {
            worst_time = loop_time;
        }
        if (loop_time < best_time) {
            best_time = loop_time;
        }

        if (durin.info.cycle_count % 1000 == 0) {
                printf("loop time average: %lld worst: %lld best: %lld total: %lld\n", average_time, worst_time, best_time, end_time - last_check);
                last_check = end_time;
                best_time = 9237492743;
                worst_time = 0;
                end_time = esp_timer_get_time();
        }
        while (end_time < start_time + MAIN_LOOP_PERIOD) {
            end_time = esp_timer_get_time();
        }
    }
    vTaskDelete(NULL);
}



void init_i2c(void* arg) {
    // i2c
    nbe_i2c_init(&durin.hw.i2c_tof, I2C_NUM_TOF, PIN_TOF_SDA, PIN_TOF_SCL, I2C_TOF_HZ);
    nbe_i2c_init(&durin.hw.i2c_imu, I2C_NUM_IMU, PIN_IMU_SDA, PIN_IMU_SCL, I2C_IMU_HZ);
}

void setup(void* arg) {
    printf("booting\n");

    esp_ipc_call_blocking(RT_CORE, init_i2c, NULL);

    // misc
    printf("init misc\n");
    init_misc();
    set_led(BLUE);

    // timer
    esp_timer_early_init();

    // init_i2c(NULL);

    // init TOF
    printf("init tof\n");
    init_tof_and_expander();

    // wifi
    printf("init wifi\n");
    init_wifi();

    // servo
    printf("init servo\n");
    init_servo();

    printf("init imu\n");
    init_imu();

    // uwb
    printf("init uwb\n");
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
    printf("version 11\n");
    // while (1) {
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
    vTaskDelete(NULL);
}

void app_main() {
    durin.info.logging_enabled = EnableLogging_disabled;

    init_user_uart();

    #ifndef CONSOLE_ENABLED
    _GLOBAL_REENT->_stdout = fwopen(NULL, &durin_writefn);
    static char stdout_buf[129]; //magic extra byte for null
    setvbuf(_GLOBAL_REENT->_stdout, stdout_buf, _IOLBF, 128);
    #endif

    durin.info.init_finished = 0;
    esp_event_loop_create_default();
    xTaskCreatePinnedToCore(rt_loop, "rt_loop", 22000 * 2, NULL, 5, NULL, RT_CORE);
    xTaskCreatePinnedToCore(setup, "setup", 15000 * 2, NULL, 5, NULL, TRASH_CORE);
    return;
}



static capn_text chars_to_text(const char *chars) {
    return (capn_text) {
        .len = (int) strlen(chars),
        .str = chars,
        .seg = NULL,
    };
}

int IRAM_ATTR void_writefn(void*cookie, const char* data, int size) {
    return size;
}

int IRAM_ATTR durin_writefn(void* cookie, const char* data, int size) {
    static bool in_writefn = 0;
    if (in_writefn) {
        goto end; //prevent recursive writes
    }
    if (durin.info.logging_enabled == EnableLogging_disabled) {
        goto end;
    }
    if (durin.info.ota_in_progress) {
        goto end;
    }
    in_writefn = 1;
    struct capn c;
    capn_init_malloc(&c);
    struct capn_segment *cs = capn_root(&c).seg;
    struct DurinBase msg;
    struct TextLogging log;
    ((char*) data)[size] = '\0';
    log.log = (struct capn_text) {
        .len = size,
        .str = data,
        .seg = NULL,
    };
    int e = 0;
    uint8_t *buf = malloc(size + 100);

    uint16_t len;
    msg.textLogging = new_TextLogging(cs);
    msg.which = DurinBase_textLogging;
    DurinBase_ptr durin_ptr = new_DurinBase(cs);
    write_TextLogging(&log, msg.textLogging);
    write_DurinBase(&msg, durin_ptr);
    e = capn_setp(capn_root(&c), 0, durin_ptr.p);
    len = capn_write_mem(&c, buf, size + 100, CAPN_PACKED);

    if (durin.info.logging_enabled == EnableLogging_uart) {
        send_response(buf, len, CHANNEL_UART);
    }
    if (durin.info.logging_enabled == EnableLogging_tcp) {
        send_response(buf, len, CHANNEL_TCP);
    }
    if (durin.info.logging_enabled == EnableLogging_both) {
        send_response(buf, len, CHANNEL_TCP);
        send_response(buf, len, CHANNEL_UART);
    }
    capn_free(&c);
    free(buf);
    in_writefn = 0;

    end:
    return size;
}