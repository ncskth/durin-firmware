#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <string.h>

#include "misc.h"
#include "hardware.h"
#include "durin.h"
#include "esp_timer.h"
#include "tof_and_expander.h"

#define DEFAULT_VREF 1100 
#define VOLT_LP_GAIN 0.9995

#define RAW

#ifdef RAW
#define BAT_K 5.45
#define BAT_M 0.232
#endif

#ifdef AMP
#define BAT_K ((2.2 + 0.47) / 0.47)
#define BAT_M 0
#endif


esp_adc_cal_characteristics_t *adc_chars;
nvs_handle_t durin_nvs;
static uint64_t power_off_when = 0;

void update_persistent_data() {
    nvs_set_blob(durin_nvs, "durin_nvs", &durin_persistent, sizeof(durin_persistent));
}

void power_off() {
    gpio_set_level(PIN_3V3_EN, 0);
    gpio_set_direction(PIN_3V3_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_3V3_EN, 0);
}

void power_off_in(uint64_t delay) {
    power_off_when = esp_timer_get_time() + delay;
}

void set_led(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, r);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, g);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, b);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);   
}

void set_buzzer(uint8_t intensity) {
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_BUZZER, intensity > 0);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_BUZZER);
}

void init_misc() {
    // install gpio isr
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    durin.info.motor_enabled = false;
    durin.info.user_enabled = false;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    err = nvs_open("storage", NVS_READWRITE, &durin_nvs);
    uint len = 255;
    err = nvs_get_blob(durin_nvs, "durin_nvs", &durin_persistent, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        memcpy(durin_persistent.main_ssid, DEFAULT_SSID, sizeof(DEFAULT_SSID));
        memcpy(durin_persistent.main_password, DEFAULT_PASSWORD, sizeof(DEFAULT_PASSWORD));
        durin_persistent.node_id = 255;
        update_persistent_data();
    }

    gpio_set_direction(PIN_BUTTON_IN, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_VBAT_SENSE_GND, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_VBAT_SENSE_GND, 0);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, DEFAULT_VREF, adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("eFuse Vref\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Two Point\n");
    } else {
        printf("Default\n");
    }
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(CHANNEL_BAT_SENSE, ADC_ATTEN_11db);

    // led
    ledc_timer_config_t led_timer_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_R,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_G,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_B,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_buzz = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_BUZZER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = BUZZER_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_channel_config_t led_channel_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_R,
        .timer_sel = TIMER_LED_R,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_R,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_G,
        .timer_sel = TIMER_LED_G,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_G,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_B,
        .timer_sel = TIMER_LED_B,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_B,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
        ledc_channel_config_t led_channel_conf_buzz = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_BUZZER,
        .timer_sel = TIMER_BUZZER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_BUZZER,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_timer_config(&led_timer_conf_r);
    ledc_timer_config(&led_timer_conf_g);
    ledc_timer_config(&led_timer_conf_b);
    ledc_timer_config(&led_timer_conf_buzz);

    ledc_channel_config(&led_channel_conf_r);
    ledc_channel_config(&led_channel_conf_g);
    ledc_channel_config(&led_channel_conf_b);
    ledc_channel_config(&led_channel_conf_buzz);

    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, 0);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_BUZZER, 0);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_BUZZER);

    //disable user port
    write_expander_pin(EX_PIN_USER_EN, 0);
    configure_expander_pin(EX_PIN_USER_EN, 0);

    write_expander_pin(EX_PIN_SERVO_EN, 0);
    configure_expander_pin(EX_PIN_SERVO_EN, 0);

    durin.info.last_message_received = esp_timer_get_time();


    uint16_t raw_adc = adc1_get_raw(CHANNEL_BAT_SENSE);
    float new_battery_voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc_chars) / 1000.0;
    new_battery_voltage = BAT_K * new_battery_voltage + BAT_M;
    durin.telemetry.battery_voltage = new_battery_voltage;

    for (uint8_t i = 0; i < 100; i++) {
        uint16_t raw_adc = adc1_get_raw(CHANNEL_BAT_SENSE);
        float new_battery_voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc_chars) / 1000.0;
        new_battery_voltage = BAT_K * new_battery_voltage + BAT_M;
        durin.telemetry.battery_voltage = new_battery_voltage * (1 - 0.99) + durin.telemetry.battery_voltage * 0.99;
    }
}

void update_misc(struct pt *pt) {
    PT_BEGIN(pt);
    static uint64_t pressed_at = 0;
    static uint64_t pressed_previously = 0;
    static uint64_t last_action = 0;
    last_action = esp_timer_get_time();

    while (1) {
        uint64_t current_time = esp_timer_get_time();
        uint64_t pressed_for;
        uint8_t released;
        uint8_t pressed;
        if (esp_timer_get_time() - durin.info.last_message_received > 30*1000*1000 && durin.info.active) {
            durin.info.active = false;
            durin.info.streaming_enabled = false;
        }

        if (gpio_get_level(PIN_BUTTON_IN)) {
            released = false;
            pressed = true;
            if (!pressed_previously) {
                pressed_previously = true;
                pressed_at = current_time;
            }
            pressed_for = current_time - pressed_at;
        } else {
            pressed = false;
            if (pressed_previously) {
                released = true;
            } else {
                released = false;
            }
            pressed_for = 0;
            pressed_previously = false;
        }

        if (released && current_time - last_action > 1000000) {
            // toggle motor and user en
            durin.info.motor_enabled = !durin.info.motor_enabled;
            durin.info.user_enabled = durin.info.motor_enabled;
            write_expander_pin(EX_PIN_SERVO_EN, durin.info.motor_enabled);
            write_expander_pin(EX_PIN_USER_EN, durin.info.motor_enabled);
            last_action = current_time;
            if (durin.info.motor_enabled) {
                set_led(GREEN);
            } else {
                set_led(YELLOW);
            }
        }

        if (pressed_for > 1000000) {
            //make the esp kill itself :(
            printf("i am dead goodbye! \n");
            set_led(RED);
            vTaskDelay(100);
            power_off();
        }

        uint16_t raw_adc = adc1_get_raw(CHANNEL_BAT_SENSE);
        float new_battery_voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc_chars) / 1000.0;
        new_battery_voltage = BAT_K * new_battery_voltage + BAT_M;
        // printf("battery %f %f\n", new_battery_voltage, durin.telemetry.battery_voltage);
        durin.telemetry.battery_voltage = new_battery_voltage * (1 - VOLT_LP_GAIN) + durin.telemetry.battery_voltage * VOLT_LP_GAIN;

        if (power_off_when && esp_timer_get_time() > power_off_when) {
            printf("power off in\n");
            vTaskDelay(100);
            power_off();
        }

        if (durin.telemetry.battery_voltage < 6.7) {
            // printf("no battery\n");
            // vTaskDelay(100);
            // power_off();
        }
        PT_YIELD(pt);
    }
    PT_END(pt);
}