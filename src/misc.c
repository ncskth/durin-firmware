#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

#include "misc.h"
#include "hardware.h"
#include "durin.h"
#include "esp_timer.h"

#define DEFAULT_VREF 1100 
#define VOLT_LP_GAIN 0.995

esp_adc_cal_characteristics_t *adc_chars;

void set_led(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, r);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, g);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, b);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);   
}

void init_misc() {
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
}

void update_misc(struct pt *pt) {
    static uint64_t button_last_not_pressed = 0;

    button_last_not_pressed = esp_timer_get_time();

    PT_BEGIN(pt);

    while (1) {
        uint64_t current_time = esp_timer_get_time();
        if (gpio_get_level(PIN_BUTTON_IN) == 0) {
            button_last_not_pressed = current_time;
        }

        if (current_time - button_last_not_pressed > 100) {
            // toggle motor and user en
            durin.hw.port_expander_ouput ^= (1 << EX_PIN_USER_EN) | (1 << EX_PIN_SERVO_EN); 
        }

        if (current_time - button_last_not_pressed > 2000) {
            //make the esp kill itself :(
            gpio_set_level(PIN_3V3_EN, 0);
            gpio_set_direction(PIN_3V3_EN, GPIO_MODE_OUTPUT);
            gpio_set_level(PIN_3V3_EN, 0);
        }

        uint16_t raw_adc = adc1_get_raw(CHANNEL_BAT_SENSE);
        // float new_battery_voltage = ((float) raw_adc) / 4096 * 3.3 * 5; // 3 for the voltage divider
        float new_battery_voltage = esp_adc_cal_raw_to_voltage(raw_adc, adc_chars) / 1000.0;
        durin.telemetry.battery_voltage = new_battery_voltage * (1 - VOLT_LP_GAIN) + durin.telemetry.battery_voltage * VOLT_LP_GAIN;
        // printf("adc %f %f %d\n", durin.telemetry.battery_voltage, ((float) raw_adc) / 4096.0 * 3.3 * (2700.0 + 10000.0) / 2700.0, esp_adc_cal_raw_to_voltage(raw_adc, adc_chars));
        PT_YIELD(pt);
    }
    PT_END(pt);
}