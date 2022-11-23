#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_async.h"

#include "durin.h"
#include "hardware.h"
#include "tof_and_expander.h"
#include "pt.h"
#include "protocol.h"

#define EXPANDER_ADDRESS (0x20)

#define NUM_VL53L5CX 8
#define VL53L5CX_ADDRESS_CHAIN_START 0x41
VL53L5CX_Configuration tof_sensors[NUM_VL53L5CX];

uint16_t expander_current_output = 0;
uint16_t expander_current_input = 0;
uint16_t expander_current_configuration = ~0;

uint16_t expander_wanted_output = 0;
uint16_t expander_wanted_configuration = ~0;

#define TOF_I2C_WAIT() do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_tof))
#define TOF_I2C_WAIT_BLOCK() do {} while (nbe_i2c_is_busy(&durin.hw.i2c_tof))

void expander_write(uint16_t output);
void expander_read(uint8_t *buf);
uint16_t expander_parse(uint8_t *buf);

enum TofResolutions wanted_tof_resolution = 0;

void set_tof_resolution(enum TofResolutions resolution) {
    wanted_tof_resolution = resolution;
}

void init_tof_and_expander() {
    for (uint8_t i = 0; i < 8; i++) {
        configure_expander_pin(i, 0);
    }
    for (uint8_t i = 0; i < 8; i++) {
        write_expander_pin(i, 0);
    }

    VL53L5CX_Platform platform;
    platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    platform.nbe_i2c = &durin.hw.i2c_tof;
    tof_sensors[0].platform = platform;
    vl53l5cx_init(&tof_sensors[0]); //init all of them
    //reset all of them
    for (uint8_t i = 0; i < 8; i++) {
        write_expander_pin(i, 1);
    }

    for (uint8_t i = 0 ; i < NUM_VL53L5CX; i++) {
        //unreset sensor
        write_expander_pin(i, 0);
        VL53L5CX_Platform platform;
        platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
        platform.nbe_i2c = &durin.hw.i2c_tof;
        tof_sensors[i].platform = platform;
        uint8_t alive;
        vl53l5cx_is_alive(&tof_sensors[i], &alive);
        if (!alive) {
            durin.info.tof_sensor_alive[i] = 0;
            printf("dead tof sensor with index %d\n", i);
            continue;
        }
        printf("starting TOF %d\n", i);
        durin.info.tof_sensor_alive[i] = 1;
        vl53l5cx_set_ranging_mode(&tof_sensors[i], VL53L5CX_RANGING_MODE_CONTINUOUS);
        vl53l5cx_set_ranging_frequency_hz(&tof_sensors[i], 15);
        vl53l5cx_set_resolution(&tof_sensors[i], VL53L5CX_RESOLUTION_8X8);
        vl53l5cx_start_ranging(&tof_sensors[i]);
        vl53l5cx_set_i2c_address(&tof_sensors[i], VL53L5CX_ADDRESS_CHAIN_START + i);
    }
    durin.info.tof_resolution = TofResolutions_resolution8x8rate15Hz;
}


void build_tof_message(TofObservations_ptr *ptr, struct capn_segment *cs, uint8_t *to_send) {
    struct TofObservations tof_observations;
    uint8_t num_sensors = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (to_send[i]) {
            num_sensors++;
        }
    }
    tof_observations.observations = new_TofObservations_TofObservation_list(cs, num_sensors);
    uint8_t sensor_index = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (!to_send[i]) {
            continue;
        }
        uint8_t pixels;
        uint8_t id = i;
        struct TofObservations_TofObservation observation;
        observation.id = id;
        if (durin.info.tof_resolution == TofResolutions_resolution4x4rate60Hz) {
            pixels = 16;
        } else {
            pixels = 64;
        }
        observation.ranges = capn_new_list16(cs, pixels);
        capn_setv16(observation.ranges, 0, durin.telemetry.ranging_data[id], pixels);
        set_TofObservations_TofObservation(&observation, tof_observations.observations, sensor_index);
        sensor_index++;
    }
    *ptr = new_TofObservations(cs);
    write_TofObservations(&tof_observations, *ptr);
}

void send_tof_telemetry(uint8_t *to_send) {
    struct capn c;
    struct capn_segment *cs;    
    struct DurinBase msg;
    init_durinbase(&c, &cs, &msg);
    build_tof_message(&msg.tofObservations, cs, to_send);
    msg.which = DurinBase_tofObservations;
    uint8_t buf[1500];
    uint16_t len = 1500;
    finish_durinbase(&c, &cs, &msg, buf, &len);
    if (len == -1) {
        printf("tof telemetry LENGTH ERROR\n");
        return;
    }
    send_telemetry(buf, len);
}

void update_tof_and_expander(struct pt *pt) {
    PT_BEGIN(pt);
    static uint32_t current_port_output = 1 << 16; // only 16 outputs so this value is "invalid"
    static uint8_t tof_index;
    static uint64_t last_tof_update;
    static uint64_t last_tof_send;
    static bool tof_enabled = true;
    static uint8_t tof_update_rate = 15;
    static uint8_t img_size = 8;
    last_tof_update = esp_timer_get_time();
    while (1) {
        PT_YIELD(pt);
        if (expander_current_output != expander_wanted_output) {
            nbe_i2c_full_register_write(&durin.hw.i2c_tof, EXPANDER_ADDRESS, 2, NBE_I2C_REGISTER_8, &expander_wanted_output, 2);
            TOF_I2C_WAIT();
            expander_current_output = expander_wanted_output;
        }
        nbe_i2c_full_register_read(&durin.hw.i2c_tof, EXPANDER_ADDRESS, 0, NBE_I2C_REGISTER_8, &expander_current_input, 2);
        TOF_I2C_WAIT();

        if (tof_enabled && durin.info.active == false) {
            printf("disabled tof\n");
            for (tof_index = 0; tof_index < NUM_VL53L5CX; tof_index++) {
                if (!durin.info.tof_sensor_alive[tof_index]) {
                    continue;
                }
                vl53l5cx_stop_ranging(&tof_sensors[tof_index]);
            }
            tof_enabled = false;
        }

        if (tof_enabled == false && durin.info.active) {
            printf("enabled tof\n");
            for (tof_index = 0; tof_index < NUM_VL53L5CX; tof_index++) {
                if (!durin.info.tof_sensor_alive[tof_index]) {
                    continue;
                }
                vl53l5cx_start_ranging(&tof_sensors[tof_index]);
            }
            tof_enabled = true;
        }

        if (wanted_tof_resolution != durin.info.tof_resolution) {
            printf("changing tof resolution from %d to %d\n", durin.info.tof_resolution, wanted_tof_resolution);
            durin.info.tof_resolution = wanted_tof_resolution;
            for (tof_index = 0; tof_index < NUM_VL53L5CX; tof_index++) {
                if (!durin.info.tof_sensor_alive[tof_index]) {
                    continue;
                }
                vl53l5cx_stop_ranging(&tof_sensors[tof_index]);
                if (durin.info.tof_resolution == TofResolutions_resolution8x8rate15Hz) {
                    vl53l5cx_set_resolution(&tof_sensors[tof_index], VL53L5CX_RESOLUTION_8X8);
                    vl53l5cx_set_ranging_frequency_hz(&tof_sensors[tof_index], 15);
                } else
                if (durin.info.tof_resolution == TofResolutions_resolution4x4rate60Hz) {                    
                    vl53l5cx_set_resolution(&tof_sensors[tof_index], VL53L5CX_RESOLUTION_4X4);
                    vl53l5cx_set_ranging_frequency_hz(&tof_sensors[tof_index], 60);
                }
                vl53l5cx_start_ranging(&tof_sensors[tof_index]);
            }
            if (durin.info.tof_resolution == TofResolutions_resolution4x4rate60Hz) {
                tof_update_rate = 60;
                img_size = 4;
            } else
            if (durin.info.tof_resolution == TofResolutions_resolution8x8rate15Hz) {
                tof_update_rate = 15;
                img_size = 8;
            }
        }
        if (!durin.info.streaming_enabled) {
            continue;
        }

        // update at 15 Hz
        if (esp_timer_get_time() - last_tof_update < 1000000 / tof_update_rate) {
            continue;
        }

        last_tof_update = esp_timer_get_time();
        for (tof_index = 0; tof_index < NUM_VL53L5CX; tof_index++) {
            if (!durin.info.tof_sensor_alive[tof_index]) {
                // printf("dead tof\n");
                continue;
            }
            // printf("update tof\n");
            vl53l5cx_get_ranging_data_async_start(&tof_sensors[tof_index]);
            TOF_I2C_WAIT();
            VL53L5CX_ResultsData result;
            vl53l5cx_get_ranging_data_async_finish(&tof_sensors[tof_index], &result);
            for (uint8_t src_y = 0; src_y < img_size; src_y++) {
                for (uint8_t src_x = 0; src_x < img_size; src_x++) {
                    uint8_t target_x = img_size - 1 - src_x;
                    uint8_t target_y = src_y;
                    uint8_t status = result.target_status[src_x + src_y * img_size];
                    uint8_t status_bits = 0;
                    switch (status) {
                        case 0: // not updated
                            status_bits = 0b11;
                            break;
                        case 5: // valid 
                            status_bits = 0b00;
                            break;
                        case 6:
                        case 9: // 50% valid
                            status_bits = 0b01;
                            break;
                        default: // invalid
                            status_bits = 0b10;
                            break;
                    }
                    durin.telemetry.ranging_data[tof_index][target_x + img_size * target_y] = result.distance_mm[src_x + src_y * img_size] | (status_bits << 14);
                }
            }
        }
        if (esp_timer_get_time() - last_tof_send > durin.info.tof_stream_period * 1000) {
            last_tof_send = esp_timer_get_time();
            uint8_t one[8] =   {1,1,0,0,0,0,0,0};
            uint8_t two[8] =   {0,0,1,1,0,0,0,0};
            uint8_t three[8] = {0,0,0,0,1,1,0,0};
            uint8_t four[8] =  {0,0,0,0,0,0,1,1};
            send_tof_telemetry(one);
            send_tof_telemetry(two);
            send_tof_telemetry(three);
            send_tof_telemetry(four);
        }
    }
    PT_END(pt);
}

bool configure_expander_pin(uint8_t pin, bool is_input) {
    if (is_input) {
        expander_wanted_configuration |= 1 << pin;
    } else {
        expander_wanted_configuration &= ~(1 << pin);
    }
    if (!durin.info.init_finished) {
        printf("configured %d\n", expander_wanted_configuration);
        nbe_i2c_full_register_write(&durin.hw.i2c_tof, EXPANDER_ADDRESS, 6, NBE_I2C_REGISTER_8, &expander_wanted_configuration, 2);
        while (nbe_i2c_is_busy(&durin.hw.i2c_tof)) {}
        expander_current_configuration = expander_wanted_configuration;
    }
    return ((expander_current_configuration & (1 << pin)) == is_input);
}

bool write_expander_pin(uint8_t pin, bool value) {
    if (value) {
        expander_wanted_output |= 1 << pin;
    } else {
        expander_wanted_output &= ~(1 << pin);
    }
    if (!durin.info.init_finished) {
        printf("wrote %d\n", expander_wanted_output);
        nbe_i2c_full_register_write(&durin.hw.i2c_tof, EXPANDER_ADDRESS, 2, NBE_I2C_REGISTER_8, &expander_wanted_output, 2);
        while (nbe_i2c_is_busy(&durin.hw.i2c_tof)) {}
        expander_current_output = expander_wanted_output;
    }
    return ((expander_current_output & (1 << pin)) == value);
}

bool read_expander_pin(uint8_t pin) {
    if (!durin.info.init_finished) {
        printf("read %d\n", expander_wanted_configuration);
        nbe_i2c_full_register_read(&durin.hw.i2c_tof, EXPANDER_ADDRESS, 0, NBE_I2C_REGISTER_8, &expander_current_input, 2);   
        while (nbe_i2c_is_busy(&durin.hw.i2c_tof)) {}
    }
    return expander_current_input & (1 << pin);
}