#include "vl53l5cx_api.h"
#include "vl53l5cx_plugin_async.h"

#include "durin.h"
#include "hardware.h"
#include "tof_and_expander.h"
#include "pt.h"

#define EXPANDER_ADDRESS (0x20)

#define NUM_VL53L5CX 8
#define VL53L5CX_ADDRESS_CHAIN_START 0x41
VL53L5CX_Configuration tof_sensors[NUM_VL53L5CX];

#define TOF_I2C_WAIT() do { PT_YIELD(pt); } while (nbe_i2c_is_busy(&durin.hw.i2c_tof))
#define TOF_I2C_WAIT_BLOCK() do {} while (nbe_i2c_is_busy(&durin.hw.i2c_tof))

void expander_write(uint16_t output);
void expander_read(uint8_t *buf);
uint16_t expander_parse(uint8_t *buf);

void init_tof_and_expander() {
    durin.hw.port_expander_ouput = ~0; // everything defaults to 1
    expander_write(durin.hw.port_expander_ouput);
    TOF_I2C_WAIT_BLOCK();
    
    //reset all
    /* already 1 
    for (uint8_t i = 0; i < NUM_VL53L5CX; i++) {
        durin.hw.port_expander_ouput |= 1 << (TOF_RESET_START + i);
    }
    expander_write(durin.hw.port_expander_ouput);
    TOF_I2C_WAIT_BLOCK();
    */

    VL53L5CX_Platform platform;
    platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
    platform.nbe_i2c = &durin.hw.i2c_tof;
    tof_sensors[0].platform = platform;
    uint8_t alive;
    vl53l5cx_is_alive(&tof_sensors[0], &alive);
    //sensor should be dead now
    if (alive) {
        return; // abort if we heave a dead sensor
    }

    for (uint8_t i = 0 ; i < NUM_VL53L5CX; i++) {
        //unreset sensor
        durin.hw.port_expander_ouput &= ~(1 << (TOF_RESET_START + i));
        expander_write(durin.hw.port_expander_ouput);
        TOF_I2C_WAIT_BLOCK();

        VL53L5CX_Platform platform;
        platform.address = VL53L5CX_DEFAULT_I2C_ADDRESS;
        platform.nbe_i2c = &durin.hw.i2c_tof;
        tof_sensors[i].platform = platform;
        uint8_t alive;
        vl53l5cx_is_alive(&tof_sensors[i], &alive);
        if (!alive) {
            durin.info.tof_sensor_alive[i] = 0;
            // printf("dead tof sensor with index %d\n", i);
            continue;
        }
        durin.info.tof_sensor_alive[i] = 1;
        vl53l5cx_init(&tof_sensors[i]);
        vl53l5cx_set_ranging_mode(&tof_sensors[i], VL53L5CX_RANGING_MODE_CONTINUOUS);
        vl53l5cx_set_ranging_frequency_hz(&tof_sensors[i], 15);
        vl53l5cx_set_resolution(&tof_sensors[i], VL53L5CX_RESOLUTION_8X8);
        vl53l5cx_start_ranging(&tof_sensors[i]);
        vl53l5cx_set_i2c_address(&tof_sensors[i], VL53L5CX_ADDRESS_CHAIN_START + i);
    }
}

void update_tof_and_expander(struct pt *pt) {
    PT_BEGIN(pt);

    static uint32_t current_port_output = 1 << 16; // only 16 outputs so this value is "invalid"
    static uint8_t tof_index;
    static uint64_t last_tof_update;
    last_tof_update = esp_timer_get_time();
    while (1) {
        PT_YIELD(pt);
        if (current_port_output != durin.hw.port_expander_ouput) {
            expander_write(durin.hw.port_expander_ouput);
            TOF_I2C_WAIT();
            current_port_output = durin.hw.port_expander_ouput;
            durin.info.expander_awaiting_update = 0;
        }
        static uint8_t buf[2];
        expander_read(buf);
        TOF_I2C_WAIT();
        durin.hw.port_expander_input = expander_parse(buf);

        // update at 15 Hz
        if (esp_timer_get_time() - last_tof_update < 1000000 / 15) {
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
            for (uint8_t src_y = 0; src_y < 8; src_y++) {
                for (uint8_t src_x = 0; src_x < 8; src_x++) {
                    uint8_t target_x = 7 - src_x;
                    uint8_t target_y = src_y;
                    uint8_t status = result.target_status[src_x + src_y * 8];
                    uint8_t status_bits = 0;
                    switch (status) {
                        case 0: // not updated
                            status_bits = 0b01;
                            break;
                        case 5: // valid 
                            status_bits = 0b00;
                            break;
                        case 6:
                        case 9: // 50% valid
                            status_bits = 0b11;
                            break;
                        default: // invalid
                            status_bits = 0b10;
                            break;
                    }
                    durin.telemetry.ranging_data[tof_index][target_x + 8 * target_y] = result.distance_mm[src_x + src_y * 8] | (status_bits << 14); 
                }
            }
        }
    }
    PT_END(pt);
}


//CAN ONLY BE USED IN INIT BEFORE TOF_AND_EXPANDER_UPDATE_IS_RUNNING
void init_expander_write() {
    expander_write(durin.hw.port_expander_ouput);
}

void init_expander_read() {

}

void expander_write(uint16_t output) {
    uint8_t buf[2];
    buf[0] = output & 0x00ff;
    buf[1] = (output >> 8) & 0x00ff;
    nbe_i2c_start_write(&durin.hw.i2c_tof, EXPANDER_ADDRESS, NULL, NULL);
    nbe_i2c_write_preamble(&durin.hw.i2c_tof, buf, 2);
    nbe_i2c_stop(&durin.hw.i2c_tof);
    nbe_i2c_commit(&durin.hw.i2c_tof);
}

void expander_read(uint8_t *buf) {
    nbe_i2c_start_read(&durin.hw.i2c_tof, EXPANDER_ADDRESS, NULL, buf);
    nbe_i2c_read_ack(&durin.hw.i2c_tof, 1);
    nbe_i2c_read_nak(&durin.hw.i2c_tof, 1);
    nbe_i2c_stop(&durin.hw.i2c_tof);
    nbe_i2c_commit(&durin.hw.i2c_tof);
}

uint16_t expander_parse(uint8_t *buf) {
    return buf[0] + (buf[1] << 8);
}