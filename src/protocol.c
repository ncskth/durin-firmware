#include <string.h>

#include "protocol.h"

#include "durin.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define POWER_OFF 1

#define MOVE_ROB_CENTRIC 2
#define MOVE_WHEELS 3

#define POLL_ALL 16
#define START_STREAM 18


uint16_t id_to_len(uint8_t id) {
    switch (id) {
        case POWER_OFF:
            return 0;
        case MOVE_ROB_CENTRIC:
            return 6;
        case MOVE_WHEELS:
            return 8;
        case POLL_ALL:
            return 0;
        case START_STREAM:
            return 8; 
    }
    return 0;
}

void parse_msg(uint8_t id, uint8_t *buf) {
    durin.info.last_message_received = esp_timer_get_time();

    if (id == POWER_OFF) {
        gpio_set_level(PIN_3V3_EN, 0);
    }
    if (id == MOVE_ROB_CENTRIC) {
        struct move_robot_velocity *data = (struct move_robot_velocity*) buf; 
        durin.control.control_mode = DURIN_ROBOT_VELOCITY;
        durin.control.robot_velocity.velocity_x = data->vel_x;
        durin.control.robot_velocity.velocity_y = data->vel_y;
        durin.control.robot_velocity.rotation_cw = data->rot;
    }

    if (id == MOVE_WHEELS) {
        struct move_motor_velocity *data = (struct move_motor_velocity*) buf; 
        durin.control.control_mode = DURIN_MOTOR_VELOCITY;
        durin.control.motor_velocity.motor_1 = data->motor1;
        durin.control.motor_velocity.motor_2 = data->motor2;
        durin.control.motor_velocity.motor_3 = data->motor3;
        durin.control.motor_velocity.motor_4 = data->motor4;
    }

    if (id == START_STREAM) {
        struct start_stream *data = (struct start_stream*) buf; 
        memcpy(durin.info.telemetry_udp_address, data->ip, 4);
        durin.info.telemetry_udp_port = data->port;
        durin.info.telemetry_udp_rate = data->rate;
        durin.info.telemetry_udp_enabled = 1;
    }
}

void protocol_parse_byte(struct protocol_state *state, uint8_t byte) {
    if (state->state == 0) {
        state->id = byte;
        state->expected_len = id_to_len(state->id);
        state->current_len = 0;
        state->state = 1;
    } else {
        state->payload_buf[state->current_len] = byte;
        state->current_len += 1;
    }

    if (state->current_len == state->expected_len) {
        parse_msg(state->id, state->payload_buf);
        state->state = 0;
    }
}