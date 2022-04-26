#include <string.h>

#include "protocol.h"

#include "durin.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_timer.h"


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
        case STOP_STREAM:
            return 0;
        case POLL_SENSOR:
            return 1;
    }
    return 0;
}

void parse_msg(uint8_t id, uint8_t *buf, uint8_t *response) {
    durin.info.last_message_received = esp_timer_get_time();
    
    if (id == POWER_OFF) {
        gpio_set_level(PIN_3V3_EN, 0);
        *response = ACKNOWLEDGE; // not like you can respond to this
    }

    if (id == MOVE_ROB_CENTRIC) {
        struct move_robot_velocity *data = (struct move_robot_velocity*) buf; 
        printf("move robot x %d y %d rot %d\n", data->vel_x, data->vel_y, data->rot);

        durin.control.control_mode = DURIN_ROBOT_VELOCITY;
        durin.control.robot_velocity.velocity_x = data->vel_x;
        durin.control.robot_velocity.velocity_y = data->vel_y;
        durin.control.robot_velocity.rotation_cw = data->rot;
        *response = ACKNOWLEDGE;
    }

    if (id == MOVE_WHEELS) {
        struct move_motor_velocity *data = (struct move_motor_velocity*) buf; 
        printf("move wheels %d %d %d %d\n", data->motor1, data->motor2, data->motor3, data->motor4);
        durin.control.control_mode = DURIN_MOTOR_VELOCITY;
        durin.control.motor_velocity.motor_1 = data->motor1;
        durin.control.motor_velocity.motor_2 = data->motor2;
        durin.control.motor_velocity.motor_3 = data->motor3;
        durin.control.motor_velocity.motor_4 = data->motor4;
        *response = ACKNOWLEDGE;
    }

    if (id == START_STREAM) {
        struct start_stream *data = (struct start_stream*) buf;
        uint8_t *tmp = &data->ip;
        printf("start stream ip %d.%d.%d.%d port %d\n", tmp[0], tmp[1], tmp[2], tmp[3], data->port);
        durin.info.telemetry_udp_address = data->ip;
        durin.info.telemetry_udp_port = data->port;
        durin.info.telemetry_udp_rate = data->rate;
        durin.info.telemetry_udp_enabled = 1;
        *response = ACKNOWLEDGE;
    }

    if (id == STOP_STREAM) {
        printf("stop stream\n");
        durin.info.telemetry_udp_enabled = 0;
        *response = ACKNOWLEDGE;
    }

    if (id == POLL_SENSOR) {
        struct poll_sensor *data = (struct poll_sensor*) buf;
        *response = data->id; 
        printf("poll sensor %d\n", data->id);
    }
}

uint8_t protocol_parse_byte(struct protocol_state *state, uint8_t byte, uint8_t *response) {
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
        parse_msg(state->id, state->payload_buf, response);
        state->state = 0;
        return 1;
    }
    return 0;
}