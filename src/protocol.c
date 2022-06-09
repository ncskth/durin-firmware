#include <string.h>
#include <esp_ota_ops.h>
#include <esp_rom_crc.h>

#include "prot.h"
#include "protocol.h"
#include "misc.h"


#include "durin.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// I HATE CRC SO MUCH WHY ARE THERE 983274893247 PARAMETERS I NEED TO GET RIGHT FOR IT TO WORK
uint16_t crc_ccitt_update( uint16_t crc, uint8_t data ) {

  uint16_t ret_val;

  data ^= ( uint8_t )(crc) & (uint8_t )(0xFF);
  data ^= data << 4;

  ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
            ^ (uint8_t)(data >> 4)
            ^ ((uint16_t)data << 3));

  return ret_val;

}

uint16_t get_crc16z(uint8_t *p, uint16_t len) {

  uint16_t crc16_data=0;

  while(len--) {
      crc16_data = crc_ccitt_update(crc16_data, p[0]); p++;
  }

  return(crc16_data);

}


uint8_t response;

uint8_t protocol_parse_byte(struct protocol_state *state, uint8_t byte) {
    if (state->state == 0) {
        state->id = byte;
        state->expected_len = prot_id_to_len(state->id);
        state->current_len = 0;
        state->state = 1;
    } else {
        state->payload_buf[state->current_len] = byte;
        state->current_len += 1;
    }

    if (state->current_len == state->expected_len) {
        //printf("got id %d\n", state->id);
        prot_parse(state->id,state->payload_buf);
        durin.info.last_message_received = esp_timer_get_time();
        state->state = 0;
        return 1;
    }
    return 0;
}

void prot_rx_power_off(struct prot_power_off data) {
    printf("power off command goodbye!\n");
    vTaskDelay(100);
    power_off();
    response = prot_id_acknowledge; // hard to respond though
}

void prot_rx_move_rob_centric(struct prot_move_rob_centric data) {
    durin.control.control_id = prot_id_move_rob_centric;
    durin.control.robot_velocity.velocity_x = data.vel_x_mms;
    durin.control.robot_velocity.velocity_y = data.vel_y_mms;
    durin.control.robot_velocity.rotation_cw = data.rot_degs;
    response = prot_id_acknowledge;
}

void prot_rx_move_wheels(struct prot_move_wheels data) {
    durin.control.control_id = prot_id_move_wheels; 
    durin.control.motor_velocity.motor_1 = data.motor_1_mms;
    durin.control.motor_velocity.motor_2 = data.motor_2_mms;
    durin.control.motor_velocity.motor_3 = data.motor_3_mms;
    durin.control.motor_velocity.motor_4 = data.motor_4_mms;
    response = prot_id_acknowledge;
}

void prot_rx_start_stream(struct prot_start_stream data) {
    durin.info.telemetry_udp_address = data.ip;
    durin.info.telemetry_udp_port = data.port;
    durin.info.telemetry_udp_rate = data.rate_ms;
    durin.info.telemetry_udp_enabled = 1;
    response = prot_id_acknowledge;
}

void prot_rx_stop_stream(struct prot_stop_stream data) {
    durin.info.telemetry_udp_enabled = 0;
    response = prot_id_acknowledge;
}

void prot_rx_set_led(struct prot_set_led data) {
    set_led(data.led_r, data.led_g, data.led_b);
    response = prot_id_acknowledge;
}

void prot_rx_poll_sensor(struct prot_poll_sensor data) {
    response = data.sensor_id;
}

void prot_rx_poll_all(struct prot_poll_all data) {
    response = prot_id_poll_all;
}

void prot_rx_set_buzzer(struct prot_set_buzzer data) {
    set_buzzer(data.intensity);
    response = prot_id_acknowledge;
}

void prot_rx_set_node_id(struct prot_set_node_id data) {
    durin_persistent.node_id = data.node_id;
    update_persistent_data();
    response = prot_id_acknowledge;
}

void prot_rx_wifi_config(struct prot_wifi_config data) {
    memcpy(durin_persistent.main_ssid, data.main_ssid, sizeof(durin_persistent.main_ssid));
    memcpy(durin_persistent.main_password, data.main_password, sizeof(durin_persistent.main_password));
    update_persistent_data();
    response = prot_id_acknowledge;
}

void prot_rx_ota_packet(struct prot_ota_packet data) {
    static bool ota_in_progress = false;
    static esp_partition_t *ota_partition;
    static esp_ota_handle_t ota_handle;
    static int32_t last_frame_number;
    static uint8_t first;
    esp_err_t e;
    response = prot_id_acknowledge;
    if (!ota_in_progress) {
        first = 1;
        if (data.frame_length == 0) {
            esp_ota_mark_app_valid_cancel_rollback();
            vTaskDelay(100);
            return;
        }

        if (data.frame_number != 0) {
            goto error;
        }

        ota_partition = esp_ota_get_next_update_partition(NULL);
        if (ota_partition == NULL) {
            ota_partition = esp_partition_find(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
        }
        if (ota_partition == NULL) {
            goto error;
        }

        e = esp_ota_begin(ota_partition, 0, &ota_handle); 
        if (e != ESP_OK) {
            goto error;
        }

        ota_in_progress = true;
        last_frame_number = -1; 
    }

    //check crc
    if (data.checksum != get_crc16z(data.data, data.frame_length)) {
        goto error;
    }


    //check frame number
    if (data.frame_number != last_frame_number + 1) {
        first = 0;
        printf("frame number gotten %d  wanted %d\n", data.frame_number, last_frame_number + 1);

        goto error;
    }

    last_frame_number++;
    //write
    e = esp_ota_write(ota_handle, data.data, data.frame_length);
    if (e != ESP_OK) {
        printf("couldn't write\n");
        goto error;
    }

    //end
    if (data.frame_length < 255) {
        printf("yeah i am done %d\n", data.frame_length);
        e = esp_ota_end(ota_handle);
        esp_ota_set_boot_partition(ota_partition);
        vTaskDelay(100);
        power_off();
        if (e != ESP_OK) {
            goto error;
        } 
    }
    return;

    error:
    if (ota_in_progress) {
        esp_ota_abort(ota_handle);
        ota_in_progress = false;
    }
    response = prot_id_reject;
    return;
}