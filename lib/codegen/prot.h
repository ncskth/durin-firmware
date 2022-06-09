/*****************************
GENERATED FILE DO NOT EDIT
******************************/

#ifndef _prot_H
#define _prot_H
// if you want to use floats or doubles
#define prot_FLOAT_DEF float

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum tof_status {
  prot_tof_status_valid = 0,
  prot_tof_status_half_valid = 1,
  prot_tof_status_not_valid = 2,
  prot_tof_status_no_change = 3,
};
enum nodes {
  prot_nodes_durin = 0,
  prot_nodes_control = 1,
};
enum fields {
  prot_fields_vel_x_mms = 0,
  prot_fields_vel_y_mms = 1,
  prot_fields_rot_degs = 2,
  prot_fields_motor_1_mms = 3,
  prot_fields_motor_2_mms = 4,
  prot_fields_motor_3_mms = 5,
  prot_fields_motor_4_mms = 6,
  prot_fields_intensity = 7,
  prot_fields_led_r = 8,
  prot_fields_led_g = 9,
  prot_fields_led_b = 10,
  prot_fields_sensor_id = 11,
  prot_fields_ip = 12,
  prot_fields_port = 13,
  prot_fields_rate_ms = 14,
  prot_fields_node_id = 15,
  prot_fields_main_ssid = 16,
  prot_fields_main_password = 17,
  prot_fields_frame_number = 18,
  prot_fields_checksum = 19,
  prot_fields_frame_length = 20,
  prot_fields_data = 21,
  prot_fields_tof_1 = 22,
  prot_fields_tof_2 = 23,
  prot_fields_tof_3 = 24,
  prot_fields_tof_4 = 25,
  prot_fields_tof_5 = 26,
  prot_fields_tof_6 = 27,
  prot_fields_tof_7 = 28,
  prot_fields_tof_8 = 29,
  prot_fields_charge_percent = 30,
  prot_fields_battery_voltage_mv = 31,
  prot_fields_ax = 32,
  prot_fields_ay = 33,
  prot_fields_az = 34,
  prot_fields_gx = 35,
  prot_fields_gy = 36,
  prot_fields_gz = 37,
  prot_fields_mx = 38,
  prot_fields_my = 39,
  prot_fields_mz = 40,
  prot_fields_set_this_to_zero = 41,
};
enum messages {
  prot_messages_acknowledge = 0,
  prot_messages_reject = 1,
  prot_messages_power_off = 2,
  prot_messages_move_rob_centric = 3,
  prot_messages_move_wheels = 4,
  prot_messages_set_buzzer = 5,
  prot_messages_set_led = 6,
  prot_messages_poll_all = 7,
  prot_messages_poll_sensor = 8,
  prot_messages_start_stream = 9,
  prot_messages_stop_stream = 10,
  prot_messages_set_node_id = 11,
  prot_messages_wifi_config = 12,
  prot_messages_ota_packet = 13,
  prot_messages_tof_12 = 14,
  prot_messages_tof_34 = 15,
  prot_messages_tof_56 = 16,
  prot_messages_tof_78 = 17,
  prot_messages_misc = 18,
  prot_messages_uwb_distance = 19,
};
enum categories {
  prot_categories_none = 0,
};
#define prot_id_acknowledge 0
#define prot_id_reject 127
#define prot_id_power_off 1
#define prot_id_move_rob_centric 2
#define prot_id_move_wheels 3
#define prot_id_set_buzzer 8
#define prot_id_set_led 9
#define prot_id_poll_all 16
#define prot_id_poll_sensor 17
#define prot_id_start_stream 18
#define prot_id_stop_stream 19
#define prot_id_set_node_id 20
#define prot_id_wifi_config 21
#define prot_id_ota_packet 22
#define prot_id_tof_12 128
#define prot_id_tof_34 129
#define prot_id_tof_56 130
#define prot_id_tof_78 131
#define prot_id_misc 132
#define prot_id_uwb_distance 133
#define prot_len_acknowledge 0
#define prot_len_reject 0
#define prot_len_power_off 0
#define prot_len_move_rob_centric 6
#define prot_len_move_wheels 8
#define prot_len_set_buzzer 1
#define prot_len_set_led 3
#define prot_len_poll_all 0
#define prot_len_poll_sensor 1
#define prot_len_start_stream 8
#define prot_len_stop_stream 0
#define prot_len_set_node_id 1
#define prot_len_wifi_config 96
#define prot_len_ota_packet 260
#define prot_len_tof_12 256
#define prot_len_tof_34 256
#define prot_len_tof_56 256
#define prot_len_tof_78 256
#define prot_len_misc 21
#define prot_len_uwb_distance 1
struct prot_acknowledge {};
struct prot_reject {};
struct prot_power_off {};
struct prot_move_rob_centric {
  int16_t vel_x_mms;
  int16_t vel_y_mms;
  int16_t rot_degs;
};
struct prot_move_wheels {
  int16_t motor_1_mms;
  int16_t motor_2_mms;
  int16_t motor_3_mms;
  int16_t motor_4_mms;
};
struct prot_set_buzzer {
  uint8_t intensity;
};
struct prot_set_led {
  uint8_t led_r;
  uint8_t led_g;
  uint8_t led_b;
};
struct prot_poll_all {};
struct prot_poll_sensor {
  uint8_t sensor_id;
};
struct prot_start_stream {
  uint32_t ip;
  uint16_t port;
  uint16_t rate_ms;
};
struct prot_stop_stream {};
struct prot_set_node_id {
  uint8_t node_id;
};
struct prot_wifi_config {
  uint8_t *main_ssid;
  uint8_t *main_password;
};
struct prot_ota_packet {
  uint16_t frame_number;
  uint16_t checksum;
  uint8_t frame_length;
  uint8_t *data;
};
struct prot_tof_12 {
  uint8_t *tof_1;
  uint8_t *tof_2;
};
struct prot_tof_34 {
  uint8_t *tof_3;
  uint8_t *tof_4;
};
struct prot_tof_56 {
  uint8_t *tof_5;
  uint8_t *tof_6;
};
struct prot_tof_78 {
  uint8_t *tof_7;
  uint8_t *tof_8;
};
struct prot_misc {
  uint8_t charge_percent;
  uint16_t battery_voltage_mv;
  prot_FLOAT_DEF ax;
  prot_FLOAT_DEF ay;
  prot_FLOAT_DEF az;
  prot_FLOAT_DEF gx;
  prot_FLOAT_DEF gy;
  prot_FLOAT_DEF gz;
  prot_FLOAT_DEF mx;
  prot_FLOAT_DEF my;
  prot_FLOAT_DEF mz;
};
struct prot_uwb_distance {
  uint8_t set_this_to_zero;
};
#define scaledFloat_to_uint(value, scale, out) out = value * scale;

#define uint_to_scaledFloat(value, scale, out) value / scale;
#define packedFloat_to_uint(value, minValue, maxValue, out)                    \
  {                                                                            \
    uint64_t intMax = 1 << ((sizeof(out) * 8) - 1);                            \
    if (value < minValue) {                                                    \
      out = 0;                                                                 \
    } else if (value > maxValue) {                                             \
      out = intMax;                                                            \
    } else {                                                                   \
      prot_FLOAT_DEF ratio = (value - minValue) / (maxValue - minValue);       \
      out = 1 + ((intMax - 2)) * ratio;                                        \
    }                                                                          \
  }

#define uint_to_packedFloat(value, minValue, maxValue, out)                    \
  {                                                                            \
    uint64_t intMax = 1 << ((sizeof(out) * 8) - 1);                            \
    if (value <= 0) {                                                          \
      out = minValue - 1.0;                                                    \
    } else if (value >= intMax) {                                              \
      out = maxValue + 1.0;                                                    \
    } else {                                                                   \
      prot_FLOAT_DEF ratio = (value - 1) / (intMax - 2);                       \
      out = ratio * (maxValue - minValue) + minValue;                          \
    }                                                                          \
  }
void prot_build_acknowledge(uint8_t *buf, struct prot_acknowledge data);
void prot_rx_acknowledge(struct prot_acknowledge data);
void prot_build_reject(uint8_t *buf, struct prot_reject data);
void prot_rx_reject(struct prot_reject data);
void prot_build_power_off(uint8_t *buf, struct prot_power_off data);
void prot_rx_power_off(struct prot_power_off data);
void prot_build_move_rob_centric(uint8_t *buf,
                                 struct prot_move_rob_centric data);
void prot_rx_move_rob_centric(struct prot_move_rob_centric data);
void prot_build_move_wheels(uint8_t *buf, struct prot_move_wheels data);
void prot_rx_move_wheels(struct prot_move_wheels data);
void prot_build_set_buzzer(uint8_t *buf, struct prot_set_buzzer data);
void prot_rx_set_buzzer(struct prot_set_buzzer data);
void prot_build_set_led(uint8_t *buf, struct prot_set_led data);
void prot_rx_set_led(struct prot_set_led data);
void prot_build_poll_all(uint8_t *buf, struct prot_poll_all data);
void prot_rx_poll_all(struct prot_poll_all data);
void prot_build_poll_sensor(uint8_t *buf, struct prot_poll_sensor data);
void prot_rx_poll_sensor(struct prot_poll_sensor data);
void prot_build_start_stream(uint8_t *buf, struct prot_start_stream data);
void prot_rx_start_stream(struct prot_start_stream data);
void prot_build_stop_stream(uint8_t *buf, struct prot_stop_stream data);
void prot_rx_stop_stream(struct prot_stop_stream data);
void prot_build_set_node_id(uint8_t *buf, struct prot_set_node_id data);
void prot_rx_set_node_id(struct prot_set_node_id data);
void prot_build_wifi_config(uint8_t *buf, struct prot_wifi_config data);
void prot_rx_wifi_config(struct prot_wifi_config data);
void prot_build_ota_packet(uint8_t *buf, struct prot_ota_packet data);
void prot_rx_ota_packet(struct prot_ota_packet data);
void prot_build_tof_12(uint8_t *buf, struct prot_tof_12 data);
void prot_rx_tof_12(struct prot_tof_12 data);
void prot_build_tof_34(uint8_t *buf, struct prot_tof_34 data);
void prot_rx_tof_34(struct prot_tof_34 data);
void prot_build_tof_56(uint8_t *buf, struct prot_tof_56 data);
void prot_rx_tof_56(struct prot_tof_56 data);
void prot_build_tof_78(uint8_t *buf, struct prot_tof_78 data);
void prot_rx_tof_78(struct prot_tof_78 data);
void prot_build_misc(uint8_t *buf, struct prot_misc data);
void prot_rx_misc(struct prot_misc data);
void prot_build_uwb_distance(uint8_t *buf, struct prot_uwb_distance data);
void prot_rx_uwb_distance(struct prot_uwb_distance data);
void prot_parse(uint8_t id, uint8_t *buf);
uintmax_t prot_id_to_len(uint8_t id);
#endif
