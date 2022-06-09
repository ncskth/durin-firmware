/*****************************
GENERATED FILE DO NOT EDIT
******************************/

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "prot.h"

void prot_build_acknowledge(uint8_t *buf, struct prot_acknowledge data) {}
void __attribute__((weak)) prot_rx_acknowledge(struct prot_acknowledge data) {}
void prot_build_reject(uint8_t *buf, struct prot_reject data) {}
void __attribute__((weak)) prot_rx_reject(struct prot_reject data) {}
void prot_build_power_off(uint8_t *buf, struct prot_power_off data) {}
void __attribute__((weak)) prot_rx_power_off(struct prot_power_off data) {}
void prot_build_move_rob_centric(uint8_t *buf,
                                 struct prot_move_rob_centric data) {
  memcpy(buf + 0, &data.vel_x_mms, 2);
  memcpy(buf + 2, &data.vel_y_mms, 2);
  memcpy(buf + 4, &data.rot_degs, 2);
}
void __attribute__((weak))
prot_rx_move_rob_centric(struct prot_move_rob_centric data) {}
void prot_build_move_wheels(uint8_t *buf, struct prot_move_wheels data) {
  memcpy(buf + 0, &data.motor_1_mms, 2);
  memcpy(buf + 2, &data.motor_2_mms, 2);
  memcpy(buf + 4, &data.motor_3_mms, 2);
  memcpy(buf + 6, &data.motor_4_mms, 2);
}
void __attribute__((weak)) prot_rx_move_wheels(struct prot_move_wheels data) {}
void prot_build_set_buzzer(uint8_t *buf, struct prot_set_buzzer data) {
  memcpy(buf + 0, &data.intensity, 1);
}
void __attribute__((weak)) prot_rx_set_buzzer(struct prot_set_buzzer data) {}
void prot_build_set_led(uint8_t *buf, struct prot_set_led data) {
  memcpy(buf + 0, &data.led_r, 1);
  memcpy(buf + 1, &data.led_g, 1);
  memcpy(buf + 2, &data.led_b, 1);
}
void __attribute__((weak)) prot_rx_set_led(struct prot_set_led data) {}
void prot_build_poll_all(uint8_t *buf, struct prot_poll_all data) {}
void __attribute__((weak)) prot_rx_poll_all(struct prot_poll_all data) {}
void prot_build_poll_sensor(uint8_t *buf, struct prot_poll_sensor data) {
  memcpy(buf + 0, &data.sensor_id, 1);
}
void __attribute__((weak)) prot_rx_poll_sensor(struct prot_poll_sensor data) {}
void prot_build_start_stream(uint8_t *buf, struct prot_start_stream data) {
  memcpy(buf + 0, &data.ip, 4);
  memcpy(buf + 4, &data.port, 2);
  memcpy(buf + 6, &data.rate_ms, 2);
}
void __attribute__((weak)) prot_rx_start_stream(struct prot_start_stream data) {
}
void prot_build_stop_stream(uint8_t *buf, struct prot_stop_stream data) {}
void __attribute__((weak)) prot_rx_stop_stream(struct prot_stop_stream data) {}
void prot_build_set_node_id(uint8_t *buf, struct prot_set_node_id data) {
  memcpy(buf + 0, &data.node_id, 1);
}
void __attribute__((weak)) prot_rx_set_node_id(struct prot_set_node_id data) {}
void prot_build_wifi_config(uint8_t *buf, struct prot_wifi_config data) {
  memcpy(buf + 0, data.main_ssid, 32);
  memcpy(buf + 32, data.main_password, 64);
}
void __attribute__((weak)) prot_rx_wifi_config(struct prot_wifi_config data) {}
void prot_build_ota_packet(uint8_t *buf, struct prot_ota_packet data) {
  memcpy(buf + 0, &data.frame_number, 2);
  memcpy(buf + 2, &data.checksum, 2);
  memcpy(buf + 4, &data.frame_length, 1);
  memcpy(buf + 5, data.data, 255);
}
void __attribute__((weak)) prot_rx_ota_packet(struct prot_ota_packet data) {}
void prot_build_tof_12(uint8_t *buf, struct prot_tof_12 data) {
  memcpy(buf + 0, data.tof_1, 128);
  memcpy(buf + 128, data.tof_2, 128);
}
void __attribute__((weak)) prot_rx_tof_12(struct prot_tof_12 data) {}
void prot_build_tof_34(uint8_t *buf, struct prot_tof_34 data) {
  memcpy(buf + 0, data.tof_3, 128);
  memcpy(buf + 128, data.tof_4, 128);
}
void __attribute__((weak)) prot_rx_tof_34(struct prot_tof_34 data) {}
void prot_build_tof_56(uint8_t *buf, struct prot_tof_56 data) {
  memcpy(buf + 0, data.tof_5, 128);
  memcpy(buf + 128, data.tof_6, 128);
}
void __attribute__((weak)) prot_rx_tof_56(struct prot_tof_56 data) {}
void prot_build_tof_78(uint8_t *buf, struct prot_tof_78 data) {
  memcpy(buf + 0, data.tof_7, 128);
  memcpy(buf + 128, data.tof_8, 128);
}
void __attribute__((weak)) prot_rx_tof_78(struct prot_tof_78 data) {}
void prot_build_misc(uint8_t *buf, struct prot_misc data) {
  memcpy(buf + 0, &data.charge_percent, 1);
  memcpy(buf + 1, &data.battery_voltage_mv, 2);
  uint16_t temp_ax;
  packedFloat_to_uint(data.ax, -4, 4, temp_ax);
  memcpy(buf + 3, &temp_ax, 2);
  uint16_t temp_ay;
  packedFloat_to_uint(data.ay, -4, 4, temp_ay);
  memcpy(buf + 5, &temp_ay, 2);
  uint16_t temp_az;
  packedFloat_to_uint(data.az, -4, 4, temp_az);
  memcpy(buf + 7, &temp_az, 2);
  uint16_t temp_gx;
  packedFloat_to_uint(data.gx, -4, 4, temp_gx);
  memcpy(buf + 9, &temp_gx, 2);
  uint16_t temp_gy;
  packedFloat_to_uint(data.gy, -4, 4, temp_gy);
  memcpy(buf + 11, &temp_gy, 2);
  uint16_t temp_gz;
  packedFloat_to_uint(data.gz, -4, 4, temp_gz);
  memcpy(buf + 13, &temp_gz, 2);
  uint16_t temp_mx;
  packedFloat_to_uint(data.mx, -4, 4, temp_mx);
  memcpy(buf + 15, &temp_mx, 2);
  uint16_t temp_my;
  packedFloat_to_uint(data.my, -4, 4, temp_my);
  memcpy(buf + 17, &temp_my, 2);
  uint16_t temp_mz;
  packedFloat_to_uint(data.mz, -4, 4, temp_mz);
  memcpy(buf + 19, &temp_mz, 2);
}
void __attribute__((weak)) prot_rx_misc(struct prot_misc data) {}
void prot_build_uwb_distance(uint8_t *buf, struct prot_uwb_distance data) {
  memcpy(buf + 0, &data.set_this_to_zero, 1);
}
void __attribute__((weak)) prot_rx_uwb_distance(struct prot_uwb_distance data) {
}
void prot_parse(uint8_t id, uint8_t *buf) {
  if (id == 0) {
    struct prot_acknowledge data;
    prot_rx_acknowledge(data);
  }
  if (id == 127) {
    struct prot_reject data;
    prot_rx_reject(data);
  }
  if (id == 1) {
    struct prot_power_off data;
    prot_rx_power_off(data);
  }
  if (id == 2) {
    struct prot_move_rob_centric data;
    memcpy(&data.vel_x_mms, buf + 0, 2);
    memcpy(&data.vel_y_mms, buf + 2, 2);
    memcpy(&data.rot_degs, buf + 4, 2);
    prot_rx_move_rob_centric(data);
  }
  if (id == 3) {
    struct prot_move_wheels data;
    memcpy(&data.motor_1_mms, buf + 0, 2);
    memcpy(&data.motor_2_mms, buf + 2, 2);
    memcpy(&data.motor_3_mms, buf + 4, 2);
    memcpy(&data.motor_4_mms, buf + 6, 2);
    prot_rx_move_wheels(data);
  }
  if (id == 8) {
    struct prot_set_buzzer data;
    memcpy(&data.intensity, buf + 0, 1);
    prot_rx_set_buzzer(data);
  }
  if (id == 9) {
    struct prot_set_led data;
    memcpy(&data.led_r, buf + 0, 1);
    memcpy(&data.led_g, buf + 1, 1);
    memcpy(&data.led_b, buf + 2, 1);
    prot_rx_set_led(data);
  }
  if (id == 16) {
    struct prot_poll_all data;
    prot_rx_poll_all(data);
  }
  if (id == 17) {
    struct prot_poll_sensor data;
    memcpy(&data.sensor_id, buf + 0, 1);
    prot_rx_poll_sensor(data);
  }
  if (id == 18) {
    struct prot_start_stream data;
    memcpy(&data.ip, buf + 0, 4);
    memcpy(&data.port, buf + 4, 2);
    memcpy(&data.rate_ms, buf + 6, 2);
    prot_rx_start_stream(data);
  }
  if (id == 19) {
    struct prot_stop_stream data;
    prot_rx_stop_stream(data);
  }
  if (id == 20) {
    struct prot_set_node_id data;
    memcpy(&data.node_id, buf + 0, 1);
    prot_rx_set_node_id(data);
  }
  if (id == 21) {
    struct prot_wifi_config data;
    data.main_ssid = buf + 0;
    data.main_password = buf + 32;
    prot_rx_wifi_config(data);
  }
  if (id == 22) {
    struct prot_ota_packet data;
    memcpy(&data.frame_number, buf + 0, 2);
    memcpy(&data.checksum, buf + 2, 2);
    memcpy(&data.frame_length, buf + 4, 1);
    data.data = buf + 5;
    prot_rx_ota_packet(data);
  }
  if (id == 128) {
    struct prot_tof_12 data;
    data.tof_1 = buf + 0;
    data.tof_2 = buf + 128;
    prot_rx_tof_12(data);
  }
  if (id == 129) {
    struct prot_tof_34 data;
    data.tof_3 = buf + 0;
    data.tof_4 = buf + 128;
    prot_rx_tof_34(data);
  }
  if (id == 130) {
    struct prot_tof_56 data;
    data.tof_5 = buf + 0;
    data.tof_6 = buf + 128;
    prot_rx_tof_56(data);
  }
  if (id == 131) {
    struct prot_tof_78 data;
    data.tof_7 = buf + 0;
    data.tof_8 = buf + 128;
    prot_rx_tof_78(data);
  }
  if (id == 132) {
    struct prot_misc data;
    memcpy(&data.charge_percent, buf + 0, 1);
    memcpy(&data.battery_voltage_mv, buf + 1, 2);
    prot_FLOAT_DEF temp_ax;
    memcpy(&temp_ax, buf + 3, 2);
    uint_to_packedFloat(temp_ax, -4, 4, data.ax);
    prot_FLOAT_DEF temp_ay;
    memcpy(&temp_ay, buf + 5, 2);
    uint_to_packedFloat(temp_ay, -4, 4, data.ay);
    prot_FLOAT_DEF temp_az;
    memcpy(&temp_az, buf + 7, 2);
    uint_to_packedFloat(temp_az, -4, 4, data.az);
    prot_FLOAT_DEF temp_gx;
    memcpy(&temp_gx, buf + 9, 2);
    uint_to_packedFloat(temp_gx, -4, 4, data.gx);
    prot_FLOAT_DEF temp_gy;
    memcpy(&temp_gy, buf + 11, 2);
    uint_to_packedFloat(temp_gy, -4, 4, data.gy);
    prot_FLOAT_DEF temp_gz;
    memcpy(&temp_gz, buf + 13, 2);
    uint_to_packedFloat(temp_gz, -4, 4, data.gz);
    prot_FLOAT_DEF temp_mx;
    memcpy(&temp_mx, buf + 15, 2);
    uint_to_packedFloat(temp_mx, -4, 4, data.mx);
    prot_FLOAT_DEF temp_my;
    memcpy(&temp_my, buf + 17, 2);
    uint_to_packedFloat(temp_my, -4, 4, data.my);
    prot_FLOAT_DEF temp_mz;
    memcpy(&temp_mz, buf + 19, 2);
    uint_to_packedFloat(temp_mz, -4, 4, data.mz);
    prot_rx_misc(data);
  }
  if (id == 133) {
    struct prot_uwb_distance data;
    memcpy(&data.set_this_to_zero, buf + 0, 1);
    prot_rx_uwb_distance(data);
  }
}
bool prot_is_valid_id(uint8_t id) {
  switch (id) {
  case 0:
    return true;
    break;
  case 127:
    return true;
    break;
  case 1:
    return true;
    break;
  case 2:
    return true;
    break;
  case 3:
    return true;
    break;
  case 8:
    return true;
    break;
  case 9:
    return true;
    break;
  case 16:
    return true;
    break;
  case 17:
    return true;
    break;
  case 18:
    return true;
    break;
  case 19:
    return true;
    break;
  case 20:
    return true;
    break;
  case 21:
    return true;
    break;
  case 22:
    return true;
    break;
  case 128:
    return true;
    break;
  case 129:
    return true;
    break;
  case 130:
    return true;
    break;
  case 131:
    return true;
    break;
  case 132:
    return true;
    break;
  case 133:
    return true;
    break;
  default:
    return false;
  }
}

uintmax_t prot_id_to_len(uint8_t id) {
  switch (id) {
  case 0:
    return 0;
    break;
  case 127:
    return 0;
    break;
  case 1:
    return 0;
    break;
  case 2:
    return 6;
    break;
  case 3:
    return 8;
    break;
  case 8:
    return 1;
    break;
  case 9:
    return 3;
    break;
  case 16:
    return 0;
    break;
  case 17:
    return 1;
    break;
  case 18:
    return 8;
    break;
  case 19:
    return 0;
    break;
  case 20:
    return 1;
    break;
  case 21:
    return 96;
    break;
  case 22:
    return 260;
    break;
  case 128:
    return 256;
    break;
  case 129:
    return 256;
    break;
  case 130:
    return 256;
    break;
  case 131:
    return 256;
    break;
  case 132:
    return 21;
    break;
  case 133:
    return 1;
    break;
  default:
    return 0;
  }
}
