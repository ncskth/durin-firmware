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
  prot_fields_fallback_ssid = 18,
  prot_fields_fallback_password = 19,
  prot_fields_tof_1 = 20,
  prot_fields_tof_2 = 21,
  prot_fields_tof_3 = 22,
  prot_fields_tof_4 = 23,
  prot_fields_tof_5 = 24,
  prot_fields_tof_6 = 25,
  prot_fields_tof_7 = 26,
  prot_fields_tof_8 = 27,
  prot_fields_charge_percent = 28,
  prot_fields_battery_voltage_mv = 29,
  prot_fields_ax = 30,
  prot_fields_ay = 31,
  prot_fields_az = 32,
  prot_fields_gx = 33,
  prot_fields_gy = 34,
  prot_fields_gz = 35,
  prot_fields_mx = 36,
  prot_fields_my = 37,
  prot_fields_mz = 38,
  prot_fields_set_this_to_zero = 39,
};
enum messages {
  prot_messages_acknowledge = 0,
  prot_messages_power_off = 1,
  prot_messages_move_rob_centric = 2,
  prot_messages_move_wheel = 3,
  prot_messages_set_buzzer = 4,
  prot_messages_set_led = 5,
  prot_messages_poll_all = 6,
  prot_messages_poll_sensor = 7,
  prot_messages_start_stream = 8,
  prot_messages_stop_stream = 9,
  prot_messages_set_node_id = 10,
  prot_messages_wifi_config = 11,
  prot_messages_fallback_wifi_config = 12,
  prot_messages_tof_12 = 13,
  prot_messages_tof_34 = 14,
  prot_messages_tof_56 = 15,
  prot_messages_tof_78 = 16,
  prot_messages_misc = 17,
  prot_messages_uwb_distance = 18,
};
enum categories {
  prot_categories_none = 0,
};
#define scaledFloat_to_uint(value, scale, out) out = value * scale;

#define uint_to_scaledFloat(value, scale, out) value / scale;
#define packedFloat_to_uint(value, minValue, maxValue, out)                    \
  uint64_t intMax = 1 << (sizeof(out) * 8) - 1;                                \
  if (value < minValue) {                                                      \
    out = 0;                                                                   \
  } else if (value > maxValue) {                                               \
    out = intMax;                                                              \
  } else {                                                                     \
    prot_FLOAT_DEF ratio = (value - minValue) / (maxValue - minValue);         \
    out = 1 + ((intMax - 2)) * ratio;                                          \
  }

#define uint_to_packedFloat(value, minValue, maxValue, out)                    \
  uint64_t intMax = 1 << (sizeof(out) * 8) - 1;                                \
  if (value <= 0) {                                                            \
    out = minValue - 1.0;                                                      \
  } else if (value >= intMax) {                                                \
    out = maxValue + 1.0;                                                      \
  } else {                                                                     \
    prot_FLOAT_DEF ratio = (value - 1) / (intMax - 2);                         \
    out = ratio * (maxValue - minValue) + minValue;                            \
  }
void prot_build_acknowledge(uint8_t *buf) {}
void prot_rx_acknowledge(uint8_t id);
void __attribute__((weak)) prot_rx_acknowledge(uint8_t id) {}
void prot_build_power_off(uint8_t *buf) {}
void prot_rx_power_off(uint8_t id);
void __attribute__((weak)) prot_rx_power_off(uint8_t id) {}
void prot_build_move_rob_centric(uint8_t *buf, uint16_t vel_x_mms,
                                 uint16_t vel_y_mms, uint16_t rot_degs) {
  memcpy(buf + 0, &vel_x_mms, 2);
  memcpy(buf + 2, &vel_y_mms, 2);
  memcpy(buf + 4, &rot_degs, 2);
}
void prot_rx_move_rob_centric(uint8_t id, uint16_t vel_x_mms,
                              uint16_t vel_y_mms, uint16_t rot_degs);
void __attribute__((weak))
prot_rx_move_rob_centric(uint8_t id, uint16_t vel_x_mms, uint16_t vel_y_mms,
                         uint16_t rot_degs) {}
void prot_build_move_wheel(uint8_t *buf, uint16_t motor_1_mms,
                           uint16_t motor_2_mms, uint16_t motor_3_mms,
                           uint16_t motor_4_mms) {
  memcpy(buf + 0, &motor_1_mms, 2);
  memcpy(buf + 2, &motor_2_mms, 2);
  memcpy(buf + 4, &motor_3_mms, 2);
  memcpy(buf + 6, &motor_4_mms, 2);
}
void prot_rx_move_wheel(uint8_t id, uint16_t motor_1_mms, uint16_t motor_2_mms,
                        uint16_t motor_3_mms, uint16_t motor_4_mms);
void __attribute__((weak))
prot_rx_move_wheel(uint8_t id, uint16_t motor_1_mms, uint16_t motor_2_mms,
                   uint16_t motor_3_mms, uint16_t motor_4_mms) {}
void prot_build_set_buzzer(uint8_t *buf, uint8_t intensity) {
  memcpy(buf + 0, &intensity, 1);
}
void prot_rx_set_buzzer(uint8_t id, uint8_t intensity);
void __attribute__((weak)) prot_rx_set_buzzer(uint8_t id, uint8_t intensity) {}
void prot_build_set_led(uint8_t *buf, uint16_t led_r, uint16_t led_g,
                        uint16_t led_b) {
  memcpy(buf + 0, &led_r, 2);
  memcpy(buf + 2, &led_g, 2);
  memcpy(buf + 4, &led_b, 2);
}
void prot_rx_set_led(uint8_t id, uint16_t led_r, uint16_t led_g,
                     uint16_t led_b);
void __attribute__((weak))
prot_rx_set_led(uint8_t id, uint16_t led_r, uint16_t led_g, uint16_t led_b) {}
void prot_build_poll_all(uint8_t *buf) {}
void prot_rx_poll_all(uint8_t id);
void __attribute__((weak)) prot_rx_poll_all(uint8_t id) {}
void prot_build_poll_sensor(uint8_t *buf, uint8_t sensor_id) {
  memcpy(buf + 0, &sensor_id, 1);
}
void prot_rx_poll_sensor(uint8_t id, uint8_t sensor_id);
void __attribute__((weak)) prot_rx_poll_sensor(uint8_t id, uint8_t sensor_id) {}
void prot_build_start_stream(uint8_t *buf, uint32_t ip, uint16_t port,
                             uint16_t rate_ms) {
  memcpy(buf + 0, &ip, 4);
  memcpy(buf + 4, &port, 2);
  memcpy(buf + 6, &rate_ms, 2);
}
void prot_rx_start_stream(uint8_t id, uint32_t ip, uint16_t port,
                          uint16_t rate_ms);
void __attribute__((weak))
prot_rx_start_stream(uint8_t id, uint32_t ip, uint16_t port, uint16_t rate_ms) {
}
void prot_build_stop_stream(uint8_t *buf) {}
void prot_rx_stop_stream(uint8_t id);
void __attribute__((weak)) prot_rx_stop_stream(uint8_t id) {}
void prot_build_set_node_id(uint8_t *buf, uint8_t node_id) {
  memcpy(buf + 0, &node_id, 1);
}
void prot_rx_set_node_id(uint8_t id, uint8_t node_id);
void __attribute__((weak)) prot_rx_set_node_id(uint8_t id, uint8_t node_id) {}
void prot_build_wifi_config(uint8_t *buf, uint8_t *main_ssid,
                            uint8_t *main_password) {
  memcpy(buf + 0, main_ssid, 64);
  memcpy(buf + 64, main_password, 64);
}
void prot_rx_wifi_config(uint8_t id, uint8_t *main_ssid,
                         uint8_t *main_password);
void __attribute__((weak))
prot_rx_wifi_config(uint8_t id, uint8_t *main_ssid, uint8_t *main_password) {}
void prot_build_fallback_wifi_config(uint8_t *buf, uint8_t *fallback_ssid,
                                     uint8_t *fallback_password) {
  memcpy(buf + 0, fallback_ssid, 64);
  memcpy(buf + 64, fallback_password, 64);
}
void prot_rx_fallback_wifi_config(uint8_t id, uint8_t *fallback_ssid,
                                  uint8_t *fallback_password);
void __attribute__((weak))
prot_rx_fallback_wifi_config(uint8_t id, uint8_t *fallback_ssid,
                             uint8_t *fallback_password) {}
void prot_build_tof_12(uint8_t *buf, uint8_t *tof_1, uint8_t *tof_2) {
  memcpy(buf + 0, tof_1, 128);
  memcpy(buf + 128, tof_2, 128);
}
void prot_rx_tof_12(uint8_t id, uint8_t *tof_1, uint8_t *tof_2);
void __attribute__((weak))
prot_rx_tof_12(uint8_t id, uint8_t *tof_1, uint8_t *tof_2) {}
void prot_build_tof_34(uint8_t *buf, uint8_t *tof_3, uint8_t *tof_4) {
  memcpy(buf + 0, tof_3, 128);
  memcpy(buf + 128, tof_4, 128);
}
void prot_rx_tof_34(uint8_t id, uint8_t *tof_3, uint8_t *tof_4);
void __attribute__((weak))
prot_rx_tof_34(uint8_t id, uint8_t *tof_3, uint8_t *tof_4) {}
void prot_build_tof_56(uint8_t *buf, uint8_t *tof_5, uint8_t *tof_6) {
  memcpy(buf + 0, tof_5, 128);
  memcpy(buf + 128, tof_6, 128);
}
void prot_rx_tof_56(uint8_t id, uint8_t *tof_5, uint8_t *tof_6);
void __attribute__((weak))
prot_rx_tof_56(uint8_t id, uint8_t *tof_5, uint8_t *tof_6) {}
void prot_build_tof_78(uint8_t *buf, uint8_t *tof_7, uint8_t *tof_8) {
  memcpy(buf + 0, tof_7, 128);
  memcpy(buf + 128, tof_8, 128);
}
void prot_rx_tof_78(uint8_t id, uint8_t *tof_7, uint8_t *tof_8);
void __attribute__((weak))
prot_rx_tof_78(uint8_t id, uint8_t *tof_7, uint8_t *tof_8) {}
void prot_build_misc(uint8_t *buf, uint8_t charge_percent,
                     uint16_t battery_voltage_mv, uint16_t ax, uint16_t ay,
                     uint16_t az, uint16_t gx, uint16_t gy, uint16_t gz,
                     uint16_t mx, uint16_t my, uint16_t mz) {
  memcpy(buf + 0, &charge_percent, 1);
  memcpy(buf + 1, &battery_voltage_mv, 2);
  uint16_t temp_ax;
  packedFloat_to_uint(ax, -4, 4, temp_ax);
  memcpy(buf + 3, &temp_ax, 2);
  uint16_t temp_ay;
  packedFloat_to_uint(ay, -4, 4, temp_ay);
  memcpy(buf + 5, &temp_ay, 2);
  uint16_t temp_az;
  packedFloat_to_uint(az, -4, 4, temp_az);
  memcpy(buf + 7, &temp_az, 2);
  uint16_t temp_gx;
  packedFloat_to_uint(gx, -4, 4, temp_gx);
  memcpy(buf + 9, &temp_gx, 2);
  uint16_t temp_gy;
  packedFloat_to_uint(gy, -4, 4, temp_gy);
  memcpy(buf + 11, &temp_gy, 2);
  uint16_t temp_gz;
  packedFloat_to_uint(gz, -4, 4, temp_gz);
  memcpy(buf + 13, &temp_gz, 2);
  uint16_t temp_mx;
  packedFloat_to_uint(mx, -4, 4, temp_mx);
  memcpy(buf + 15, &temp_mx, 2);
  uint16_t temp_my;
  packedFloat_to_uint(my, -4, 4, temp_my);
  memcpy(buf + 17, &temp_my, 2);
  uint16_t temp_mz;
  packedFloat_to_uint(mz, -4, 4, temp_mz);
  memcpy(buf + 19, &temp_mz, 2);
}
void prot_rx_misc(uint8_t id, uint8_t charge_percent,
                  uint16_t battery_voltage_mv, uint16_t ax, uint16_t ay,
                  uint16_t az, uint16_t gx, uint16_t gy, uint16_t gz,
                  uint16_t mx, uint16_t my, uint16_t mz);
void __attribute__((weak))
prot_rx_misc(uint8_t id, uint8_t charge_percent, uint16_t battery_voltage_mv,
             uint16_t ax, uint16_t ay, uint16_t az, uint16_t gx, uint16_t gy,
             uint16_t gz, uint16_t mx, uint16_t my, uint16_t mz) {}
void prot_build_uwb_distance(uint8_t *buf, uint8_t set_this_to_zero) {
  memcpy(buf + 0, &set_this_to_zero, 1);
}
void prot_rx_uwb_distance(uint8_t id, uint8_t set_this_to_zero);
void __attribute__((weak))
prot_rx_uwb_distance(uint8_t id, uint8_t set_this_to_zero) {}
void prot_parse(uint8_t id, uint8_t *buf) {
  if (id == 0) {
    prot_rx_acknowledge(id);
  }
  if (id == 1) {
    prot_rx_power_off(id);
  }
  if (id == 2) {
    uint16_t vel_x_mms;
    uint16_t vel_y_mms;
    uint16_t rot_degs;
    memcpy(&vel_x_mms, buf + 0, 2);
    memcpy(&vel_y_mms, buf + 2, 2);
    memcpy(&rot_degs, buf + 4, 2);
    prot_rx_move_rob_centric(id, vel_x_mms, vel_y_mms, rot_degs);
  }
  if (id == 3) {
    uint16_t motor_1_mms;
    uint16_t motor_2_mms;
    uint16_t motor_3_mms;
    uint16_t motor_4_mms;
    memcpy(&motor_1_mms, buf + 0, 2);
    memcpy(&motor_2_mms, buf + 2, 2);
    memcpy(&motor_3_mms, buf + 4, 2);
    memcpy(&motor_4_mms, buf + 6, 2);
    prot_rx_move_wheel(id, motor_1_mms, motor_2_mms, motor_3_mms, motor_4_mms);
  }
  if (id == 8) {
    uint8_t intensity;
    memcpy(&intensity, buf + 0, 1);
    prot_rx_set_buzzer(id, intensity);
  }
  if (id == 9) {
    uint16_t led_r;
    uint16_t led_g;
    uint16_t led_b;
    memcpy(&led_r, buf + 0, 2);
    memcpy(&led_g, buf + 2, 2);
    memcpy(&led_b, buf + 4, 2);
    prot_rx_set_led(id, led_r, led_g, led_b);
  }
  if (id == 16) {
    prot_rx_poll_all(id);
  }
  if (id == 17) {
    uint8_t sensor_id;
    memcpy(&sensor_id, buf + 0, 1);
    prot_rx_poll_sensor(id, sensor_id);
  }
  if (id == 18) {
    uint32_t ip;
    uint16_t port;
    uint16_t rate_ms;
    memcpy(&ip, buf + 0, 4);
    memcpy(&port, buf + 4, 2);
    memcpy(&rate_ms, buf + 6, 2);
    prot_rx_start_stream(id, ip, port, rate_ms);
  }
  if (id == 19) {
    prot_rx_stop_stream(id);
  }
  if (id == 20) {
    uint8_t node_id;
    memcpy(&node_id, buf + 0, 1);
    prot_rx_set_node_id(id, node_id);
  }
  if (id == 21) {
    uint8_t *main_ssid;
    uint8_t *main_password;
    memcpy(main_ssid, buf + 0, 64);
    memcpy(main_password, buf + 64, 64);
    prot_rx_wifi_config(id, main_ssid, main_password);
  }
  if (id == 22) {
    uint8_t *fallback_ssid;
    uint8_t *fallback_password;
    memcpy(fallback_ssid, buf + 0, 64);
    memcpy(fallback_password, buf + 64, 64);
    prot_rx_fallback_wifi_config(id, fallback_ssid, fallback_password);
  }
  if (id == 128) {
    uint8_t *tof_1;
    uint8_t *tof_2;
    memcpy(tof_1, buf + 0, 128);
    memcpy(tof_2, buf + 128, 128);
    prot_rx_tof_12(id, tof_1, tof_2);
  }
  if (id == 129) {
    uint8_t *tof_3;
    uint8_t *tof_4;
    memcpy(tof_3, buf + 0, 128);
    memcpy(tof_4, buf + 128, 128);
    prot_rx_tof_34(id, tof_3, tof_4);
  }
  if (id == 130) {
    uint8_t *tof_5;
    uint8_t *tof_6;
    memcpy(tof_5, buf + 0, 128);
    memcpy(tof_6, buf + 128, 128);
    prot_rx_tof_56(id, tof_5, tof_6);
  }
  if (id == 131) {
    uint8_t *tof_7;
    uint8_t *tof_8;
    memcpy(tof_7, buf + 0, 128);
    memcpy(tof_8, buf + 128, 128);
    prot_rx_tof_78(id, tof_7, tof_8);
  }
  if (id == 132) {
    uint8_t charge_percent;
    uint16_t battery_voltage_mv;
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
    uint16_t mx;
    uint16_t my;
    uint16_t mz;
    memcpy(&charge_percent, buf + 0, 1);
    memcpy(&battery_voltage_mv, buf + 1, 2);
    uint16_t temp_ax;
    memcpy(&temp_ax, buf + 3, 2);
    uint_to_packedFloat(temp_ax, -4, 4, ax);
    uint16_t temp_ay;
    memcpy(&temp_ay, buf + 5, 2);
    uint_to_packedFloat(temp_ay, -4, 4, ay);
    uint16_t temp_az;
    memcpy(&temp_az, buf + 7, 2);
    uint_to_packedFloat(temp_az, -4, 4, az);
    uint16_t temp_gx;
    memcpy(&temp_gx, buf + 9, 2);
    uint_to_packedFloat(temp_gx, -4, 4, gx);
    uint16_t temp_gy;
    memcpy(&temp_gy, buf + 11, 2);
    uint_to_packedFloat(temp_gy, -4, 4, gy);
    uint16_t temp_gz;
    memcpy(&temp_gz, buf + 13, 2);
    uint_to_packedFloat(temp_gz, -4, 4, gz);
    uint16_t temp_mx;
    memcpy(&temp_mx, buf + 15, 2);
    uint_to_packedFloat(temp_mx, -4, 4, mx);
    uint16_t temp_my;
    memcpy(&temp_my, buf + 17, 2);
    uint_to_packedFloat(temp_my, -4, 4, my);
    uint16_t temp_mz;
    memcpy(&temp_mz, buf + 19, 2);
    uint_to_packedFloat(temp_mz, -4, 4, mz);
    prot_rx_misc(id, charge_percent, battery_voltage_mv, ax, ay, az, gx, gy, gz,
                 mx, my, mz);
  }
  if (id == 133) {
    uint8_t set_this_to_zero;
    memcpy(&set_this_to_zero, buf + 0, 1);
    prot_rx_uwb_distance(id, set_this_to_zero);
  }
}
bool is_valid_id(uint8_t id) {
  switch (id) {
  case 0:
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

uint8_t id_to_len(uint8_t id) {
  switch (id) {
  case 0:
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
    return 6;
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
    return 128;
    break;
  case 22:
    return 128;
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

#endif
