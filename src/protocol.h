#include <stdint.h>

struct __attribute__((packed)) start_stream {
    uint8_t ip[4];
    uint16_t port;
    uint16_t rate;
};

struct __attribute__((packed)) move_robot_velocity {
    int16_t vel_x;
    int16_t vel_y;
    int16_t rot;
};

struct __attribute__((packed)) move_motor_velocity {
    int16_t motor1;
    int16_t motor2;
    int16_t motor3;
    int16_t motor4;
};

struct __attribute__((packed)) tof_package_1 {
    uint16_t tof_0[8*8];
    uint16_t tof_1[8*8];
};

struct __attribute__((packed)) tof_package_2 {
    uint16_t tof_2[8*8];
    uint16_t tof_3[8*8];
};

struct __attribute__((packed)) tof_package_3 {
    uint16_t tof_4[8*8];
    uint16_t tof_5[8*8];
};

struct __attribute__((packed)) tof_package_4 {
    uint16_t tof_6[8*8];
    uint16_t tof_7[8*8];
};

struct __attribute__((packed)) misc_package {
    uint8_t charge_percent;
    uint16_t battery_voltage;
    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
    uint16_t mx;
    uint16_t my;
    uint16_t mz;
};

struct protocol_state {
    uint8_t state;
    uint8_t payload_buf[32];
    uint16_t expected_len;
    uint16_t current_len;
    uint8_t id;

};

void protocol_parse_byte(struct protocol_state *state, uint8_t byte);