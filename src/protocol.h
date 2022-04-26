#include <stdint.h>

#define POWER_OFF 1

#define MOVE_ROB_CENTRIC 2
#define MOVE_WHEELS 3

#define POLL_ALL 16
#define POLL_SENSOR 17
#define START_STREAM 18
#define STOP_STREAM 19

#define TOF_UDP_1 128
#define TOF_UDP_2 129
#define TOF_UDP_3 130
#define TOF_UDP_4 131
#define MISC_UDP 132
#define UWB_UDP 133
#define ACKNOWLEDGE 0

struct __attribute__((packed)) start_stream {
    uint32_t ip;
    uint16_t port;
    uint16_t rate;
};

struct __attribute__((packed)) poll_sensor {
    uint8_t id;
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

struct __attribute__((packed)) tof_package {
    uint16_t tof_0[8*8];
    uint16_t tof_1[8*8];
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

uint8_t protocol_parse_byte(struct protocol_state *state, uint8_t byte, uint8_t *response);