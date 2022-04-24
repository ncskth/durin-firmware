#include <stdint.h>

#include "nbe_i2c.h"
#include "dynamixel.h"

enum control_mode {
    DURIN_MOTOR_VELOCITY,
    DURIN_ROBOT_VELOCITY,
};

struct durin_telemetry {
    float battery_voltage;
    float ranging_data[8][8*8];
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float heading;
    float vx, vy;
    float px, py;
};

struct durin_control {
    float target_vx;
    float target_vy;
    float target_py;
    enum control_mode control_mode;
    union {
        struct {
            int16_t velocity_x;
            int16_t velocity_y;
            int16_t rotation_cw;
        } robot_velocity;

        struct {
            int16_t motor_1;
            int16_t motor_2;
            int16_t motor_3;
            int16_t motor_4;
        } motor_velocity;
    };
};

struct durin_info {
    uint8_t robot_id;
    uint64_t cycle_count;
    uint64_t last_message_received;
    uint8_t init_finished;
    uint8_t wifi_connected;
    uint8_t telemetry_udp_enabled;
    uint16_t telemetry_udp_port;
    uint32_t telemetry_udp_address;
    uint16_t telemetry_udp_rate; 
};

struct durin_hardware {
    nbe_i2c_t i2c_tof;
    nbe_i2c_t i2c_imu;
    uint16_t port_expander_ouput;
    uint16_t port_expander_input;
    dynamixel_t dx;
};

struct durin {
    struct durin_telemetry telemetry;
    struct durin_control control;
    struct durin_info info;
    struct durin_hardware hw;
};

extern volatile struct durin durin;
