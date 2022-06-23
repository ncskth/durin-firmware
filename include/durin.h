#include <stdint.h>

#include "nbe_i2c.h"
#include "dynamixel.h"

#define GREEN 0,255,0
#define BLUE 0,0,255
#define RED 255,0,0
#define YELLOW 255,80,0
#define PINK 255, 20, 20

#define DEFAULT_SSID "NCSpeople"
#define DEFAULT_PASSWORD "peopleNCS"

#define NUM_NODES 6

struct durin_persistent {
    uint8_t node_id;
    uint8_t main_ssid[32];
    uint8_t main_password[64];
};

struct durin_telemetry {
    float battery_voltage;
    uint16_t ranging_data[8][8*8];
    uint16_t distance_to_node[NUM_NODES];
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    float heading;
    float vx, vy;
    float px, py;
};

struct durin_control {
    float target_vx;
    float target_vy;
    float target_py;
    uint8_t control_id;
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
    uint64_t cycle_count;
    uint64_t last_message_received;
    uint8_t init_finished;
    uint8_t wifi_connected;
    uint8_t telemetry_udp_enabled;
    uint16_t telemetry_udp_port;
    uint32_t telemetry_udp_address;
    uint16_t telemetry_udp_rate; 
    uint8_t tof_sensor_alive[8];
    uint8_t expander_awaiting_update;
    uint8_t uwb_attempts[NUM_NODES];
    uint8_t motor_enabled;
    uint8_t user_enabled;
    uint8_t failed_node_polls[NUM_NODES];
    bool stationary;
};

struct durin_hardware {
    nbe_i2c_t i2c_tof;
    nbe_i2c_t i2c_imu;
    uint16_t port_expander_output;
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
extern volatile struct durin_persistent durin_persistent;