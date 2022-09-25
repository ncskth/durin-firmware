#ifndef DURIN_H
#define DURIN_H

#include <stdint.h>

#include "nbe_i2c.h"
#include "dynamixel.h"
#include "capnp_c.h"
#include "schema.capnp.h"

#define GREEN 0,255,0
#define BLUE 0,0,255
#define RED 255,0,0
#define YELLOW 255,80,0
#define PINK 255, 20, 20

#define DEFAULT_SSID "NCSpeople"
#define DEFAULT_PASSWORD "peopleNCS"

#define CAPN_PACKED 0

struct distance_measurement {
    uint8_t id;
    uint32_t distance;
    int32_t position_x; // in mm
    int32_t position_y; // in mm
    int32_t position_z; // in mm
    uint8_t flags;
    uint16_t error; // in mm
};

struct durin_persistent {
    uint8_t node_id;
    uint8_t main_ssid[32];
    uint8_t main_password[64];
};

struct durin_telemetry {
    float battery_voltage;
    uint16_t ranging_data[8][8*8];
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    float pos_x, pos_y, pos_z;
    uint8_t fix_type;
    struct distance_measurement distance_data[32];
    uint8_t distance_index;
};

struct durin_info {
    uint64_t cycle_count;
    uint64_t last_message_received;
    uint8_t init_finished;
    uint8_t wifi_connected;
    bool streaming_enabled;
    enum EnableStreaming_destination_which telemetry_destination;
    uint16_t telemetry_udp_port;
    uint32_t telemetry_udp_address;
    uint8_t tof_sensor_alive[8];
    uint8_t expander_awaiting_update;
    uint8_t motor_enabled;
    uint8_t user_enabled;
    uint16_t tof_stream_period;
    uint16_t position_stream_period;
    uint16_t imu_stream_period;
    uint16_t systemstatus_stream_period;
    enum EnableLogging_which logging_enabled;
    bool ota_in_progress;
};

struct durin_control {
    enum DurinBase_which control_type;
    union {
        struct SetRobotVelocity setRobotVelocity;
        struct SetWheelVelocity setWheelVelocity;
    };
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
    struct durin_info info;
    struct durin_hardware hw;
    struct durin_control control;
};


enum comm_channel{
    CHANNEL_UART,
    CHANNEL_TCP,
};

void send_response(uint8_t *buf, uint16_t len, enum comm_channel where);
void send_telemetry(uint8_t *buf, uint16_t len);
void send_printf(char *format, ...);

void set_led(uint8_t r, uint8_t g, uint8_t b);
void set_buzzer(uint8_t intensity);
void update_persistent_data();
void power_off();

extern volatile struct durin durin;
extern volatile struct durin_persistent durin_persistent;

#endif