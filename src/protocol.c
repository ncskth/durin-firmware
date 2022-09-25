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
#include "user_uart.h"
#include "wifi.h"

#include "capnp_c.h"
#include "schema.capnp.h"

#define fast_acknowledge(response, cs) {\
    struct Acknowledge data;\
    response->message.acknowledge = new_Acknowledge(cs);\
    write_Acknowledge(&data, response->message.acknowledge);\
    response->message_which = DurinBase_message_acknowledge;\
}\

#define fast_reject(response, cs) {\
    struct Reject data;\
    response->message.reject = new_Reject(cs);\
    write_Reject(&data, response->message.reject);\
    response->message_which = DurinBase_message_reject;\
}\

void decode_message(uint8_t* buf, uint16_t len, enum comm_channel where);

void handle_enableStreaming(EnableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs);

void handle_disableStreaming(DisableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setRobotVelocity(SetRobotVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setWheelVelocity(SetWheelVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setLed(SetLed_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setBuzzer(SetBuzzer_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_otaUpdate(OtaUpdate_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_otaUpdateBegin(OtaUpdateBegin_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_otaUdateCommit(OtaUpdateCommit_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_getImuMeasurement(GetImuMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_getPosition(GetPosition_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_getSystemStatus(GetSystemStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_getTofObservations(GetTofObservations_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_getDistanceMeasurement(GetDistanceMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setImuStreamPeriod(SetImuStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setPositionStreamPeriod(SetPositionStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setSystemStatusStreamPeriod(SetSystemStatusStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_setTofStreamPeriod(SetTofStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_powerOff(PowerOff_ptr msg, struct DurinBase *response, struct capn_segment *cs);
void handle_enableLogging(EnableLogging_ptr msg, struct DurinBase *response, struct capn_segment *cs);

uint16_t build_packet(uint8_t *buf_in, uint16_t len_in, uint8_t *buf_out) {
    buf_out[0] = '\n';
    buf_out[1] = len_in & 0xff;
    buf_out[2] = (len_in >> 8) &0xff;
    memcpy(buf_out + 3, buf_in, len_in);
    return len_in + 3;
}


void send_response(uint8_t *buf, uint16_t len, enum comm_channel where) {
    uint8_t packet_buf[2048];
    uint16_t packet_len = build_packet(buf, len, packet_buf);
    if (where == CHANNEL_TCP) {
        send_tcp(packet_buf, packet_len);
    }
    if (where == CHANNEL_UART) {
        send_uart(packet_buf, packet_len);
    }
}

void send_telemetry(uint8_t *buf, uint16_t len) {
    uint8_t packet_buf[2048];
    uint16_t packet_len = build_packet(buf, len, packet_buf);
    if (durin.info.streaming_enabled == false) {
        return;
    }
    if (durin.info.telemetry_destination == EnableStreaming_destination_uartAndUdp) {
        send_uart(packet_buf, packet_len);
        send_udp(packet_buf, packet_len);
    }
    if (durin.info.telemetry_destination == EnableStreaming_destination_uartOnly) {
        send_uart(packet_buf, packet_len);
    }
    if (durin.info.telemetry_destination == EnableStreaming_destination_udpOnly) {
        send_udp(packet_buf, packet_len);
    }
}


void protocol_parse_byte(struct protocol_state *state, uint8_t byte) {
    switch(state->state) {
        case 0:
            printf("got \\n\n");
            if (byte == '\n') {
                state->state = 1;
            }
            break;

        case 1:
            printf("got len 1\n");
            state->expected_len = byte;
            state->state = 2;   
            break;

        case 2:
            state->expected_len += byte << 8;
            state->current_len = 0;
            state->state = 3;
            if (state->expected_len == 0) {
                decode_message(state->payload_buf, state->current_len, state->channel);
                state->state = 0;
            }
            printf("got len 2 %d\n", state->expected_len);
            break;

        case 3:
            state->payload_buf[state->current_len] = byte;
            state->current_len++;
            if (state->current_len == state->expected_len) {
                decode_message(state->payload_buf, state->current_len, state->channel);
                state->state = 0;
                printf("decoding message\n");
            }
            break;
    }
}

void decode_message(uint8_t* buf, uint16_t len, enum comm_channel where) {
    struct capn read_c;
    capn_init_mem(&read_c, buf, len, CAPN_PACKED /* packed */);
    DurinBase_ptr rroot;
    struct DurinBase base;
    rroot.p = capn_getp(capn_root(&read_c), 0 /* off */, 1 /* resolve */);
    read_DurinBase(&base, rroot);

 
    struct capn response_c;
    capn_init_malloc(&response_c);
    struct capn_segment *cs = capn_root(&response_c).seg;    
    struct DurinBase durin_response;

    switch (base.message_which) {
        case DurinBase_message_enableStreaming:
            handle_enableStreaming(base.message.enableStreaming, &durin_response, cs);
            break;
        
        case DurinBase_message_disableStreaming:
            handle_disableStreaming(base.message.disableStreaming, &durin_response, cs);
            break;
        
        case DurinBase_message_powerOff:
            handle_powerOff(base.message.powerOff, &durin_response, cs);
            break;
        
        case DurinBase_message_setRobotVelocity:
            handle_setRobotVelocity(base.message.setRobotVelocity, &durin_response, cs);
            break;
        
        case DurinBase_message_setWheelVelocity:
            handle_setWheelVelocity(base.message.setWheelVelocity, &durin_response, cs);
            break;

        case DurinBase_message_setLed:
            handle_setLed(base.message.setLed, &durin_response, cs);
            break;

        case DurinBase_message_setBuzzer:
            handle_setBuzzer(base.message.setBuzzer, &durin_response, cs);
            break;

        case DurinBase_message_otaUpdate:
            handle_otaUpdate(base.message.otaUpdate, &durin_response, cs);
            break;

        case DurinBase_message_otaUpdateCommit:
            handle_otaUdateCommit(base.message.otaUpdateCommit, &durin_response, cs);
            break;

        case DurinBase_message_getImuMeasurement:
            handle_getImuMeasurement(base.message.getImuMeasurement, &durin_response, cs);
            break;

        case DurinBase_message_getPosition:
            handle_getPosition(base.message.getPosition, &durin_response, cs);
            break;

        case DurinBase_message_getSystemStatus:
            handle_getSystemStatus(base.message.getSystemStatus, &durin_response, cs);
            break;

        case DurinBase_message_getTofObservations:
            handle_getTofObservations(base.message.getTofObservations, &durin_response, cs);
            break;

        case DurinBase_message_getDistanceMeasurement:
            handle_getDistanceMeasurement(base.message.getDistanceMeasurement, &durin_response, cs);
            break;
        
        case DurinBase_message_setImuStreamPeriod:
            handle_setImuStreamPeriod(base.message.setImuStreamPeriod, &durin_response, cs);
            break;

        case DurinBase_message_setPositionStreamPeriod:
            handle_setPositionStreamPeriod(base.message.setPositionStreamPeriod, &durin_response, cs);
            break;
        
        case DurinBase_message_setSystemStatusStreamPeriod:
            handle_setSystemStatusStreamPeriod(base.message.setSystemStatusStreamPeriod, &durin_response, cs);
            break;
        
        case DurinBase_message_setTofStreamPeriod:
            handle_setTofStreamPeriod(base.message.setTofStreamPeriod, &durin_response, cs);
            break;

        case DurinBase_message_otaUpdateBegin:
            handle_otaUpdateBegin(base.message.otaUpdateBegin, &durin_response, cs);
            break;
        
        case DurinBase_message_enableLogging:
            handle_enableLogging(base.message.enableLogging, &durin_response, cs);
            break;

        default:
            fast_reject((&durin_response), cs);
    }


    DurinBase_ptr durin_response_ptr = new_DurinBase(cs);
    write_DurinBase(&durin_response, durin_response_ptr);
    capn_setp(capn_root(&response_c), 0, durin_response_ptr.p);

    uint8_t response_buf[2048];
    uint16_t response_len = capn_write_mem(&response_c, response_buf, sizeof(response_buf), CAPN_PACKED);
    capn_free(&read_c);
    capn_free(&response_c);
    send_response(response_buf, response_len, where);
}

void handle_enableStreaming(EnableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct EnableStreaming data;
    uint8_t *raw_ip = (uint8_t*) &durin.info.telemetry_udp_address;
    read_EnableStreaming(&data, msg);
    durin.info.telemetry_destination = data.destination_which;
    durin.info.streaming_enabled = true;
    // i think oyu need to resolve the pointer which capn_len does. Took a long time to figure out...
    if (capn_len(data.destination.udpOnly.ip) != 4) {
        fast_reject(response, cs);
        return;
    }
    
    if (data.destination_which == EnableStreaming_destination_udpOnly) {
        raw_ip[0] = capn_get8(data.destination.udpOnly.ip, 0);
        raw_ip[1] = capn_get8(data.destination.udpOnly.ip, 1);
        raw_ip[2] = capn_get8(data.destination.udpOnly.ip, 2);
        raw_ip[3] = capn_get8(data.destination.udpOnly.ip, 3);
        durin.info.telemetry_udp_port = data.destination.udpOnly.port;
    } else
    if (data.destination_which == EnableStreaming_destination_uartAndUdp) {
        raw_ip[0] = capn_get8(data.destination.uartAndUdp.ip, 0);
        raw_ip[1] = capn_get8(data.destination.uartAndUdp.ip, 1);
        raw_ip[2] = capn_get8(data.destination.uartAndUdp.ip, 2);
        raw_ip[3] = capn_get8(data.destination.uartAndUdp.ip, 3);
        durin.info.telemetry_udp_port = data.destination.uartAndUdp.port;
    } else {
        fast_reject(response, cs);
        return;
    }

    fast_acknowledge(response, cs);
}

void handle_disableStreaming(DisableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    durin.info.streaming_enabled = false;
}

void handle_powerOff(PowerOff_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    printf("power off command goodbye!\n");
    fast_acknowledge(response, cs);    
    vTaskDelay(100);
    power_off();
}

void handle_setRobotVelocity(SetRobotVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    read_SetRobotVelocity(&durin.control.setRobotVelocity, msg);
    fast_acknowledge(response, cs);
}

void handle_setWheelVelocity(SetWheelVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    read_SetWheelVelocity(&durin.control.setWheelVelocity, msg);

    fast_acknowledge(response, cs);
}

void handle_setLed(SetLed_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetLed data;
    read_SetLed(&data, msg);
    set_led(data.ledR, data.ledG, data.ledB);
    fast_acknowledge(response, cs);
}

void handle_setBuzzer(SetBuzzer_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetBuzzer data;
    read_SetBuzzer(&data, msg);
    set_buzzer(data.enabled);
    fast_acknowledge(response, cs);
}

void handle_getImuMeasurement(GetImuMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct ImuMeasurement imu;
    imu.accelerometerXG = durin.telemetry.ax;
    imu.accelerometerYG = durin.telemetry.ay;
    imu.accelerometerZG = durin.telemetry.az;
    imu.gyroscopeXRads = durin.telemetry.gx;
    imu.gyroscopeYRads = durin.telemetry.gy;
    imu.gyroscopeZRads = durin.telemetry.gz;
    imu.magnetometerXUt = durin.telemetry.mx;
    imu.magnetometerYUt = durin.telemetry.my;
    imu.magnetometerZUt = durin.telemetry.mz;
    response->message.imuMeasurement = new_ImuMeasurement(cs);
    write_ImuMeasurement(&imu, response->message.imuMeasurement);
    response->message_which = DurinBase_message_imuMeasurement;
}

void handle_getPosition(GetPosition_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct Position position;
    if (durin.telemetry.fix_type == 3) {
        position.position.vector.x = durin.telemetry.pos_x;
        position.position.vector.y = durin.telemetry.pos_y;
        position.position.vector.z = durin.telemetry.pos_z;
        position.position_which = Position_position_vector;
    } else {
        position.position_which = Position_position_unknown;
    }

    response->message.position = new_Position(cs);
    write_Position(&position, response->message.position);
    response->message_which = DurinBase_message_position;
}

void handle_getSystemStatus(GetSystemStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SystemStatus status;
    status.batteryMv = durin.telemetry.battery_voltage * 1000;
    status.batteryPercent = 0; //TODO: 

    response->message.systemStatus = new_SystemStatus(cs);
    write_SystemStatus(&status, response->message.systemStatus);
    response->message_which = DurinBase_message_systemStatus;
}

void handle_getTofObservations(GetTofObservations_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct GetTofObservations request;
    struct TofObservations tof_observations;
    read_GetTofObservations(&request, msg);
    tof_observations.observations = new_TofObservations_TofObservation_list(cs, capn_len(request.ids));
    for (uint8_t i = 0; i < capn_len(request.ids); i++) {
        uint8_t id = capn_get8(request.ids, i);
        struct TofObservations_TofObservation observation;
        observation.id = id;
        uint8_t pixels = 64;
        observation.ranges = capn_new_list16(cs, pixels);
        capn_setv16(observation.ranges, 0, durin.telemetry.ranging_data[id], pixels);
        set_TofObservations_TofObservation(&observation, tof_observations.observations, i);        
    }

    response->message.tofObservations = new_TofObservations(cs);
    write_TofObservations(&tof_observations, response->message.tofObservations);
    response->message_which = DurinBase_message_tofObservations;
}

void handle_getDistanceMeasurement(GetDistanceMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    
}

void handle_setImuStreamPeriod(SetImuStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetImuStreamPeriod data;
    read_SetImuStreamPeriod(&data, msg);
    durin.info.imu_stream_period = data.periodMs;
}

void handle_setPositionStreamPeriod(SetPositionStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetPositionStreamPeriod data;
    read_SetPositionStreamPeriod(&data, msg);
    durin.info.position_stream_period = data.periodMs;
}

void handle_setSystemStatusStreamPeriod(SetSystemStatusStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetSystemStatusStreamPeriod data;
    read_SetSystemStatusStreamPeriod(&data, msg);
    durin.info.systemstatus_stream_period = data.periodMs;
}

void handle_setTofStreamPeriod(SetTofStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct SetTofStreamPeriod data;
    read_SetTofStreamPeriod(&data, msg);
    durin.info.tof_stream_period = data.periodMs;
}


esp_ota_handle_t ota_handle;
esp_partition_t *ota_partition;
bool ota_started = false;
void handle_otaUpdateBegin(OtaUpdateBegin_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    uint8_t stop = false;
    ota_partition = esp_ota_get_next_update_partition(NULL);
    bool ret = esp_ota_begin(ota_partition, 0, &ota_handle);
    if (ret == ESP_OK) {
        fast_acknowledge(response, cs);
        ota_started = 1;
    } else {
        fast_reject(response, cs);
    }
    ota_started = ret;
}

void handle_otaUpdate(OtaUpdate_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct OtaUpdate data;
    read_OtaUpdate(&data, msg);
    bool ret = esp_ota_write(ota_handle, capn_len(data.data), capn_len(data.data));
    if (ret == ESP_OK) {
        fast_acknowledge(response, cs);
    } else {
        fast_reject(response, cs);
    }
}

void handle_otaUdateCommit(OtaUpdateCommit_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    if (ota_started) {
        esp_ota_end(ota_handle);
        esp_ota_set_boot_partition(ota_partition);
        printf("Update done, restarting\n");
        vTaskDelay(2000);
        esp_restart();
    } else {
        const esp_partition_t *part = esp_ota_get_running_partition();
        esp_ota_img_states_t state;
        esp_ota_get_state_partition(part, &state);
        if (state == ESP_OTA_IMG_NEW || state == ESP_OTA_IMG_PENDING_VERIFY) {
            int e = esp_ota_mark_app_valid_cancel_rollback();
            if (e == 0) {
                fast_acknowledge(response, cs);
                printf("confirmed update\n");
            } else {
                fast_reject(response, cs);
                printf("Something went wrong when confirming\n");
            }
        } else {
            printf("confirmation not needed\n");
            fast_reject(response, cs);
        }
    } 
}

void handle_enableLogging(EnableLogging_ptr msg, struct DurinBase *response, struct capn_segment *cs) {
    struct EnableLogging data;
    read_EnableLogging(&data, msg);
    durin.info.logging_enabled = data.enabled;
    fast_acknowledge(response, cs);
}