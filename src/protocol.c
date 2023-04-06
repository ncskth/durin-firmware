#include <string.h>
#include <esp_ota_ops.h>
#include <esp_rom_crc.h>
#include <esp_netif.h>
#include <esp_system.h>

#include "prot.h"
#include "protocol.h"
#include "misc.h"


#include "durin.h"
#include "hardware.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "user_uart.h"
#include "wifi.h"
#include "tof_and_expander.h"

#include "capnp_c.h"
#include "schema.capnp.h"

#define HEADER_BYTE '*'

#define fast_acknowledge(response, cs) {\
    struct Acknowledge data;\
    response->acknowledge = new_Acknowledge(cs);\
    write_Acknowledge(&data, response->acknowledge);\
    response->which = DurinBase_acknowledge;\
}\

#define fast_reject(response, cs) {\
    struct Reject data;\
    response->reject = new_Reject(cs);\
    write_Reject(&data, response->reject);\
    response->which = DurinBase_reject;\
}\

void decode_message(uint8_t* buf, uint16_t len, enum comm_channel where);

void handle_enableStreaming(EnableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_disableStreaming(DisableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setRobotVelocity(SetRobotVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setWheelVelocity(SetWheelVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setLed(SetLed_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setBuzzer(SetBuzzer_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_otaUpdate(OtaUpdate_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_otaUpdateBegin(OtaUpdateBegin_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_otaUdateCommit(OtaUpdateCommit_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getImuMeasurement(GetImuMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getPosition(GetPosition_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getSystemStatus(GetSystemStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getTofObservations(GetTofObservations_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getDistanceMeasurement(GetDistanceMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setImuStreamPeriod(SetImuStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setPositionStreamPeriod(SetPositionStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setSystemStatusStreamPeriod(SetSystemStatusStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setTofStreamPeriod(SetTofStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_powerOff(PowerOff_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_enableLogging(EnableLogging_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setWifiConfig(SetWifiConfig_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setNodeId(SetNodeId_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setTofResolution(SetTofResolution_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_ping(Ping_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_getSystemInfo(GetSystemInfo_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_setUwbStreamPeriod(SetUwbStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);
void handle_enableTofStatus(EnableTofStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel);

void init_durinbase(struct capn *c, struct capn_segment **cs, struct DurinBase *msg) {
    capn_init_malloc(c);
    *cs = capn_root(c).seg;
}

void finish_durinbase(struct capn *c, struct capn_segment **cs, struct DurinBase *msg, uint8_t *buf, uint16_t* len) {
    DurinBase_ptr durin_ptr = new_DurinBase(*cs);
    write_DurinBase(msg, durin_ptr);
    int e = capn_setp(capn_root(c), 0, durin_ptr.p);
    *len = capn_write_mem(c, buf, *len, CAPN_PACKED);
    capn_free(c);
}

uint16_t build_packet(uint8_t *buf_in, uint16_t len_in, uint8_t *buf_out, uint8_t is_telemetry) {
    buf_out[0] = HEADER_BYTE;
    buf_out[1] = len_in & 0xff;
    buf_out[2] = (len_in >> 8) &0x0f;
    if (is_telemetry) {
        buf_out[2] |= 1 << 4;
    }
    memcpy(buf_out + 3, buf_in, len_in);
    uint8_t checksum = 0;
    for (size_t i = 0; i < len_in + 3; i++) {
        checksum ^= buf_out[i];
    }
    buf_out[len_in + 3] = checksum;
    return len_in + 4;
}

void send_response(uint8_t *buf, uint16_t len, enum comm_channel where) {
    uint8_t packet_buf[len + 50];
    uint16_t packet_len = build_packet(buf, len, packet_buf, 0);
    if (where == CHANNEL_TCP) {
        send_tcp(packet_buf, packet_len);
    }
    if (where == CHANNEL_UART) {
        send_uart(packet_buf, packet_len);
    }
}

void send_telemetry(uint8_t *buf, uint16_t len) {
    uint8_t packet_buf[len + 50];
    uint16_t packet_len = build_packet(buf, len, packet_buf, 1);
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
    uint8_t checksum;
    switch(state->state) {
        case 0:
            if (byte == HEADER_BYTE) {
                state->state = 1;
            }
            break;

        case 1:
            state->expected_len = byte;
            state->state = 2;
            break;

        case 2:
            state->expected_len += byte << 8;
            state->current_len = 0;
            state->state = 3;
            if (state->expected_len > sizeof(state->payload_buf)) {
                printf("PROTOCOL PARSE LENGTH ERROR\n");
                state = 0;
                break;
            }
            if (state->expected_len == 0) {
                state->state = 4;
            }
            break;

        case 3:
            state->payload_buf[state->current_len] = byte;
            state->current_len++;
            if (state->current_len == state->expected_len) {
                state->state = 4;
            }
            break;

        case 4:
            checksum = HEADER_BYTE;
            checksum ^= (state->current_len >> 8) & 0xff;
            checksum ^= state->current_len & 0xff;
            for (size_t i = 0; i < state->current_len; i++) {
                checksum ^= state->payload_buf[i];
            }
            if (checksum == byte) {
                decode_message(state->payload_buf, state->current_len, state->channel);
            } else {
                printf("invalid checksum %d %d\n", byte, checksum);
            }
            state->state = 0;
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
    printf("decoding message %d from %d\n", base.which, where);
    switch (base.which) {
        case DurinBase_enableStreaming:
            handle_enableStreaming(base.enableStreaming, &durin_response, cs, where);
            break;

        case DurinBase_disableStreaming:
            handle_disableStreaming(base.disableStreaming, &durin_response, cs, where);
            break;

        case DurinBase_powerOff:
            handle_powerOff(base.powerOff, &durin_response, cs, where);
            break;

        case DurinBase_setRobotVelocity:
            handle_setRobotVelocity(base.setRobotVelocity, &durin_response, cs, where);
            break;

        case DurinBase_setWheelVelocity:
            handle_setWheelVelocity(base.setWheelVelocity, &durin_response, cs, where);
            break;

        case DurinBase_setLed:
            handle_setLed(base.setLed, &durin_response, cs, where);
            break;

        case DurinBase_setBuzzer:
            handle_setBuzzer(base.setBuzzer, &durin_response, cs, where);
            break;

        case DurinBase_otaUpdate:
            handle_otaUpdate(base.otaUpdate, &durin_response, cs, where);
            break;

        case DurinBase_otaUpdateCommit:
            handle_otaUdateCommit(base.otaUpdateCommit, &durin_response, cs, where);
            break;

        case DurinBase_getImuMeasurement:
            handle_getImuMeasurement(base.getImuMeasurement, &durin_response, cs, where);
            break;

        case DurinBase_getPosition:
            handle_getPosition(base.getPosition, &durin_response, cs, where);
            break;

        case DurinBase_getSystemStatus:
            handle_getSystemStatus(base.getSystemStatus, &durin_response, cs, where);
            break;

        case DurinBase_getTofObservations:
            handle_getTofObservations(base.getTofObservations, &durin_response, cs, where);
            break;

        case DurinBase_setImuStreamPeriod:
            handle_setImuStreamPeriod(base.setImuStreamPeriod, &durin_response, cs, where);
            break;

        case DurinBase_setPositionStreamPeriod:
            handle_setPositionStreamPeriod(base.setPositionStreamPeriod, &durin_response, cs, where);
            break;

        case DurinBase_setSystemStatusStreamPeriod:
            handle_setSystemStatusStreamPeriod(base.setSystemStatusStreamPeriod, &durin_response, cs, where);
            break;

        case DurinBase_setTofStreamPeriod:
            handle_setTofStreamPeriod(base.setTofStreamPeriod, &durin_response, cs, where);
            break;

        case DurinBase_otaUpdateBegin:
            handle_otaUpdateBegin(base.otaUpdateBegin, &durin_response, cs, where);
            break;

        case DurinBase_enableLogging:
            handle_enableLogging(base.enableLogging, &durin_response, cs, where);
            break;

        case DurinBase_setWifiConfig:
            handle_setWifiConfig(base.setWifiConfig, &durin_response, cs, where);
            break;

        case DurinBase_setNodeId:
            handle_setNodeId(base.setNodeId, &durin_response, cs, where);
            break;

        case DurinBase_setTofResolution:
            handle_setTofResolution(base.setTofResolution, &durin_response, cs, where);
            break;

        case DurinBase_ping:
            handle_ping(base.ping, &durin_response, cs, where);
            break;

        case DurinBase_getSystemInfo:
            handle_getSystemInfo(base.getSystemInfo, &durin_response, cs, where);
            break;

        case DurinBase_setUwbStreamPeriod:
            handle_setUwbStreamPeriod(base.setUwbStreamPeriod, &durin_response, cs, where);
            break;

        case DurinBase_enableTofStatus:
            handle_enableTofStatus(base.enableTofStatus, &durin_response, cs, where);
            break;

        default:
            fast_reject((&durin_response), cs);
    }


    DurinBase_ptr durin_response_ptr = new_DurinBase(cs);
    write_DurinBase(&durin_response, durin_response_ptr);
    capn_setp(capn_root(&response_c), 0, durin_response_ptr.p);

    durin.info.last_message_received = esp_timer_get_time();
    durin.info.active = true;
    uint8_t response_buf[2048];
    uint16_t response_len = capn_write_mem(&response_c, response_buf, sizeof(response_buf), CAPN_PACKED);
    capn_free(&read_c);
    capn_free(&response_c);
    if (response_len == -1) {
        printf("PROTOCOL RESPONSE LENGTH ERROR\n");
        return;
    }
    send_response(response_buf, response_len, where);
}

void handle_ping(Ping_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    fast_acknowledge(response, cs);
}

void handle_enableStreaming(EnableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct EnableStreaming data;
    uint8_t *raw_ip = (uint8_t*) &durin.info.telemetry_udp_address;
    read_EnableStreaming(&data, msg);
    durin.info.telemetry_destination = data.destination_which;
    durin.info.streaming_enabled = true;

    if (data.destination_which == EnableStreaming_destination_udpOnly && channel == CHANNEL_TCP) {
        if (capn_len(data.destination.udpOnly.ip) != 4) {
            fast_reject(response, cs);
            return;
        }
        raw_ip[0] = capn_get8(data.destination.udpOnly.ip, 0);
        raw_ip[1] = capn_get8(data.destination.udpOnly.ip, 1);
        raw_ip[2] = capn_get8(data.destination.udpOnly.ip, 2);
        raw_ip[3] = capn_get8(data.destination.udpOnly.ip, 3);
        durin.info.telemetry_udp_port = data.destination.udpOnly.port;
    } else
    if (data.destination_which == EnableStreaming_destination_uartAndUdp) {
        if (capn_len(data.destination.uartAndUdp.ip) != 4) {
            fast_reject(response, cs);
            return;
        }
        raw_ip[0] = capn_get8(data.destination.uartAndUdp.ip, 0);
        raw_ip[1] = capn_get8(data.destination.uartAndUdp.ip, 1);
        raw_ip[2] = capn_get8(data.destination.uartAndUdp.ip, 2);
        raw_ip[3] = capn_get8(data.destination.uartAndUdp.ip, 3);
        durin.info.telemetry_udp_port = data.destination.uartAndUdp.port;
    } else
    if (data.destination_which == EnableStreaming_destination_uartOnly) {

    } else {
        fast_reject(response, cs);
        return;
    }
    fast_acknowledge(response, cs);
}

void handle_disableStreaming(DisableStreaming_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    durin.info.streaming_enabled = false;
    fast_acknowledge(response, cs);
}

void handle_powerOff(PowerOff_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    printf("power off command goodbye!\n");
    fast_acknowledge(response, cs);
    vTaskDelay(100);
    power_off();
}

void handle_setRobotVelocity(SetRobotVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    read_SetRobotVelocity(&durin.control.setRobotVelocity, msg);
    durin.control.control_type = DurinBase_setRobotVelocity;
    fast_acknowledge(response, cs);
}

void handle_setWheelVelocity(SetWheelVelocity_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    read_SetWheelVelocity(&durin.control.setWheelVelocity, msg);
    durin.control.control_type = DurinBase_setWheelVelocity;
    fast_acknowledge(response, cs);
}

void handle_setLed(SetLed_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetLed data;
    read_SetLed(&data, msg);
    set_led(data.ledR, data.ledG, data.ledB);
    fast_acknowledge(response, cs);
}

void handle_setBuzzer(SetBuzzer_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetBuzzer data;
    read_SetBuzzer(&data, msg);
    set_buzzer(data.enabled);
    fast_acknowledge(response, cs);
}

void handle_getImuMeasurement(GetImuMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct ImuMeasurement data;
    data.accelerometerX = durin.telemetry.raw_ax;
    data.accelerometerY = durin.telemetry.raw_ay;
    data.accelerometerZ = durin.telemetry.raw_az;
    data.gyroscopeX = durin.telemetry.raw_gx;
    data.gyroscopeY = durin.telemetry.raw_gy;
    data.gyroscopeZ = durin.telemetry.raw_gz;
    data.magnetometerX = durin.telemetry.raw_mx;
    data.magnetometerY = durin.telemetry.raw_my;
    data.magnetometerZ = durin.telemetry.raw_mz;
    response->imuMeasurement = new_ImuMeasurement(cs);
    write_ImuMeasurement(&data, response->imuMeasurement);
    response->which = DurinBase_imuMeasurement;
}

void handle_getPosition(GetPosition_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct Position position;
    if (durin.telemetry.fix_type == 3) {
        position.vectorMm.x = durin.telemetry.pos_x;
        position.vectorMm.y = durin.telemetry.pos_y;
        position.vectorMm.z = durin.telemetry.pos_z;
        position.which = Position_vectorMm;
    } else {
        position.which = Position_unknown;
    }

    response->position = new_Position(cs);
    write_Position(&position, response->position);
    response->which = DurinBase_position;
}

void handle_getSystemStatus(GetSystemStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SystemStatus status;
    status.batteryMv = durin.telemetry.battery_voltage * 1000;
    status.batteryPercent = 0; //TODO:
    status.batteryDischarge = 0;
    response->systemStatus = new_SystemStatus(cs);
    write_SystemStatus(&status, response->systemStatus);
    response->which = DurinBase_systemStatus;
}

void handle_setTofResolution(SetTofResolution_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetTofResolution data;
    read_SetTofResolution(&data, msg);
    set_tof_resolution(data.resolution);
    fast_acknowledge(response, cs);
}

void handle_getTofObservations(GetTofObservations_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    uint8_t to_send[8] = {1,1,1,1,1,1,1,1};
    build_tof_message(&response->tofObservations, cs, to_send);
    response->which = DurinBase_tofObservations;
}

void handle_getDistanceMeasurement(GetDistanceMeasurement_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    fast_acknowledge(response, cs);
}

void handle_setImuStreamPeriod(SetImuStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetImuStreamPeriod data;
    read_SetImuStreamPeriod(&data, msg);
    durin.info.imu_stream_period = data.periodMs;
    fast_acknowledge(response, cs);
}

void handle_setPositionStreamPeriod(SetPositionStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetPositionStreamPeriod data;
    read_SetPositionStreamPeriod(&data, msg);
    durin.info.position_stream_period = data.periodMs;
    fast_acknowledge(response, cs);
}

void handle_setSystemStatusStreamPeriod(SetSystemStatusStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetSystemStatusStreamPeriod data;
    read_SetSystemStatusStreamPeriod(&data, msg);
    durin.info.systemstatus_stream_period = data.periodMs;
    fast_acknowledge(response, cs);
}

void handle_setTofStreamPeriod(SetTofStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetTofStreamPeriod data;
    read_SetTofStreamPeriod(&data, msg);
    durin.info.tof_stream_period = data.periodMs;
    fast_acknowledge(response, cs);
}

void handle_setUwbStreamPeriod(SetUwbStreamPeriod_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetUwbStreamPeriod data;
    read_SetUwbStreamPeriod(&data, msg);
    durin.info.uwb_stream_period = data.periodMs;
    fast_acknowledge(response, cs);
}

esp_ota_handle_t ota_handle;
esp_partition_t *ota_partition;
void handle_otaUpdateBegin(OtaUpdateBegin_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    uint8_t stop = false;
    ota_partition = esp_ota_get_next_update_partition(NULL);
    bool ret = esp_ota_begin(ota_partition, 0, &ota_handle);
    if (ret == ESP_OK) {
        fast_acknowledge(response, cs);
        durin.info.ota_in_progress = 1;
    } else {
        fast_reject(response, cs);
    }
}

void handle_otaUpdate(OtaUpdate_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct OtaUpdate data;
    read_OtaUpdate(&data, msg);
    uint16_t len = capn_len(data.data);
    bool ret = esp_ota_write(ota_handle, data.data.p.data, len);
    if (ret == ESP_OK) {
        fast_acknowledge(response, cs);
    } else {
        fast_reject(response, cs);
    }
}

void handle_otaUdateCommit(OtaUpdateCommit_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    if (durin.info.ota_in_progress) {
        esp_ota_end(ota_handle);
        esp_ota_set_boot_partition(ota_partition);
        printf("Update done, restarting\n");
        vTaskDelay(1000);
        power_off();
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

void handle_enableLogging(EnableLogging_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct EnableLogging data;
    read_EnableLogging(&data, msg);
    durin.info.logging_enabled = data.which;
    fast_acknowledge(response, cs);
}

void handle_setWifiConfig(SetWifiConfig_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetWifiConfig data;
    read_SetWifiConfig(&data, msg);

    if (data.index >= DURIN_MAX_WIFI_CONFIGURATIONS) {
        fast_reject(response, cs);
        return;
    }
    memcpy(durin_persistent.wifi_configurations[data.index].ssid, data.ssid.str, sizeof(durin_persistent.wifi_configurations[0].ssid));
    memcpy(durin_persistent.wifi_configurations[data.index].password, data.password.str, sizeof(durin_persistent.wifi_configurations[0].password));
    update_persistent_data();
    fast_acknowledge(response, cs);
}

void handle_setNodeId(SetNodeId_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SetNodeId data;
    read_SetNodeId(&data, msg);
    durin_persistent.node_id = data.nodeId;
    update_persistent_data();
    fast_acknowledge(response, cs);
}

void handle_getSystemInfo(GetSystemInfo_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct SystemInfo data;

    char ip_pretty[32];
    tcpip_adapter_ip_info_t ipInfo;
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);
    char* ip_raw = (char*) &ipInfo.ip.addr;
    sprintf(ip_pretty, "%d.%d.%d.%d", ip_raw[0], ip_raw[1], ip_raw[2], ip_raw[3]);
    data.ip = (struct capn_text) {
        .len = strlen(ip_pretty),
        .str = ip_pretty,
        .seg = NULL,
    };

    uint8_t mac_raw[6];
    char mac_pretty[32];
    esp_efuse_mac_get_default(mac_raw);
    sprintf(mac_pretty, "%02x:%02x:%02x:%02x:%02x:%02x", mac_raw[0], mac_raw[1], mac_raw[2], mac_raw[3], mac_raw[4], mac_raw[5]);
    data.mac = (struct capn_text) {
        .len = strlen(mac_pretty),
        .str = mac_pretty,
        .seg = NULL,
    };

    char hostname[16];
    sprintf(hostname, "durin%d", durin_persistent.node_id);
    data.hostname = (struct capn_text) {
        .len = strlen(hostname),
        .str = hostname,
        .seg = NULL,
    };

    data.id = durin_persistent.node_id;
    data.uptimeMs = esp_timer_get_time() / 1000;

    response->systemInfo = new_SystemInfo(cs);
    write_SystemInfo(&data, response->systemInfo);
    response->which = DurinBase_systemInfo;
}

void handle_enableTofStatus(EnableTofStatus_ptr msg, struct DurinBase *response, struct capn_segment *cs, enum comm_channel channel) {
    struct EnableTofStatus data;
    read_EnableTofStatus(&data, msg);
    durin.info.tof_status_enabled = data.enabled;
}