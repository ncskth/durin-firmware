#ifdef __cplusplus
extern "C" {
#endif

#include "dynamixel.h"

#define DX_INS_PING 0x01
#define DX_INS_READ 0x02
#define DX_INS_WRITE 0x03
#define DX_INS_REG_WRITE 0x04
#define DX_INS_ACTION 0x05
#define DX_INS_FACTORY_RESET 0x06
#define DX_INS_REBOOT 0x08
#define DX_INS_CLEAR 0x10
#define DX_INS_CONTROL_TABLLE_BACKUP 0x20
#define DX_INS_RETURN 0x55
#define DX_INS_SYNC_READ 0x82
#define DX_INS_SYN_WRITE 0x83
#define DX_INS_FAST_SYNC_READ0 0x8A
#define DX_INS_BULK_READ 0x92
#define DX_INS_BULK_WRITE 0x93
#define DX_INS_FAST_BULK_READ 0x9A

#define DX_ADDR_BAUD_RATE 8
#define DX_ADDR_TORQUE_ENABLE 64
#define DX_ADDR_GOAL_VELOCITY 104
#define DX_ADDR_GOAL_PWM 100
#define DX_ADDR_OPERATING_MODE 11
#define DX_ADDR_STATUS_RETURN_LEVEL 68

#define DX_HEADER_0 0xFF
#define DX_HEADER_1 0xFF
#define DX_HEADER_2 0xFD
#define DX_HEADER_3 0x00

static uint16_t update_crc(uint16_t crc_accum, uint8_t *buf, uint16_t len);
static void dx_send_packet(dynamixel_t *dx, uint8_t id, uint8_t inst, uint8_t *buf, uint8_t len);
static void dx_send_buf(dynamixel_t *dx, uint8_t *buf, uint16_t len); // only hardware specific function
static void dx_parse_message(dynamixel_t *dx);
static void dx_read(dynamixel_t *dx);

//i stole this from the official library
static uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    uint16_t i, j;
    static const uint16_t crc_table[256] = { 0x0000,
        0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
        0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
        0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
        0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
        0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
        0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
        0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
        0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
        0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
        0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
        0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
        0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
        0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
        0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
        0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
        0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
        0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
        0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
        0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
        0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
        0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
        0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
        0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
        0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
        0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
        0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
        0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
        0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
        0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
        0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
        0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
        0x820D, 0x8207, 0x0202 };

    for (j = 0; j < data_blk_size; j++) {
        i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}

static void dx_send_buf(dynamixel_t *dx, uint8_t* buf, uint16_t len) {
    uint8_t lol[1];

    //just to silence a warning
    if (buf == NULL) {
        buf = lol;    
    }
    dx->expected_self_bytes += len;
    uart_write_bytes(dx->uart_num, buf, len);
}

static void dx_send_packet(dynamixel_t *dx, uint8_t id, uint8_t inst, uint8_t *buf, uint8_t len) {
    uint8_t header[] = {DX_HEADER_0, DX_HEADER_1, DX_HEADER_2, DX_HEADER_3,
        id, ((len + 3) & 0x0F), ((len + 3) & 0xF0) >> 8, inst
    };
    dx_send_buf(dx, header, sizeof(header));
    dx_send_buf(dx, buf, len); 

    uint16_t crc = 0;
    crc = update_crc(crc, header, sizeof(header));
    crc = update_crc(crc, buf, len);
    uint8_t crc_buf[] = {(crc & 0x0F), (crc & 0xF0) >> 8};
    dx_send_buf(dx, &crc, sizeof(crc_buf));
    
    
    // uint8_t out_buf[len + sizeof(header) + sizeof(crc_buf)];
    // memcpy(out_buf, header, sizeof(header));
    // memcpy(out_buf + sizeof(header), buf, len);
    // memcpy(out_buf + sizeof(header) + len, crc_buf, 2);
    // dx_send_buf(dx, out_buf, len + sizeof(header) + sizeof(crc_buf));
}

void dx_init(dynamixel_t *dx, uint8_t uart_num) {
    dx->uart_num = uart_num;
    dx->expected_self_bytes = 0;
    dx->state = DX_STATE_HEADER_0;
}

void dx_parse_byte(dynamixel_t *dx, uint8_t byte) {
    if (dx->expected_self_bytes > 0) {
        dx->expected_self_bytes--;
        return;
    }
    switch (dx->state) {
        case DX_STATE_HEADER_0:
            if (byte == DX_STATE_HEADER_0) {
                dx->state = DX_STATE_HEADER_1;
            } else {
                dx->state = DX_STATE_HEADER_0;
            }
            break;

        case DX_STATE_HEADER_1:
            if (byte == DX_STATE_HEADER_1) {
                dx->state = DX_STATE_HEADER_2;
            } else {
                dx->state = DX_STATE_HEADER_0;
            }
            break;

        case DX_STATE_HEADER_2:
            if (byte == DX_STATE_HEADER_2) {
                dx->state = DX_STATE_HEADER_3;
            } else {
                dx->state = DX_STATE_HEADER_0;
            }
            break;

        case DX_STATE_HEADER_3:
            if (byte == DX_STATE_HEADER_3) {
                dx->state = DX_STATE_ID;
            } else {
                dx->state = DX_STATE_HEADER_0;
            }
            break;

        case DX_STATE_ID:
            dx->rx_id = byte;
            dx->state = DX_STATE_LENGTH_0;
            break;

        case DX_STATE_LENGTH_0:
            dx->rx_params_length = byte & 0x0F;
            dx->state = DX_STATE_LENGTH_1;
            break;

        case DX_STATE_LENGTH_1:
            dx->rx_params_length = (byte & 0x0F) << 8;
            dx->rx_buf_index = 0;
            dx->state = DX_STATE_INSTRUCTION;
            break;

        case DX_STATE_INSTRUCTION:
            dx->rx_instruction = byte;
            dx->state = DX_STATE_ERROR;
            break;
        
        case DX_STATE_ERROR:
            dx->rx_error = byte;
            dx->state = DX_STATE_PARAM;
            if (dx->rx_params_length == 4) {
                dx->state = DX_STATE_CRC_0;
            }
            break;

        case DX_STATE_PARAM:
            dx->rx_buf[dx->rx_buf_index] = byte;
            dx->rx_buf_index++;
            if (dx->rx_buf_index == dx->rx_params_length - 4) {
                dx->state = DX_STATE_CRC_0;
            }
            break;

        case DX_STATE_CRC_0:
            dx->rx_crc = byte & 0x0F;
            dx->state = DX_STATE_CRC_1;
            break;

        case DX_STATE_CRC_1:
            dx->rx_crc = (byte & 0x0F) << 8;
            dx->state = DX_STATE_HEADER_0;
            break;
        
        default:
            dx->state = DX_STATE_HEADER_0;
        
        //TODO: actually use the data
    }
}

void dx_ping(dynamixel_t *dx, uint8_t id) {
    dx_send_packet(dx, id, DX_INS_PING, NULL, 0);
}

void dx_set_goal_velocity(dynamixel_t *dx, uint8_t id, float rpm, uint8_t sync) {
    uint8_t buf[6];
    buf[0] = DX_ADDR_GOAL_VELOCITY;
    buf[1] = 0;
    int16_t integer_rpm = rpm / 0.229;
    buf[2] = integer_rpm & 0x00ff;
    buf[3] = (integer_rpm >> 8) & 0x00ff;
    buf[4] = (integer_rpm >> 16) & 0x00ff;
    buf[5] = (integer_rpm >> 24) & 0x00ff;

    dx_send_packet(dx, id, sync ? DX_INS_REG_WRITE : DX_INS_WRITE, buf, 6);
}


void dx_test(dynamixel_t *dx) {
    uint8_t buf[6];
    buf[0] = DX_ADDR_GOAL_VELOCITY;
    buf[1] = 0;
    buf[2] = 200;
    buf[3] = 0;
    buf[4] = 0;
    buf[5] = 0;
    dx_send_packet(dx, 1, DX_INS_REG_WRITE, buf, 6);
}

void dx_set_goal_percent(dynamixel_t *dx, uint8_t id, float percent, uint8_t sync) {
    uint8_t buf[4];
    buf[0] = DX_ADDR_GOAL_VELOCITY;
    buf[1] = 0;
    int16_t integer_percent = percent / 0.113;
    buf[2] = integer_percent & 0x00ff;
    buf[3] = (integer_percent >> 8) & 0x00ff;
    dx_send_packet(dx, id, sync ? DX_INS_REG_WRITE : DX_INS_WRITE, buf, 4);
}

void dx_action(dynamixel_t *dx, uint8_t id) {
    dx_send_packet(dx, id, DX_INS_ACTION, NULL, 0);
}

void dx_enable_torque(dynamixel_t *dx, uint8_t id, uint8_t enabled) {
    uint8_t buf[3];
    buf[0] = DX_ADDR_TORQUE_ENABLE;
    buf[1] = 0;
    buf[2] = enabled;
    dx_send_packet(dx, id, DX_INS_WRITE, buf, 3);
}

void dx_set_baud(dynamixel_t *dx, uint8_t id, dx_baud_t baud) {
    uint8_t buf[3];
    buf[0] = DX_ADDR_BAUD_RATE;
    buf[1] = 0;
    buf[2] = baud;
    dx_send_packet(dx, id, DX_INS_WRITE, buf, 3);
}

void dx_set_operating_mode(dynamixel_t *dx, uint8_t id, dx_operating_mode_t mode) {
    uint8_t buf[3];
    buf[0] = DX_ADDR_OPERATING_MODE;
    buf[1] = 0;
    buf[2] = mode;
    dx_send_packet(dx, id, DX_INS_WRITE, buf, 3);
}

void dx_set_status_return_level(dynamixel_t *dx, uint8_t id, dx_status_return_level_t level) {
    uint8_t buf[3];
    buf[0] = DX_ADDR_STATUS_RETURN_LEVEL;
    buf[1] = 0;
    buf[2] = level;
    dx_send_packet(dx, id, DX_INS_WRITE, buf, 3);
}

#ifdef __cplusplus
extern "C" }
#endif