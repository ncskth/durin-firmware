#include <stdint.h>

#define UWB_BROADCAST_ID 0xff
#define UWB_MAGIC_WORD "cskth"
#define BROADCAST_RESPONSE_SLOT_MS 10 // 10ms

#define UWB_CONFIG {\
    9,               /* Channel number. */\
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */\
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */\
    9,               /* TX preamble code. Used in TX only. */\
    9,               /* RX preamble code. Used in RX only. */\
    2,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */\
    DWT_BR_6M8,      /* Data rate. */\
    DWT_PHRMODE_STD, /* PHY header mode. */\
    DWT_PHRRATE_STD, /* PHY header rate. */\
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */\
    DWT_STS_MODE_OFF, /* STS disabled */\
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */\
    DWT_PDOA_M0      /* PDOA mode off */\
}

#define UWB_HAS_3D_POSITION_BITMASK (1 << 0)
#define UWB_HAS_2D_POSITION_BITMASK (1 << 1)
#define UWB_IS_ANCHOR_BITMASK (1 << 2)

#define GET_BROADCAST_WAIT_MS(id) (BROADCAST_RESPONSE_SLOT_MS * 1.5 + BROADCAST_RESPONSE_SLOT_MS * id)
#define BROADCAST_BUSY_FOR_MS (BROADCAST_RESPONSE_SLOT_MS * 257.5)
#define SINGLE_BUSY_FOR_MS (1) //or until it has gotten a response

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING = 0,
    UWB_MSG_RESPONSE_RANGING = 1,
    UWB_MSG_POLL_ALIVE = 2,
    UWB_MSG_IS_ALIVE = 3,
    UWB_MSG_GIVE_WORD = 4,
    UWB_MSG_SYSTEM_STATUS = 5,
    UWB_MSG_QUIET = 6,
};

enum uwb_purposes {
    UWB_PURPOSE_BEACON_ORIGIN = 0,
    UWB_PURPOSE_BEACON_X = 1,
    UWB_PURPOSE_BEACON_Y = 2,
    UWB_PURPOSE_BEACON_Z = 3,
    UWB_PURPOSE_BEACON_REPEATER = 4,
    UWB_PURPOSE_BEACON_PASSIVE = 5,
    UWB_PURPOSE_USER = 6,
};

enum uwb_system_status {
    UWB_SYSTEM_STATUS_CALIBRATING = 0,
    UWB_SYSTEM_STATUS_ERROR = 1,
    UWB_SYSTEM_STATUS_GOOD = 2,
};

#pragma pack(push, 1)
struct uwb_header {
    char magic[5]; //{"cskth"}
    uint8_t msg_type;
    uint8_t sender;
    uint8_t receiver;
    uint8_t purpose;
};

struct uwb_poll_ranging {
    struct uwb_header header;
};

struct uwb_response_ranging {
    struct uwb_header header;
    uint32_t rx_timestamp;
    uint32_t tx_timestamp;
    int32_t position_x; // in mm
    int32_t position_y; // in mm
    int32_t position_z; // in mm
    uint8_t flags;
    uint16_t error; // in mm
};

struct uwb_poll_alive {
    struct uwb_header header;
};

struct uwb_is_alive {
    struct uwb_header header;
};

struct uwb_give_word {
    struct uwb_header header;
    uint32_t duration;   
};

struct uwb_system_status_msg {
    struct uwb_header header;
    uint8_t status;
};

struct uwb_quiet {
    struct uwb_header header;
    uint16_t duration;
};
#pragma pack(pop)