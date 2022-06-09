#include <stdint.h>

#include "prot.h"

//global variables are ugly but yeah, this tells what durin should respond with
extern uint8_t response;

struct protocol_state {
    uint8_t state;
    uint8_t payload_buf[512];
    uint16_t expected_len;
    uint16_t current_len;
    uint8_t id;
};

uint8_t protocol_parse_byte(struct protocol_state *state, uint8_t byte);