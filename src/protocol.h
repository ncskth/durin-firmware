#include <stdint.h>

#include "prot.h"
#include "durin.h"

struct protocol_state {
    uint8_t state;
    uint8_t payload_buf[2048];
    uint16_t expected_len;
    uint16_t current_len;
    uint8_t id;
    enum comm_channel channel;
};

void protocol_parse_byte(struct protocol_state *state, uint8_t byte);

void init_durinbase(struct capn *c, struct capn_segment **cs, struct DurinBase *msg);
void finish_durinbase(struct capn *c, struct capn_segment **cs, struct DurinBase *msg, uint8_t *buf, uint16_t *len);