#include "pt.h"

#include <stdint.h>

void send_uart(uint8_t *buf, uint16_t len);

void init_user_uart();
void update_user_uart(struct pt *pt);