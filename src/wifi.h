#include "pt.h"

void send_tcp(uint8_t *buf, uint16_t len);
void send_udp(uint8_t *buf, uint16_t len);

void init_wifi();
void update_wifi(struct pt *pt);
void update_tcp_server(struct pt *pt);