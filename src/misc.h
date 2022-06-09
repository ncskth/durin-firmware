#include "pt.h"

void set_led(uint8_t r, uint8_t g, uint8_t b);
void set_buzzer(uint8_t intensity);
void update_persistent_data();
void power_off();

void init_misc();
void update_misc(struct pt *pt);