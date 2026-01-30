// Crazyflie 2.1+ HAL - LED
//
// Debug LED control (PD2 blue LED).

#include "../hal.h"
#include "platform.h"

void hal_led_on(void) {
    platform_led_on();
}

void hal_led_off(void) {
    platform_led_off();
}

void hal_led_toggle(void) {
    platform_led_toggle();
}
