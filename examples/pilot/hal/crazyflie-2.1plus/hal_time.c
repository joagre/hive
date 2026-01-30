// Crazyflie 2.1+ HAL - Time
//
// Timing functions: delay, get time.

#include "../hal.h"
#include "platform.h"

void hal_delay_ms(uint32_t ms) {
    platform_delay_ms(ms);
}

uint32_t hal_get_time_ms(void) {
    return platform_get_time_ms();
}
