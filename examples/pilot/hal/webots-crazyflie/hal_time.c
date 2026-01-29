// Webots Crazyflie HAL - Time
//
// Timing functions using Webots simulation time.

#include "../hal.h"
#include <webots/robot.h>

#include "hal_internal.h"

void hal_delay_ms(uint32_t ms) {
    // Advance simulation time by stepping
    uint32_t steps = (ms + TIME_STEP_MS - 1) / TIME_STEP_MS;
    for (uint32_t i = 0; i < steps; i++) {
        wb_robot_step(TIME_STEP_MS);
    }
}

uint32_t hal_get_time_ms(void) {
    return (uint32_t)(wb_robot_get_time() * 1000.0);
}
