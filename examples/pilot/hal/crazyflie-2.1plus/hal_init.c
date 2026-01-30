// Crazyflie 2.1+ HAL - Initialization
//
// Platform lifecycle: init, cleanup, self-test, calibration, arm/disarm.

#include "../hal.h"
#include "platform.h"

int hal_init(void) {
    // SWO debug output is initialized by platform_init()
    return platform_init();
}

void hal_cleanup(void) {
    platform_disarm();
}

bool hal_self_test(void) {
    return platform_self_test();
}

void hal_calibrate(void) {
    platform_calibrate();
}

void hal_arm(void) {
    platform_arm();
}

void hal_disarm(void) {
    platform_disarm();
}
