// Crazyflie 2.1+ HAL - Initialization
//
// Platform lifecycle: init, cleanup, self-test, calibration, arm/disarm.

#include "../hal.h"
#include "debug_swo.h"
#include "platform.h"

// Crazyflie 2.1+ runs at 168 MHz
#define CPU_FREQ_HZ 168000000
#define SWO_BAUD 2000000

int hal_init(void) {
    // Initialize SWO debug output first so we can log during init
    debug_swo_init(CPU_FREQ_HZ, SWO_BAUD);
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
