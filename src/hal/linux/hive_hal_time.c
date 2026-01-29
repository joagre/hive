// Hardware Abstraction Layer - Linux Time Implementation
//
// Time functions using POSIX clock_gettime.
// Critical sections are no-ops (single-threaded, no ISRs).

#include "hal/hive_hal_time.h"
#include <time.h>

uint64_t hive_hal_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint32_t hive_hal_critical_enter(void) {
    // No-op on Linux - single-threaded, no ISRs
    return 0;
}

void hive_hal_critical_exit(uint32_t state) {
    // No-op on Linux
    (void)state;
}
