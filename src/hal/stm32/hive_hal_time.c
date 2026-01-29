// Hardware Abstraction Layer - STM32 Time Implementation
//
// Time functions using tick counter from hive_hal_timer.c.
// Critical sections using PRIMASK interrupt disable.

#include "hal/hive_hal_time.h"

// Get tick count from timer subsystem
extern uint32_t hive_timer_get_ticks(void);

// Tick period in microseconds (from hive_hal_timer.c)
#ifndef HIVE_TIMER_TICK_US
#define HIVE_TIMER_TICK_US 1000
#endif

uint64_t hive_hal_get_time_us(void) {
    return (uint64_t)hive_timer_get_ticks() * HIVE_TIMER_TICK_US;
}

uint32_t hive_hal_critical_enter(void) {
    uint32_t primask;
    __asm__ volatile("mrs %0, primask\n"
                     "cpsid i\n"
                     : "=r"(primask)
                     :
                     : "memory");
    return primask;
}

void hive_hal_critical_exit(uint32_t state) {
    __asm__ volatile("msr primask, %0" : : "r"(state) : "memory");
}
