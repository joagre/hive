// Hardware Abstraction Layer - Time Template
//
// This file provides a documented template for time functions.
// Copy to src/hal/<platform>/hive_hal_time.c and implement.

#include "hal/hive_hal_time.h"

// Get current monotonic time in microseconds.
//
// Requirements:
//   - Must be monotonically increasing (never go backwards)
//   - Resolution should be at least 1ms (microsecond preferred)
//   - Should not wrap during runtime (64-bit provides ~584,000 years)
//
// Examples:
//   - Linux: clock_gettime(CLOCK_MONOTONIC)
//   - STM32: SysTick counter or hardware timer
//   - Bare metal: Hardware timer with known frequency
//
uint64_t hive_hal_get_time_us(void) {
    // TODO: Implement for your platform
    // Example for a platform with a tick counter:
    //   extern uint32_t get_tick_count(void);
    //   return (uint64_t)get_tick_count() * TICK_PERIOD_US;
    return 0;
}

// Enter critical section (disable interrupts).
//
// Requirements:
//   - Must disable interrupts that could call runtime APIs
//   - Must return state that allows nesting (save previous state)
//   - Must be fast (called frequently in hot paths)
//
// Returns:
//   Previous interrupt state to restore in hive_hal_critical_exit()
//
// Examples:
//   - Linux (single-threaded): Return 0 (no-op)
//   - ARM Cortex-M: Save PRIMASK, then CPSID I
//   - RISC-V: Save and clear MIE bit
//
uint32_t hive_hal_critical_enter(void) {
    // TODO: Implement for your platform
    // Example for ARM Cortex-M:
    //   uint32_t primask;
    //   __asm__ volatile("mrs %0, primask\n" "cpsid i\n" : "=r"(primask) :: "memory");
    //   return primask;
    return 0;
}

// Exit critical section (restore interrupts).
//
// Parameters:
//   state - Value returned from hive_hal_critical_enter()
//
// Requirements:
//   - Must restore previous interrupt state (supports nesting)
//   - Must be fast
//
void hive_hal_critical_exit(uint32_t state) {
    // TODO: Implement for your platform
    // Example for ARM Cortex-M:
    //   __asm__ volatile("msr primask, %0" :: "r"(state) : "memory");
    (void)state;
}
