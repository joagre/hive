// Hardware Abstraction Layer - Time and Critical Sections
//
// These are the most fundamental HAL primitives. Every platform needs them.
// A porter implements these 4 functions to get basic time support.

#ifndef HIVE_HAL_TIME_H
#define HIVE_HAL_TIME_H

#include <stdint.h>

// Get current monotonic time in microseconds.
// Must be callable from any context (including ISR on embedded).
// Resolution should be at least 1ms, preferably better.
//
// Linux: clock_gettime(CLOCK_MONOTONIC)
// STM32: DWT->CYCCNT or SysTick + tick counter
uint64_t hive_hal_get_time_us(void);

// Enter critical section.
// Linux: No-op (single-threaded, no ISRs)
// STM32: Disable interrupts, return previous PRIMASK
//
// Returns opaque state value to pass to hive_hal_critical_exit().
// Must support nesting (save/restore, not just enable/disable).
uint32_t hive_hal_critical_enter(void);

// Exit critical section.
// Restores interrupt state from hive_hal_critical_enter().
void hive_hal_critical_exit(uint32_t state);

// Compiler memory barrier.
// Prevents compiler from reordering memory accesses across this point.
// Used when reading volatile flags set by ISR.
static inline void hive_hal_memory_barrier(void) {
    __asm__ volatile("" ::: "memory");
}

#endif // HIVE_HAL_TIME_H
