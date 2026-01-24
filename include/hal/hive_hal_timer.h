// Hardware Abstraction Layer - Timer
//
// Abstracts platform-specific timer operations:
// - Linux: timerfd + epoll with optional simulation mode
// - STM32: Software timer wheel driven by hardware tick ISR
//
// The common hive_timer.c wrapper handles:
// - Subsystem initialization guards
// - Argument validation
// - hive_sleep() implementation (timer + selective receive)
//
// HAL implementations manage their own timer pools and provide the core
// operations.

#ifndef HIVE_HAL_TIMER_H
#define HIVE_HAL_TIMER_H

#include "hive_types.h"
#include <stdint.h>

// Timer ID type (also defined in hive_timer.h)
typedef uint32_t timer_id_t;

// Initialize timer subsystem.
// Linux: Initialize timer pool
// STM32: Initialize timer pool and wheel
hive_status_t hive_hal_timer_init(void);

// Cleanup timer subsystem.
// Cancels all active timers and releases resources.
void hive_hal_timer_cleanup(void);

// Create a timer.
// interval_us: Delay/interval in microseconds
// periodic: true for periodic timer, false for one-shot
// owner: Actor ID that owns this timer
// out: Output timer ID
// Returns: HIVE_SUCCESS or error status
//
// Timer messages are sent to the owner actor with:
// - class = HIVE_MSG_TIMER
// - tag = timer_id_t
// - no payload
hive_status_t hive_hal_timer_create(uint32_t interval_us, bool periodic,
                                    actor_id_t owner, timer_id_t *out);

// Cancel a timer.
// id: Timer ID to cancel
// Returns: HIVE_SUCCESS or error status
hive_status_t hive_hal_timer_cancel(timer_id_t id);

// Get current time in microseconds.
// Returns monotonic time suitable for measuring elapsed durations.
// In simulation mode, returns simulated time.
uint64_t hive_hal_timer_get_time(void);

// Advance simulation time and fire due timers.
// delta_us: Microseconds to advance
//
// On first call, enables simulation mode (disables real-time timers on Linux).
// Calling this function enables deterministic time control for testing.
//
// Linux: Converts existing timerfd timers to software timers
// STM32: Directly advances tick counter and processes expired timers
void hive_hal_timer_advance_time(uint64_t delta_us);

#endif // HIVE_HAL_TIMER_H
