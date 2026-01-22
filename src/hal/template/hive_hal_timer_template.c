// Hardware Abstraction Layer - Timer Template
//
// Template for implementing the timer HAL on a new platform.
// Copy this file to src/hal/<platform>/hive_hal_timer_<platform>.c
// and implement the functions according to your platform's capabilities.
//
// The timer HAL provides:
// - One-shot and periodic timers
// - Timer cancellation
// - Time queries (for simulation mode)
// - Time advancement (for testing/simulation)

#include "hal/hive_hal_timer.h"
#include "hive_timer.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_actor.h"
#include "hive_ipc.h"
#include <stdbool.h>

// =============================================================================
// Timer Entry Structure
// =============================================================================

// Each active timer needs tracking information
typedef struct timer_entry {
    timer_id id;
    actor_id owner;
    bool periodic;
    uint64_t interval_us; // Interval for periodic timers
    uint64_t expiry_us;   // When timer fires (absolute time)
    struct timer_entry *next;
    // Platform-specific fields:
    // - Linux: int fd (timerfd), io_source source
    // - STM32: uint32_t expiry_ticks, uint32_t interval_ticks
} timer_entry;

// =============================================================================
// Static Storage
// =============================================================================

// Timer entry pool
static timer_entry s_timer_pool[HIVE_TIMER_ENTRY_POOL_SIZE];
static bool s_timer_used[HIVE_TIMER_ENTRY_POOL_SIZE];
static hive_pool s_timer_pool_mgr;

// Timer subsystem state
static struct {
    bool initialized;
    timer_entry *timers; // Active timers list
    timer_id next_id;
    uint64_t current_time_us; // For simulation mode
    // Platform-specific fields as needed
} s_hal_timer = {0};

// =============================================================================
// HAL Implementation
// =============================================================================

hive_status hive_hal_timer_init(void) {
    if (s_hal_timer.initialized) {
        return HIVE_SUCCESS;
    }

    // Initialize timer entry pool
    hive_pool_init(&s_timer_pool_mgr, s_timer_pool, s_timer_used,
                   sizeof(timer_entry), HIVE_TIMER_ENTRY_POOL_SIZE);

    // Initialize timer state
    s_hal_timer.timers = NULL;
    s_hal_timer.next_id = 1;
    s_hal_timer.current_time_us = 0;

    // TODO: Platform-specific initialization
    // - Linux: Nothing special needed (timerfd created per-timer)
    // - STM32: Configure SysTick or hardware timer to call hive_timer_tick_isr()

    s_hal_timer.initialized = true;
    return HIVE_SUCCESS;
}

void hive_hal_timer_cleanup(void) {
    if (!s_hal_timer.initialized) {
        return;
    }

    // Clean up all active timers
    timer_entry *entry = s_hal_timer.timers;
    while (entry) {
        timer_entry *next = entry->next;
        // TODO: Platform-specific cleanup (close timerfd, etc.)
        hive_pool_free(&s_timer_pool_mgr, entry);
        entry = next;
    }
    s_hal_timer.timers = NULL;

    s_hal_timer.initialized = false;
}

hive_status hive_hal_timer_create(uint32_t interval_us, bool periodic,
                                  actor_id owner, timer_id *out) {
    // Allocate timer entry from pool
    timer_entry *entry = hive_pool_alloc(&s_timer_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Timer entry pool exhausted");
    }

    // Initialize common fields
    entry->id = s_hal_timer.next_id++;
    entry->owner = owner;
    entry->periodic = periodic;
    entry->interval_us = interval_us;
    entry->expiry_us = s_hal_timer.current_time_us + interval_us;
    entry->next = s_hal_timer.timers;

    // TODO: Platform-specific timer setup
    //
    // Option A: Hardware timer (Linux with timerfd)
    //   - Create timerfd with timerfd_create()
    //   - Set timer with timerfd_settime()
    //   - Register with event system via hive_hal_event_register()
    //   - When timerfd fires, send HIVE_MSG_TIMER to owner
    //
    // Option B: Software timer wheel (STM32, simulation)
    //   - Store expiry time/ticks
    //   - Check expiry in periodic tick handler or advance_time()
    //   - When expired, call hive_ipc_notify_internal(owner, owner,
    //       HIVE_MSG_TIMER, entry->id, NULL, 0)

    s_hal_timer.timers = entry;
    *out = entry->id;
    return HIVE_SUCCESS;
}

hive_status hive_hal_timer_cancel(timer_id id) {
    // Find and remove timer from list
    timer_entry **pp = &s_hal_timer.timers;
    while (*pp) {
        timer_entry *entry = *pp;
        if (entry->id == id) {
            *pp = entry->next;
            // TODO: Platform-specific cleanup (close timerfd, etc.)
            hive_pool_free(&s_timer_pool_mgr, entry);
            return HIVE_SUCCESS;
        }
        pp = &entry->next;
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Timer not found");
}

uint64_t hive_hal_timer_get_time(void) {
    // TODO: Return current time in microseconds
    //
    // Option A: Real time (Linux)
    //   struct timespec ts;
    //   clock_gettime(CLOCK_MONOTONIC, &ts);
    //   return ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    //
    // Option B: Tick-based time (STM32)
    //   return tick_count * TICK_PERIOD_US;
    //
    // Option C: Simulation time
    //   return s_hal_timer.current_time_us;

    return s_hal_timer.current_time_us;
}

void hive_hal_timer_advance_time(uint64_t delta_us) {
    if (!s_hal_timer.initialized) {
        return;
    }

    // Advance simulation time
    s_hal_timer.current_time_us += delta_us;

    // Fire all expired timers
    // Note: Iterate carefully as firing may cause timer creation/cancellation
    bool fired_any;
    do {
        fired_any = false;
        timer_entry **pp = &s_hal_timer.timers;

        while (*pp) {
            timer_entry *entry = *pp;

            if (entry->expiry_us <= s_hal_timer.current_time_us) {
                // Timer expired - send message to owner
                actor *a = hive_actor_get(entry->owner);
                if (a) {
                    hive_ipc_notify_internal(entry->owner, entry->owner,
                                             HIVE_MSG_TIMER, entry->id, NULL,
                                             0);
                }

                fired_any = true;

                if (entry->periodic && a) {
                    // Reschedule periodic timer
                    entry->expiry_us += entry->interval_us;
                    pp = &entry->next;
                } else {
                    // Remove one-shot or dead actor's timer
                    *pp = entry->next;
                    hive_pool_free(&s_timer_pool_mgr, entry);
                }
            } else {
                pp = &entry->next;
            }
        }
    } while (fired_any);
}

// =============================================================================
// Platform-Specific Functions (examples)
// =============================================================================

// For STM32: Call this from your SysTick or timer interrupt handler
// void hive_timer_tick_isr(void) {
//     s_hal_timer.tick_count++;
//     s_hal_timer.tick_pending = true;
// }

// For STM32: Call this from the scheduler main loop
// void hive_timer_process_pending(void) {
//     if (!s_hal_timer.tick_pending) return;
//     s_hal_timer.tick_pending = false;
//     // Check and fire expired timers...
// }

// For Linux: Timer event handler (called when timerfd becomes readable)
// void hive_timer_handle_event(io_source *source) {
//     timer_entry *entry = source->data.timer;
//     // Read timerfd to acknowledge
//     // Send HIVE_MSG_TIMER to owner
//     // If one-shot, cleanup timer
// }
