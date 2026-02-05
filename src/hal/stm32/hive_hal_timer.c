// Hardware Abstraction Layer - STM32 Timer Implementation
//
// Software timer wheel driven by hardware tick interrupt (SysTick or TIMx).
// Provides hive_timer_id_t-based API compatible with Linux implementation.
// Default tick resolution: 1ms (HIVE_TIMER_TICK_US)

#include "hal/hive_hal_timer.h"
#include "hive_timer.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include <string.h>
#include <stdbool.h>

// STM32-specific timer implementation using a software timer wheel
// Hardware timer (SysTick or TIMx) drives the tick at HIVE_TIMER_TICK_US
// interval Default: 1000us (1ms) tick resolution

#ifndef HIVE_TIMER_TICK_US
#define HIVE_TIMER_TICK_US 1000 // 1ms tick
#endif

// Active timer entry
typedef struct timer_entry_t {
    hive_timer_id_t id;
    hive_actor_id_t owner;
    uint32_t expiry_ticks;   // When timer expires (absolute tick count)
    uint32_t interval_ticks; // For periodic timers (0 = one-shot)
    bool periodic;
    struct timer_entry_t *next;
} timer_entry_t;

// Static pool for timer entries
static timer_entry_t s_timer_pool[HIVE_TIMER_ENTRY_POOL_SIZE];
static bool s_timer_used[HIVE_TIMER_ENTRY_POOL_SIZE];
static hive_pool_t s_timer_pool_mgr;

// Timer subsystem state
static struct {
    bool initialized;
    timer_entry_t *timers; // Active timers list (sorted by expiry)
    hive_timer_id_t next_id;
    volatile uint32_t tick_count; // Current tick count (updated by ISR)
    volatile bool tick_pending;   // Set by ISR, cleared by scheduler
} s_hal_timer = {0};

// Convert microseconds to ticks (rounding up)
static uint32_t us_to_ticks(uint32_t us) {
    return (us + HIVE_TIMER_TICK_US - 1) / HIVE_TIMER_TICK_US;
}

// Called by hardware timer ISR (SysTick or TIMx)
// This function must be called from the timer interrupt handler
void hive_timer_tick_isr(void) {
    s_hal_timer.tick_count++;
    s_hal_timer.tick_pending = true;
}

// Get current tick count
uint32_t hive_timer_get_ticks(void) {
    return s_hal_timer.tick_count;
}

// Process expired timers (called by scheduler in main loop)
void hive_timer_process_pending(void) {
    if (!s_hal_timer.tick_pending) {
        return;
    }
    s_hal_timer.tick_pending = false;

    uint32_t now = s_hal_timer.tick_count;

    // Process all expired timers
    timer_entry_t **pp = &s_hal_timer.timers;
    while (*pp) {
        timer_entry_t *entry = *pp;

        // Check if timer expired (handle wrap-around)
        int32_t delta = (int32_t)(entry->expiry_ticks - now);
        if (delta <= 0) {
            // Timer expired - deliver message to owner
            actor_t *a = hive_actor_get(entry->owner);
            if (a) {
                hive_ipc_notify_internal(entry->owner, entry->owner,
                                         HIVE_MSG_TIMER, HIVE_ID_NONE,
                                         entry->id, NULL, 0);
            }

            if (entry->periodic && a) {
                // Reschedule periodic timer
                entry->expiry_ticks = now + entry->interval_ticks;
                pp = &entry->next;
            } else {
                // Remove one-shot or dead actor_t's timer
                *pp = entry->next;
                hive_pool_free(&s_timer_pool_mgr, entry);
            }
        } else {
            pp = &entry->next;
        }
    }
}

// Handle timer event from scheduler (compatibility with io_source_t interface)
void hive_timer_handle_event(io_source_t *source) {
    (void)source;
    // On STM32, timer processing is done via hive_timer_process_pending()
    // This function exists for API compatibility but shouldn't be called
}

hive_status_t hive_hal_timer_init(void) {
    if (s_hal_timer.initialized) {
        return HIVE_SUCCESS;
    }

    // Initialize timer entry pool
    hive_pool_init(&s_timer_pool_mgr, s_timer_pool, s_timer_used,
                   sizeof(timer_entry_t), HIVE_TIMER_ENTRY_POOL_SIZE);

    // Initialize timer state
    s_hal_timer.timers = NULL;
    s_hal_timer.next_id = 1;
    s_hal_timer.tick_count = 0;
    s_hal_timer.tick_pending = false;

    // Hardware timer initialization should be done by the application
    // (e.g., configure SysTick to call hive_timer_tick_isr every
    // HIVE_TIMER_TICK_US)

    s_hal_timer.initialized = true;
    return HIVE_SUCCESS;
}

void hive_hal_timer_cleanup(void) {
    if (!s_hal_timer.initialized) {
        return;
    }

    // Clean up all active timers
    timer_entry_t *entry = s_hal_timer.timers;
    while (entry) {
        timer_entry_t *next = entry->next;
        hive_pool_free(&s_timer_pool_mgr, entry);
        entry = next;
    }
    s_hal_timer.timers = NULL;

    s_hal_timer.initialized = false;
}

hive_status_t hive_hal_timer_create(uint32_t interval_us, bool periodic,
                                    hive_actor_id_t owner,
                                    hive_timer_id_t *out) {
    // Allocate timer entry from pool
    timer_entry_t *entry = hive_pool_alloc(&s_timer_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Timer entry pool exhausted");
    }

    // Calculate expiry
    uint32_t ticks = us_to_ticks(interval_us);
    if (ticks == 0)
        ticks = 1; // Minimum 1 tick

    // Initialize timer entry
    entry->id = s_hal_timer.next_id++;
    entry->owner = owner;
    entry->expiry_ticks = s_hal_timer.tick_count + ticks;
    entry->interval_ticks = periodic ? ticks : 0;
    entry->periodic = periodic;

    // Insert into list (simple append - could optimize with sorted insert)
    entry->next = s_hal_timer.timers;
    s_hal_timer.timers = entry;

    *out = entry->id;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_timer_cancel(hive_timer_id_t id) {
    // Find and remove timer from list
    timer_entry_t *found = NULL;
    SLIST_FIND_REMOVE(s_hal_timer.timers, entry->id == id, found);

    if (found) {
        hive_pool_free(&s_timer_pool_mgr, found);
        return HIVE_SUCCESS;
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Timer not found");
}

uint64_t hive_hal_timer_get_time(void) {
    return (uint64_t)s_hal_timer.tick_count * HIVE_TIMER_TICK_US;
}

// Advance simulation time (microseconds) and process expired timers
// On STM32, this directly advances the tick counter (similar to ISR)
void hive_hal_timer_advance_time(uint64_t delta_us) {
    if (!s_hal_timer.initialized) {
        return;
    }

    // Convert microseconds to ticks
    uint32_t ticks = us_to_ticks((uint32_t)delta_us);

    // Advance tick count
    s_hal_timer.tick_count += ticks;
    s_hal_timer.tick_pending = true;

    // Process expired timers immediately
    hive_timer_process_pending();
}
