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
typedef struct timer_entry {
    timer_id id;
    actor_id owner;
    uint32_t expiry_ticks;   // When timer expires (absolute tick count)
    uint32_t interval_ticks; // For periodic timers (0 = one-shot)
    bool periodic;
    struct timer_entry *next;
} timer_entry;

// Static pool for timer entries
static timer_entry g_timer_pool[HIVE_TIMER_ENTRY_POOL_SIZE];
static bool g_timer_used[HIVE_TIMER_ENTRY_POOL_SIZE];
static hive_pool g_timer_pool_mgr;

// Timer subsystem state
static struct {
    bool initialized;
    timer_entry *timers; // Active timers list (sorted by expiry)
    timer_id next_id;
    volatile uint32_t tick_count; // Current tick count (updated by ISR)
    volatile bool tick_pending;   // Set by ISR, cleared by scheduler
} g_timer = {0};

// Convert microseconds to ticks (rounding up)
static uint32_t us_to_ticks(uint32_t us) {
    return (us + HIVE_TIMER_TICK_US - 1) / HIVE_TIMER_TICK_US;
}

// Called by hardware timer ISR (SysTick or TIMx)
// This function must be called from the timer interrupt handler
void hive_timer_tick_isr(void) {
    g_timer.tick_count++;
    g_timer.tick_pending = true;
}

// Get current tick count
uint32_t hive_timer_get_ticks(void) {
    return g_timer.tick_count;
}

// Process expired timers (called by scheduler in main loop)
void hive_timer_process_pending(void) {
    if (!g_timer.tick_pending) {
        return;
    }
    g_timer.tick_pending = false;

    uint32_t now = g_timer.tick_count;

    // Process all expired timers
    timer_entry **pp = &g_timer.timers;
    while (*pp) {
        timer_entry *entry = *pp;

        // Check if timer expired (handle wrap-around)
        int32_t delta = (int32_t)(entry->expiry_ticks - now);
        if (delta <= 0) {
            // Timer expired - send message to owner
            actor *a = hive_actor_get(entry->owner);
            if (a) {
                hive_ipc_notify_internal(entry->owner, entry->owner,
                                         HIVE_MSG_TIMER, entry->id, NULL, 0);
            }

            if (entry->periodic && a) {
                // Reschedule periodic timer
                entry->expiry_ticks = now + entry->interval_ticks;
                pp = &entry->next;
            } else {
                // Remove one-shot or dead actor's timer
                *pp = entry->next;
                hive_pool_free(&g_timer_pool_mgr, entry);
            }
        } else {
            pp = &entry->next;
        }
    }
}

// Handle timer event from scheduler (compatibility with io_source interface)
void hive_timer_handle_event(io_source *source) {
    (void)source;
    // On STM32, timer processing is done via hive_timer_process_pending()
    // This function exists for API compatibility but shouldn't be called
}

// Initialize timer subsystem
hive_status hive_timer_init(void) {
    HIVE_INIT_GUARD(g_timer.initialized);

    // Initialize timer entry pool
    hive_pool_init(&g_timer_pool_mgr, g_timer_pool, g_timer_used,
                   sizeof(timer_entry), HIVE_TIMER_ENTRY_POOL_SIZE);

    // Initialize timer state
    g_timer.timers = NULL;
    g_timer.next_id = 1;
    g_timer.tick_count = 0;
    g_timer.tick_pending = false;

    // Hardware timer initialization should be done by the application
    // (e.g., configure SysTick to call hive_timer_tick_isr every
    // HIVE_TIMER_TICK_US)

    g_timer.initialized = true;
    return HIVE_SUCCESS;
}

// Cleanup timer subsystem
void hive_timer_cleanup(void) {
    HIVE_CLEANUP_GUARD(g_timer.initialized);

    // Clean up all active timers
    timer_entry *entry = g_timer.timers;
    while (entry) {
        timer_entry *next = entry->next;
        hive_pool_free(&g_timer_pool_mgr, entry);
        entry = next;
    }
    g_timer.timers = NULL;

    g_timer.initialized = false;
}

// Create a timer (one-shot or periodic)
static hive_status create_timer(uint32_t interval_us, bool periodic,
                                timer_id *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(g_timer.initialized, "Timer");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Allocate timer entry from pool
    timer_entry *entry = hive_pool_alloc(&g_timer_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Timer entry pool exhausted");
    }

    // Calculate expiry
    uint32_t ticks = us_to_ticks(interval_us);
    if (ticks == 0)
        ticks = 1; // Minimum 1 tick

    // Initialize timer entry
    entry->id = g_timer.next_id++;
    entry->owner = current->id;
    entry->expiry_ticks = g_timer.tick_count + ticks;
    entry->interval_ticks = periodic ? ticks : 0;
    entry->periodic = periodic;

    // Insert into list (simple append - could optimize with sorted insert)
    entry->next = g_timer.timers;
    g_timer.timers = entry;

    *out = entry->id;
    return HIVE_SUCCESS;
}

hive_status hive_timer_after(uint32_t delay_us, timer_id *out) {
    return create_timer(delay_us, false, out);
}

hive_status hive_timer_every(uint32_t interval_us, timer_id *out) {
    return create_timer(interval_us, true, out);
}

hive_status hive_timer_cancel(timer_id id) {
    HIVE_REQUIRE_INIT(g_timer.initialized, "Timer");

    // Find and remove timer from list
    timer_entry *found = NULL;
    SLIST_FIND_REMOVE(g_timer.timers, entry->id == id, found);

    if (found) {
        hive_pool_free(&g_timer_pool_mgr, found);
        return HIVE_SUCCESS;
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Timer not found");
}

hive_status hive_sleep(uint32_t delay_us) {
    // Create one-shot timer
    timer_id timer;
    hive_status s = hive_timer_after(delay_us, &timer);
    if (HIVE_FAILED(s)) {
        return s;
    }

    // Wait specifically for THIS timer message
    // Other messages remain in mailbox (selective receive)
    hive_message msg;
    return hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg,
                               -1);
}

// Advance simulation time (microseconds) and process expired timers
// On STM32, this directly advances the tick counter (similar to ISR)
void hive_timer_advance_time(uint64_t delta_us) {
    if (!g_timer.initialized) {
        return;
    }

    // Convert microseconds to ticks
    uint32_t ticks = us_to_ticks((uint32_t)delta_us);

    // Advance tick count
    g_timer.tick_count += ticks;
    g_timer.tick_pending = true;

    // Process expired timers immediately
    hive_timer_process_pending();
}

// Get current time in microseconds
// Resolution is limited by HIVE_TIMER_TICK_US (default 1ms).
// For sub-millisecond precision, reduce HIVE_TIMER_TICK_US or use
// platform-specific high-resolution timers (DWT cycle counter).
uint64_t hive_get_time(void) {
    return (uint64_t)g_timer.tick_count * HIVE_TIMER_TICK_US;
}
