// Unified Scheduler Implementation
//
// Platform-independent scheduler using HAL event interface.
// All platform-specific event handling (epoll, WFI) is in the HAL layer.

#include "hive_scheduler.h"
#include "hive_static_config.h"
#include "hive_actor.h"
#include "hive_context.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_internal.h"
#include "hal/hive_hal_event.h"
#include "hal/hive_hal_time.h"
#include <stdbool.h>
#include <stdint.h>

// Rate limit for pool wait warnings (1 second)
#define POOL_WAIT_WARN_INTERVAL_US 1000000

// External function to get actor_t table
extern actor_table_t *hive_actor_get_table(void);

// Pool wait queue entry
typedef struct {
    actor_id_t actor;
    hive_priority_level_t priority;
    bool used;
} pool_wait_entry_t;

// Scheduler state
static struct {
    hive_context_t scheduler_ctx;
    bool shutdown_requested;
    bool initialized;
    size_t last_run_idx[HIVE_PRIORITY_COUNT]; // Round-robin index per priority

    // Pool wait queue (for blocking on pool exhaustion)
    pool_wait_entry_t pool_wait_queue[HIVE_MAX_ACTORS];
    size_t pool_wait_count;
    uint64_t pool_wait_last_warn_us; // For rate-limited warnings
} s_scheduler = {0};

// Run a single actor_t: context switch, check stack, handle exit/yield
static void run_single_actor(actor_t *a) {
    HIVE_LOG_TRACE("Scheduler: Running actor_t %u (prio=%d)", a->id,
                   a->priority);
    a->state = ACTOR_STATE_RUNNING;
    hive_actor_set_current(a);

    // Context switch to actor_t
    hive_context_switch(&s_scheduler.scheduler_ctx, &a->ctx);

    // Actor has yielded or exited
    HIVE_LOG_TRACE("Scheduler: Actor %u yielded, state=%d", a->id, a->state);
    hive_actor_set_current(NULL);

    // If actor_t is dead, free its resources
    if (a->state == ACTOR_STATE_DEAD) {
        hive_actor_free(a);
    }
    // If actor_t is still running (yielded), mark as ready
    else if (a->state == ACTOR_STATE_RUNNING) {
        a->state = ACTOR_STATE_READY;
    }
}

// Find next runnable actor_t (priority-based round-robin)
static actor_t *find_next_runnable(void) {
    actor_table_t *table = hive_actor_get_table();
    if (!table || !table->actors) {
        return NULL;
    }

    // Search by priority level (CRITICAL first, LOW last)
    for (hive_priority_level_t prio = HIVE_PRIORITY_CRITICAL;
         prio < HIVE_PRIORITY_COUNT; prio++) {
        // Round-robin within priority level - start from after last run actor_t
        size_t start_idx =
            (s_scheduler.last_run_idx[prio] + 1) % table->max_actors;

        for (size_t i = 0; i < table->max_actors; i++) {
            size_t idx = (start_idx + i) % table->max_actors;
            actor_t *a = &table->actors[idx];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                s_scheduler.last_run_idx[prio] = idx;
                HIVE_LOG_TRACE("Scheduler: Found runnable actor_t %u (prio=%d)",
                               a->id, prio);
                return a;
            }
        }
    }

    HIVE_LOG_TRACE("Scheduler: No runnable actors found");
    return NULL;
}

hive_status_t hive_scheduler_init(void) {
    s_scheduler.shutdown_requested = false;

    // Initialize round-robin indices
    for (int i = 0; i < HIVE_PRIORITY_COUNT; i++) {
        s_scheduler.last_run_idx[i] = 0;
    }

    // Initialize HAL event system
    hive_status_t status = hive_hal_event_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    s_scheduler.initialized = true;
    return HIVE_SUCCESS;
}

void hive_scheduler_cleanup(void) {
    hive_hal_event_cleanup();
    s_scheduler.initialized = false;
}

void hive_scheduler_run(void) {
    if (!s_scheduler.initialized) {
        HIVE_LOG_ERROR("Scheduler not initialized");
        return;
    }

    actor_table_t *table = hive_actor_get_table();
    if (!table) {
        HIVE_LOG_ERROR("Actor table not initialized");
        return;
    }

    HIVE_LOG_INFO("Scheduler started");

    while (!s_scheduler.shutdown_requested && table->num_actors > 0) {
        // Poll for pending events (non-blocking)
        hive_hal_event_poll();

        // Find next runnable actor_t
        actor_t *next = find_next_runnable();

        if (next) {
            run_single_actor(next);
        } else {
            // No runnable actors - wait for I/O events
            hive_hal_event_wait(HIVE_EPOLL_POLL_TIMEOUT_MS);
        }
    }

    HIVE_LOG_INFO("Scheduler stopped");
}

hive_status_t hive_scheduler_run_until_blocked(void) {
    if (!s_scheduler.initialized) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Scheduler not initialized");
    }

    actor_table_t *table = hive_actor_get_table();
    if (!table) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Actor table not initialized");
    }

    // Run actors until all are blocked (WAITING) or dead
    while (!s_scheduler.shutdown_requested && table->num_actors > 0) {
        // Poll for I/O events (non-blocking)
        hive_hal_event_poll();

        // Find next ready actor_t
        actor_t *next = find_next_runnable();
        if (!next) {
            // No ready actors - all are blocked or dead
            break;
        }

        run_single_actor(next);
    }

    return HIVE_SUCCESS;
}

void hive_scheduler_shutdown(void) {
    s_scheduler.shutdown_requested = true;
}

void hive_scheduler_yield(void) {
    actor_t *current = hive_actor_current();
    if (!current) {
        HIVE_LOG_ERROR("yield called outside actor_t context");
        return;
    }

    // Switch back to scheduler
    hive_context_switch(&current->ctx, &s_scheduler.scheduler_ctx);
}

bool hive_scheduler_should_stop(void) {
    return s_scheduler.shutdown_requested;
}

// =============================================================================
// Pool Wait Queue Implementation
// =============================================================================

void hive_scheduler_pool_wait(void) {
    actor_t *current = hive_actor_current();
    if (!current) {
        HIVE_LOG_ERROR("pool_wait called outside actor context");
        return;
    }

    // Rate-limited warning
    uint64_t now = hive_hal_get_time_us();
    if (now - s_scheduler.pool_wait_last_warn_us >=
        POOL_WAIT_WARN_INTERVAL_US) {
        HIVE_LOG_WARN("Actor %u (%s) blocking on pool exhaustion",
                      (unsigned)current->id,
                      current->name ? current->name : "unnamed");
        s_scheduler.pool_wait_last_warn_us = now;
    }

    // Find empty slot in wait queue (FIFO within priority)
    for (size_t i = 0; i < HIVE_MAX_ACTORS; i++) {
        if (!s_scheduler.pool_wait_queue[i].used) {
            s_scheduler.pool_wait_queue[i].actor = current->id;
            s_scheduler.pool_wait_queue[i].priority = current->priority;
            s_scheduler.pool_wait_queue[i].used = true;
            s_scheduler.pool_wait_count++;

            HIVE_LOG_TRACE("Actor %u added to pool wait queue (count=%zu)",
                           current->id, s_scheduler.pool_wait_count);

            // Mark as waiting and yield
            current->state = ACTOR_STATE_WAITING;
            hive_scheduler_yield();
            return;
        }
    }

    // This should never happen (queue bounded by HIVE_MAX_ACTORS)
    HIVE_LOG_ERROR("Pool wait queue full - this should never happen");
}

void hive_scheduler_pool_wake_one(void) {
    if (s_scheduler.pool_wait_count == 0) {
        return;
    }

    // Find highest priority waiter (FIFO within same priority)
    size_t best_idx = SIZE_MAX;
    hive_priority_level_t best_prio = HIVE_PRIORITY_COUNT;

    for (size_t i = 0; i < HIVE_MAX_ACTORS; i++) {
        if (s_scheduler.pool_wait_queue[i].used) {
            hive_priority_level_t prio =
                s_scheduler.pool_wait_queue[i].priority;
            if (prio < best_prio) {
                best_prio = prio;
                best_idx = i;
            }
        }
    }

    if (best_idx == SIZE_MAX) {
        return; // No waiters found (shouldn't happen if count > 0)
    }

    // Wake the waiter
    actor_id_t actor_id = s_scheduler.pool_wait_queue[best_idx].actor;
    s_scheduler.pool_wait_queue[best_idx].used = false;
    s_scheduler.pool_wait_count--;

    actor_t *a = hive_actor_get(actor_id);
    if (a && a->state == ACTOR_STATE_WAITING) {
        HIVE_LOG_TRACE("Waking actor %u from pool wait queue (count=%zu)",
                       actor_id, s_scheduler.pool_wait_count);
        a->state = ACTOR_STATE_READY;
    }
}

void hive_scheduler_pool_wait_remove(actor_id_t id) {
    for (size_t i = 0; i < HIVE_MAX_ACTORS; i++) {
        if (s_scheduler.pool_wait_queue[i].used &&
            s_scheduler.pool_wait_queue[i].actor == id) {
            s_scheduler.pool_wait_queue[i].used = false;
            s_scheduler.pool_wait_count--;
            HIVE_LOG_TRACE("Removed actor %u from pool wait queue (count=%zu)",
                           id, s_scheduler.pool_wait_count);
            return;
        }
    }
}
