#include "rt_scheduler.h"
#include "rt_actor.h"
#include "rt_context.h"
#include "rt_link.h"
#include "rt_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

// External function to get actor table
extern actor_table *rt_actor_get_table(void);

// External functions to process I/O completions
extern void rt_file_process_completions(void);
extern void rt_net_process_completions(void);
extern void rt_timer_process_completions(void);

// Scheduler state
static struct {
    rt_context scheduler_ctx;
    bool       shutdown_requested;
    bool       initialized;
    size_t     last_run_idx[RT_PRIO_COUNT];  // Last run actor index for each priority
} g_scheduler = {0};

rt_status rt_scheduler_init(void) {
    g_scheduler.shutdown_requested = false;
    g_scheduler.initialized = true;
    return RT_SUCCESS;
}

void rt_scheduler_cleanup(void) {
    g_scheduler.initialized = false;
}

// Find next runnable actor (priority-based round-robin)
static actor *find_next_runnable(void) {
    actor_table *table = rt_actor_get_table();
    if (!table || !table->actors) {
        return NULL;
    }

    // Search by priority level
    for (rt_priority prio = RT_PRIO_CRITICAL; prio < RT_PRIO_COUNT; prio++) {
        // Round-robin within priority level - start from after last run actor
        size_t start_idx = (g_scheduler.last_run_idx[prio] + 1) % table->max_actors;

        for (size_t i = 0; i < table->max_actors; i++) {
            size_t idx = (start_idx + i) % table->max_actors;
            actor *a = &table->actors[idx];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                g_scheduler.last_run_idx[prio] = idx;
                RT_LOG_TRACE("Scheduler: Found runnable actor %u (prio=%d)", a->id, prio);
                return a;
            }
        }
    }

    RT_LOG_TRACE("Scheduler: No runnable actors found");
    return NULL;
}

void rt_scheduler_run(void) {
    if (!g_scheduler.initialized) {
        RT_LOG_ERROR("Scheduler not initialized");
        return;
    }

    actor_table *table = rt_actor_get_table();
    if (!table) {
        RT_LOG_ERROR("Actor table not initialized");
        return;
    }

    RT_LOG_INFO("Scheduler started");

    while (!g_scheduler.shutdown_requested && table->num_actors > 0) {
        // Process I/O completions
        rt_file_process_completions();
        rt_net_process_completions();
        rt_timer_process_completions();

        // Find next runnable actor
        actor *next = find_next_runnable();

        if (next) {
            // Switch to actor
            RT_LOG_TRACE("Scheduler: Switching to actor %u", next->id);
            next->state = ACTOR_STATE_RUNNING;
            rt_actor_set_current(next);

            // Context switch to actor
            rt_context_switch(&g_scheduler.scheduler_ctx, &next->ctx);

            // Actor has yielded or exited
            RT_LOG_TRACE("Scheduler: Actor %u yielded, state=%d", next->id, next->state);
            rt_actor_set_current(NULL);

            // If actor is dead, free its resources
            if (next->state == ACTOR_STATE_DEAD) {
                rt_actor_free(next);
            }
            // If actor is still running, mark as ready
            else if (next->state == ACTOR_STATE_RUNNING) {
                next->state = ACTOR_STATE_READY;
            }

        } else {
            // No runnable actors - they may be blocked on I/O
            // Sleep briefly to allow I/O operations to complete
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000}; // 100us
            nanosleep(&ts, NULL);
        }
    }

    RT_LOG_INFO("Scheduler stopped");
}

void rt_scheduler_shutdown(void) {
    g_scheduler.shutdown_requested = true;
}

void rt_scheduler_yield(void) {
    actor *current = rt_actor_current();
    if (!current) {
        RT_LOG_ERROR("yield called outside actor context");
        return;
    }

    // Switch back to scheduler
    rt_context_switch(&current->ctx, &g_scheduler.scheduler_ctx);
}

bool rt_scheduler_should_stop(void) {
    return g_scheduler.shutdown_requested;
}
