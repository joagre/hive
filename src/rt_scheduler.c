#include "rt_scheduler.h"
#include "rt_actor.h"
#include "rt_context.h"
#include "rt_log.h"
#include <stdio.h>
#include <stdbool.h>
#include <time.h>

// External function to get actor table
extern actor_table *rt_actor_get_table(void);

// External functions to process I/O completions
extern void rt_file_process_completions(void);
extern void rt_net_process_completions(void);

// Scheduler state
static struct {
    rt_context scheduler_ctx;
    bool       shutdown_requested;
    bool       initialized;
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
        // Round-robin within priority level
        for (size_t i = 0; i < table->max_actors; i++) {
            actor *a = &table->actors[i];
            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                return a;
            }
        }
    }

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

        // Find next runnable actor
        actor *next = find_next_runnable();

        if (next) {
            // Switch to actor
            next->state = ACTOR_STATE_RUNNING;
            rt_actor_set_current(next);

            // Context switch to actor
            rt_context_switch(&g_scheduler.scheduler_ctx, &next->ctx);

            // Actor has yielded or exited
            rt_actor_set_current(NULL);

            // If actor is still running, mark as ready
            if (next->state == ACTOR_STATE_RUNNING) {
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
