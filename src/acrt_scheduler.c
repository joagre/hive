#include "acrt_scheduler.h"
#include "acrt_static_config.h"
#include "acrt_actor.h"
#include "acrt_context.h"
#include "acrt_link.h"
#include "acrt_log.h"
#include "acrt_internal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/epoll.h>
#include <unistd.h>

// External function to get actor table
extern actor_table *acrt_actor_get_table(void);

// Stack overflow detection (must match acrt_actor.c)
#define STACK_GUARD_PATTERN 0xDEADBEEFCAFEBABEULL
#define STACK_GUARD_SIZE 8

// Scheduler state
static struct {
    acrt_context scheduler_ctx;
    bool       shutdown_requested;
    bool       initialized;
    size_t     last_run_idx[ACRT_PRIORITY_COUNT];  // Last run actor index for each priority
    int        epoll_fd;                      // Event loop file descriptor
} g_scheduler = {0};

// Check stack guard patterns for overflow detection
static bool check_stack_guard(actor *a) {
    if (!a || !a->stack) {
        return true;  // No stack to check
    }

    uint64_t *guard_low = (uint64_t *)a->stack;
    uint64_t *guard_high = (uint64_t *)((uint8_t *)a->stack + a->stack_size - STACK_GUARD_SIZE);

    return (*guard_low == STACK_GUARD_PATTERN && *guard_high == STACK_GUARD_PATTERN);
}

acrt_status acrt_scheduler_init(void) {
    g_scheduler.shutdown_requested = false;
    g_scheduler.initialized = true;

    // Create epoll instance for event loop
    g_scheduler.epoll_fd = epoll_create1(0);
    if (g_scheduler.epoll_fd < 0) {
        g_scheduler.initialized = false;
        return ACRT_ERROR(ACRT_ERR_IO, "Failed to create epoll");
    }

    return ACRT_SUCCESS;
}

void acrt_scheduler_cleanup(void) {
    if (g_scheduler.epoll_fd >= 0) {
        close(g_scheduler.epoll_fd);
        g_scheduler.epoll_fd = -1;
    }
    g_scheduler.initialized = false;
}

// Find next runnable actor (priority-based round-robin)
static actor *find_next_runnable(void) {
    actor_table *table = acrt_actor_get_table();
    if (!table || !table->actors) {
        return NULL;
    }

    // Search by priority level
    for (acrt_priority_level prio = ACRT_PRIORITY_CRITICAL; prio < ACRT_PRIORITY_COUNT; prio++) {
        // Round-robin within priority level - start from after last run actor
        size_t start_idx = (g_scheduler.last_run_idx[prio] + 1) % table->max_actors;

        for (size_t i = 0; i < table->max_actors; i++) {
            size_t idx = (start_idx + i) % table->max_actors;
            actor *a = &table->actors[idx];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                g_scheduler.last_run_idx[prio] = idx;
                ACRT_LOG_TRACE("Scheduler: Found runnable actor %u (prio=%d)", a->id, prio);
                return a;
            }
        }
    }

    ACRT_LOG_TRACE("Scheduler: No runnable actors found");
    return NULL;
}

void acrt_scheduler_run(void) {
    if (!g_scheduler.initialized) {
        ACRT_LOG_ERROR("Scheduler not initialized");
        return;
    }

    actor_table *table = acrt_actor_get_table();
    if (!table) {
        ACRT_LOG_ERROR("Actor table not initialized");
        return;
    }

    ACRT_LOG_INFO("Scheduler started");

    while (!g_scheduler.shutdown_requested && table->num_actors > 0) {
        // Find next runnable actor
        actor *next = find_next_runnable();

        if (next) {
            // Switch to actor
            ACRT_LOG_TRACE("Scheduler: Switching to actor %u", next->id);
            next->state = ACTOR_STATE_RUNNING;
            acrt_actor_set_current(next);

            // Context switch to actor
            acrt_context_switch(&g_scheduler.scheduler_ctx, &next->ctx);

            // Check for stack overflow
            if (!check_stack_guard(next)) {
                ACRT_LOG_ERROR("Actor %u stack overflow detected", next->id);
                next->exit_reason = ACRT_EXIT_CRASH_STACK;
                next->state = ACTOR_STATE_DEAD;
            }

            // Actor has yielded or exited
            ACRT_LOG_TRACE("Scheduler: Actor %u yielded, state=%d", next->id, next->state);
            acrt_actor_set_current(NULL);

            // If actor is dead, free its resources
            if (next->state == ACTOR_STATE_DEAD) {
                acrt_actor_free(next);
            }
            // If actor is still running, mark as ready
            else if (next->state == ACTOR_STATE_RUNNING) {
                next->state = ACTOR_STATE_READY;
            }

        } else {
            // No runnable actors - wait for I/O events
            struct epoll_event events[64];
            // Short timeout to allow checking for actors made ready by IPC/bus/link
            // (those don't use epoll, they directly set actor state to READY)
            int timeout_ms = 10;
            int n = epoll_wait(g_scheduler.epoll_fd, events, 64, timeout_ms);

            // Dispatch epoll events
            for (int i = 0; i < n; i++) {
                io_source *source = events[i].data.ptr;

                if (source->type == IO_SOURCE_TIMER) {
                    acrt_timer_handle_event(source);
                } else if (source->type == IO_SOURCE_NETWORK) {
                    acrt_net_handle_event(source);
                }
                // Future: IO_SOURCE_WAKEUP
            }
        }
    }

    ACRT_LOG_INFO("Scheduler stopped");
}

void acrt_scheduler_shutdown(void) {
    g_scheduler.shutdown_requested = true;
}

void acrt_scheduler_yield(void) {
    actor *current = acrt_actor_current();
    if (!current) {
        ACRT_LOG_ERROR("yield called outside actor context");
        return;
    }

    // Switch back to scheduler
    acrt_context_switch(&current->ctx, &g_scheduler.scheduler_ctx);
}

bool acrt_scheduler_should_stop(void) {
    return g_scheduler.shutdown_requested;
}

int acrt_scheduler_get_epoll_fd(void) {
    return g_scheduler.epoll_fd;
}
