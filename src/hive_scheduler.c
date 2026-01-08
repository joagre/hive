#include "hive_scheduler.h"
#include "hive_static_config.h"
#include "hive_actor.h"
#include "hive_context.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_internal.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include <sys/epoll.h>
#include <unistd.h>

// External function to get actor table
extern actor_table *hive_actor_get_table(void);

// Stack overflow detection (must match hive_actor.c)
#define STACK_GUARD_PATTERN 0xDEADBEEFCAFEBABEULL
#define STACK_GUARD_SIZE 8

// Scheduler state
static struct {
    hive_context scheduler_ctx;
    bool       shutdown_requested;
    bool       initialized;
    size_t     last_run_idx[HIVE_PRIORITY_COUNT];  // Last run actor index for each priority
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

hive_status hive_scheduler_init(void) {
    g_scheduler.shutdown_requested = false;
    g_scheduler.initialized = true;

    // Create epoll instance for event loop
    g_scheduler.epoll_fd = epoll_create1(0);
    if (g_scheduler.epoll_fd < 0) {
        g_scheduler.initialized = false;
        return HIVE_ERROR(HIVE_ERR_IO, "Failed to create epoll");
    }

    return HIVE_SUCCESS;
}

void hive_scheduler_cleanup(void) {
    if (g_scheduler.epoll_fd >= 0) {
        close(g_scheduler.epoll_fd);
        g_scheduler.epoll_fd = -1;
    }
    g_scheduler.initialized = false;
}

// Find next runnable actor (priority-based round-robin)
static actor *find_next_runnable(void) {
    actor_table *table = hive_actor_get_table();
    if (!table || !table->actors) {
        return NULL;
    }

    // Search by priority level
    for (hive_priority_level prio = HIVE_PRIORITY_CRITICAL; prio < HIVE_PRIORITY_COUNT; prio++) {
        // Round-robin within priority level - start from after last run actor
        size_t start_idx = (g_scheduler.last_run_idx[prio] + 1) % table->max_actors;

        for (size_t i = 0; i < table->max_actors; i++) {
            size_t idx = (start_idx + i) % table->max_actors;
            actor *a = &table->actors[idx];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                g_scheduler.last_run_idx[prio] = idx;
                HIVE_LOG_TRACE("Scheduler: Found runnable actor %u (prio=%d)", a->id, prio);
                return a;
            }
        }
    }

    HIVE_LOG_TRACE("Scheduler: No runnable actors found");
    return NULL;
}

void hive_scheduler_run(void) {
    if (!g_scheduler.initialized) {
        HIVE_LOG_ERROR("Scheduler not initialized");
        return;
    }

    actor_table *table = hive_actor_get_table();
    if (!table) {
        HIVE_LOG_ERROR("Actor table not initialized");
        return;
    }

    HIVE_LOG_INFO("Scheduler started");

    while (!g_scheduler.shutdown_requested && table->num_actors > 0) {
        // Find next runnable actor
        actor *next = find_next_runnable();

        if (next) {
            // Switch to actor
            HIVE_LOG_TRACE("Scheduler: Switching to actor %u", next->id);
            next->state = ACTOR_STATE_RUNNING;
            hive_actor_set_current(next);

            // Context switch to actor
            hive_context_switch(&g_scheduler.scheduler_ctx, &next->ctx);

            // Check for stack overflow
            if (!check_stack_guard(next)) {
                HIVE_LOG_ERROR("Actor %u stack overflow detected", next->id);
                next->exit_reason = HIVE_EXIT_CRASH_STACK;
                next->state = ACTOR_STATE_DEAD;
            }

            // Actor has yielded or exited
            HIVE_LOG_TRACE("Scheduler: Actor %u yielded, state=%d", next->id, next->state);
            hive_actor_set_current(NULL);

            // If actor is dead, free its resources
            if (next->state == ACTOR_STATE_DEAD) {
                hive_actor_free(next);
            }
            // If actor is still running, mark as ready
            else if (next->state == ACTOR_STATE_RUNNING) {
                next->state = ACTOR_STATE_READY;
            }

        } else {
            // No runnable actors - wait for I/O events
            struct epoll_event events[HIVE_EPOLL_MAX_EVENTS];
            // Short timeout to allow checking for actors made ready by IPC/bus/link
            // (those don't use epoll, they directly set actor state to READY)
            int n = epoll_wait(g_scheduler.epoll_fd, events,
                               HIVE_EPOLL_MAX_EVENTS, HIVE_EPOLL_POLL_TIMEOUT_MS);

            // Dispatch epoll events
            for (int i = 0; i < n; i++) {
                io_source *source = events[i].data.ptr;

                if (source->type == IO_SOURCE_TIMER) {
                    hive_timer_handle_event(source);
                } else if (source->type == IO_SOURCE_NETWORK) {
                    hive_net_handle_event(source);
                }
                // Future: IO_SOURCE_WAKEUP
            }
        }
    }

    HIVE_LOG_INFO("Scheduler stopped");
}

void hive_scheduler_shutdown(void) {
    g_scheduler.shutdown_requested = true;
}

void hive_scheduler_yield(void) {
    actor *current = hive_actor_current();
    if (!current) {
        HIVE_LOG_ERROR("yield called outside actor context");
        return;
    }

    // Switch back to scheduler
    hive_context_switch(&current->ctx, &g_scheduler.scheduler_ctx);
}

bool hive_scheduler_should_stop(void) {
    return g_scheduler.shutdown_requested;
}

int hive_scheduler_get_epoll_fd(void) {
    return g_scheduler.epoll_fd;
}
