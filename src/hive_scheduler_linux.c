#include "hive_scheduler.h"
#include "hive_static_config.h"
#include "hive_actor.h"
#include "hive_context.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_internal.h"
#include <stdbool.h>
#include <stdint.h>
#include <sys/epoll.h>
#include <unistd.h>

// External function to get actor table
extern actor_table *hive_actor_get_table(void);

// Scheduler state
static struct {
    hive_context scheduler_ctx;
    bool shutdown_requested;
    bool initialized;
    size_t last_run_idx[HIVE_PRIORITY_COUNT]; // Last run actor index for each
                                              // priority
    int epoll_fd;                             // Event loop file descriptor
} g_scheduler = {0};

// Dispatch pending epoll events (timeout_ms: -1=block, 0=poll, >0=wait)
static void dispatch_epoll_events(int timeout_ms) {
    struct epoll_event events[HIVE_EPOLL_MAX_EVENTS];
    int n = epoll_wait(g_scheduler.epoll_fd, events, HIVE_EPOLL_MAX_EVENTS,
                       timeout_ms);

    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;

        if (source->type == IO_SOURCE_TIMER) {
            hive_timer_handle_event(source);
        }
#if HIVE_ENABLE_NET
        else if (source->type == IO_SOURCE_NETWORK) {
            hive_net_handle_event(source);
        }
#endif
    }
}

// Run a single actor: context switch, check stack, handle exit/yield
static void run_single_actor(actor *a) {
    HIVE_LOG_TRACE("Scheduler: Running actor %u (prio=%d)", a->id, a->priority);
    a->state = ACTOR_STATE_RUNNING;
    hive_actor_set_current(a);

    // Context switch to actor
    hive_context_switch(&g_scheduler.scheduler_ctx, &a->ctx);

    // Actor has yielded or exited
    HIVE_LOG_TRACE("Scheduler: Actor %u yielded, state=%d", a->id, a->state);
    hive_actor_set_current(NULL);

    // If actor is dead, free its resources
    if (a->state == ACTOR_STATE_DEAD) {
        hive_actor_free(a);
    }
    // If actor is still running (yielded), mark as ready
    else if (a->state == ACTOR_STATE_RUNNING) {
        a->state = ACTOR_STATE_READY;
    }
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
    for (hive_priority_level prio = HIVE_PRIORITY_CRITICAL;
         prio < HIVE_PRIORITY_COUNT; prio++) {
        // Round-robin within priority level - start from after last run actor
        size_t start_idx =
            (g_scheduler.last_run_idx[prio] + 1) % table->max_actors;

        for (size_t i = 0; i < table->max_actors; i++) {
            size_t idx = (start_idx + i) % table->max_actors;
            actor *a = &table->actors[idx];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                g_scheduler.last_run_idx[prio] = idx;
                HIVE_LOG_TRACE("Scheduler: Found runnable actor %u (prio=%d)",
                               a->id, prio);
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
        actor *next = find_next_runnable();

        if (next) {
            run_single_actor(next);
        } else {
            // No runnable actors - wait for I/O events with short timeout
            // (IPC/bus/link don't use epoll, they directly set actor state)
            dispatch_epoll_events(HIVE_EPOLL_POLL_TIMEOUT_MS);
        }
    }

    HIVE_LOG_INFO("Scheduler stopped");
}

hive_status hive_scheduler_run_until_blocked(void) {
    if (!g_scheduler.initialized) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Scheduler not initialized");
    }

    actor_table *table = hive_actor_get_table();
    if (!table) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Actor table not initialized");
    }

    // Run actors until all are blocked (WAITING) or dead
    while (!g_scheduler.shutdown_requested && table->num_actors > 0) {
        // Poll for I/O events (non-blocking) - handles timerfd events in
        // real-time mode
        dispatch_epoll_events(0);

        // Find next ready actor
        actor *next = find_next_runnable();
        if (!next) {
            // No ready actors - all are blocked or dead
            break;
        }

        run_single_actor(next);
    }

    return HIVE_SUCCESS;
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
