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

// Dispatch pending epoll events (timeout_ms: -1=block, 0=poll, >0=wait)
static void dispatch_epoll_events(int timeout_ms) {
    struct epoll_event events[HIVE_EPOLL_MAX_EVENTS];
    int n = epoll_wait(g_scheduler.epoll_fd, events, HIVE_EPOLL_MAX_EVENTS, timeout_ms);

    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;

        if (source->type == IO_SOURCE_TIMER) {
            hive_timer_handle_event(source);
        } else if (source->type == IO_SOURCE_NETWORK) {
            hive_net_handle_event(source);
        }
    }
}

// Run a single actor: context switch, check stack, handle exit/yield
static void run_single_actor(actor *a) {
    HIVE_LOG_TRACE("Scheduler: Running actor %u (prio=%d)", a->id, a->priority);
    a->state = ACTOR_STATE_RUNNING;
    hive_actor_set_current(a);

    // Context switch to actor
    hive_context_switch(&g_scheduler.scheduler_ctx, &a->ctx);

    // Check for stack overflow
    if (!check_stack_guard(a)) {
        HIVE_LOG_ERROR("Actor %u stack overflow detected", a->id);
        a->exit_reason = HIVE_EXIT_CRASH_STACK;
        a->state = ACTOR_STATE_DEAD;
    }

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

hive_status hive_scheduler_step(void) {
    if (!g_scheduler.initialized) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Scheduler not initialized");
    }

    actor_table *table = hive_actor_get_table();
    if (!table) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Actor table not initialized");
    }

    // Poll for I/O events (non-blocking)
    dispatch_epoll_events(0);

    // Run each READY actor exactly once (priority order)
    bool ran_any = false;

    for (hive_priority_level prio = HIVE_PRIORITY_CRITICAL; prio < HIVE_PRIORITY_COUNT; prio++) {
        for (size_t i = 0; i < table->max_actors; i++) {
            actor *a = &table->actors[i];

            if (a->state == ACTOR_STATE_READY && a->priority == prio) {
                run_single_actor(a);
                ran_any = true;
            }
        }
    }

    return ran_any ? HIVE_SUCCESS : HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No actors ready");
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
