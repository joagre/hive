#include "rt_timer.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include "rt_pool.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_log.h"
#include "rt_io_source.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>


// Active timer entry
typedef struct timer_entry {
    timer_id            id;
    actor_id            owner;
    int                 fd;        // timerfd
    bool                periodic;
    struct timer_entry *next;
    io_source           source;    // For epoll registration
} timer_entry;

// Static pool for timer entries
static timer_entry g_timer_pool[RT_TIMER_ENTRY_POOL_SIZE];
static bool g_timer_used[RT_TIMER_ENTRY_POOL_SIZE];
static rt_pool g_timer_pool_mgr;

// Timer subsystem state (simplified - no worker thread!)
static struct {
    bool        initialized;
    timer_entry *timers;      // Active timers list
    timer_id    next_id;
} g_timer = {0};

// Helper: Close timer fd and remove from epoll
static void timer_close_fd(timer_entry *entry) {
    int epoll_fd = rt_scheduler_get_epoll_fd();
    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, entry->fd, NULL);
    close(entry->fd);
}

// Forward declarations for internal functions
rt_status rt_timer_init(void);
void rt_timer_cleanup(void);

// Handle timer event from scheduler (called when timerfd fires)
void rt_timer_handle_event(io_source *source) {
    timer_entry *entry = source->data.timer;

    // Read timerfd to acknowledge
    uint64_t expirations;
    ssize_t n = read(entry->fd, &expirations, sizeof(expirations));
    (void)n;  // Suppress unused result warning

    // Get the actor
    actor *a = rt_actor_get(entry->owner);
    if (!a) {
        // Actor is dead - cleanup timer
        timer_close_fd(entry);
        SLIST_REMOVE(g_timer.timers, entry);
        rt_pool_free(&g_timer_pool_mgr, entry);
        return;
    }

    // Send timer tick message to actor
    // Use RT_MSG_TIMER class with timer_id as tag, sender is the owning actor
    // No payload needed - timer_id is encoded in the tag
    rt_status status = rt_ipc_cast_ex(entry->owner, entry->owner, RT_MSG_TIMER,
                                       entry->id, NULL, 0);
    if (RT_FAILED(status)) {
        RT_LOG_ERROR("Failed to send timer tick: %s", status.msg);
        // Don't cleanup timer - try again next tick
        return;
    }

    // If one-shot, cleanup
    if (!entry->periodic) {
        timer_close_fd(entry);
        SLIST_REMOVE(g_timer.timers, entry);
        rt_pool_free(&g_timer_pool_mgr, entry);
    }
}

// Initialize timer subsystem
rt_status rt_timer_init(void) {
    RT_INIT_GUARD(g_timer.initialized);

    // Initialize timer entry pool
    rt_pool_init(&g_timer_pool_mgr, g_timer_pool, g_timer_used,
                 sizeof(timer_entry), RT_TIMER_ENTRY_POOL_SIZE);

    // Initialize timer state
    g_timer.timers = NULL;
    g_timer.next_id = 1;

    g_timer.initialized = true;
    return RT_SUCCESS;
}

// Cleanup timer subsystem
void rt_timer_cleanup(void) {
    RT_CLEANUP_GUARD(g_timer.initialized);

    // Clean up all active timers
    timer_entry *entry = g_timer.timers;
    while (entry) {
        timer_entry *next = entry->next;
        timer_close_fd(entry);
        rt_pool_free(&g_timer_pool_mgr, entry);
        entry = next;
    }
    g_timer.timers = NULL;

    g_timer.initialized = false;
}

// Create a timer (one-shot or periodic)
static rt_status create_timer(uint32_t interval_us, bool periodic, timer_id *out) {
    if (!out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_timer.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Timer subsystem not initialized");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Create timerfd
    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (tfd < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Set timer
    // Note: timerfd treats (0, 0) as "disarm timer", so we use minimum 1ns for zero delay
    struct itimerspec its;
    if (interval_us == 0) {
        its.it_value.tv_sec = 0;
        its.it_value.tv_nsec = 1;  // Minimum 1 nanosecond to avoid disarming
    } else {
        its.it_value.tv_sec = interval_us / RT_USEC_PER_SEC;
        its.it_value.tv_nsec = (interval_us % RT_USEC_PER_SEC) * 1000;
    }

    if (periodic) {
        // Periodic - set interval
        its.it_interval.tv_sec = its.it_value.tv_sec;
        its.it_interval.tv_nsec = its.it_value.tv_nsec;
    } else {
        // One-shot - no interval
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = 0;
    }

    if (timerfd_settime(tfd, 0, &its, NULL) < 0) {
        close(tfd);
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Allocate timer entry from pool
    timer_entry *entry = rt_pool_alloc(&g_timer_pool_mgr);
    if (!entry) {
        close(tfd);
        return RT_ERROR(RT_ERR_NOMEM, "Timer entry pool exhausted");
    }

    // Initialize timer entry
    entry->id = g_timer.next_id++;
    entry->owner = current->id;
    entry->fd = tfd;
    entry->periodic = periodic;
    entry->next = g_timer.timers;
    g_timer.timers = entry;

    // Setup io_source for epoll
    entry->source.type = IO_SOURCE_TIMER;
    entry->source.data.timer = entry;

    // Add to scheduler's epoll
    int epoll_fd = rt_scheduler_get_epoll_fd();
    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.ptr = &entry->source;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, tfd, &ev) < 0) {
        SLIST_REMOVE(g_timer.timers, entry);
        close(tfd);
        rt_pool_free(&g_timer_pool_mgr, entry);
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Done! No worker thread, no queue, no blocking!
    *out = entry->id;
    return RT_SUCCESS;
}

rt_status rt_timer_after(uint32_t delay_us, timer_id *out) {
    return create_timer(delay_us, false, out);
}

rt_status rt_timer_every(uint32_t interval_us, timer_id *out) {
    return create_timer(interval_us, true, out);
}

rt_status rt_timer_cancel(timer_id id) {
    if (!g_timer.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Timer subsystem not initialized");
    }

    // Find and remove timer from list
    timer_entry *found = NULL;
    SLIST_FIND_REMOVE(g_timer.timers, entry->id == id, found);

    if (found) {
        timer_close_fd(found);
        rt_pool_free(&g_timer_pool_mgr, found);
        return RT_SUCCESS;
    }

    return RT_ERROR(RT_ERR_INVALID, "Timer not found");
}
