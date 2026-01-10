#include "hive_timer.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include "hive_io_source.h"
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
static timer_entry g_timer_pool[HIVE_TIMER_ENTRY_POOL_SIZE];
static bool g_timer_used[HIVE_TIMER_ENTRY_POOL_SIZE];
static hive_pool g_timer_pool_mgr;

// Timer subsystem state
static struct {
    bool        initialized;
    timer_entry *timers;      // Active timers list
    timer_id    next_id;
} g_timer = {0};

// Helper: Close timer fd and remove from epoll
static void timer_close_fd(timer_entry *entry) {
    int epoll_fd = hive_scheduler_get_epoll_fd();
    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, entry->fd, NULL);
    close(entry->fd);
}

// Handle timer event from scheduler (called when timerfd fires)
void hive_timer_handle_event(io_source *source) {
    timer_entry *entry = source->data.timer;

    // Read timerfd to acknowledge
    uint64_t expirations;
    ssize_t n = read(entry->fd, &expirations, sizeof(expirations));
    (void)n;  // Suppress unused result warning

    // Get the actor
    actor *a = hive_actor_get(entry->owner);
    if (!a) {
        // Actor is dead - cleanup timer
        timer_close_fd(entry);
        SLIST_REMOVE(g_timer.timers, entry);
        hive_pool_free(&g_timer_pool_mgr, entry);
        return;
    }

    // Send timer tick message to actor
    // Use HIVE_MSG_TIMER class with timer_id as tag, sender is the owning actor
    // No payload needed - timer_id is encoded in the tag
    hive_status status = hive_ipc_notify_internal(entry->owner, entry->owner, HIVE_MSG_TIMER,
                                                entry->id, NULL, 0);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to send timer tick: %s", status.msg);
        // Don't cleanup timer - try again next tick
        return;
    }

    // If one-shot, cleanup
    if (!entry->periodic) {
        timer_close_fd(entry);
        SLIST_REMOVE(g_timer.timers, entry);
        hive_pool_free(&g_timer_pool_mgr, entry);
    }
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
        timer_close_fd(entry);
        hive_pool_free(&g_timer_pool_mgr, entry);
        entry = next;
    }
    g_timer.timers = NULL;

    g_timer.initialized = false;
}

// Create a timer (one-shot or periodic)
static hive_status create_timer(uint32_t interval_us, bool periodic, timer_id *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    if (!g_timer.initialized) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Timer subsystem not initialized");
    }

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Create timerfd
    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if (tfd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, strerror(errno));
    }

    // Set timer
    // Note: timerfd treats (0, 0) as "disarm timer", so we use minimum 1ns for zero delay
    struct itimerspec its;
    if (interval_us == 0) {
        its.it_value.tv_sec = 0;
        its.it_value.tv_nsec = 1;  // Minimum 1 nanosecond to avoid disarming
    } else {
        its.it_value.tv_sec = interval_us / HIVE_USEC_PER_SEC;
        its.it_value.tv_nsec = (interval_us % HIVE_USEC_PER_SEC) * 1000;
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
        return HIVE_ERROR(HIVE_ERR_IO, strerror(errno));
    }

    // Allocate timer entry from pool
    timer_entry *entry = hive_pool_alloc(&g_timer_pool_mgr);
    if (!entry) {
        close(tfd);
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Timer entry pool exhausted");
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
    int epoll_fd = hive_scheduler_get_epoll_fd();
    struct epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.ptr = &entry->source;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, tfd, &ev) < 0) {
        SLIST_REMOVE(g_timer.timers, entry);
        close(tfd);
        hive_pool_free(&g_timer_pool_mgr, entry);
        return HIVE_ERROR(HIVE_ERR_IO, strerror(errno));
    }

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
    if (!g_timer.initialized) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Timer subsystem not initialized");
    }

    // Find and remove timer from list
    timer_entry *found = NULL;
    SLIST_FIND_REMOVE(g_timer.timers, entry->id == id, found);

    if (found) {
        timer_close_fd(found);
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
    hive_msg_class class = HIVE_MSG_TIMER;
    uint32_t tag = timer;
    hive_message msg;

    return hive_ipc_recv_match(NULL, &class, &tag, &msg, -1);
}
