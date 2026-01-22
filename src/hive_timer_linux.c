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
#include "hal/hive_hal_event.h"
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>

// Active timer entry
typedef struct timer_entry {
    timer_id id;
    actor_id owner;
    int fd; // timerfd (only used in real-time mode)
    bool periodic;
    uint64_t expiry_us;   // Expiry time in microseconds (simulation mode)
    uint64_t interval_us; // Interval for periodic timers (simulation mode)
    struct timer_entry *next;
    io_source source; // For epoll registration
} timer_entry;

// Static pool for timer entries
static timer_entry s_timer_pool[HIVE_TIMER_ENTRY_POOL_SIZE];
static bool s_timer_used[HIVE_TIMER_ENTRY_POOL_SIZE];
static hive_pool s_timer_pool_mgr;

// Timer subsystem state
static struct {
    bool initialized;
    timer_entry *timers; // Active timers list
    timer_id next_id;
    bool sim_mode;        // Simulation time mode (enabled by hive_advance_time)
    uint64_t sim_time_us; // Current simulation time in microseconds
} s_timer = {0};

// Helper: Close timer fd and remove from event system (only in real-time mode)
static void timer_close_fd(timer_entry *entry) {
    if (entry->fd >= 0) {
        hive_hal_event_unregister(entry->fd);
        close(entry->fd);
        entry->fd = -1;
    }
}

// Handle timer event from scheduler (called when timerfd fires)
void hive_timer_handle_event(io_source *source) {
    timer_entry *entry = source->data.timer;

    // Read timerfd to acknowledge
    uint64_t expirations;
    ssize_t n = read(entry->fd, &expirations, sizeof(expirations));
    (void)n; // Suppress unused result warning

    // Get the actor
    actor *a = hive_actor_get(entry->owner);
    if (!a) {
        // Actor is dead - cleanup timer
        timer_close_fd(entry);
        SLIST_REMOVE(s_timer.timers, entry);
        hive_pool_free(&s_timer_pool_mgr, entry);
        return;
    }

    // Send timer tick message to actor
    // Use HIVE_MSG_TIMER class with timer_id as tag, sender is the owning actor
    // No payload needed - timer_id is encoded in the tag
    hive_status status = hive_ipc_notify_internal(
        entry->owner, entry->owner, HIVE_MSG_TIMER, entry->id, NULL, 0);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to send timer tick: %s", status.msg);
        // Don't cleanup timer - try again next tick
        return;
    }

    // If one-shot, cleanup
    if (!entry->periodic) {
        timer_close_fd(entry);
        SLIST_REMOVE(s_timer.timers, entry);
        hive_pool_free(&s_timer_pool_mgr, entry);
    }
}

// Initialize timer subsystem
hive_status hive_timer_init(void) {
    HIVE_INIT_GUARD(s_timer.initialized);

    // Initialize timer entry pool
    hive_pool_init(&s_timer_pool_mgr, s_timer_pool, s_timer_used,
                   sizeof(timer_entry), HIVE_TIMER_ENTRY_POOL_SIZE);

    // Initialize timer state
    s_timer.timers = NULL;
    s_timer.next_id = 1;

    s_timer.initialized = true;
    return HIVE_SUCCESS;
}

// Cleanup timer subsystem
void hive_timer_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_timer.initialized);

    // Clean up all active timers
    timer_entry *entry = s_timer.timers;
    while (entry) {
        timer_entry *next = entry->next;
        timer_close_fd(entry);
        hive_pool_free(&s_timer_pool_mgr, entry);
        entry = next;
    }
    s_timer.timers = NULL;

    s_timer.initialized = false;
}

// Create a timer (one-shot or periodic)
static hive_status create_timer(uint32_t interval_us, bool periodic,
                                timer_id *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(s_timer.initialized, "Timer");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Allocate timer entry from pool
    timer_entry *entry = hive_pool_alloc(&s_timer_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Timer entry pool exhausted");
    }

    // Initialize common fields
    entry->id = s_timer.next_id++;
    entry->owner = current->id;
    entry->periodic = periodic;
    entry->interval_us =
        interval_us; // Always store for potential sim mode conversion
    entry->next = s_timer.timers;

    if (s_timer.sim_mode) {
        // Simulation mode: store expiry time, no timerfd
        entry->fd = -1;
        entry->expiry_us = s_timer.sim_time_us + interval_us;
        HIVE_LOG_DEBUG(
            "Timer %u created in sim mode (expiry=%lu, sim_time=%lu)",
            entry->id, (unsigned long)entry->expiry_us,
            (unsigned long)s_timer.sim_time_us);
    } else {
        // Real-time mode: use timerfd
        int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
        if (tfd < 0) {
            hive_pool_free(&s_timer_pool_mgr, entry);
            return HIVE_ERROR(HIVE_ERR_IO, "timerfd_create failed");
        }

        // Set timer
        // Note: timerfd treats (0, 0) as "disarm timer", so we use minimum 1ns
        // for zero delay
        struct itimerspec its;
        if (interval_us == 0) {
            its.it_value.tv_sec = 0;
            its.it_value.tv_nsec = 1; // Minimum 1 nanosecond to avoid disarming
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
            hive_pool_free(&s_timer_pool_mgr, entry);
            return HIVE_ERROR(HIVE_ERR_IO, "timerfd_settime failed");
        }

        entry->fd = tfd;
        entry->interval_us = interval_us;
        entry->expiry_us = 0; // Not used in real-time mode

        // Setup io_source for event system
        entry->source.type = IO_SOURCE_TIMER;
        entry->source.data.timer = entry;

        // Register with HAL event system (timers use read events)
        hive_status reg_status =
            hive_hal_event_register(tfd, HIVE_EVENT_READ, &entry->source);
        if (HIVE_FAILED(reg_status)) {
            close(tfd);
            hive_pool_free(&s_timer_pool_mgr, entry);
            return reg_status;
        }
    }

    s_timer.timers = entry;
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
    HIVE_REQUIRE_INIT(s_timer.initialized, "Timer");

    // Find and remove timer from list
    timer_entry *found = NULL;
    SLIST_FIND_REMOVE(s_timer.timers, entry->id == id, found);

    if (found) {
        timer_close_fd(found);
        hive_pool_free(&s_timer_pool_mgr, found);
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

// Advance simulation time and fire due timers
// Calling this function enables simulation mode (disables timerfd)
void hive_timer_advance_time(uint64_t delta_us) {
    if (!s_timer.initialized) {
        return;
    }

    // Enable simulation mode on first call
    if (!s_timer.sim_mode) {
        s_timer.sim_mode = true;
        HIVE_LOG_INFO("Simulation time mode enabled");

        // Convert any existing timerfd-based timers to simulation mode
        for (timer_entry *entry = s_timer.timers; entry; entry = entry->next) {
            if (entry->fd >= 0) {
                // Close timerfd and unregister from epoll
                timer_close_fd(entry);
                // Set expiry based on interval (fires on first advance after
                // interval)
                entry->expiry_us = entry->interval_us;
            }
        }
    }

    // Advance time
    s_timer.sim_time_us += delta_us;

    // Fire all due timers
    // We need to iterate carefully since firing a timer may cause actor to
    // create/cancel timers
    bool fired_any;
    do {
        fired_any = false;
        timer_entry *entry = s_timer.timers;
        timer_entry *prev = NULL;

        while (entry) {
            timer_entry *next = entry->next;

            // Check if timer is due (only for simulation mode timers)
            if (entry->fd < 0 && entry->expiry_us <= s_timer.sim_time_us) {
                // Get the actor
                actor *a = hive_actor_get(entry->owner);
                if (!a) {
                    // Actor is dead - cleanup timer
                    if (prev) {
                        prev->next = next;
                    } else {
                        s_timer.timers = next;
                    }
                    hive_pool_free(&s_timer_pool_mgr, entry);
                    entry = next;
                    continue;
                }

                // Send timer tick message to actor
                HIVE_LOG_DEBUG(
                    "Timer %u fired for actor %u (sim_time=%lu, expiry=%lu)",
                    entry->id, entry->owner, (unsigned long)s_timer.sim_time_us,
                    (unsigned long)entry->expiry_us);

                hive_status status = hive_ipc_notify_internal(
                    entry->owner, entry->owner, HIVE_MSG_TIMER, entry->id, NULL,
                    0);

                if (HIVE_FAILED(status)) {
                    HIVE_LOG_ERROR("Failed to send timer tick: %s", status.msg);
                    prev = entry;
                    entry = next;
                    continue;
                }

                fired_any = true;

                if (entry->periodic) {
                    // Reschedule periodic timer
                    entry->expiry_us += entry->interval_us;
                    prev = entry;
                } else {
                    // Remove one-shot timer
                    if (prev) {
                        prev->next = next;
                    } else {
                        s_timer.timers = next;
                    }
                    hive_pool_free(&s_timer_pool_mgr, entry);
                }
            } else {
                prev = entry;
            }

            entry = next;
        }
    } while (fired_any); // Repeat if we fired any (handles multiple fires for
        // large delta)
}

// Get current time in microseconds
uint64_t hive_get_time(void) {
    if (s_timer.sim_mode) {
        return s_timer.sim_time_us;
    }

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000 + (uint64_t)ts.tv_nsec / 1000;
}
