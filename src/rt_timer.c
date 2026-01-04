#include "rt_timer.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_spsc.h"
#include "rt_ipc.h"
#include "rt_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>

// Forward declarations for internal functions
rt_status rt_timer_init(void);
void rt_timer_cleanup(void);
void rt_timer_process_completions(void);

// Timer operation types
typedef enum {
    TIMER_OP_AFTER,
    TIMER_OP_EVERY,
    TIMER_OP_CANCEL,
} timer_op_type;

// Timer operation request
typedef struct {
    timer_op_type op;
    actor_id      requester;
    uint32_t      interval_us;
    timer_id      id;  // For cancel
} timer_request;

// Timer operation completion types
typedef enum {
    TIMER_COMP_CREATED,  // Timer created successfully
    TIMER_COMP_TICK,     // Timer tick (deliver message to actor)
    TIMER_COMP_ERROR,    // Operation failed
} timer_comp_type;

// Timer operation completion
typedef struct {
    timer_comp_type type;
    actor_id        requester;
    timer_id        id;
    rt_status       status;
} timer_completion;

// Active timer entry
typedef struct timer_entry {
    timer_id            id;
    actor_id            owner;
    int                 fd;        // timerfd
    bool                periodic;
    struct timer_entry *next;
} timer_entry;

// Timer subsystem state
static struct {
    rt_spsc_queue  request_queue;
    rt_spsc_queue  completion_queue;
    pthread_t      worker_thread;
    bool           running;
    bool           initialized;

    // Active timers (managed by worker thread)
    timer_entry   *timers;
    timer_id       next_id;
    pthread_mutex_t timers_lock;
    int            epoll_fd;
} g_timer = {0};

// Timer worker thread
static void *timer_worker_thread(void *arg) {
    (void)arg;

    RT_LOG_DEBUG("Timer worker thread started");

    // Create epoll instance
    g_timer.epoll_fd = epoll_create1(0);
    if (g_timer.epoll_fd < 0) {
        RT_LOG_ERROR("Failed to create epoll: %s", strerror(errno));
        return NULL;
    }

    struct epoll_event events[16];

    while (g_timer.running) {
        // Process timer requests
        timer_request req;
        while (rt_spsc_pop(&g_timer.request_queue, &req)) {
            timer_completion comp = {
                .type = TIMER_COMP_CREATED,
                .requester = req.requester,
                .status = RT_SUCCESS
            };

            switch (req.op) {
                case TIMER_OP_AFTER:
                case TIMER_OP_EVERY: {
                    // Create timerfd
                    int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
                    if (tfd < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    }

                    // Set timer
                    struct itimerspec its;
                    its.it_value.tv_sec = req.interval_us / 1000000;
                    its.it_value.tv_nsec = (req.interval_us % 1000000) * 1000;

                    if (req.op == TIMER_OP_EVERY) {
                        // Periodic - set interval
                        its.it_interval.tv_sec = its.it_value.tv_sec;
                        its.it_interval.tv_nsec = its.it_value.tv_nsec;
                    } else {
                        // One-shot - no interval
                        its.it_interval.tv_sec = 0;
                        its.it_interval.tv_nsec = 0;
                    }

                    if (timerfd_settime(tfd, 0, &its, NULL) < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        close(tfd);
                        break;
                    }

                    // Create timer entry
                    timer_entry *entry = malloc(sizeof(timer_entry));
                    if (!entry) {
                        comp.status = RT_ERROR(RT_ERR_NOMEM, "Failed to allocate timer entry");
                        close(tfd);
                        break;
                    }

                    pthread_mutex_lock(&g_timer.timers_lock);
                    entry->id = g_timer.next_id++;
                    entry->owner = req.requester;
                    entry->fd = tfd;
                    entry->periodic = (req.op == TIMER_OP_EVERY);
                    entry->next = g_timer.timers;
                    g_timer.timers = entry;
                    pthread_mutex_unlock(&g_timer.timers_lock);

                    // Add to epoll
                    struct epoll_event ev;
                    ev.events = EPOLLIN;
                    ev.data.ptr = entry;
                    if (epoll_ctl(g_timer.epoll_fd, EPOLL_CTL_ADD, tfd, &ev) < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        pthread_mutex_lock(&g_timer.timers_lock);
                        // Remove from list
                        timer_entry **prev = &g_timer.timers;
                        while (*prev && *prev != entry) {
                            prev = &(*prev)->next;
                        }
                        if (*prev) {
                            *prev = entry->next;
                        }
                        pthread_mutex_unlock(&g_timer.timers_lock);
                        close(tfd);
                        free(entry);
                        break;
                    }

                    comp.id = entry->id;
                    break;
                }

                case TIMER_OP_CANCEL: {
                    pthread_mutex_lock(&g_timer.timers_lock);
                    timer_entry **prev = &g_timer.timers;
                    timer_entry *entry = NULL;
                    while (*prev) {
                        if ((*prev)->id == req.id) {
                            entry = *prev;
                            *prev = entry->next;
                            break;
                        }
                        prev = &(*prev)->next;
                    }
                    pthread_mutex_unlock(&g_timer.timers_lock);

                    if (entry) {
                        epoll_ctl(g_timer.epoll_fd, EPOLL_CTL_DEL, entry->fd, NULL);
                        close(entry->fd);
                        free(entry);
                    } else {
                        comp.status = RT_ERROR(RT_ERR_INVALID, "Timer not found");
                    }
                    break;
                }
            }

            // Push completion
            while (!rt_spsc_push(&g_timer.completion_queue, &comp)) {
                struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000};
                nanosleep(&ts, NULL);
            }
        }

        // Wait for timer events (with timeout to check for new requests)
        int nfds = epoll_wait(g_timer.epoll_fd, events, 16, 100); // 100ms timeout

        for (int i = 0; i < nfds; i++) {
            timer_entry *entry = (timer_entry *)events[i].data.ptr;

            // Read timerfd to acknowledge
            uint64_t expirations;
            ssize_t n = read(entry->fd, &expirations, sizeof(expirations));
            (void)n;  // Suppress unused result warning

            // Send tick completion to scheduler
            timer_completion tick_comp = {
                .type = TIMER_COMP_TICK,
                .requester = entry->owner,
                .id = entry->id,
                .status = RT_SUCCESS
            };

            while (!rt_spsc_push(&g_timer.completion_queue, &tick_comp)) {
                struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000};
                nanosleep(&ts, NULL);
            }

            // If one-shot, remove timer
            if (!entry->periodic) {
                pthread_mutex_lock(&g_timer.timers_lock);
                timer_entry **prev = &g_timer.timers;
                while (*prev && *prev != entry) {
                    prev = &(*prev)->next;
                }
                if (*prev) {
                    *prev = entry->next;
                }
                pthread_mutex_unlock(&g_timer.timers_lock);

                epoll_ctl(g_timer.epoll_fd, EPOLL_CTL_DEL, entry->fd, NULL);
                close(entry->fd);
                free(entry);
            }
        }
    }

    // Cleanup
    close(g_timer.epoll_fd);

    RT_LOG_DEBUG("Timer worker thread exiting");
    return NULL;
}

// Initialize timer subsystem
rt_status rt_timer_init(void) {
    if (g_timer.initialized) {
        return RT_SUCCESS;
    }

    // Initialize queues
    rt_status status = rt_spsc_init(&g_timer.request_queue, sizeof(timer_request), 64);
    if (RT_FAILED(status)) {
        return status;
    }

    status = rt_spsc_init(&g_timer.completion_queue, sizeof(timer_completion), 64);
    if (RT_FAILED(status)) {
        rt_spsc_destroy(&g_timer.request_queue);
        return status;
    }

    // Initialize timer state
    g_timer.timers = NULL;
    g_timer.next_id = 1;
    pthread_mutex_init(&g_timer.timers_lock, NULL);

    // Start worker thread
    g_timer.running = true;
    if (pthread_create(&g_timer.worker_thread, NULL, timer_worker_thread, NULL) != 0) {
        pthread_mutex_destroy(&g_timer.timers_lock);
        rt_spsc_destroy(&g_timer.request_queue);
        rt_spsc_destroy(&g_timer.completion_queue);
        return RT_ERROR(RT_ERR_IO, "Failed to create timer worker thread");
    }

    g_timer.initialized = true;
    return RT_SUCCESS;
}

// Cleanup timer subsystem
void rt_timer_cleanup(void) {
    if (!g_timer.initialized) {
        return;
    }

    // Stop worker thread
    g_timer.running = false;
    pthread_join(g_timer.worker_thread, NULL);

    // Clean up timers
    pthread_mutex_lock(&g_timer.timers_lock);
    timer_entry *entry = g_timer.timers;
    while (entry) {
        timer_entry *next = entry->next;
        close(entry->fd);
        free(entry);
        entry = next;
    }
    g_timer.timers = NULL;
    pthread_mutex_unlock(&g_timer.timers_lock);

    pthread_mutex_destroy(&g_timer.timers_lock);

    // Cleanup queues
    rt_spsc_destroy(&g_timer.request_queue);
    rt_spsc_destroy(&g_timer.completion_queue);

    g_timer.initialized = false;
}

// Process timer completions (called by scheduler)
void rt_timer_process_completions(void) {
    if (!g_timer.initialized) {
        return;
    }

    timer_completion comp;
    while (rt_spsc_pop(&g_timer.completion_queue, &comp)) {
        actor *a = rt_actor_get(comp.requester);
        if (!a) {
            continue;
        }

        switch (comp.type) {
            case TIMER_COMP_CREATED:
            case TIMER_COMP_ERROR:
                // Timer create/cancel completion - wake blocked actor
                if (a->state == ACTOR_STATE_BLOCKED) {
                    a->io_status = comp.status;
                    a->io_result_fd = comp.id;  // Return timer_id in fd field
                    a->state = ACTOR_STATE_READY;
                }
                break;

            case TIMER_COMP_TICK:
                // Timer tick - inject message into actor's mailbox
                {
                    mailbox_entry *entry = malloc(sizeof(mailbox_entry));
                    if (!entry) {
                        break;  // Drop tick if out of memory
                    }

                    entry->sender = RT_SENDER_TIMER;
                    entry->len = sizeof(timer_id);
                    entry->data = malloc(sizeof(timer_id));
                    if (!entry->data) {
                        free(entry);
                        break;
                    }
                    *(timer_id *)entry->data = comp.id;
                    entry->borrow_ptr = NULL;
                    entry->next = NULL;

                    // Add to actor's mailbox
                    if (a->mbox.tail) {
                        a->mbox.tail->next = entry;
                    } else {
                        a->mbox.head = entry;
                    }
                    a->mbox.tail = entry;
                    a->mbox.count++;

                    // Wake actor if blocked waiting for message
                    if (a->state == ACTOR_STATE_BLOCKED) {
                        a->state = ACTOR_STATE_READY;
                    }
                }
                break;
        }
    }
}

// Submit timer operation and block
static rt_status submit_and_block(timer_request *req) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    if (!g_timer.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Timer subsystem not initialized");
    }

    req->requester = current->id;

    // Submit request
    while (!rt_spsc_push(&g_timer.request_queue, req)) {
        rt_yield();
    }

    // Block waiting for completion
    current->state = ACTOR_STATE_BLOCKED;
    rt_yield();

    // When we wake up, the operation is complete
    return current->io_status;
}

rt_status rt_timer_after(uint32_t delay_us, timer_id *out) {
    if (!out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    timer_request req;
    req.op = TIMER_OP_AFTER;
    req.interval_us = delay_us;

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *out = (timer_id)current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_timer_every(uint32_t interval_us, timer_id *out) {
    if (!out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    timer_request req;
    req.op = TIMER_OP_EVERY;
    req.interval_us = interval_us;

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *out = (timer_id)current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_timer_cancel(timer_id id) {
    timer_request req;
    req.op = TIMER_OP_CANCEL;
    req.id = id;

    return submit_and_block(&req);
}

bool rt_timer_is_tick(const rt_message *msg) {
    return msg && msg->sender == RT_SENDER_TIMER;
}
