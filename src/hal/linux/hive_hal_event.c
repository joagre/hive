// Hardware Abstraction Layer - Linux Event Implementation
//
// Event system uses epoll for efficient I/O multiplexing.

#include "hal/hive_hal_event.h"
#include "hive_io_source.h"
#include "hive_static_config.h"
#include <sys/epoll.h>
#include <unistd.h>
#include <stdbool.h>

// Forward declarations for event handlers
extern void hive_timer_handle_event(io_source_t *source);
#if HIVE_ENABLE_TCP
extern void hive_tcp_handle_event(io_source_t *source);
#endif

// Event system state
static struct {
    int epoll_fd;
    bool initialized;
} s_event = {.epoll_fd = -1, .initialized = false};

// Dispatch events from epoll
static void dispatch_epoll_events(int timeout_ms) {
    if (!s_event.initialized || s_event.epoll_fd < 0) {
        return;
    }

    struct epoll_event events[HIVE_EPOLL_MAX_EVENTS];
    int n =
        epoll_wait(s_event.epoll_fd, events, HIVE_EPOLL_MAX_EVENTS, timeout_ms);

    for (int i = 0; i < n; i++) {
        io_source_t *source = events[i].data.ptr;

        if (source->type == IO_SOURCE_TIMER) {
            hive_timer_handle_event(source);
        }
#if HIVE_ENABLE_TCP
        else if (source->type == IO_SOURCE_TCP) {
            hive_tcp_handle_event(source);
        }
#endif
    }
}

hive_status_t hive_hal_event_init(void) {
    if (s_event.initialized) {
        return HIVE_SUCCESS;
    }

    s_event.epoll_fd = epoll_create1(0);
    if (s_event.epoll_fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "Failed to create epoll");
    }

    s_event.initialized = true;
    return HIVE_SUCCESS;
}

void hive_hal_event_cleanup(void) {
    if (!s_event.initialized) {
        return;
    }

    if (s_event.epoll_fd >= 0) {
        close(s_event.epoll_fd);
        s_event.epoll_fd = -1;
    }

    s_event.initialized = false;
}

void hive_hal_event_poll(void) {
    dispatch_epoll_events(0);
}

void hive_hal_event_wait(int timeout_ms) {
    dispatch_epoll_events(timeout_ms);
}

hive_status_t hive_hal_event_register(int fd, uint32_t events,
                                      io_source_t *source) {
    if (!s_event.initialized || s_event.epoll_fd < 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Event system not initialized");
    }

    // Map platform-neutral flags to epoll flags
    uint32_t epoll_events = 0;
    if (events & HIVE_EVENT_READ) {
        epoll_events |= EPOLLIN;
    }
    if (events & HIVE_EVENT_WRITE) {
        epoll_events |= EPOLLOUT;
    }

    struct epoll_event ev;
    ev.events = epoll_events;
    ev.data.ptr = source;

    if (epoll_ctl(s_event.epoll_fd, EPOLL_CTL_ADD, fd, &ev) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "epoll_ctl ADD failed");
    }

    return HIVE_SUCCESS;
}

void hive_hal_event_unregister(int fd) {
    if (!s_event.initialized || s_event.epoll_fd < 0) {
        return;
    }

    epoll_ctl(s_event.epoll_fd, EPOLL_CTL_DEL, fd, NULL);
}

// -----------------------------------------------------------------------------
// HAL Event Signaling - Linux stub implementation
// -----------------------------------------------------------------------------
// Linux doesn't have hardware interrupts in userspace, but we provide the API
// for compatibility. Webots simulation doesn't use ESB radio.

#include <stdatomic.h>

static atomic_uint_fast32_t s_event_flags = 0;
static uint32_t s_event_allocated = 0;

hive_hal_event_id_t hive_hal_event_create(void) {
    for (uint8_t i = 0; i < HIVE_HAL_EVENT_MAX; i++) {
        if (!(s_event_allocated & (1U << i))) {
            s_event_allocated |= (1U << i);
            return i;
        }
    }
    return HIVE_HAL_EVENT_INVALID;
}

void hive_hal_event_destroy(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        s_event_allocated &= ~(1U << id);
        atomic_fetch_and(&s_event_flags, ~(1U << id));
    }
}

void hive_hal_event_signal(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        atomic_fetch_or(&s_event_flags, (1U << id));
    }
}

bool hive_hal_event_is_set(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        return (atomic_load(&s_event_flags) & (1U << id)) != 0;
    }
    return false;
}

void hive_hal_event_clear(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        atomic_fetch_and(&s_event_flags, ~(1U << id));
    }
}
