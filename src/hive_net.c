#include "hive_net.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_runtime.h"
#include "hive_log.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_pool.h"
#include "hive_io_source.h"
#include "hal/hive_hal_event.h"
#include "hal/hive_hal_net.h"
#include <string.h>

// Network operation types (used in io_source.data.net.operation)
enum {
    NET_OP_ACCEPT,
    NET_OP_CONNECT,
    NET_OP_RECV,
    NET_OP_SEND,
};

// Static pool for io_source entries
static io_source s_io_source_pool[HIVE_IO_SOURCE_POOL_SIZE];
static bool s_io_source_used[HIVE_IO_SOURCE_POOL_SIZE];
static hive_pool s_io_source_pool_mgr;

// Network I/O subsystem state
static struct {
    bool initialized;
} s_net = {0};

// Handle network event from scheduler (called when socket ready)
void hive_net_handle_event(io_source *source) {
    net_io_data *net = &source->data.net;

    // Get the actor
    actor *a = hive_actor_get(net->actor);
    if (!a) {
        // Actor is dead - cleanup
        hive_hal_event_unregister(net->fd);
        hive_pool_free(&s_io_source_pool_mgr, source);
        return;
    }

    // Perform the actual I/O based on operation type
    hive_status status = HIVE_SUCCESS;

    switch (net->operation) {
    case NET_OP_ACCEPT: {
        int conn_fd;
        status = hive_hal_net_accept(net->fd, &conn_fd);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - this shouldn't happen with epoll, but handle it
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_fd = conn_fd;
        }
        break;
    }

    case NET_OP_CONNECT: {
        // Check if connection succeeded
        status = hive_hal_net_connect_check(net->fd);
        if (HIVE_FAILED(status)) {
            hive_hal_net_close(net->fd);
        } else {
            a->io_result_fd = net->fd;
        }
        break;
    }

    case NET_OP_RECV: {
        size_t received = 0;
        status = hive_hal_net_recv(net->fd, net->buf, net->len, &received);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - shouldn't happen with epoll
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_bytes = received;
        }
        break;
    }

    case NET_OP_SEND: {
        size_t sent = 0;
        status = hive_hal_net_send(net->fd, net->buf, net->len, &sent);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - shouldn't happen with epoll
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_bytes = sent;
        }
        break;
    }

    default:
        status = HIVE_ERROR(HIVE_ERR_INVALID, "Unknown network operation");
        break;
    }

    // Remove from event system (one-shot operation)
    hive_hal_event_unregister(net->fd);

    // Store result in actor
    a->io_status = status;

    // Wake actor
    a->state = ACTOR_STATE_READY;

    // Free io_source
    hive_pool_free(&s_io_source_pool_mgr, source);
}

// Initialize network I/O subsystem
hive_status hive_net_init(void) {
    HIVE_INIT_GUARD(s_net.initialized);

    // Initialize HAL network subsystem
    hive_status status = hive_hal_net_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Initialize io_source pool
    hive_pool_init(&s_io_source_pool_mgr, s_io_source_pool, s_io_source_used,
                   sizeof(io_source), HIVE_IO_SOURCE_POOL_SIZE);

    s_net.initialized = true;
    return HIVE_SUCCESS;
}

// Cleanup network I/O subsystem
void hive_net_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_net.initialized);

    hive_hal_net_cleanup();
    s_net.initialized = false;
}

// Helper: Try non-blocking I/O, register with event system if would block
static hive_status try_or_wait(int fd, uint32_t hal_events, int operation,
                               void *buf, size_t len, int32_t timeout_ms) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Non-blocking mode: timeout=0 means "poll once and return immediately"
    if (timeout_ms == 0) {
        return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "Operation would block");
    }

    // Create timeout timer if needed (timeout > 0)
    // Note: timeout < 0 means "wait forever" (no timer)
    timer_id timeout_timer = TIMER_ID_INVALID;
    if (timeout_ms > 0) {
        hive_status status =
            hive_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
        if (HIVE_FAILED(status)) {
            return status; // Timer pool exhausted
        }
    }

    // Allocate io_source from pool
    io_source *source = hive_pool_alloc(&s_io_source_pool_mgr);
    if (!source) {
        if (timeout_timer != TIMER_ID_INVALID) {
            hive_timer_cancel(timeout_timer);
        }
        return HIVE_ERROR(HIVE_ERR_NOMEM, "io_source pool exhausted");
    }

    // Setup io_source for event system
    source->type = IO_SOURCE_NETWORK;
    source->data.net.fd = fd;
    source->data.net.buf = buf;
    source->data.net.len = len;
    source->data.net.actor = current->id;
    source->data.net.operation = operation;

    // Register with HAL event system
    hive_status reg_status = hive_hal_event_register(fd, hal_events, source);
    if (HIVE_FAILED(reg_status)) {
        hive_pool_free(&s_io_source_pool_mgr, source);
        if (timeout_timer != TIMER_ID_INVALID) {
            hive_timer_cancel(timeout_timer);
        }
        return reg_status;
    }

    // Block actor until I/O ready
    current->state = ACTOR_STATE_WAITING;
    hive_yield();

    // When we resume, check for timeout
    hive_status timeout_status = hive_mailbox_handle_timeout(
        current, timeout_timer, "Network I/O operation timed out");
    if (HIVE_FAILED(timeout_status)) {
        // Timeout occurred - cleanup event registration
        hive_hal_event_unregister(fd);
        hive_pool_free(&s_io_source_pool_mgr, source);
        return timeout_status;
    }

    // Return the result stored by the event handler
    return current->io_status;
}

hive_status hive_net_listen(uint16_t port, int *fd_out) {
    if (!fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL fd_out pointer");
    }

    HIVE_REQUIRE_INIT(s_net.initialized, "Network I/O");

    // Create socket (HAL sets it to non-blocking and SO_REUSEADDR)
    int fd;
    hive_status status = hive_hal_net_socket(&fd);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Bind to port
    status = hive_hal_net_bind(fd, port);
    if (HIVE_FAILED(status)) {
        hive_hal_net_close(fd);
        return status;
    }

    // Start listening
    status = hive_hal_net_listen(fd, HIVE_NET_LISTEN_BACKLOG);
    if (HIVE_FAILED(status)) {
        hive_hal_net_close(fd);
        return status;
    }

    *fd_out = fd;
    return HIVE_SUCCESS;
}

hive_status hive_net_accept(int listen_fd, int *conn_fd_out,
                            int32_t timeout_ms) {
    if (!conn_fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL conn_fd_out pointer");
    }

    HIVE_REQUIRE_INIT(s_net.initialized, "Network I/O");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Try immediate accept with non-blocking
    int conn_fd;
    hive_status status = hive_hal_net_accept(listen_fd, &conn_fd);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        *conn_fd_out = conn_fd;
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register interest in epoll and yield
    status = try_or_wait(listen_fd, HIVE_EVENT_READ, NET_OP_ACCEPT, NULL, 0,
                         timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *conn_fd_out = current->io_result_fd;
    return HIVE_SUCCESS;
}

hive_status hive_net_connect(const char *ip, uint16_t port, int *fd_out,
                             int32_t timeout_ms) {
    if (!ip || !fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL ip or fd_out pointer");
    }

    HIVE_REQUIRE_INIT(s_net.initialized, "Network I/O");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Create non-blocking socket
    int fd;
    hive_status status = hive_hal_net_socket(&fd);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Try non-blocking connect (HAL parses IP address)
    status = hive_hal_net_connect(fd, ip, port);

    if (HIVE_SUCCEEDED(status)) {
        // Connected immediately (rare but possible on localhost)
        *fd_out = fd;
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_INPROGRESS) {
        // Error (not just "in progress")
        hive_hal_net_close(fd);
        return status;
    }

    // Connection in progress - add to epoll and wait for writable
    status =
        try_or_wait(fd, HIVE_EVENT_WRITE, NET_OP_CONNECT, NULL, 0, timeout_ms);
    if (HIVE_FAILED(status)) {
        hive_hal_net_close(fd);
        return status;
    }

    // Result stored by event handler
    *fd_out = current->io_result_fd;
    return HIVE_SUCCESS;
}

hive_status hive_net_close(int fd) {
    // Close is synchronous and fast
    return hive_hal_net_close(fd);
}

hive_status hive_net_recv(int fd, void *buf, size_t len, size_t *received,
                          int32_t timeout_ms) {
    if (!buf || !received) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL buffer or received pointer");
    }

    HIVE_REQUIRE_INIT(s_net.initialized, "Network I/O");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Try immediate non-blocking recv
    hive_status status = hive_hal_net_recv(fd, buf, len, received);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register interest in epoll and yield
    status =
        try_or_wait(fd, HIVE_EVENT_READ, NET_OP_RECV, buf, len, timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *received = current->io_result_bytes;
    return HIVE_SUCCESS;
}

hive_status hive_net_send(int fd, const void *buf, size_t len, size_t *sent,
                          int32_t timeout_ms) {
    if (!buf || !sent) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL buffer or sent pointer");
    }

    HIVE_REQUIRE_INIT(s_net.initialized, "Network I/O");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Try immediate non-blocking send
    hive_status status = hive_hal_net_send(fd, buf, len, sent);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register interest in epoll and yield
    status = try_or_wait(fd, HIVE_EVENT_WRITE, NET_OP_SEND, (void *)buf, len,
                         timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *sent = current->io_result_bytes;
    return HIVE_SUCCESS;
}
