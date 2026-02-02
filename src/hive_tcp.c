#include "hive_tcp.h"
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
#include "hal/hive_hal_tcp.h"
#include <string.h>

// TCP operation types (used in io_source_t.data.tcp.operation)
enum {
    TCP_OP_ACCEPT,
    TCP_OP_CONNECT,
    TCP_OP_RECV,
    TCP_OP_SEND,
};

// Static pool for io_source_t entries
static io_source_t s_io_source_pool[HIVE_IO_SOURCE_POOL_SIZE];
static bool s_io_source_used[HIVE_IO_SOURCE_POOL_SIZE];
static hive_pool_t s_io_source_pool_mgr;

// TCP subsystem state
static struct {
    bool initialized;
} s_tcp = {0};

// Handle TCP event from scheduler (called when socket ready)
void hive_tcp_handle_event(io_source_t *source) {
    tcp_io_data_t *tcp = &source->data.tcp;

    // Get the actor_t
    actor_t *a = hive_actor_get(tcp->actor);
    if (!a) {
        // Actor is dead - cleanup
        hive_hal_event_unregister(tcp->fd);
        hive_pool_free(&s_io_source_pool_mgr, source);
        return;
    }

    // Perform the actual I/O based on operation type
    hive_status_t status = HIVE_SUCCESS;

    switch (tcp->operation) {
    case TCP_OP_ACCEPT: {
        int conn_fd;
        status = hive_hal_tcp_accept(tcp->fd, &conn_fd);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - handle gracefully
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_fd = conn_fd;
        }
        break;
    }

    case TCP_OP_CONNECT: {
        // Check if connection succeeded
        status = hive_hal_tcp_connect_check(tcp->fd);
        if (HIVE_FAILED(status)) {
            hive_hal_tcp_close(tcp->fd);
        } else {
            a->io_result_fd = tcp->fd;
        }
        break;
    }

    case TCP_OP_RECV: {
        size_t received = 0;
        status = hive_hal_tcp_recv(tcp->fd, tcp->buf, tcp->len, &received);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - keep waiting
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_bytes = received;
        }
        break;
    }

    case TCP_OP_SEND: {
        size_t sent = 0;
        status = hive_hal_tcp_send(tcp->fd, tcp->buf, tcp->len, &sent);
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            // Still not ready - keep waiting
            return; // Keep waiting
        }
        if (HIVE_SUCCEEDED(status)) {
            a->io_result_bytes = sent;
        }
        break;
    }

    default:
        status = HIVE_ERROR(HIVE_ERR_INVALID, "Unknown TCP operation");
        break;
    }

    // Remove from event system (one-shot operation)
    hive_hal_event_unregister(tcp->fd);

    // Store result in actor_t
    a->io_status = status;

    // Wake actor_t
    a->state = ACTOR_STATE_READY;

    // Free io_source_t
    hive_pool_free(&s_io_source_pool_mgr, source);
}

// Initialize TCP subsystem
hive_status_t hive_tcp_init(void) {
    HIVE_INIT_GUARD(s_tcp.initialized);

    // Initialize HAL TCP subsystem
    hive_status_t status = hive_hal_tcp_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Initialize io_source_t pool
    hive_pool_init(&s_io_source_pool_mgr, s_io_source_pool, s_io_source_used,
                   sizeof(io_source_t), HIVE_IO_SOURCE_POOL_SIZE);

    s_tcp.initialized = true;
    return HIVE_SUCCESS;
}

// Cleanup TCP subsystem
void hive_tcp_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_tcp.initialized);

    hive_hal_tcp_cleanup();
    s_tcp.initialized = false;
}

// Helper: Try non-blocking I/O, register with event system if would block
static hive_status_t try_or_wait(int fd, uint32_t hal_events, int operation,
                                 void *buf, size_t len, int32_t timeout_ms) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Non-blocking mode: timeout=0 means "poll once and return immediately"
    if (timeout_ms == 0) {
        return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "Operation would block");
    }

    // Create timeout timer if needed (timeout > 0)
    // Note: timeout < 0 means "wait forever" (no timer)
    hive_timer_id_t timeout_timer = HIVE_TIMER_ID_INVALID;
    if (timeout_ms > 0) {
        hive_status_t status =
            hive_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
        if (HIVE_FAILED(status)) {
            return status; // Timer pool exhausted
        }
    }

    // Allocate io_source_t from pool
    io_source_t *source = hive_pool_alloc(&s_io_source_pool_mgr);
    if (!source) {
        if (timeout_timer != HIVE_TIMER_ID_INVALID) {
            hive_timer_cancel(timeout_timer);
        }
        return HIVE_ERROR(HIVE_ERR_NOMEM, "io_source_t pool exhausted");
    }

    // Setup io_source_t for event system
    source->type = IO_SOURCE_TCP;
    source->data.tcp.fd = fd;
    source->data.tcp.buf = buf;
    source->data.tcp.len = len;
    source->data.tcp.actor = current->id;
    source->data.tcp.operation = operation;

    // Register with HAL event system
    hive_status_t reg_status = hive_hal_event_register(fd, hal_events, source);
    if (HIVE_FAILED(reg_status)) {
        hive_pool_free(&s_io_source_pool_mgr, source);
        if (timeout_timer != HIVE_TIMER_ID_INVALID) {
            hive_timer_cancel(timeout_timer);
        }
        return reg_status;
    }

    // Block actor_t until I/O ready
    current->state = ACTOR_STATE_WAITING;
    hive_yield();

    // When we resume, check for timeout
    hive_status_t timeout_status = hive_mailbox_handle_timeout(
        current, timeout_timer, "TCP operation timed out");
    if (HIVE_FAILED(timeout_status)) {
        // Timeout occurred - cleanup event registration
        hive_hal_event_unregister(fd);
        hive_pool_free(&s_io_source_pool_mgr, source);
        return timeout_status;
    }

    // Return the result stored by the event handler
    return current->io_status;
}

hive_status_t hive_tcp_listen(uint16_t port, int *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(s_tcp.initialized, "TCP");

    // Create socket (HAL sets it to non-blocking and SO_REUSEADDR)
    int fd;
    hive_status_t status = hive_hal_tcp_socket(&fd);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Bind to port
    status = hive_hal_tcp_bind(fd, port);
    if (HIVE_FAILED(status)) {
        hive_hal_tcp_close(fd);
        return status;
    }

    // Start listening
    status = hive_hal_tcp_listen(fd, HIVE_TCP_LISTEN_BACKLOG);
    if (HIVE_FAILED(status)) {
        hive_hal_tcp_close(fd);
        return status;
    }

    *out = fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_tcp_accept(int listen_fd, int *out, int32_t timeout_ms) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(s_tcp.initialized, "TCP");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Try immediate accept with non-blocking
    int conn_fd;
    hive_status_t status = hive_hal_tcp_accept(listen_fd, &conn_fd);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        *out = conn_fd;
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register event interest and yield
    status = try_or_wait(listen_fd, HIVE_EVENT_READ, TCP_OP_ACCEPT, NULL, 0,
                         timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *out = current->io_result_fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_tcp_connect(const char *ip, uint16_t port, int *out,
                               int32_t timeout_ms) {
    if (!ip || !out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL ip or out pointer");
    }

    HIVE_REQUIRE_INIT(s_tcp.initialized, "TCP");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Create non-blocking socket
    int fd;
    hive_status_t status = hive_hal_tcp_socket(&fd);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Try non-blocking connect (HAL parses IP address)
    status = hive_hal_tcp_connect(fd, ip, port);

    if (HIVE_SUCCEEDED(status)) {
        // Connected immediately (rare but possible on localhost)
        *out = fd;
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_INPROGRESS) {
        // Error (not just "in progress")
        hive_hal_tcp_close(fd);
        return status;
    }

    // Connection in progress - register event interest and wait for writable
    status =
        try_or_wait(fd, HIVE_EVENT_WRITE, TCP_OP_CONNECT, NULL, 0, timeout_ms);
    if (HIVE_FAILED(status)) {
        hive_hal_tcp_close(fd);
        return status;
    }

    // Result stored by event handler
    *out = current->io_result_fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_tcp_close(int fd) {
    // Close is synchronous and fast
    return hive_hal_tcp_close(fd);
}

hive_status_t hive_tcp_recv(int fd, void *buf, size_t len, size_t *bytes_read,
                            int32_t timeout_ms) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(s_tcp.initialized, "TCP");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Try immediate non-blocking recv
    hive_status_t status = hive_hal_tcp_recv(fd, buf, len, bytes_read);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register event interest and yield
    status =
        try_or_wait(fd, HIVE_EVENT_READ, TCP_OP_RECV, buf, len, timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *bytes_read = current->io_result_bytes;
    return HIVE_SUCCESS;
}

hive_status_t hive_tcp_send(int fd, const void *buf, size_t len,
                            size_t *bytes_written, int32_t timeout_ms) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(s_tcp.initialized, "TCP");

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Try immediate non-blocking send
    hive_status_t status = hive_hal_tcp_send(fd, buf, len, bytes_written);

    if (HIVE_SUCCEEDED(status)) {
        // Success immediately!
        return HIVE_SUCCESS;
    }

    if (status.code != HIVE_ERR_WOULDBLOCK) {
        // Error (not just "would block")
        return status;
    }

    // Would block - register event interest and yield
    status = try_or_wait(fd, HIVE_EVENT_WRITE, TCP_OP_SEND, (void *)buf, len,
                         timeout_ms);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *bytes_written = current->io_result_bytes;
    return HIVE_SUCCESS;
}
