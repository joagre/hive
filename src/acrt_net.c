#include "acrt_net.h"
#include "acrt_internal.h"
#include "acrt_static_config.h"
#include "acrt_actor.h"
#include "acrt_scheduler.h"
#include "acrt_runtime.h"
#include "acrt_log.h"
#include "acrt_timer.h"
#include "acrt_ipc.h"
#include "acrt_pool.h"
#include "acrt_io_source.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

// Forward declarations for internal functions
acrt_status acrt_net_init(void);
void acrt_net_cleanup(void);

// Network operation types (used in io_source.data.net.operation)
enum {
    NET_OP_ACCEPT,
    NET_OP_CONNECT,
    NET_OP_RECV,
    NET_OP_SEND,
};

// Static pool for io_source entries
static io_source g_io_source_pool[ACRT_IO_SOURCE_POOL_SIZE];
static bool g_io_source_used[ACRT_IO_SOURCE_POOL_SIZE];
static acrt_pool g_io_source_pool_mgr;

// Network I/O subsystem state (simplified - no worker thread!)
static struct {
    bool initialized;
} g_net = {0};

// Set socket to non-blocking mode
static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Handle network event from scheduler (called when socket ready)
void acrt_net_handle_event(io_source *source) {
    net_io_data *net = &source->data.net;

    // Get the actor
    actor *a = acrt_actor_get(net->actor);
    if (!a) {
        // Actor is dead - cleanup
        int epoll_fd = acrt_scheduler_get_epoll_fd();
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, net->fd, NULL);
        acrt_pool_free(&g_io_source_pool_mgr, source);
        return;
    }

    // Perform the actual I/O based on operation type
    acrt_status status = ACRT_SUCCESS;
    ssize_t n = 0;

    switch (net->operation) {
        case NET_OP_ACCEPT: {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int conn_fd = accept(net->fd, (struct sockaddr *)&client_addr, &client_len);
            if (conn_fd < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Still not ready - this shouldn't happen with epoll, but handle it
                    return;  // Keep waiting
                } else {
                    status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
                }
            } else {
                set_nonblocking(conn_fd);
                a->io_result_fd = conn_fd;
            }
            break;
        }

        case NET_OP_CONNECT: {
            // Check if connection succeeded
            int error = 0;
            socklen_t len = sizeof(error);
            if (getsockopt(net->fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
                status = ACRT_ERROR(ACRT_ERR_IO, error ? strerror(error) : "Connection failed");
                close(net->fd);
            } else {
                a->io_result_fd = net->fd;
            }
            break;
        }

        case NET_OP_RECV: {
            n = recv(net->fd, net->buf, net->len, 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Still not ready - shouldn't happen with epoll
                    return;  // Keep waiting
                } else {
                    status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
                }
            } else {
                a->io_result_bytes = (size_t)n;
            }
            break;
        }

        case NET_OP_SEND: {
            n = send(net->fd, net->buf, net->len, 0);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Still not ready - shouldn't happen with epoll
                    return;  // Keep waiting
                } else {
                    status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
                }
            } else {
                a->io_result_bytes = (size_t)n;
            }
            break;
        }

        default:
            status = ACRT_ERROR(ACRT_ERR_INVALID, "Unknown network operation");
            break;
    }

    // Remove from epoll (one-shot operation)
    int epoll_fd = acrt_scheduler_get_epoll_fd();
    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, net->fd, NULL);

    // Store result in actor
    a->io_status = status;

    // Wake actor
    a->state = ACTOR_STATE_READY;

    // Free io_source
    acrt_pool_free(&g_io_source_pool_mgr, source);
}

// Initialize network I/O subsystem
acrt_status acrt_net_init(void) {
    ACRT_INIT_GUARD(g_net.initialized);

    // Initialize io_source pool
    acrt_pool_init(&g_io_source_pool_mgr, g_io_source_pool, g_io_source_used,
                 sizeof(io_source), ACRT_IO_SOURCE_POOL_SIZE);

    g_net.initialized = true;
    return ACRT_SUCCESS;
}

// Cleanup network I/O subsystem
void acrt_net_cleanup(void) {
    ACRT_CLEANUP_GUARD(g_net.initialized);
    g_net.initialized = false;
}

// Helper: Try non-blocking I/O, add to epoll if would block
static acrt_status try_or_epoll(int fd, uint32_t epoll_events, int operation,
                               void *buf, size_t len, int32_t timeout_ms) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Non-blocking mode: timeout=0 means "poll once and return immediately"
    if (timeout_ms == 0) {
        return ACRT_ERROR(ACRT_ERR_WOULDBLOCK, "Operation would block");
    }

    // Create timeout timer if needed (timeout > 0)
    // Note: timeout < 0 means "wait forever" (no timer)
    timer_id timeout_timer = TIMER_ID_INVALID;
    if (timeout_ms > 0) {
        acrt_status status = acrt_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
        if (ACRT_FAILED(status)) {
            return status;  // Timer pool exhausted
        }
    }

    // Allocate io_source from pool
    io_source *source = acrt_pool_alloc(&g_io_source_pool_mgr);
    if (!source) {
        if (timeout_timer != TIMER_ID_INVALID) {
            acrt_timer_cancel(timeout_timer);
        }
        return ACRT_ERROR(ACRT_ERR_NOMEM, "io_source pool exhausted");
    }

    // Setup io_source for epoll
    source->type = IO_SOURCE_NETWORK;
    source->data.net.fd = fd;
    source->data.net.buf = buf;
    source->data.net.len = len;
    source->data.net.actor = current->id;
    source->data.net.operation = operation;

    // Add to scheduler's epoll
    int epoll_fd = acrt_scheduler_get_epoll_fd();
    struct epoll_event ev;
    ev.events = epoll_events;
    ev.data.ptr = source;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) < 0) {
        acrt_pool_free(&g_io_source_pool_mgr, source);
        if (timeout_timer != TIMER_ID_INVALID) {
            acrt_timer_cancel(timeout_timer);
        }
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    // Block actor until I/O ready
    current->state = ACTOR_STATE_WAITING;
    acrt_yield();

    // When we resume, check for timeout
    acrt_status timeout_status = acrt_mailbox_handle_timeout(current, timeout_timer, "Network I/O operation timed out");
    if (ACRT_FAILED(timeout_status)) {
        // Timeout occurred - cleanup epoll registration
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
        acrt_pool_free(&g_io_source_pool_mgr, source);
        return timeout_status;
    }

    // Return the result stored by the event handler
    return current->io_status;
}

acrt_status acrt_net_listen(uint16_t port, int *fd_out) {
    if (!fd_out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    // Create socket (synchronous, doesn't block)
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    // Set SO_REUSEADDR to avoid "Address already in use" errors
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        acrt_status status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
        close(fd);
        return status;
    }

    if (listen(fd, ACRT_NET_LISTEN_BACKLOG) < 0) {
        acrt_status status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
        close(fd);
        return status;
    }

    set_nonblocking(fd);
    *fd_out = fd;
    return ACRT_SUCCESS;
}

acrt_status acrt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms) {
    if (!conn_fd_out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Try immediate accept with non-blocking
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int conn_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);

    if (conn_fd >= 0) {
        // Success immediately!
        set_nonblocking(conn_fd);
        *conn_fd_out = conn_fd;
        return ACRT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    acrt_status status = try_or_epoll(listen_fd, EPOLLIN, NET_OP_ACCEPT, NULL, 0, timeout_ms);
    if (ACRT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *conn_fd_out = current->io_result_fd;
    return ACRT_SUCCESS;
}

acrt_status acrt_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms) {
    if (!ip || !fd_out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Parse numeric IPv4 address (DNS resolution not supported - would block scheduler)
    struct sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) != 1) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid IPv4 address (hostnames not supported)");
    }

    // Create non-blocking socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    set_nonblocking(fd);

    // Try non-blocking connect
    if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        if (errno != EINPROGRESS) {
            acrt_status status = ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
            close(fd);
            return status;
        }

        // Connection in progress - add to epoll and wait for writable
        acrt_status status = try_or_epoll(fd, EPOLLOUT, NET_OP_CONNECT, NULL, 0, timeout_ms);
        if (ACRT_FAILED(status)) {
            close(fd);
            return status;
        }

        // Result stored by event handler
        *fd_out = current->io_result_fd;
        return ACRT_SUCCESS;
    }

    // Connected immediately (rare but possible on localhost)
    *fd_out = fd;
    return ACRT_SUCCESS;
}

acrt_status acrt_net_close(int fd) {
    // Close is synchronous and fast
    if (close(fd) < 0) {
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }
    return ACRT_SUCCESS;
}

acrt_status acrt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms) {
    if (!buf || !received) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Try immediate non-blocking recv
    ssize_t n = recv(fd, buf, len, MSG_DONTWAIT);
    if (n >= 0) {
        // Success immediately!
        *received = (size_t)n;
        return ACRT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    acrt_status status = try_or_epoll(fd, EPOLLIN, NET_OP_RECV, buf, len, timeout_ms);
    if (ACRT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *received = current->io_result_bytes;
    return ACRT_SUCCESS;
}

acrt_status acrt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms) {
    if (!buf || !sent) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Try immediate non-blocking send
    ssize_t n = send(fd, buf, len, MSG_DONTWAIT);
    if (n >= 0) {
        // Success immediately!
        *sent = (size_t)n;
        return ACRT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return ACRT_ERROR(ACRT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    acrt_status status = try_or_epoll(fd, EPOLLOUT, NET_OP_SEND, (void *)buf, len, timeout_ms);
    if (ACRT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *sent = current->io_result_bytes;
    return ACRT_SUCCESS;
}
