#include "rt_net.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_log.h"
#include "rt_timer.h"
#include "rt_ipc.h"
#include "rt_pool.h"
#include "rt_io_source.h"
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
rt_status rt_net_init(void);
void rt_net_cleanup(void);

// Network operation types (used in io_source.data.net.operation)
enum {
    NET_OP_ACCEPT,
    NET_OP_CONNECT,
    NET_OP_RECV,
    NET_OP_SEND,
};

// Static pool for io_source entries
static io_source g_io_source_pool[RT_IO_SOURCE_POOL_SIZE];
static bool g_io_source_used[RT_IO_SOURCE_POOL_SIZE];
static rt_pool g_io_source_pool_mgr;

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
void rt_net_handle_event(io_source *source) {
    net_io_data *net = &source->data.net;

    // Get the actor
    actor *a = rt_actor_get(net->actor);
    if (!a) {
        // Actor is dead - cleanup
        int epoll_fd = rt_scheduler_get_epoll_fd();
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, net->fd, NULL);
        rt_pool_free(&g_io_source_pool_mgr, source);
        return;
    }

    // Perform the actual I/O based on operation type
    rt_status status = RT_SUCCESS;
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
                    status = RT_ERROR(RT_ERR_IO, strerror(errno));
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
                status = RT_ERROR(RT_ERR_IO, error ? strerror(error) : "Connection failed");
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
                    status = RT_ERROR(RT_ERR_IO, strerror(errno));
                }
            } else {
                a->io_result_nbytes = (size_t)n;
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
                    status = RT_ERROR(RT_ERR_IO, strerror(errno));
                }
            } else {
                a->io_result_nbytes = (size_t)n;
            }
            break;
        }

        default:
            status = RT_ERROR(RT_ERR_INVALID, "Unknown network operation");
            break;
    }

    // Remove from epoll (one-shot operation)
    int epoll_fd = rt_scheduler_get_epoll_fd();
    epoll_ctl(epoll_fd, EPOLL_CTL_DEL, net->fd, NULL);

    // Store result in actor
    a->io_status = status;

    // Wake actor
    a->state = ACTOR_STATE_READY;

    // Free io_source
    rt_pool_free(&g_io_source_pool_mgr, source);
}

// Initialize network I/O subsystem
rt_status rt_net_init(void) {
    RT_INIT_GUARD(g_net.initialized);

    // Initialize io_source pool
    rt_pool_init(&g_io_source_pool_mgr, g_io_source_pool, g_io_source_used,
                 sizeof(io_source), RT_IO_SOURCE_POOL_SIZE);

    g_net.initialized = true;
    return RT_SUCCESS;
}

// Cleanup network I/O subsystem
void rt_net_cleanup(void) {
    RT_CLEANUP_GUARD(g_net.initialized);
    g_net.initialized = false;
}

// Helper: Try non-blocking I/O, add to epoll if would block
static rt_status try_or_epoll(int fd, uint32_t epoll_events, int operation,
                               void *buf, size_t len, int32_t timeout_ms) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Create timeout timer if needed
    timer_id timeout_timer = TIMER_ID_INVALID;
    if (timeout_ms > 0) {
        rt_status status = rt_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
        if (RT_FAILED(status)) {
            return status;  // Timer pool exhausted
        }
    }

    // Allocate io_source from pool
    io_source *source = rt_pool_alloc(&g_io_source_pool_mgr);
    if (!source) {
        if (timeout_timer != TIMER_ID_INVALID) {
            rt_timer_cancel(timeout_timer);
        }
        return RT_ERROR(RT_ERR_NOMEM, "io_source pool exhausted");
    }

    // Setup io_source for epoll
    source->type = IO_SOURCE_NETWORK;
    source->data.net.fd = fd;
    source->data.net.buf = buf;
    source->data.net.len = len;
    source->data.net.actor = current->id;
    source->data.net.operation = operation;

    // Add to scheduler's epoll
    int epoll_fd = rt_scheduler_get_epoll_fd();
    struct epoll_event ev;
    ev.events = epoll_events;
    ev.data.ptr = source;

    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, fd, &ev) < 0) {
        rt_pool_free(&g_io_source_pool_mgr, source);
        if (timeout_timer != TIMER_ID_INVALID) {
            rt_timer_cancel(timeout_timer);
        }
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Block actor until I/O ready
    current->state = ACTOR_STATE_BLOCKED;
    rt_yield();

    // When we resume, check for timeout
    rt_status timeout_status = rt_mailbox_handle_timeout(current, timeout_timer, "Network I/O operation timed out");
    if (RT_FAILED(timeout_status)) {
        // Timeout occurred - cleanup epoll registration
        epoll_ctl(epoll_fd, EPOLL_CTL_DEL, fd, NULL);
        rt_pool_free(&g_io_source_pool_mgr, source);
        return timeout_status;
    }

    // Return the result stored by the event handler
    return current->io_status;
}

rt_status rt_net_listen(uint16_t port, int *fd_out) {
    if (!fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    // Create socket (synchronous, doesn't block)
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Set SO_REUSEADDR to avoid "Address already in use" errors
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        rt_status status = RT_ERROR(RT_ERR_IO, strerror(errno));
        close(fd);
        return status;
    }

    if (listen(fd, 5) < 0) {
        rt_status status = RT_ERROR(RT_ERR_IO, strerror(errno));
        close(fd);
        return status;
    }

    set_nonblocking(fd);
    *fd_out = fd;
    return RT_SUCCESS;
}

rt_status rt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms) {
    if (!conn_fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Try immediate accept with non-blocking
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int conn_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);

    if (conn_fd >= 0) {
        // Success immediately!
        set_nonblocking(conn_fd);
        *conn_fd_out = conn_fd;
        return RT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    rt_status status = try_or_epoll(listen_fd, EPOLLIN, NET_OP_ACCEPT, NULL, 0, timeout_ms);
    if (RT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *conn_fd_out = current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms) {
    if (!ip || !fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Parse numeric IPv4 address (DNS resolution not supported - would block scheduler)
    struct sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) != 1) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid IPv4 address (hostnames not supported)");
    }

    // Create non-blocking socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    set_nonblocking(fd);

    // Try non-blocking connect
    if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        if (errno != EINPROGRESS) {
            rt_status status = RT_ERROR(RT_ERR_IO, strerror(errno));
            close(fd);
            return status;
        }

        // Connection in progress - add to epoll and wait for writable
        rt_status status = try_or_epoll(fd, EPOLLOUT, NET_OP_CONNECT, NULL, 0, timeout_ms);
        if (RT_FAILED(status)) {
            close(fd);
            return status;
        }

        // Result stored by event handler
        *fd_out = current->io_result_fd;
        return RT_SUCCESS;
    }

    // Connected immediately (rare but possible on localhost)
    *fd_out = fd;
    return RT_SUCCESS;
}

rt_status rt_net_close(int fd) {
    // Close is synchronous and fast
    if (close(fd) < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }
    return RT_SUCCESS;
}

rt_status rt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms) {
    if (!buf || !received) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Try immediate non-blocking recv
    ssize_t n = recv(fd, buf, len, MSG_DONTWAIT);
    if (n >= 0) {
        // Success immediately!
        *received = (size_t)n;
        return RT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    rt_status status = try_or_epoll(fd, EPOLLIN, NET_OP_RECV, buf, len, timeout_ms);
    if (RT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *received = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms) {
    if (!buf || !sent) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_net.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Try immediate non-blocking send
    ssize_t n = send(fd, buf, len, MSG_DONTWAIT);
    if (n >= 0) {
        // Success immediately!
        *sent = (size_t)n;
        return RT_SUCCESS;
    }

    if (errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error (not just "would block")
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    // Would block - register interest in epoll and yield
    rt_status status = try_or_epoll(fd, EPOLLOUT, NET_OP_SEND, (void *)buf, len, timeout_ms);
    if (RT_FAILED(status)) {
        return status;
    }

    // Result stored by event handler
    *sent = current->io_result_nbytes;
    return RT_SUCCESS;
}
