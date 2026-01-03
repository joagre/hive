#include "rt_net.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_spsc.h"
#include "rt_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>

// Forward declarations for internal functions
rt_status rt_net_init(void);
void rt_net_cleanup(void);
void rt_net_process_completions(void);

// Network operation types
typedef enum {
    NET_OP_LISTEN,
    NET_OP_ACCEPT,
    NET_OP_CONNECT,
    NET_OP_CLOSE,
    NET_OP_RECV,
    NET_OP_SEND,
} net_op_type;

// Network operation request
typedef struct {
    net_op_type op;
    actor_id    requester;
    int32_t     timeout_ms;

    // Operation-specific data
    union {
        struct {
            uint16_t port;
        } listen;

        struct {
            int fd;
        } accept;

        struct {
            char     host[256];
            uint16_t port;
            int      pending_fd;  // For tracking in-progress connection
        } connect;

        struct {
            int fd;
        } close;

        struct {
            int    fd;
            void  *buf;
            size_t len;
        } rw;
    } data;
} net_request;

// Network operation completion
typedef struct {
    actor_id    requester;
    rt_status   status;

    // Result data
    union {
        int    fd;       // For listen, accept, connect
        size_t nbytes;   // For recv/send
    } result;
} net_completion;

// Network I/O subsystem state
static struct {
    rt_spsc_queue  request_queue;
    rt_spsc_queue  completion_queue;
    pthread_t      worker_thread;
    bool           running;
    bool           initialized;
} g_net_io = {0};

// Set socket to non-blocking mode
static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Network I/O worker thread
static void *net_worker_thread(void *arg) {
    (void)arg;

    RT_LOG_DEBUG("Network I/O worker thread started");

    while (g_net_io.running) {
        net_request req;

        // Try to get a request
        if (!rt_spsc_pop(&g_net_io.request_queue, &req)) {
            // No requests, sleep briefly
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 1000000}; // 1ms
            nanosleep(&ts, NULL);
            continue;
        }

        RT_LOG_TRACE("Network worker: processing op=%d from requester=%u", req.op, req.requester);

        // Process request
        net_completion comp = {
            .requester = req.requester,
            .status = RT_SUCCESS
        };

        switch (req.op) {
            case NET_OP_LISTEN: {
                int fd = socket(AF_INET, SOCK_STREAM, 0);
                if (fd < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    break;
                }

                // Set SO_REUSEADDR to avoid "Address already in use" errors
                int opt = 1;
                setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

                struct sockaddr_in addr = {0};
                addr.sin_family = AF_INET;
                addr.sin_addr.s_addr = INADDR_ANY;
                addr.sin_port = htons(req.data.listen.port);

                if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    close(fd);
                    break;
                }

                if (listen(fd, 5) < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    close(fd);
                    break;
                }

                set_nonblocking(fd);
                comp.result.fd = fd;
                break;
            }

            case NET_OP_ACCEPT: {
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);

                // For blocking accept, use select with polling to avoid blocking other requests
                if (req.timeout_ms != 0) {
                    fd_set readfds;
                    FD_ZERO(&readfds);
                    FD_SET(req.data.accept.fd, &readfds);

                    // Use 100ms polling interval to allow processing other requests
                    struct timeval tv;
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000; // 100ms

                    RT_LOG_TRACE("Network worker: accept polling on fd=%d", req.data.accept.fd);
                    int ret = select(req.data.accept.fd + 1, &readfds, NULL, NULL, &tv);
                    RT_LOG_TRACE("Network worker: accept select returned %d", ret);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue request and process other requests
                        RT_LOG_TRACE("Network worker: accept timeout, requeuing");
                        if (!rt_spsc_push(&g_net_io.request_queue, &req)) {
                            RT_LOG_TRACE("Network worker: accept requeue failed - queue full");
                        }
                        continue; // Don't push completion, just loop
                    }
                }

                int conn_fd = accept(req.data.accept.fd, (struct sockaddr *)&client_addr, &client_len);
                if (conn_fd < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        comp.status = RT_ERROR(RT_ERR_WOULDBLOCK, "Would block");
                    } else {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    }
                    break;
                }

                set_nonblocking(conn_fd);
                comp.result.fd = conn_fd;
                break;
            }

            case NET_OP_CONNECT: {
                int fd;

                // Check if this is a retry (pending_fd already set)
                if (req.data.connect.pending_fd > 0) {
                    fd = req.data.connect.pending_fd;
                } else {
                    // First time: create socket and start connect
                    struct hostent *server = gethostbyname(req.data.connect.host);
                    if (!server) {
                        comp.status = RT_ERROR(RT_ERR_IO, "Host not found");
                        break;
                    }

                    fd = socket(AF_INET, SOCK_STREAM, 0);
                    if (fd < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    }

                    struct sockaddr_in serv_addr = {0};
                    serv_addr.sin_family = AF_INET;
                    memcpy(&serv_addr.sin_addr.s_addr, server->h_addr_list[0], server->h_length);
                    serv_addr.sin_port = htons(req.data.connect.port);

                    // Set non-blocking for connect
                    set_nonblocking(fd);

                    if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                        if (errno != EINPROGRESS) {
                            comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                            close(fd);
                            break;
                        }
                        // Connection in progress - store fd for polling
                        req.data.connect.pending_fd = fd;
                    } else {
                        // Connected immediately
                        comp.result.fd = fd;
                        break;
                    }
                }

                // Poll for connection completion with 100ms timeout
                fd_set writefds;
                FD_ZERO(&writefds);
                FD_SET(fd, &writefds);

                struct timeval tv;
                tv.tv_sec = 0;
                tv.tv_usec = 100000; // 100ms polling

                int ret = select(fd + 1, NULL, &writefds, NULL, &tv);
                if (ret < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    close(fd);
                    break;
                } else if (ret == 0) {
                    // Timeout - requeue request and process other requests
                    if (!rt_spsc_push(&g_net_io.request_queue, &req)) {
                        // Queue full
                    }
                    continue; // Don't push completion, just loop
                }

                // Check if connection succeeded
                int error = 0;
                socklen_t len = sizeof(error);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, error ? strerror(error) : "Connection failed");
                    close(fd);
                    break;
                }

                comp.result.fd = fd;
                break;
            }

            case NET_OP_CLOSE: {
                if (close(req.data.close.fd) < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                }
                break;
            }

            case NET_OP_RECV: {
                // For blocking recv, use select with timeout
                // Note: Even for infinite timeout (-1), we use polling to avoid blocking the worker thread
                if (req.timeout_ms != 0) {
                    fd_set readfds;
                    FD_ZERO(&readfds);
                    FD_SET(req.data.rw.fd, &readfds);

                    // Use 100ms polling interval for infinite timeout to avoid blocking other requests
                    struct timeval tv;
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000; // 100ms

                    int ret = select(req.data.rw.fd + 1, &readfds, NULL, NULL, &tv);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue request and try again later
                        if (!rt_spsc_push(&g_net_io.request_queue, &req)) {
                            // Queue full, skip this iteration
                        }
                        continue; // Don't push completion, just loop
                    }
                }

                ssize_t n = recv(req.data.rw.fd, req.data.rw.buf, req.data.rw.len, 0);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        comp.status = RT_ERROR(RT_ERR_WOULDBLOCK, "Would block");
                    } else {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    }
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }

            case NET_OP_SEND: {
                // For blocking send, use select with timeout
                // Note: Even for infinite timeout (-1), we use polling to avoid blocking the worker thread
                if (req.timeout_ms != 0) {
                    fd_set writefds;
                    FD_ZERO(&writefds);
                    FD_SET(req.data.rw.fd, &writefds);

                    // Use 100ms polling interval for infinite timeout to avoid blocking other requests
                    struct timeval tv;
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000; // 100ms

                    int ret = select(req.data.rw.fd + 1, NULL, &writefds, NULL, &tv);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue request and try again later
                        if (!rt_spsc_push(&g_net_io.request_queue, &req)) {
                            // Queue full, skip this iteration
                        }
                        continue; // Don't push completion, just loop
                    }
                }

                ssize_t n = send(req.data.rw.fd, req.data.rw.buf, req.data.rw.len, 0);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        comp.status = RT_ERROR(RT_ERR_WOULDBLOCK, "Would block");
                    } else {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    }
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }
        }

        // Push completion
        while (!rt_spsc_push(&g_net_io.completion_queue, &comp)) {
            // Completion queue full, wait briefly
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 100000}; // 100us
            nanosleep(&ts, NULL);
        }
        RT_LOG_TRACE("Network worker: pushed completion for requester=%u, status=%d",
                     comp.requester, comp.status.code);
    }

    RT_LOG_DEBUG("Network I/O worker thread exiting");
    return NULL;
}

// Initialize network I/O subsystem
rt_status rt_net_init(void) {
    if (g_net_io.initialized) {
        return RT_SUCCESS;
    }

    // Initialize queues (power of 2 capacity)
    rt_status status = rt_spsc_init(&g_net_io.request_queue, sizeof(net_request), 64);
    if (RT_FAILED(status)) {
        return status;
    }

    status = rt_spsc_init(&g_net_io.completion_queue, sizeof(net_completion), 64);
    if (RT_FAILED(status)) {
        rt_spsc_destroy(&g_net_io.request_queue);
        return status;
    }

    // Start worker thread
    g_net_io.running = true;
    if (pthread_create(&g_net_io.worker_thread, NULL, net_worker_thread, NULL) != 0) {
        rt_spsc_destroy(&g_net_io.request_queue);
        rt_spsc_destroy(&g_net_io.completion_queue);
        return RT_ERROR(RT_ERR_IO, "Failed to create network I/O worker thread");
    }

    g_net_io.initialized = true;
    return RT_SUCCESS;
}

// Cleanup network I/O subsystem
void rt_net_cleanup(void) {
    if (!g_net_io.initialized) {
        return;
    }

    // Stop worker thread
    g_net_io.running = false;
    pthread_join(g_net_io.worker_thread, NULL);

    // Cleanup queues
    rt_spsc_destroy(&g_net_io.request_queue);
    rt_spsc_destroy(&g_net_io.completion_queue);

    g_net_io.initialized = false;
}

// Process network completions (called by scheduler)
void rt_net_process_completions(void) {
    if (!g_net_io.initialized) {
        return;
    }

    net_completion comp;
    while (rt_spsc_pop(&g_net_io.completion_queue, &comp)) {
        // Find the actor that made the request
        actor *a = rt_actor_get(comp.requester);
        if (a && a->state == ACTOR_STATE_BLOCKED) {
            // Store completion result in actor
            a->io_status = comp.status;
            a->io_result_fd = comp.result.fd;
            a->io_result_nbytes = comp.result.nbytes;

            // Wake up the actor
            a->state = ACTOR_STATE_READY;
        }
    }
}

// Submit network operation and block
static rt_status submit_and_block(net_request *req) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    if (!g_net_io.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Network I/O subsystem not initialized");
    }

    req->requester = current->id;

    // Submit request
    while (!rt_spsc_push(&g_net_io.request_queue, req)) {
        // Request queue full, yield and try again
        rt_yield();
    }

    // Block waiting for completion
    current->state = ACTOR_STATE_BLOCKED;
    rt_yield();

    // When we wake up, the operation is complete
    // Return the result stored by the completion handler
    return current->io_status;
}

rt_status rt_net_listen(uint16_t port, int *fd_out) {
    if (!fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    net_request req = {
        .op = NET_OP_LISTEN,
        .data.listen.port = port
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *fd_out = current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms) {
    if (!conn_fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    net_request req = {
        .op = NET_OP_ACCEPT,
        .timeout_ms = timeout_ms,
        .data.accept.fd = listen_fd
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *conn_fd_out = current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_net_connect(const char *host, uint16_t port, int *fd_out, int32_t timeout_ms) {
    if (!host || !fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    net_request req = {
        .op = NET_OP_CONNECT,
        .timeout_ms = timeout_ms
    };

    strncpy(req.data.connect.host, host, sizeof(req.data.connect.host) - 1);
    req.data.connect.port = port;

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *fd_out = current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_net_close(int fd) {
    net_request req = {
        .op = NET_OP_CLOSE,
        .data.close.fd = fd
    };

    return submit_and_block(&req);
}

rt_status rt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms) {
    if (!buf || !received) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    net_request req = {
        .op = NET_OP_RECV,
        .timeout_ms = timeout_ms,
        .data.rw = {
            .fd = fd,
            .buf = buf,
            .len = len
        }
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *received = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms) {
    if (!buf || !sent) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    net_request req = {
        .op = NET_OP_SEND,
        .timeout_ms = timeout_ms,
        .data.rw = {
            .fd = fd,
            .buf = (void *)buf,  // Cast away const for union
            .len = len
        }
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *sent = current->io_result_nbytes;
    return RT_SUCCESS;
}
