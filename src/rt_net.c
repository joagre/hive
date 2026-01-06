#include "rt_net.h"
#include "rt_static_config.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_scheduler_wakeup.h"
#include "rt_runtime.h"
#include "rt_spsc.h"
#include "rt_log.h"
#include "rt_timer.h"
#include "rt_ipc.h"
#include "rt_pool.h"
#include "rt_internal.h"
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

// External declarations for timeout handling (pools from rt_ipc.c)
extern rt_pool g_mailbox_pool_mgr;
extern rt_pool g_message_pool_mgr;

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

// Static buffers for network I/O queues
static uint8_t g_net_request_buffer[sizeof(net_request) * RT_COMPLETION_QUEUE_SIZE];
static uint8_t g_net_completion_buffer[sizeof(net_completion) * RT_COMPLETION_QUEUE_SIZE];

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

// Poll file descriptor for readiness with 100ms timeout
// Returns: 1 = ready, 0 = timeout, -1 = error
static int poll_fd(int fd, bool for_write) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    struct timeval tv = { .tv_sec = 0, .tv_usec = RT_NET_SELECT_TIMEOUT_US };

    if (for_write) {
        return select(fd + 1, NULL, &fds, NULL, &tv);
    } else {
        return select(fd + 1, &fds, NULL, NULL, &tv);
    }
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
            struct timespec ts = {.tv_sec = 0, .tv_nsec = RT_WORKER_IDLE_SLEEP_NS};
            nanosleep(&ts, NULL);
            continue;
        }


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
                // For blocking accept, poll to avoid blocking other requests
                if (req.timeout_ms != 0) {
                    int ret = poll_fd(req.data.accept.fd, false);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue and process other requests
                        rt_spsc_push(&g_net_io.request_queue, &req);
                        continue;
                    }
                }

                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
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

                // Poll for connection completion
                int ret = poll_fd(fd, true);
                if (ret < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                    close(fd);
                    break;
                } else if (ret == 0) {
                    // Timeout - requeue and process other requests
                    rt_spsc_push(&g_net_io.request_queue, &req);
                    continue;
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
                // Poll to avoid blocking the worker thread
                if (req.timeout_ms != 0) {
                    int ret = poll_fd(req.data.rw.fd, false);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue and process other requests
                        rt_spsc_push(&g_net_io.request_queue, &req);
                        continue;
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
                // Poll to avoid blocking the worker thread
                if (req.timeout_ms != 0) {
                    int ret = poll_fd(req.data.rw.fd, true);
                    if (ret < 0) {
                        comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                        break;
                    } else if (ret == 0) {
                        // Timeout - requeue and process other requests
                        rt_spsc_push(&g_net_io.request_queue, &req);
                        continue;
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
            struct timespec ts = {.tv_sec = 0, .tv_nsec = RT_COMPLETION_RETRY_SLEEP_NS};
            nanosleep(&ts, NULL);
        }

        // Wake up scheduler to process completion
        rt_scheduler_wakeup_signal();
    }

    RT_LOG_DEBUG("Network I/O worker thread exiting");
    return NULL;
}

// Initialize network I/O subsystem
rt_status rt_net_init(void) {
    if (g_net_io.initialized) {
        return RT_SUCCESS;
    }

    // Initialize queues with static buffers (power of 2 capacity)
    rt_status status = rt_spsc_init(&g_net_io.request_queue, g_net_request_buffer,
                                     sizeof(net_request), RT_COMPLETION_QUEUE_SIZE);
    if (RT_FAILED(status)) {
        return status;
    }

    status = rt_spsc_init(&g_net_io.completion_queue, g_net_completion_buffer,
                          sizeof(net_completion), RT_COMPLETION_QUEUE_SIZE);
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

    // Create timeout timer if needed (like rt_ipc_recv)
    timer_id timeout_timer = TIMER_ID_INVALID;
    if (req->timeout_ms > 0) {
        rt_status status = rt_timer_after((uint32_t)req->timeout_ms * 1000, &timeout_timer);
        if (RT_FAILED(status)) {
            return status;  // Timer pool exhausted
        }
    }

    // Submit request
    while (!rt_spsc_push(&g_net_io.request_queue, req)) {
        // Request queue full, yield and try again
        rt_yield();
    }

    // Block waiting for completion
    current->state = ACTOR_STATE_BLOCKED;
    rt_yield();

    // When we wake up, check if timeout occurred (like rt_ipc_recv)
    if (timeout_timer != TIMER_ID_INVALID) {
        // Check if first message in mailbox is timer tick
        if (current->mbox.head && current->mbox.head->sender == RT_SENDER_TIMER) {
            // Timeout occurred - dequeue timer message
            mailbox_entry *entry = current->mbox.head;
            current->mbox.head = entry->next;
            if (current->mbox.head == NULL) {
                current->mbox.tail = NULL;
            }
            current->mbox.count--;

            // Free timer message resources
            if (entry->data) {
                message_data_entry *msg_data = DATA_TO_MSG_ENTRY(entry->data);
                rt_pool_free(&g_message_pool_mgr, msg_data);
            }
            rt_pool_free(&g_mailbox_pool_mgr, entry);

            return RT_ERROR(RT_ERR_TIMEOUT, "Network I/O operation timed out");
        } else {
            // I/O completed before timeout - cancel timer
            rt_timer_cancel(timeout_timer);
        }
    }

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
