#include "rt_file.h"
#include "rt_static_config.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_scheduler_wakeup.h"
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

// Forward declarations for internal functions
rt_status rt_file_init(void);
void rt_file_cleanup(void);
void rt_file_process_completions(void);

// File operation types
typedef enum {
    FILE_OP_OPEN,
    FILE_OP_CLOSE,
    FILE_OP_READ,
    FILE_OP_PREAD,
    FILE_OP_WRITE,
    FILE_OP_PWRITE,
    FILE_OP_SYNC,
} file_op_type;

// File operation request
typedef struct {
    file_op_type op;
    actor_id     requester;

    // Operation-specific data
    union {
        struct {
            char path[256];
            int  flags;
            int  mode;
        } open;

        struct {
            int fd;
        } close;

        struct {
            int    fd;
            void  *buf;
            size_t len;
            size_t offset;  // For pread/pwrite
        } rw;

        struct {
            int fd;
        } sync;
    } data;
} file_request;

// File operation completion
typedef struct {
    actor_id    requester;
    rt_status   status;

    // Result data
    union {
        int    fd;       // For open
        size_t nbytes;   // For read/write
    } result;
} file_completion;

// Static buffers for file I/O queues
static uint8_t g_file_request_buffer[sizeof(file_request) * RT_COMPLETION_QUEUE_SIZE];
static uint8_t g_file_completion_buffer[sizeof(file_completion) * RT_COMPLETION_QUEUE_SIZE];

// File I/O subsystem state
static struct {
    rt_spsc_queue  request_queue;
    rt_spsc_queue  completion_queue;
    pthread_t      worker_thread;
    bool           running;
    bool           initialized;
} g_file_io = {0};

// File I/O worker thread
static void *file_worker_thread(void *arg) {
    (void)arg;

    RT_LOG_DEBUG("File I/O worker thread started");

    while (g_file_io.running) {
        file_request req;

        // Try to get a request
        if (!rt_spsc_pop(&g_file_io.request_queue, &req)) {
            // No requests, sleep briefly
            struct timespec ts = {.tv_sec = 0, .tv_nsec = RT_WORKER_IDLE_SLEEP_NS};
            nanosleep(&ts, NULL);
            continue;
        }

        // Process request
        file_completion comp = {
            .requester = req.requester,
            .status = RT_SUCCESS
        };

        switch (req.op) {
            case FILE_OP_OPEN: {
                int fd = open(req.data.open.path, req.data.open.flags, req.data.open.mode);
                if (fd < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                } else {
                    comp.result.fd = fd;
                }
                break;
            }

            case FILE_OP_CLOSE: {
                if (close(req.data.close.fd) < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                }
                break;
            }

            case FILE_OP_READ: {
                ssize_t n = read(req.data.rw.fd, req.data.rw.buf, req.data.rw.len);
                if (n < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }

            case FILE_OP_PREAD: {
                ssize_t n = pread(req.data.rw.fd, req.data.rw.buf, req.data.rw.len, req.data.rw.offset);
                if (n < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }

            case FILE_OP_WRITE: {
                ssize_t n = write(req.data.rw.fd, req.data.rw.buf, req.data.rw.len);
                if (n < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }

            case FILE_OP_PWRITE: {
                ssize_t n = pwrite(req.data.rw.fd, req.data.rw.buf, req.data.rw.len, req.data.rw.offset);
                if (n < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                } else {
                    comp.result.nbytes = (size_t)n;
                }
                break;
            }

            case FILE_OP_SYNC: {
                if (fsync(req.data.sync.fd) < 0) {
                    comp.status = RT_ERROR(RT_ERR_IO, strerror(errno));
                }
                break;
            }
        }

        // Push completion
        while (!rt_spsc_push(&g_file_io.completion_queue, &comp)) {
            // Completion queue full, wait briefly
            struct timespec ts = {.tv_sec = 0, .tv_nsec = RT_COMPLETION_RETRY_SLEEP_NS};
            nanosleep(&ts, NULL);
        }

        // Wake up scheduler to process completion
        rt_scheduler_wakeup_signal();
    }

    RT_LOG_DEBUG("File I/O worker thread exiting");
    return NULL;
}

// Initialize file I/O subsystem
rt_status rt_file_init(void) {
    if (g_file_io.initialized) {
        return RT_SUCCESS;
    }

    // Initialize queues with static buffers (power of 2 capacity)
    rt_status status = rt_spsc_init(&g_file_io.request_queue, g_file_request_buffer,
                                     sizeof(file_request), RT_COMPLETION_QUEUE_SIZE);
    if (RT_FAILED(status)) {
        return status;
    }

    status = rt_spsc_init(&g_file_io.completion_queue, g_file_completion_buffer,
                          sizeof(file_completion), RT_COMPLETION_QUEUE_SIZE);
    if (RT_FAILED(status)) {
        rt_spsc_destroy(&g_file_io.request_queue);
        return status;
    }

    // Start worker thread
    g_file_io.running = true;
    if (pthread_create(&g_file_io.worker_thread, NULL, file_worker_thread, NULL) != 0) {
        rt_spsc_destroy(&g_file_io.request_queue);
        rt_spsc_destroy(&g_file_io.completion_queue);
        return RT_ERROR(RT_ERR_IO, "Failed to create file I/O worker thread");
    }

    g_file_io.initialized = true;
    return RT_SUCCESS;
}

// Cleanup file I/O subsystem
void rt_file_cleanup(void) {
    if (!g_file_io.initialized) {
        return;
    }

    // Stop worker thread
    g_file_io.running = false;
    pthread_join(g_file_io.worker_thread, NULL);

    // Cleanup queues
    rt_spsc_destroy(&g_file_io.request_queue);
    rt_spsc_destroy(&g_file_io.completion_queue);

    g_file_io.initialized = false;
}

// Process file completions (called by scheduler)
void rt_file_process_completions(void) {
    if (!g_file_io.initialized) {
        return;
    }

    file_completion comp;
    while (rt_spsc_pop(&g_file_io.completion_queue, &comp)) {
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

// Submit file operation and block
static rt_status submit_and_block(file_request *req) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    if (!g_file_io.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    req->requester = current->id;

    // Submit request
    while (!rt_spsc_push(&g_file_io.request_queue, req)) {
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

rt_status rt_file_open(const char *path, int flags, int mode, int *fd_out) {
    if (!path || !fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    file_request req = {
        .op = FILE_OP_OPEN,
    };

    strncpy(req.data.open.path, path, sizeof(req.data.open.path) - 1);
    req.data.open.flags = flags;
    req.data.open.mode = mode;

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *fd_out = current->io_result_fd;
    return RT_SUCCESS;
}

rt_status rt_file_close(int fd) {
    file_request req = {
        .op = FILE_OP_CLOSE,
        .data.close.fd = fd
    };

    return submit_and_block(&req);
}

rt_status rt_file_read(int fd, void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    file_request req = {
        .op = FILE_OP_READ,
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
    *actual = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    file_request req = {
        .op = FILE_OP_PREAD,
        .data.rw = {
            .fd = fd,
            .buf = buf,
            .len = len,
            .offset = offset
        }
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *actual = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_file_write(int fd, const void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    file_request req = {
        .op = FILE_OP_WRITE,
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
    *actual = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    file_request req = {
        .op = FILE_OP_PWRITE,
        .data.rw = {
            .fd = fd,
            .buf = (void *)buf,  // Cast away const for union
            .len = len,
            .offset = offset
        }
    };

    rt_status status = submit_and_block(&req);
    if (RT_FAILED(status)) {
        return status;
    }

    actor *current = rt_actor_current();
    *actual = current->io_result_nbytes;
    return RT_SUCCESS;
}

rt_status rt_file_sync(int fd) {
    file_request req = {
        .op = FILE_OP_SYNC,
        .data.sync.fd = fd
    };

    return submit_and_block(&req);
}
