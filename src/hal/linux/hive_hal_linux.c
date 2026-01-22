// Hardware Abstraction Layer - Linux Implementation
//
// Time functions using POSIX clock_gettime.
// Critical sections are no-ops (single-threaded, no ISRs).
// Event system uses epoll for efficient I/O multiplexing.
// File I/O uses POSIX file operations.
// Network I/O uses BSD sockets.

#include "hal/hive_hal_time.h"
#include "hal/hive_hal_event.h"
#include "hive_io_source.h"
#include "hive_static_config.h"
#include <time.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <errno.h>

#if HIVE_ENABLE_FILE
#include "hal/hive_hal_file.h"
#include "hive_file.h"
#include <fcntl.h>
#endif

#if HIVE_ENABLE_NET
#include "hal/hive_hal_net.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#ifndef HIVE_ENABLE_FILE
#include <fcntl.h>
#endif
#endif

// Forward declarations for event handlers
extern void hive_timer_handle_event(io_source *source);
#if HIVE_ENABLE_NET
extern void hive_net_handle_event(io_source *source);
#endif

// Event system state
static struct {
    int epoll_fd;
    bool initialized;
} s_event = {.epoll_fd = -1, .initialized = false};

uint64_t hive_hal_get_time_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

uint32_t hive_hal_critical_enter(void) {
    // No-op on Linux - single-threaded, no ISRs
    return 0;
}

void hive_hal_critical_exit(uint32_t state) {
    // No-op on Linux
    (void)state;
}

// =============================================================================
// Event System Implementation (epoll-based)
// =============================================================================

// Dispatch events from epoll
static void dispatch_epoll_events(int timeout_ms) {
    if (!s_event.initialized || s_event.epoll_fd < 0) {
        return;
    }

    struct epoll_event events[HIVE_EPOLL_MAX_EVENTS];
    int n =
        epoll_wait(s_event.epoll_fd, events, HIVE_EPOLL_MAX_EVENTS, timeout_ms);

    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;

        if (source->type == IO_SOURCE_TIMER) {
            hive_timer_handle_event(source);
        }
#if HIVE_ENABLE_NET
        else if (source->type == IO_SOURCE_NETWORK) {
            hive_net_handle_event(source);
        }
#endif
    }
}

hive_status hive_hal_event_init(void) {
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

hive_status hive_hal_event_register(int fd, uint32_t events,
                                    io_source *source) {
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

// =============================================================================
// File I/O Implementation (POSIX-based)
// =============================================================================

#if HIVE_ENABLE_FILE

// Map HIVE_O_* flags to POSIX O_* flags
static int hive_flags_to_posix(int hive_flags) {
    int posix_flags = 0;

    // Access mode (mutually exclusive)
    int access = hive_flags & 0x0003;
    if (access == HIVE_O_RDONLY)
        posix_flags |= O_RDONLY;
    else if (access == HIVE_O_WRONLY)
        posix_flags |= O_WRONLY;
    else if (access == HIVE_O_RDWR)
        posix_flags |= O_RDWR;

    // Additional flags
    if (hive_flags & HIVE_O_CREAT)
        posix_flags |= O_CREAT;
    if (hive_flags & HIVE_O_TRUNC)
        posix_flags |= O_TRUNC;
    if (hive_flags & HIVE_O_APPEND)
        posix_flags |= O_APPEND;

    return posix_flags;
}

hive_status hive_hal_file_init(void) {
    // No initialization needed for POSIX file I/O
    return HIVE_SUCCESS;
}

void hive_hal_file_cleanup(void) {
    // No cleanup needed for POSIX file I/O
}

hive_status hive_hal_file_open(const char *path, int flags, int mode,
                               int *fd_out) {
    int posix_flags = hive_flags_to_posix(flags);
    int fd = open(path, posix_flags, mode);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "open failed");
    }
    *fd_out = fd;
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_close(int fd) {
    if (close(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "close failed");
    }
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_read(int fd, void *buf, size_t len,
                               size_t *bytes_read) {
    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "read failed");
    }
    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                size_t *bytes_read) {
    ssize_t n = pread(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pread failed");
    }
    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_write(int fd, const void *buf, size_t len,
                                size_t *bytes_written) {
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "write failed");
    }
    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                 size_t offset, size_t *bytes_written) {
    ssize_t n = pwrite(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pwrite failed");
    }
    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_hal_file_sync(int fd) {
    if (fsync(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "fsync failed");
    }
    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_FILE

// =============================================================================
// Network I/O Implementation (BSD sockets)
// =============================================================================

#if HIVE_ENABLE_NET

// Set socket to non-blocking mode
static int set_socket_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

hive_status hive_hal_net_init(void) {
    // No initialization needed for BSD sockets
    return HIVE_SUCCESS;
}

void hive_hal_net_cleanup(void) {
    // No cleanup needed for BSD sockets
}

hive_status hive_hal_net_socket(int *fd_out) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "socket failed");
    }

    // Set non-blocking
    if (set_socket_nonblocking(fd) < 0) {
        close(fd);
        return HIVE_ERROR(HIVE_ERR_IO, "fcntl failed");
    }

    // Set SO_REUSEADDR to avoid "Address already in use" errors
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    *fd_out = fd;
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_bind(int fd, uint16_t port) {
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "bind failed");
    }

    return HIVE_SUCCESS;
}

hive_status hive_hal_net_listen(int fd, int backlog) {
    if (listen(fd, backlog) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "listen failed");
    }
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_accept(int listen_fd, int *conn_fd_out) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int conn_fd =
        accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);

    if (conn_fd < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "no pending connections");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "accept failed");
    }

    // Set new connection to non-blocking
    if (set_socket_nonblocking(conn_fd) < 0) {
        close(conn_fd);
        return HIVE_ERROR(HIVE_ERR_IO, "fcntl failed");
    }

    *conn_fd_out = conn_fd;
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_connect(int fd, const char *ip, uint16_t port) {
    struct sockaddr_in serv_addr = {0};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip, &serv_addr.sin_addr) != 1) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "Invalid IPv4 address (hostnames not supported)");
    }

    if (connect(fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        if (errno == EINPROGRESS) {
            return HIVE_ERROR(HIVE_ERR_INPROGRESS, "connection in progress");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "connect failed");
    }

    // Connected immediately (rare, usually localhost)
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_connect_check(int fd) {
    int error = 0;
    socklen_t len = sizeof(error);

    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "connect failed");
    }

    return HIVE_SUCCESS;
}

hive_status hive_hal_net_close(int fd) {
    if (close(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "close failed");
    }
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_recv(int fd, void *buf, size_t len, size_t *received) {
    ssize_t n = recv(fd, buf, len, MSG_DONTWAIT);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "no data available");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "recv failed");
    }

    *received = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_hal_net_send(int fd, const void *buf, size_t len,
                              size_t *sent) {
    ssize_t n = send(fd, buf, len, MSG_DONTWAIT);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "send buffer full");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "send failed");
    }

    *sent = (size_t)n;
    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_NET
