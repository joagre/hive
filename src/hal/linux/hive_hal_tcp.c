// Hardware Abstraction Layer - Linux TCP Implementation
//
// TCP uses BSD sockets.

#include "hive_static_config.h"

#if HIVE_ENABLE_TCP

#include "hal/hive_hal_tcp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

// Set socket to non-blocking mode
static int set_socket_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        return -1;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

hive_status_t hive_hal_tcp_init(void) {
    // No initialization needed for BSD sockets
    return HIVE_SUCCESS;
}

void hive_hal_tcp_cleanup(void) {
    // No cleanup needed for BSD sockets
}

hive_status_t hive_hal_tcp_socket(int *out) {
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

    *out = fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_bind(int fd, uint16_t port) {
    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "bind failed");
    }

    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_listen(int fd, int backlog) {
    if (listen(fd, backlog) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "listen failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_accept(int listen_fd, int *out) {
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

    *out = conn_fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_connect(int fd, const char *ip, uint16_t port) {
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

hive_status_t hive_hal_tcp_connect_check(int fd) {
    int error = 0;
    socklen_t len = sizeof(error);

    if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "connect failed");
    }

    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_close(int fd) {
    if (close(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "close failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_recv(int fd, void *buf, size_t len,
                                size_t *bytes_read) {
    ssize_t n = recv(fd, buf, len, MSG_DONTWAIT);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "no data available");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "recv failed");
    }

    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_tcp_send(int fd, const void *buf, size_t len,
                                size_t *bytes_written) {
    ssize_t n = send(fd, buf, len, MSG_DONTWAIT);

    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "send buffer full");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "send failed");
    }

    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_TCP
