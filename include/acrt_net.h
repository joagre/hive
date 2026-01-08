#ifndef ACRT_NET_H
#define ACRT_NET_H

#include "acrt_types.h"
#include <stdint.h>

// Network operations
// timeout_ms: ACRT_TIMEOUT_NONBLOCKING (0) returns ACRT_ERR_WOULDBLOCK if would block
//             ACRT_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns ACRT_ERR_TIMEOUT if exceeded

// Listen on a port for incoming connections
acrt_status acrt_net_listen(uint16_t port, int *fd_out);

// Accept incoming connection
acrt_status acrt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms);

// Connect to remote server (ip must be numeric IPv4 address, e.g. "192.168.1.1")
acrt_status acrt_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms);

// Close network socket
acrt_status acrt_net_close(int fd);

// Receive data from socket
acrt_status acrt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms);

// Send data to socket
acrt_status acrt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms);

#endif // ACRT_NET_H
