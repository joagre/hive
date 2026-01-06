#ifndef RT_NET_H
#define RT_NET_H

#include "rt_types.h"
#include <stdint.h>

// Network operations
// All operations with timeout parameter support:
//   timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if would block
//   timeout_ms < 0:   block forever
//   timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded

// Listen on a port for incoming connections
rt_status rt_net_listen(uint16_t port, int *fd_out);

// Accept incoming connection
rt_status rt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms);

// Connect to remote server (ip must be numeric IPv4 address, e.g. "192.168.1.1")
rt_status rt_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms);

// Close network socket
rt_status rt_net_close(int fd);

// Receive data from socket
rt_status rt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms);

// Send data to socket
rt_status rt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms);

#endif // RT_NET_H
