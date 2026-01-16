#ifndef HIVE_NET_H
#define HIVE_NET_H

#include "hive_types.h"
#include <stdint.h>

// Network operations
// timeout_ms: HIVE_TIMEOUT_NONBLOCKING (0) returns HIVE_ERR_WOULDBLOCK if would
// block
//             HIVE_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns HIVE_ERR_TIMEOUT if
//             exceeded

// Listen on a port for incoming connections
hive_status hive_net_listen(uint16_t port, int *fd_out);

// Accept incoming connection
hive_status hive_net_accept(int listen_fd, int *conn_fd_out,
                            int32_t timeout_ms);

// Connect to remote server (ip must be numeric IPv4 address, e.g.
// "192.168.1.1")
hive_status hive_net_connect(const char *ip, uint16_t port, int *fd_out,
                             int32_t timeout_ms);

// Close network socket
hive_status hive_net_close(int fd);

// Receive data from socket
hive_status hive_net_recv(int fd, void *buf, size_t len, size_t *received,
                          int32_t timeout_ms);

// Send data to socket
hive_status hive_net_send(int fd, const void *buf, size_t len, size_t *sent,
                          int32_t timeout_ms);

#endif // HIVE_NET_H
