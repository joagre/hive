// Hardware Abstraction Layer - Network I/O
//
// Abstracts platform-specific socket operations:
// - Linux: BSD sockets
// - STM32: Stubs (future lwIP support)
//
// All operations are NON-BLOCKING. The common hive_net.c wrapper handles:
// - Subsystem initialization guards
// - Argument validation
// - Async operation management (event registration, timeouts, actor wakeup)
//
// HAL implementations only need to provide the low-level socket operations.

#ifndef HIVE_HAL_NET_H
#define HIVE_HAL_NET_H

#include "hive_types.h"
#include <stddef.h>
#include <stdint.h>

// Initialize network subsystem.
// Linux: No-op (sockets always available)
// STM32: Initialize lwIP stack (future)
hive_status hive_hal_net_init(void);

// Cleanup network subsystem.
// Linux: No-op
// STM32: Cleanup lwIP stack (future)
void hive_hal_net_cleanup(void);

// Create a TCP socket and set it to non-blocking mode.
// fd_out: Output socket file descriptor
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_net_socket(int *fd_out);

// Bind socket to a port (listen on all interfaces).
// fd: Socket file descriptor
// port: Port number to bind to
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_net_bind(int fd, uint16_t port);

// Start listening for connections.
// fd: Socket file descriptor
// backlog: Maximum pending connections
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_net_listen(int fd, int backlog);

// Accept a connection (non-blocking).
// listen_fd: Listening socket file descriptor
// conn_fd_out: Output connected socket file descriptor
// Returns: HIVE_SUCCESS if connection accepted
//          HIVE_ERR_WOULDBLOCK if no pending connections
//          Other error status on failure
hive_status hive_hal_net_accept(int listen_fd, int *conn_fd_out);

// Initiate a connection (non-blocking).
// fd: Socket file descriptor
// ip: IPv4 address string (e.g., "192.168.1.1")
// port: Port number to connect to
// Returns: HIVE_SUCCESS if connected immediately (rare, localhost)
//          HIVE_ERR_INPROGRESS if connection in progress (normal)
//          Other error status on failure
hive_status hive_hal_net_connect(int fd, const char *ip, uint16_t port);

// Check if async connect completed successfully.
// fd: Socket file descriptor
// Returns: HIVE_SUCCESS if connected
//          Error status if connection failed
hive_status hive_hal_net_connect_check(int fd);

// Close a socket.
// fd: Socket file descriptor
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_net_close(int fd);

// Receive data (non-blocking).
// fd: Socket file descriptor
// buf: Buffer to receive data
// len: Maximum bytes to receive
// received: Output - actual bytes received
// Returns: HIVE_SUCCESS if data received (received may be 0 for EOF)
//          HIVE_ERR_WOULDBLOCK if no data available
//          Other error status on failure
hive_status hive_hal_net_recv(int fd, void *buf, size_t len, size_t *received);

// Send data (non-blocking).
// fd: Socket file descriptor
// buf: Data to send
// len: Bytes to send
// sent: Output - actual bytes sent
// Returns: HIVE_SUCCESS if data sent
//          HIVE_ERR_WOULDBLOCK if send buffer full
//          Other error status on failure
hive_status hive_hal_net_send(int fd, const void *buf, size_t len,
                              size_t *sent);

#endif // HIVE_HAL_NET_H
