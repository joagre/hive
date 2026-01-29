// Hardware Abstraction Layer - TCP I/O Template
//
// This file provides a documented template for TCP I/O functions.
// Copy to src/hal/<platform>/hive_hal_tcp.c and implement.
//
// Compile with HIVE_ENABLE_TCP=1 to enable.

#include "hive_static_config.h"

#if HIVE_ENABLE_TCP

#include "hal/hive_hal_tcp.h"

// Initialize TCP subsystem.
//
// Examples:
//   - Linux: No-op (BSD sockets always available)
//   - STM32: Initialize lwIP stack
//
hive_status_t hive_hal_tcp_init(void) {
    // TODO: Implement for your platform
    return HIVE_SUCCESS;
}

// Cleanup TCP subsystem.
void hive_hal_tcp_cleanup(void) {
    // TODO: Implement for your platform
}

// Create a TCP socket (non-blocking).
hive_status_t hive_hal_tcp_socket(int *out) {
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Bind socket to port.
hive_status_t hive_hal_tcp_bind(int fd, uint16_t port) {
    (void)fd;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Start listening for connections.
hive_status_t hive_hal_tcp_listen(int fd, int backlog) {
    (void)fd;
    (void)backlog;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Accept a connection (non-blocking).
//
// Returns:
//   HIVE_SUCCESS - Connection accepted, fd in out
//   HIVE_ERR_WOULDBLOCK - No pending connections
//   Other error on failure
//
hive_status_t hive_hal_tcp_accept(int listen_fd, int *out) {
    (void)listen_fd;
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Initiate a connection (non-blocking).
//
// Returns:
//   HIVE_SUCCESS - Connected immediately (rare, localhost)
//   HIVE_ERR_INPROGRESS - Connection in progress (normal)
//   Other error on failure
//
hive_status_t hive_hal_tcp_connect(int fd, const char *ip, uint16_t port) {
    (void)fd;
    (void)ip;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Check if async connect completed.
hive_status_t hive_hal_tcp_connect_check(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Close a socket.
hive_status_t hive_hal_tcp_close(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Receive data (non-blocking).
//
// Returns:
//   HIVE_SUCCESS - Data received (bytes_read=0 means EOF)
//   HIVE_ERR_WOULDBLOCK - No data available
//   Other error on failure
//
hive_status_t hive_hal_tcp_recv(int fd, void *buf, size_t len,
                                size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

// Send data (non-blocking).
//
// Returns:
//   HIVE_SUCCESS - Data sent
//   HIVE_ERR_WOULDBLOCK - Send buffer full
//   Other error on failure
//
hive_status_t hive_hal_tcp_send(int fd, const void *buf, size_t len,
                                size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not implemented");
}

#endif // HIVE_ENABLE_TCP
