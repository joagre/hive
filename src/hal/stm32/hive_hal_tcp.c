// Hardware Abstraction Layer - STM32 TCP I/O Implementation
//
// TCP I/O stubs (future lwIP support).

#include "hive_static_config.h"

#if HIVE_ENABLE_TCP

#include "hal/hive_hal_tcp.h"

hive_status_t hive_hal_tcp_init(void) {
    // Future: Initialize lwIP stack
    return HIVE_SUCCESS;
}

void hive_hal_tcp_cleanup(void) {
    // Future: Cleanup lwIP stack
}

hive_status_t hive_hal_tcp_socket(int *out) {
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_bind(int fd, uint16_t port) {
    (void)fd;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_listen(int fd, int backlog) {
    (void)fd;
    (void)backlog;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_accept(int listen_fd, int *out) {
    (void)listen_fd;
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_connect(int fd, const char *ip, uint16_t port) {
    (void)fd;
    (void)ip;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_connect_check(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_close(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_recv(int fd, void *buf, size_t len,
                                size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

hive_status_t hive_hal_tcp_send(int fd, const void *buf, size_t len,
                                size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "TCP not supported on STM32");
}

#endif // HIVE_ENABLE_TCP
