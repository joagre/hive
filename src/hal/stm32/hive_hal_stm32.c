// Hardware Abstraction Layer - STM32 Implementation
//
// Time functions using tick counter from hive_timer_stm32.c.
// Critical sections using PRIMASK interrupt disable.
// Event system uses software timer wheel + WFI for idle.

#include "hal/hive_hal_time.h"
#include "hal/hive_hal_event.h"
#include "hive_io_source.h"

// Get tick count from timer subsystem
extern uint32_t hive_timer_get_ticks(void);

// Process pending software timers (from hive_timer_stm32.c)
extern void hive_timer_process_pending(void);

// Tick period in microseconds (from hive_timer_stm32.c)
#ifndef HIVE_TIMER_TICK_US
#define HIVE_TIMER_TICK_US 1000
#endif

uint64_t hive_hal_get_time_us(void) {
    return (uint64_t)hive_timer_get_ticks() * HIVE_TIMER_TICK_US;
}

uint32_t hive_hal_critical_enter(void) {
    uint32_t primask;
    __asm__ volatile("mrs %0, primask\n"
                     "cpsid i\n"
                     : "=r"(primask)
                     :
                     : "memory");
    return primask;
}

void hive_hal_critical_exit(uint32_t state) {
    __asm__ volatile("msr primask, %0" : : "r"(state) : "memory");
}

// =============================================================================
// Event System Implementation (software timer + WFI)
// =============================================================================

hive_status hive_hal_event_init(void) {
    // No initialization needed - software timers are managed by timer subsystem
    return HIVE_SUCCESS;
}

void hive_hal_event_cleanup(void) {
    // No cleanup needed
}

void hive_hal_event_poll(void) {
    // Process any pending timer events
    hive_timer_process_pending();
}

void hive_hal_event_wait(int timeout_ms) {
    // Process pending timers first
    hive_timer_process_pending();

    // If timeout allows waiting, use WFI to sleep until interrupt
    if (timeout_ms != 0) {
        __asm__ volatile("wfi");
    }
}

hive_status hive_hal_event_register(int fd, uint32_t events,
                                    io_source *source) {
    // STM32 doesn't use file descriptors for timers - software timer wheel
    // This is a no-op but returns success for compatibility
    (void)fd;
    (void)events;
    (void)source;
    return HIVE_SUCCESS;
}

void hive_hal_event_unregister(int fd) {
    // No-op on STM32
    (void)fd;
}

// =============================================================================
// Network I/O Stubs (future lwIP support)
// =============================================================================

#if HIVE_ENABLE_NET

#include "hal/hive_hal_net.h"

hive_status hive_hal_net_init(void) {
    // Future: Initialize lwIP stack
    return HIVE_SUCCESS;
}

void hive_hal_net_cleanup(void) {
    // Future: Cleanup lwIP stack
}

hive_status hive_hal_net_socket(int *fd_out) {
    (void)fd_out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_bind(int fd, uint16_t port) {
    (void)fd;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_listen(int fd, int backlog) {
    (void)fd;
    (void)backlog;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_accept(int listen_fd, int *conn_fd_out) {
    (void)listen_fd;
    (void)conn_fd_out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_connect(int fd, const char *ip, uint16_t port) {
    (void)fd;
    (void)ip;
    (void)port;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_connect_check(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_close(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_recv(int fd, void *buf, size_t len, size_t *received) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)received;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

hive_status hive_hal_net_send(int fd, const void *buf, size_t len,
                              size_t *sent) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)sent;
    return HIVE_ERROR(HIVE_ERR_INVALID, "Network not supported on STM32");
}

#endif // HIVE_ENABLE_NET
