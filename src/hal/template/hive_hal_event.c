// Hardware Abstraction Layer - Event Template
//
// This file provides a documented template for event functions.
// Copy to src/hal/<platform>/hive_hal_event.c and implement.

#include "hal/hive_hal_event.h"
#include "hive_io_source.h"

// Forward declarations for event handlers (defined in hive core)
extern void hive_timer_handle_event(io_source_t *source);
#if HIVE_ENABLE_TCP
extern void hive_tcp_handle_event(io_source_t *source);
#endif

// Initialize event system.
//
// Called once during hive_init().
//
// Examples:
//   - Linux: Create epoll fd
//   - STM32: Initialize software timer wheel, configure SysTick
//   - Bare metal: Setup interrupt handlers
//
hive_status_t hive_hal_event_init(void) {
    // TODO: Implement for your platform
    return HIVE_SUCCESS;
}

// Cleanup event system.
//
// Called during hive_cleanup().
//
void hive_hal_event_cleanup(void) {
    // TODO: Implement for your platform
}

// Poll for events without blocking.
//
// Called by scheduler to check for I/O readiness.
// Must not block - return immediately even if no events.
//
// Actions to perform:
//   - Check all registered I/O sources for readiness
//   - Process expired timers
//   - Call appropriate handlers (hive_timer_handle_event, hive_tcp_handle_event)
//
void hive_hal_event_poll(void) {
    // TODO: Implement for your platform
    // Example: Check timer wheel, process any expired timers
}

// Wait for events with timeout.
//
// Parameters:
//   timeout_ms - Maximum time to wait:
//     < 0: Wait indefinitely
//     = 0: Poll only (no wait)
//     > 0: Wait up to timeout_ms milliseconds
//
// Called by scheduler when all actors are blocked.
//
// Examples:
//   - Linux: epoll_wait(epoll_fd, events, max, timeout_ms)
//   - STM32: WFI (Wait For Interrupt) then process timer wheel
//   - Bare metal: Sleep until interrupt, then check events
//
void hive_hal_event_wait(int timeout_ms) {
    // TODO: Implement for your platform
    // Example for STM32:
    //   hive_timer_process_pending();  // Process expired timers first
    //   if (timeout_ms != 0) {
    //       __asm__ volatile("wfi");    // Sleep until interrupt
    //   }
    (void)timeout_ms;
}

// Register an I/O source for event notification.
//
// Parameters:
//   fd - File descriptor or handle to watch
//   events - Events to watch for (HIVE_EVENT_READ, HIVE_EVENT_WRITE)
//   source - io_source_t pointer passed to handler when event occurs
//
// Called when an actor needs to wait for I/O.
// The source->type determines which handler to call when event fires.
//
// Examples:
//   - Linux: epoll_ctl(EPOLL_CTL_ADD, fd, ...)
//   - STM32 timers: Add to software timer wheel
//   - STM32 TCP: Add to lwIP event list
//
hive_status_t hive_hal_event_register(int fd, uint32_t events,
                                      io_source_t *source) {
    // TODO: Implement for your platform
    // For platforms without file descriptors (STM32 timers), this may be a no-op
    (void)fd;
    (void)events;
    (void)source;
    return HIVE_SUCCESS;
}

// Unregister an I/O source.
//
// Parameters:
//   fd - File descriptor or handle to stop watching
//
// Called when I/O operation completes or is cancelled.
//
void hive_hal_event_unregister(int fd) {
    // TODO: Implement for your platform
    (void)fd;
}
