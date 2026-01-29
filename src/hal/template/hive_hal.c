// Hardware Abstraction Layer - Template for New Ports
//
// This file provides a documented template for porting the Hive runtime
// to a new platform. Copy this file to src/hal/<platform>/hive_hal_<platform>.c
// and implement each function for your target hardware.
//
// Required sections (must implement all):
//   1. Time Functions - Microsecond time and critical sections
//   2. Event Functions - Event loop primitives
//
// Optional sections (implement if needed):
//   3. File I/O - Enable with HIVE_ENABLE_FILE=1
//   4. TCP I/O - Enable with HIVE_ENABLE_TCP=1
//
// See also:
//   - hive_hal_context_defs.h - Context struct definition (required)
//   - hive_context_<arch>.S - Context switch assembly (required)
//   - README.md in this directory for porting guide

#include "hal/hive_hal_time.h"
#include "hal/hive_hal_event.h"
#include "hive_io_source.h"

// =============================================================================
// SECTION 1: TIME FUNCTIONS (Required)
// =============================================================================

// Get current monotonic time in microseconds.
//
// Requirements:
//   - Must be monotonically increasing (never go backwards)
//   - Resolution should be at least 1ms (microsecond preferred)
//   - Should not wrap during runtime (64-bit provides ~584,000 years)
//
// Examples:
//   - Linux: clock_gettime(CLOCK_MONOTONIC)
//   - STM32: SysTick counter or hardware timer
//   - Bare metal: Hardware timer with known frequency
//
uint64_t hive_hal_get_time_us(void) {
    // TODO: Implement for your platform
    // Example for a platform with a tick counter:
    //   extern uint32_t get_tick_count(void);
    //   return (uint64_t)get_tick_count() * TICK_PERIOD_US;
    return 0;
}

// Enter critical section (disable interrupts).
//
// Requirements:
//   - Must disable interrupts that could call runtime APIs
//   - Must return state that allows nesting (save previous state)
//   - Must be fast (called frequently in hot paths)
//
// Returns:
//   Previous interrupt state to restore in hive_hal_critical_exit()
//
// Examples:
//   - Linux (single-threaded): Return 0 (no-op)
//   - ARM Cortex-M: Save PRIMASK, then CPSID I
//   - RISC-V: Save and clear MIE bit
//
uint32_t hive_hal_critical_enter(void) {
    // TODO: Implement for your platform
    // Example for ARM Cortex-M:
    //   uint32_t primask;
    //   __asm__ volatile("mrs %0, primask\n" "cpsid i\n" : "=r"(primask) :: "memory");
    //   return primask;
    return 0;
}

// Exit critical section (restore interrupts).
//
// Parameters:
//   state - Value returned from hive_hal_critical_enter()
//
// Requirements:
//   - Must restore previous interrupt state (supports nesting)
//   - Must be fast
//
void hive_hal_critical_exit(uint32_t state) {
    // TODO: Implement for your platform
    // Example for ARM Cortex-M:
    //   __asm__ volatile("msr primask, %0" :: "r"(state) : "memory");
    (void)state;
}

// =============================================================================
// SECTION 2: EVENT FUNCTIONS (Required)
// =============================================================================

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
// Called when an actor_t needs to wait for I/O.
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

// =============================================================================
// SECTION 3: FILE I/O (Optional - compile with HIVE_ENABLE_FILE=1)
// =============================================================================

#if HIVE_ENABLE_FILE

#include "hal/hive_hal_file.h"

// Initialize file I/O subsystem.
hive_status_t hive_hal_file_init(void) {
    // TODO: Implement for your platform
    // Examples:
    //   - Linux: No-op (POSIX always available)
    //   - STM32: Initialize flash file system
    return HIVE_SUCCESS;
}

// Cleanup file I/O subsystem.
void hive_hal_file_cleanup(void) {
    // TODO: Implement for your platform
}

// Open a file.
//
// Parameters:
//   path - File path
//   flags - Open flags (HIVE_O_RDONLY, HIVE_O_WRONLY, HIVE_O_CREAT, etc.)
//   mode - File permissions (for HIVE_O_CREAT)
//   out - Output file descriptor
//
// Note: STM32 only supports virtual paths like "/log", "/config"
//
hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *out) {
    (void)path;
    (void)flags;
    (void)mode;
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Close a file.
hive_status_t hive_hal_file_close(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Read from file (sequential).
hive_status_t hive_hal_file_read(int fd, void *buf, size_t len,
                                 size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Read from file at offset (random access).
hive_status_t hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                  size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Write to file (sequential).
hive_status_t hive_hal_file_write(int fd, const void *buf, size_t len,
                                  size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Write to file at offset (random access).
hive_status_t hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                   size_t offset, size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Sync file to storage.
hive_status_t hive_hal_file_sync(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

#endif // HIVE_ENABLE_FILE

// =============================================================================
// SECTION 4: TCP I/O (Optional - compile with HIVE_ENABLE_TCP=1)
// =============================================================================

#if HIVE_ENABLE_TCP

#include "hal/hive_hal_tcp.h"

// Initialize TCP subsystem.
hive_status_t hive_hal_tcp_init(void) {
    // TODO: Implement for your platform
    // Examples:
    //   - Linux: No-op (BSD sockets always available)
    //   - STM32: Initialize lwIP stack
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
