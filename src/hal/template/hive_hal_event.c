// Hardware Abstraction Layer - Event Template
//
// This file provides a documented template for event functions.
// Copy to src/hal/<platform>/hive_hal_event.c and implement.

#include "hal/hive_hal_event.h"
#include "hive_io_source.h"
#include <stdbool.h>
#include <stdint.h>

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

// ============================================================================
// HAL Event Signaling (ISR-to-actor communication)
// ============================================================================
//
// These functions enable interrupt service routines (ISRs) to signal actors
// waiting in hive_select(). This allows efficient, interrupt-driven I/O
// without polling.
//
// Usage pattern:
//   1. Actor calls hive_hal_event_create() to get an event ID
//   2. Actor waits using hive_select() with HIVE_SEL_HAL_EVENT source
//   3. ISR calls hive_hal_event_signal(id) when hardware event occurs
//   4. Scheduler wakes actor, actor processes the event
//   5. hive_select() automatically clears the event flag
//
// Implementation uses a 32-bit bitmask for up to 32 events.
// Signal must be ISR-safe (single atomic store on ARM Cortex-M).
// Clear must use interrupt disable/enable to prevent races.

// Event state - static storage for event flags and allocation tracking
static volatile uint32_t s_event_flags = 0; // Which events are signaled
static uint32_t s_event_allocated = 0;      // Which event IDs are in use

// Create a new HAL event.
//
// Returns:
//   Event ID (0-31) on success
//   HIVE_HAL_EVENT_INVALID if pool exhausted
//
// Called from actor context during initialization.
//
hive_hal_event_id_t hive_hal_event_create(void) {
    // Find first free bit
    for (uint8_t i = 0; i < HIVE_HAL_EVENT_MAX; i++) {
        if (!(s_event_allocated & (1U << i))) {
            s_event_allocated |= (1U << i);
            return i;
        }
    }
    return HIVE_HAL_EVENT_INVALID;
}

// Destroy a HAL event and return it to the pool.
//
// Parameters:
//   id - Event ID to destroy
//
// Called from actor context during cleanup.
//
void hive_hal_event_destroy(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        s_event_allocated &= ~(1U << id);
        s_event_flags &= ~(1U << id);
    }
}

// Signal an event (ISR-SAFE).
//
// Parameters:
//   id - Event ID to signal
//
// **CRITICAL: This function MUST be callable from ISR context!**
//
// Implementation requirements:
//   - ARM Cortex-M: Single 32-bit store is atomic, no special handling needed
//   - x86-64/Linux: Use atomic_fetch_or() or equivalent
//   - Other platforms: Ensure single-instruction flag set
//
// This function sets a flag that will wake actors waiting in hive_select().
// The scheduler checks these flags in hive_hal_event_wait() or scan_sources().
//
void hive_hal_event_signal(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        // TODO: Ensure this is atomic on your platform
        // On ARM Cortex-M: Single store is atomic, this is fine
        // On x86-64: Use atomic_fetch_or(&s_event_flags, 1U << id, ...)
        s_event_flags |= (1U << id);
    }
}

// Check if an event is set (non-blocking).
//
// Parameters:
//   id - Event ID to check
//
// Returns:
//   true if event is signaled, false otherwise
//
// Called from actor context (scheduler) to check event state.
//
bool hive_hal_event_is_set(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        return (s_event_flags & (1U << id)) != 0;
    }
    return false;
}

// Clear an event flag.
//
// Parameters:
//   id - Event ID to clear
//
// Called from actor context after event is consumed.
//
// **CRITICAL: Must prevent race with ISR calling signal()!**
//
// Implementation requirements:
//   - ARM Cortex-M: Disable interrupts around read-modify-write
//   - x86-64/Linux: Use atomic_fetch_and() or equivalent
//   - Other platforms: Ensure atomic clear
//
void hive_hal_event_clear(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        // TODO: Make this atomic on your platform
        // On ARM Cortex-M:
        //   __asm__ volatile("cpsid i" ::: "memory");
        //   s_event_flags &= ~(1U << id);
        //   __asm__ volatile("cpsie i" ::: "memory");
        // On x86-64:
        //   atomic_fetch_and(&s_event_flags, ~(1U << id), ...)
        s_event_flags &= ~(1U << id);
    }
}
