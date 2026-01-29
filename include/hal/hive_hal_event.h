// Hardware Abstraction Layer - Event System
//
// Abstracts the platform event loop:
// - Linux: epoll-based event loop
// - STM32: Software timer processing + WFI
//
// Each platform provides:
// - hive_hal_event_init/cleanup() for lifecycle
// - hive_hal_event_poll/wait() for event processing
// - hive_hal_event_register/unregister() for I/O source registration

#ifndef HIVE_HAL_EVENT_H
#define HIVE_HAL_EVENT_H

#include "hive_types.h"

// Forward declaration (defined in hive_io_source.h)
typedef struct io_source_t io_source_t;

// Event flags for registration (platform-neutral)
#define HIVE_EVENT_READ 0x01  // fd is ready for reading (EPOLLIN on Linux)
#define HIVE_EVENT_WRITE 0x02 // fd is ready for writing (EPOLLOUT on Linux)

// Initialize the event system.
// Linux: Creates epoll instance
// STM32: No-op (software timers don't need init)
hive_status_t hive_hal_event_init(void);

// Cleanup the event system.
// Linux: Closes epoll instance
// STM32: No-op
void hive_hal_event_cleanup(void);

// Poll for pending events without blocking.
// Dispatches any ready events (timer, TCP, etc.)
// Linux: epoll_wait with timeout=0
// STM32: Process pending software timers
void hive_hal_event_poll(void);

// Wait for events with optional timeout.
// timeout_ms: -1 = block forever, 0 = same as poll, >0 = wait up to ms
// Dispatches any events that occur.
// Linux: epoll_wait with timeout
// STM32: Process timers, then WFI if timeout != 0
void hive_hal_event_wait(int timeout_ms);

// Register an I/O source with the event system.
// fd: File descriptor to monitor
// events: Event flags (HIVE_EVENT_READ, HIVE_EVENT_WRITE)
// source: io_source_t describing the event type and callback data
// Linux: Adds fd to epoll with specified events
// STM32: No-op (returns success but does nothing - timers are software)
hive_status_t hive_hal_event_register(int fd, uint32_t events,
                                      io_source_t *source);

// Unregister an I/O source from the event system.
// fd: File descriptor to remove
// Linux: Removes fd from epoll
// STM32: No-op
void hive_hal_event_unregister(int fd);

#endif // HIVE_HAL_EVENT_H
