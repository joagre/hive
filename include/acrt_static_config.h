#ifndef ACRT_STATIC_CONFIG_H
#define ACRT_STATIC_CONFIG_H

#include <stddef.h>

// =============================================================================
// Compile-Time Configuration for Static Memory Allocation
// =============================================================================
// All memory is allocated statically except for actor stacks.
// Edit these values and recompile to change system limits.

// -----------------------------------------------------------------------------
// Actor System Configuration
// -----------------------------------------------------------------------------

// Maximum number of concurrent actors
#ifndef ACRT_MAX_ACTORS
#define ACRT_MAX_ACTORS 64
#endif

// Default stack size for actors (can be overridden per actor)
#ifndef ACRT_DEFAULT_STACK_SIZE
#define ACRT_DEFAULT_STACK_SIZE 65536
#endif

// Stack arena size for actor stacks (when malloc_stack = false)
// Should be sized for peak actor count × average stack size
// Example: 20 actors × 64KB = 1.3 MB (with overhead)
#ifndef ACRT_STACK_ARENA_SIZE
#define ACRT_STACK_ARENA_SIZE (1 * 1024 * 1024)  // 1 MB default
#endif

// -----------------------------------------------------------------------------
// Mailbox and Message Configuration
// -----------------------------------------------------------------------------

// Size of global mailbox entry pool (shared by all actors)
// Each send consumes one entry until message is received
#ifndef ACRT_MAILBOX_ENTRY_POOL_SIZE
#define ACRT_MAILBOX_ENTRY_POOL_SIZE 256
#endif

// Size of global message data pool (for message payloads)
// Messages are allocated from this pool
#ifndef ACRT_MESSAGE_DATA_POOL_SIZE
#define ACRT_MESSAGE_DATA_POOL_SIZE 256
#endif

// Maximum message size (bytes, includes 4-byte header)
#ifndef ACRT_MAX_MESSAGE_SIZE
#define ACRT_MAX_MESSAGE_SIZE 256
#endif

// -----------------------------------------------------------------------------
// Bus Configuration
// -----------------------------------------------------------------------------

// Maximum number of buses in the system
#ifndef ACRT_MAX_BUSES
#define ACRT_MAX_BUSES 32
#endif

// Maximum entries per bus (ring buffer size)
#ifndef ACRT_MAX_BUS_ENTRIES
#define ACRT_MAX_BUS_ENTRIES 64
#endif

// Maximum subscribers per bus
#ifndef ACRT_MAX_BUS_SUBSCRIBERS
#define ACRT_MAX_BUS_SUBSCRIBERS 32
#endif

// -----------------------------------------------------------------------------
// Link and Monitor Configuration
// -----------------------------------------------------------------------------

// Size of global link entry pool
// Each acrt_link() call consumes two entries (bidirectional)
#ifndef ACRT_LINK_ENTRY_POOL_SIZE
#define ACRT_LINK_ENTRY_POOL_SIZE 128
#endif

// Size of global monitor entry pool
// Each acrt_monitor() call consumes one entry
#ifndef ACRT_MONITOR_ENTRY_POOL_SIZE
#define ACRT_MONITOR_ENTRY_POOL_SIZE 128
#endif

// -----------------------------------------------------------------------------
// Timer Configuration
// -----------------------------------------------------------------------------

// Size of global timer entry pool
// Each acrt_timer_after() or acrt_timer_every() consumes one entry
#ifndef ACRT_TIMER_ENTRY_POOL_SIZE
#define ACRT_TIMER_ENTRY_POOL_SIZE 64
#endif

// -----------------------------------------------------------------------------
// I/O Source Pool Configuration
// -----------------------------------------------------------------------------

// Size of io_source pool for tracking pending I/O operations in event loop
// Each pending network I/O operation consumes one io_source until completed
// Defined in acrt_io_source.h: ACRT_IO_SOURCE_POOL_SIZE = 128

// -----------------------------------------------------------------------------
// Timing Constants
// -----------------------------------------------------------------------------

// Microseconds per second (constant for time conversions)
#define ACRT_USEC_PER_SEC 1000000

// -----------------------------------------------------------------------------
// Scheduler Configuration
// -----------------------------------------------------------------------------

// Maximum epoll events to process per scheduler iteration
#ifndef ACRT_EPOLL_MAX_EVENTS
#define ACRT_EPOLL_MAX_EVENTS 64
#endif

// Epoll poll timeout in milliseconds (defensive wakeup interval)
#ifndef ACRT_EPOLL_POLL_TIMEOUT_MS
#define ACRT_EPOLL_POLL_TIMEOUT_MS 10
#endif

// -----------------------------------------------------------------------------
// Network Configuration
// -----------------------------------------------------------------------------

// TCP listen backlog (queued connections)
#ifndef ACRT_NET_LISTEN_BACKLOG
#define ACRT_NET_LISTEN_BACKLOG 5
#endif

#endif // ACRT_STATIC_CONFIG_H
