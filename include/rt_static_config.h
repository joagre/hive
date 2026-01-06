#ifndef RT_STATIC_CONFIG_H
#define RT_STATIC_CONFIG_H

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
#ifndef RT_MAX_ACTORS
#define RT_MAX_ACTORS 64
#endif

// Default stack size for actors (can be overridden per actor)
#ifndef RT_DEFAULT_STACK_SIZE
#define RT_DEFAULT_STACK_SIZE 65536
#endif

// Stack arena size for actor stacks (when malloc_stack = false)
// Should be sized for peak actor count × average stack size
// Example: 20 actors × 64KB = 1.3 MB (with overhead)
#ifndef RT_STACK_ARENA_SIZE
#define RT_STACK_ARENA_SIZE (1 * 1024 * 1024)  // 1 MB default
#endif

// -----------------------------------------------------------------------------
// Mailbox and Message Configuration
// -----------------------------------------------------------------------------

// Size of global mailbox entry pool (shared by all actors)
// Each IPC_ASYNC send consumes one entry until message is received
#ifndef RT_MAILBOX_ENTRY_POOL_SIZE
#define RT_MAILBOX_ENTRY_POOL_SIZE 256
#endif

// Size of global message data pool (for IPC_ASYNC payloads)
// Messages are allocated from this pool
#ifndef RT_MESSAGE_DATA_POOL_SIZE
#define RT_MESSAGE_DATA_POOL_SIZE 256
#endif

// Size of global sync buffer pool (for IPC_SYNC payloads)
// Each concurrent IPC_SYNC send consumes one buffer until released
// Separate pool prevents IPC_ASYNC/SYNC contention
#ifndef RT_SYNC_BUFFER_POOL_SIZE
#define RT_SYNC_BUFFER_POOL_SIZE 64
#endif

// Maximum message size (bytes)
#ifndef RT_MAX_MESSAGE_SIZE
#define RT_MAX_MESSAGE_SIZE 256
#endif

// -----------------------------------------------------------------------------
// Bus Configuration
// -----------------------------------------------------------------------------

// Maximum number of buses in the system
#ifndef RT_MAX_BUSES
#define RT_MAX_BUSES 32
#endif

// Maximum entries per bus (ring buffer size)
#ifndef RT_MAX_BUS_ENTRIES
#define RT_MAX_BUS_ENTRIES 64
#endif

// Maximum subscribers per bus
#ifndef RT_MAX_BUS_SUBSCRIBERS
#define RT_MAX_BUS_SUBSCRIBERS 32
#endif

// -----------------------------------------------------------------------------
// Link and Monitor Configuration
// -----------------------------------------------------------------------------

// Size of global link entry pool
// Each rt_link() call consumes two entries (bidirectional)
#ifndef RT_LINK_ENTRY_POOL_SIZE
#define RT_LINK_ENTRY_POOL_SIZE 128
#endif

// Size of global monitor entry pool
// Each rt_monitor() call consumes one entry
#ifndef RT_MONITOR_ENTRY_POOL_SIZE
#define RT_MONITOR_ENTRY_POOL_SIZE 128
#endif

// -----------------------------------------------------------------------------
// Timer Configuration
// -----------------------------------------------------------------------------

// Size of global timer entry pool
// Each rt_timer_after() or rt_timer_every() consumes one entry
#ifndef RT_TIMER_ENTRY_POOL_SIZE
#define RT_TIMER_ENTRY_POOL_SIZE 64
#endif

// -----------------------------------------------------------------------------
// I/O Source Pool Configuration
// -----------------------------------------------------------------------------

// Size of io_source pool for tracking pending I/O operations in event loop
// Each pending network I/O operation consumes one io_source until completed
// Defined in rt_io_source.h: RT_IO_SOURCE_POOL_SIZE = 128

// -----------------------------------------------------------------------------
// Timing Constants
// -----------------------------------------------------------------------------

// Microseconds per second (constant for time conversions)
#define RT_USEC_PER_SEC 1000000

#endif // RT_STATIC_CONFIG_H
