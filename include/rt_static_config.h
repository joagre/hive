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

// -----------------------------------------------------------------------------
// Mailbox and Message Configuration
// -----------------------------------------------------------------------------

// Size of global mailbox entry pool (shared by all actors)
// Each IPC_COPY send consumes one entry until message is received
#ifndef RT_MAILBOX_ENTRY_POOL_SIZE
#define RT_MAILBOX_ENTRY_POOL_SIZE 256
#endif

// Size of global message data pool (for IPC_COPY payloads)
// Messages are allocated from this pool
#ifndef RT_MESSAGE_DATA_POOL_SIZE
#define RT_MESSAGE_DATA_POOL_SIZE 256
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
// I/O Completion Queue Configuration
// -----------------------------------------------------------------------------

// Size of completion queues for each I/O subsystem (file, net, timer)
#ifndef RT_COMPLETION_QUEUE_SIZE
#define RT_COMPLETION_QUEUE_SIZE 64
#endif

#endif // RT_STATIC_CONFIG_H
