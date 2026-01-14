#ifndef HIVE_STATIC_CONFIG_H
#define HIVE_STATIC_CONFIG_H

#include <stddef.h>

// =============================================================================
// Compile-Time Configuration for Static Memory Allocation
// =============================================================================
// All memory is allocated statically except for actor stacks.
// Edit these values and recompile to change system limits.

// -----------------------------------------------------------------------------
// Feature Toggles
// -----------------------------------------------------------------------------
// Enable/disable optional subsystems. Set to 0 to disable.

// Network I/O subsystem (BSD sockets on Linux, lwIP on STM32)
#ifndef HIVE_ENABLE_NET
#define HIVE_ENABLE_NET 1
#endif

// File I/O subsystem (POSIX on Linux, FATFS/littlefs on STM32)
#ifndef HIVE_ENABLE_FILE
#define HIVE_ENABLE_FILE 1
#endif

// -----------------------------------------------------------------------------
// Actor System Configuration
// -----------------------------------------------------------------------------

// Maximum number of concurrent actors
#ifndef HIVE_MAX_ACTORS
#define HIVE_MAX_ACTORS 64
#endif

// Default stack size for actors (can be overridden per actor)
#ifndef HIVE_DEFAULT_STACK_SIZE
#define HIVE_DEFAULT_STACK_SIZE 65536
#endif

// Stack arena size for actor stacks (when malloc_stack = false)
// Should be sized for peak actor count × average stack size
// Example: 20 actors × 64KB = 1.3 MB (with overhead)
#ifndef HIVE_STACK_ARENA_SIZE
#define HIVE_STACK_ARENA_SIZE (1 * 1024 * 1024)  // 1 MB default
#endif

// -----------------------------------------------------------------------------
// Mailbox and Message Configuration
// -----------------------------------------------------------------------------

// Size of global mailbox entry pool (shared by all actors)
// Each send consumes one entry until message is received
#ifndef HIVE_MAILBOX_ENTRY_POOL_SIZE
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 256
#endif

// Size of global message data pool (for message payloads)
// Messages are allocated from this pool
#ifndef HIVE_MESSAGE_DATA_POOL_SIZE
#define HIVE_MESSAGE_DATA_POOL_SIZE 256
#endif

// Maximum message size (bytes, includes 4-byte header)
#ifndef HIVE_MAX_MESSAGE_SIZE
#define HIVE_MAX_MESSAGE_SIZE 256
#endif

// -----------------------------------------------------------------------------
// Bus Configuration
// -----------------------------------------------------------------------------

// Maximum number of buses in the system
#ifndef HIVE_MAX_BUSES
#define HIVE_MAX_BUSES 32
#endif

// Maximum entries per bus (ring buffer size)
#ifndef HIVE_MAX_BUS_ENTRIES
#define HIVE_MAX_BUS_ENTRIES 64
#endif

// Maximum subscribers per bus
#ifndef HIVE_MAX_BUS_SUBSCRIBERS
#define HIVE_MAX_BUS_SUBSCRIBERS 32
#endif

// -----------------------------------------------------------------------------
// Link and Monitor Configuration
// -----------------------------------------------------------------------------

// Size of global link entry pool
// Each hive_link() call consumes two entries (bidirectional)
#ifndef HIVE_LINK_ENTRY_POOL_SIZE
#define HIVE_LINK_ENTRY_POOL_SIZE 128
#endif

// Size of global monitor entry pool
// Each hive_monitor() call consumes one entry
#ifndef HIVE_MONITOR_ENTRY_POOL_SIZE
#define HIVE_MONITOR_ENTRY_POOL_SIZE 128
#endif

// -----------------------------------------------------------------------------
// Timer Configuration
// -----------------------------------------------------------------------------

// Size of global timer entry pool
// Each hive_timer_after() or hive_timer_every() consumes one entry
#ifndef HIVE_TIMER_ENTRY_POOL_SIZE
#define HIVE_TIMER_ENTRY_POOL_SIZE 64
#endif

// -----------------------------------------------------------------------------
// I/O Source Pool Configuration
// -----------------------------------------------------------------------------

// Size of io_source pool for tracking pending I/O operations in event loop
// Each pending network I/O operation consumes one io_source until completed
// Defined in hive_io_source.h: HIVE_IO_SOURCE_POOL_SIZE = 128

// -----------------------------------------------------------------------------
// Timing Constants
// -----------------------------------------------------------------------------

// Microseconds per second (constant for time conversions)
#define HIVE_USEC_PER_SEC 1000000

// -----------------------------------------------------------------------------
// Scheduler Configuration
// -----------------------------------------------------------------------------

// Maximum epoll events to process per scheduler iteration
#ifndef HIVE_EPOLL_MAX_EVENTS
#define HIVE_EPOLL_MAX_EVENTS 64
#endif

// Epoll poll timeout in milliseconds (defensive wakeup interval)
#ifndef HIVE_EPOLL_POLL_TIMEOUT_MS
#define HIVE_EPOLL_POLL_TIMEOUT_MS 10
#endif

// -----------------------------------------------------------------------------
// Network Configuration
// -----------------------------------------------------------------------------

// TCP listen backlog (queued connections)
#ifndef HIVE_NET_LISTEN_BACKLOG
#define HIVE_NET_LISTEN_BACKLOG 5
#endif

// -----------------------------------------------------------------------------
// STM32 Flash File I/O Configuration
// -----------------------------------------------------------------------------
// These are defaults that can be overridden by board-specific -D flags.
// Flash sector addresses/sizes MUST be defined per-board (no defaults here).

// Ring buffer size for deferred writes (bytes)
// Larger buffer = more tolerance for flash write delays
#ifndef HIVE_FILE_RING_SIZE
#define HIVE_FILE_RING_SIZE (8 * 1024)  // 8 KB default
#endif

// Flash write block size (bytes)
// Smaller = lower latency, larger = more efficient
// Must be word-aligned (multiple of 4)
#ifndef HIVE_FILE_BLOCK_SIZE
#define HIVE_FILE_BLOCK_SIZE 256
#endif

// -----------------------------------------------------------------------------
// Logging Configuration
// -----------------------------------------------------------------------------

// Maximum log entry text payload size (bytes, excluding header)
#ifndef HIVE_LOG_MAX_ENTRY_SIZE
#define HIVE_LOG_MAX_ENTRY_SIZE 128
#endif

// Enable logging to stdout/stderr (for development/debugging)
// Default: enabled on Linux, disabled on STM32
// Override with -DHIVE_LOG_TO_STDOUT=0 or -DHIVE_LOG_TO_STDOUT=1
#ifndef HIVE_LOG_TO_STDOUT
#ifdef HIVE_PLATFORM_STM32
#define HIVE_LOG_TO_STDOUT 0
#else
#define HIVE_LOG_TO_STDOUT 1
#endif
#endif

// Enable logging to file (compile in file logging code)
// Default: enabled when HIVE_ENABLE_FILE=1, disabled otherwise
// File logging still requires hive_log_file_open() call at runtime
// Override with -DHIVE_LOG_TO_FILE=0 or -DHIVE_LOG_TO_FILE=1
#ifndef HIVE_LOG_TO_FILE
#if HIVE_ENABLE_FILE
#define HIVE_LOG_TO_FILE 1
#else
#define HIVE_LOG_TO_FILE 0
#endif
#endif

// Log file path
// Default: "/var/tmp/hive.log" on Linux, "/log" on STM32 (maps to flash sector)
// Override with -DHIVE_LOG_FILE_PATH='"/path/to/log"'
#ifndef HIVE_LOG_FILE_PATH
#ifdef HIVE_PLATFORM_STM32
#define HIVE_LOG_FILE_PATH "/log"
#else
#define HIVE_LOG_FILE_PATH "/var/tmp/hive.log"
#endif
#endif

#endif // HIVE_STATIC_CONFIG_H
