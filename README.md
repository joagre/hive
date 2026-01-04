# Actor Runtime for Embedded Systems

A complete actor-based runtime with a minimalistic design philosophy. Features cooperative multitasking and message passing inspired by Erlang's actor model.

**Current platform:** x86-64 Linux (fully implemented)
**Future platform:** STM32/ARM Cortex-M with FreeRTOS (see `spec.md`)

The runtime is minimalistic by design: predictable behavior, no heap allocation in hot paths, and fast context switching via manual assembly. Despite this simplicity, it provides a complete actor system with linking, monitoring, timers, networking, and file I/O.

## Features

- ✅ **Static memory allocation** - Deterministic footprint, zero fragmentation, safety-critical ready
- ✅ Cooperative multitasking with manual x86-64 context switching
- ✅ Priority-based round-robin scheduler (4 priority levels)
- ✅ Actor lifecycle management (spawn, exit)
- ✅ IPC with COPY and BORROW modes
- ✅ Blocking and non-blocking message receive
- ✅ Actor linking and monitoring (bidirectional links, unidirectional monitors)
- ✅ Exit notifications with exit reasons (normal, crash, killed)
- ✅ Timers (one-shot and periodic with timerfd/epoll)
- ✅ Network I/O (async TCP with worker thread)
- ✅ File I/O (async read/write with worker thread)
- ✅ Bus (pub-sub with retention policies)

## Building

```bash
make
```

## Running Examples

```bash
# Basic IPC example
./build/pingpong

# Actor linking example (bidirectional links)
./build/link_demo

# Supervisor pattern example (monitoring workers)
./build/supervisor

# File I/O example
./build/fileio

# Network echo server (listens on port 8080)
./build/echo

# Timer example (one-shot and periodic)
./build/timer

# Bus pub-sub example
./build/bus
```

## Project Structure

```
.
├── include/          # Public headers
│   ├── rt_types.h          # Core types and error handling
│   ├── rt_static_config.h  # Compile-time configuration and limits
│   ├── rt_pool.h           # Memory pool allocator
│   ├── rt_context.h        # Context switching interface
│   ├── rt_actor.h          # Actor management
│   ├── rt_ipc.h            # Inter-process communication
│   ├── rt_link.h           # Actor linking and monitoring
│   ├── rt_scheduler.h      # Scheduler interface
│   ├── rt_runtime.h        # Runtime initialization and public API
│   ├── rt_timer.h          # Timer subsystem
│   ├── rt_file.h           # File I/O subsystem
│   ├── rt_net.h            # Network I/O subsystem
│   ├── rt_bus.h            # Bus pub-sub subsystem
│   ├── rt_spsc.h           # Lock-free SPSC queue
│   └── rt_log.h            # Logging utilities
├── src/              # Implementation
│   ├── rt_pool.c        # Pool allocator implementation
│   ├── rt_actor.c       # Actor table and lifecycle
│   ├── rt_context.c     # Context initialization
│   ├── rt_context_asm.S # x86-64 context switch assembly
│   ├── rt_ipc.c         # Mailbox and message passing with pools
│   ├── rt_link.c        # Linking and monitoring implementation
│   ├── rt_runtime.c     # Runtime init and actor spawning
│   ├── rt_scheduler.c   # Cooperative scheduler
│   ├── rt_timer.c       # Timer worker thread (timerfd/epoll)
│   ├── rt_file.c        # File I/O worker thread
│   ├── rt_net.c         # Network I/O worker thread
│   ├── rt_bus.c         # Bus pub-sub implementation
│   ├── rt_spsc.c        # Lock-free queue implementation
│   └── rt_log.c         # Logging implementation
├── examples/         # Example programs
│   ├── pingpong.c       # Classic ping-pong actor example
│   ├── link_demo.c      # Actor linking example
│   ├── supervisor.c     # Supervisor pattern with monitoring
│   ├── timer.c          # Timer example (one-shot and periodic)
│   ├── fileio.c         # File I/O example
│   ├── bus.c            # Bus pub-sub example
│   └── echo.c           # Network echo server
├── build/            # Build artifacts (generated)
├── Makefile          # Build system
├── spec.md           # Full specification
├── CLAUDE.md         # Development guide for Claude Code
└── README.md         # This file
```

## Quick Start

### Basic Actor Example

```c
#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>

void my_actor(void *arg) {
    printf("Hello from actor %u\n", rt_self());
    rt_exit();
}

int main(void) {
    rt_config cfg = RT_CONFIG_DEFAULT;
    rt_init(&cfg);

    rt_spawn(my_actor, NULL);

    rt_run();
    rt_cleanup();

    return 0;
}
```

### Sending Messages

```c
// Send a COPY message (async)
int data = 42;
rt_ipc_send(target_actor, &data, sizeof(data), IPC_COPY);

// Send a BORROW message (zero-copy, blocks until consumed)
int data = 42;
rt_ipc_send(target_actor, &data, sizeof(data), IPC_BORROW);
```

### Receiving Messages

```c
// Blocking receive
rt_message msg;
rt_ipc_recv(&msg, -1);  // Block until message arrives
printf("Received %zu bytes from actor %u\n", msg.len, msg.sender);

// Non-blocking receive
if (rt_ipc_recv(&msg, 0) == RT_OK) {
    // Process message
}
```

### Using Timers

```c
#include "rt_timer.h"

// One-shot timer (fires once after delay)
timer_id timer;
rt_timer_after(500000, &timer);  // 500ms delay

// Periodic timer (fires repeatedly)
timer_id periodic;
rt_timer_every(200000, &periodic);  // Every 200ms

// Receive timer ticks
rt_message msg;
rt_ipc_recv(&msg, -1);
if (rt_timer_is_tick(&msg)) {
    timer_id *tick_id = (timer_id *)msg.data;
    printf("Timer %u fired\n", *tick_id);
}

// Cancel timer
rt_timer_cancel(periodic);
```

### File I/O

```c
#include "rt_file.h"

// Open file
int fd;
rt_file_open("test.txt", O_RDWR | O_CREAT, 0644, &fd);

// Write data
const char *data = "Hello, world!";
size_t written;
rt_file_write(fd, data, strlen(data), &written);

// Read data
char buffer[128];
size_t nread;
rt_file_read(fd, buffer, sizeof(buffer), &nread);

// Close file
rt_file_close(fd);
```

### Network I/O

```c
#include "rt_net.h"

// Create listening socket
int listen_fd;
rt_net_listen(8080, &listen_fd);

// Accept connection (blocking)
int client_fd;
rt_net_accept(listen_fd, &client_fd, -1);

// Receive data (blocking)
char buffer[1024];
size_t received;
rt_net_recv(client_fd, buffer, sizeof(buffer), &received, -1);

// Send data (blocking)
size_t sent;
rt_net_send(client_fd, buffer, received, &sent, -1);

// Close socket
rt_net_close(client_fd);
```

### Actor Linking and Monitoring

```c
#include "rt_link.h"

// Bidirectional linking - both actors receive exit notifications
actor_id other = rt_spawn(other_actor, NULL);
rt_link(other);  // Link to the other actor

// Receive exit notification when linked actor dies
rt_message msg;
rt_ipc_recv(&msg, -1);
if (rt_is_exit_msg(&msg)) {
    rt_exit_msg exit_info;
    rt_decode_exit(&msg, &exit_info);
    printf("Actor %u died, reason: %d\n", exit_info.actor, exit_info.reason);
    // exit_info.reason: RT_EXIT_NORMAL, RT_EXIT_CRASH, or RT_EXIT_KILLED
}

// Unidirectional monitoring - only monitor receives exit notification
uint32_t monitor_ref;
rt_monitor(other, &monitor_ref);  // Monitor the other actor

// Stop monitoring
rt_demonitor(monitor_ref);

// Unlink (remove bidirectional link)
rt_unlink(other);
```

## API Overview

### Runtime Initialization

- `rt_init(config)` - Initialize the runtime
- `rt_run()` - Run the scheduler (blocks until all actors exit)
- `rt_cleanup()` - Cleanup and free resources
- `rt_shutdown()` - Request graceful shutdown

### Actor Management

- `rt_spawn(fn, arg)` - Spawn actor with default config
- `rt_spawn_ex(fn, arg, config)` - Spawn actor with custom config
- `rt_exit()` - Terminate current actor
- `rt_self()` - Get current actor's ID
- `rt_yield()` - Voluntarily yield to scheduler

### IPC

- `rt_ipc_send(to, data, len, mode)` - Send message
- `rt_ipc_recv(msg, timeout)` - Receive message
- `rt_ipc_release(msg)` - Release borrowed message
- `rt_ipc_pending()` - Check if messages are available
- `rt_ipc_count()` - Get number of pending messages

### Linking and Monitoring

- `rt_link(target)` - Create bidirectional link
- `rt_unlink(target)` - Remove bidirectional link
- `rt_monitor(target, out_ref)` - Create unidirectional monitor
- `rt_demonitor(ref)` - Remove monitor
- `rt_is_exit_msg(msg)` - Check if message is exit notification
- `rt_decode_exit(msg, out)` - Extract exit information

### Timers

- `rt_timer_after(delay_us, out)` - Create one-shot timer
- `rt_timer_every(interval_us, out)` - Create periodic timer
- `rt_timer_cancel(id)` - Cancel a timer
- `rt_timer_is_tick(msg)` - Check if message is a timer tick

### File I/O

- `rt_file_open(path, flags, mode, out_fd)` - Open file
- `rt_file_close(fd)` - Close file
- `rt_file_read(fd, buf, count, out_bytes)` - Read from file
- `rt_file_write(fd, buf, count, out_bytes)` - Write to file

### Network I/O

- `rt_net_listen(port, out_fd)` - Create TCP listening socket (backlog hardcoded to 5)
- `rt_net_accept(listen_fd, out_fd, timeout_ms)` - Accept incoming connection
- `rt_net_connect(host, port, out_fd, timeout_ms)` - Connect to remote host
- `rt_net_send(fd, buf, len, out_sent, timeout_ms)` - Send data
- `rt_net_recv(fd, buf, len, out_recv, timeout_ms)` - Receive data
- `rt_net_close(fd)` - Close socket

## Design Principles

1. **Minimalistic**: Only essential features, no bloat
2. **Predictable**: Cooperative scheduling, no surprises
3. **Modern C11**: Clean, safe, standards-compliant code
4. **Static allocation**: Deterministic memory, zero fragmentation, compile-time footprint
5. **No heap in hot paths**: O(1) pool allocation for all runtime operations
6. **Explicit control**: Actors yield explicitly, no preemption

## Implementation Notes

### Context Switching

The x86-64 context switch saves and restores callee-saved registers (rbx, rbp, r12-r15) and the stack pointer. This is implemented in hand-written assembly for performance and control.

### Scheduling

The scheduler uses a simple priority-based round-robin algorithm:
1. Find the highest-priority READY actor
2. Context switch to it
3. When it yields, mark it READY again
4. Repeat

### IPC Modes

- **IPC_COPY**: Message data is copied to receiver's mailbox. Sender continues immediately.
- **IPC_BORROW**: Message data remains on sender's stack. Sender blocks until receiver calls `rt_ipc_release()`. Provides automatic backpressure.

## Implementation Details

### I/O Subsystems

All I/O operations (timers, file, network) are handled by dedicated worker threads that communicate with the scheduler via lock-free SPSC queues. This design:

- Keeps the actor runtime non-blocking
- Allows multiple I/O operations to run concurrently
- Uses lock-free queues for efficient cross-thread communication
- Automatically handles actor cleanup (cancels timers, closes files/sockets)

**Timer subsystem**: Uses Linux `timerfd_create()` and `epoll` for efficient multi-timer management.

**File I/O subsystem**: Worker thread processes read/write requests asynchronously.

**Network I/O subsystem**: Worker thread handles socket operations (accept, connect, send, recv) asynchronously.

## Memory Configuration

### Static Allocation Architecture

The runtime uses **static memory allocation** for deterministic behavior suitable for embedded systems. All memory pools are configured at compile time via `include/rt_static_config.h`, providing:

- **Deterministic footprint**: Total memory usage known at compile/link time
- **Zero fragmentation**: No malloc in hot paths (message passing, timers, etc.)
- **Predictable allocation**: Pool exhaustion returns clear errors instead of allocation failures
- **Safety-critical ready**: Suitable for DO-178C and similar certifications

**Only one malloc**: Actor stacks are the only dynamic allocation (at spawn time, ~64KB each).

### Memory Footprint Calculation

The total memory footprint consists of two components:

**1. Static Data (BSS segment)** - calculated from `rt_static_config.h`:

| Component | Formula | Default Size |
|-----------|---------|--------------|
| Actor table | `RT_MAX_ACTORS × ~200 bytes` | 12.8 KB |
| Mailbox pool | `RT_MAILBOX_ENTRY_POOL_SIZE × ~40 bytes` | 10.2 KB |
| Message pool | `RT_MESSAGE_DATA_POOL_SIZE × RT_MAX_MESSAGE_SIZE` | 64 KB |
| Link pool | `RT_LINK_ENTRY_POOL_SIZE × ~16 bytes` | 2 KB |
| Monitor pool | `RT_MONITOR_ENTRY_POOL_SIZE × ~16 bytes` | 2 KB |
| Timer pool | `RT_TIMER_ENTRY_POOL_SIZE × ~40 bytes` | 2.5 KB |
| Bus tables | `RT_MAX_BUSES × ~3 KB` | 96 KB |
| Completion queues | `3 subsystems × ~20 KB` | 60 KB |
| **Total static** | **(measured)** | **~231 KB** |

**2. Dynamic Memory (Heap)** - actor stacks only:

```
Heap = (Number of actors) × (Average stack size)
```

Example: 20 actors × 32 KB = 640 KB

**Total footprint** = Static data + Dynamic stacks = ~871 KB (default configuration, 20 actors)

### Configuring Memory Limits

Edit `include/rt_static_config.h` to adjust memory usage for your application:

```c
// Maximum number of concurrent actors
#define RT_MAX_ACTORS 64

// IPC pools (message passing hot path)
#define RT_MAILBOX_ENTRY_POOL_SIZE 256    // Mailbox entries
#define RT_MESSAGE_DATA_POOL_SIZE  256    // Message data buffers
#define RT_MAX_MESSAGE_SIZE        256    // Max bytes per message

// Actor relationship pools
#define RT_LINK_ENTRY_POOL_SIZE    128    // Bidirectional links
#define RT_MONITOR_ENTRY_POOL_SIZE 128    // Unidirectional monitors

// Timer pool
#define RT_TIMER_ENTRY_POOL_SIZE   64     // Active timers

// I/O completion queues (one per subsystem)
#define RT_COMPLETION_QUEUE_SIZE   64     // Must be power of 2

// Bus configuration
#define RT_MAX_BUSES               32     // Maximum number of buses
#define RT_MAX_BUS_ENTRIES         64     // Entries per bus
#define RT_MAX_BUS_SUBSCRIBERS     32     // Subscribers per bus

// Actor stack configuration
#define RT_DEFAULT_STACK_SIZE      65536  // 64 KB default
```

### Configuration Examples

**Small embedded system (drone sensor node):**
```c
#define RT_MAX_ACTORS              16     // Few actors
#define RT_MAX_BUSES               8      // Fewer buses
#define RT_MAILBOX_ENTRY_POOL_SIZE 64     // Limited messaging
#define RT_MESSAGE_DATA_POOL_SIZE  64
#define RT_TIMER_ENTRY_POOL_SIZE   16
#define RT_DEFAULT_STACK_SIZE      16384  // 16 KB stacks
// Static: ~60 KB, Dynamic: 16 actors × 16 KB = 256 KB
// Total: ~316 KB
```

**High-throughput system (network gateway):**
```c
#define RT_MAX_ACTORS              128    // Many concurrent actors
#define RT_MAILBOX_ENTRY_POOL_SIZE 512    // Heavy messaging
#define RT_MESSAGE_DATA_POOL_SIZE  512
#define RT_MAX_MESSAGE_SIZE        1024   // Larger messages
#define RT_COMPLETION_QUEUE_SIZE   128
// Static: ~650 KB, Dynamic: varies by active actors
```

**Memory-constrained MCU (STM32F4 with 192KB RAM):**
```c
#define RT_MAX_ACTORS              8
#define RT_MAX_BUSES               4      // Minimal buses
#define RT_MAX_BUS_ENTRIES         16     // Small buffers
#define RT_MAILBOX_ENTRY_POOL_SIZE 32
#define RT_MESSAGE_DATA_POOL_SIZE  32
#define RT_MAX_MESSAGE_SIZE        128    // Smaller messages
#define RT_LINK_ENTRY_POOL_SIZE    16
#define RT_MONITOR_ENTRY_POOL_SIZE 16
#define RT_TIMER_ENTRY_POOL_SIZE   8
#define RT_DEFAULT_STACK_SIZE      8192   // 8 KB stacks
// Static: ~20 KB, Dynamic: 8 actors × 8 KB = 64 KB
// Total: ~84 KB (fits in 192 KB RAM with margin)
```

### Sizing Guidelines

**RT_MAX_ACTORS**: Set to maximum concurrent actors needed
- Default (64): General-purpose development
- Small (8-16): Constrained embedded systems
- Large (128+): Server-like applications

**RT_MAILBOX_ENTRY_POOL_SIZE**: Set to peak concurrent messages
- Formula: `(RT_MAX_ACTORS × avg_mailbox_depth × 1.5)`
- Default (256): ~4 messages per actor with margin
- Increase if seeing "Mailbox pool exhausted" errors

**RT_MESSAGE_DATA_POOL_SIZE**: Set to peak concurrent messages
- Should match or exceed `RT_MAILBOX_ENTRY_POOL_SIZE`
- Shared by IPC, timers, links, and bus
- Increase if seeing "Message pool exhausted" errors

**RT_MAX_MESSAGE_SIZE**: Set to largest message payload
- Default (256 bytes): Most use cases
- Small (64-128): Memory-constrained systems
- Large (512-1024): Network packets or file data
- Note: All messages use this size (some waste for small messages)

**RT_TIMER_ENTRY_POOL_SIZE**: Set to maximum active timers
- Count both one-shot and periodic timers
- Increase if seeing "Timer pool exhausted" errors

**RT_COMPLETION_QUEUE_SIZE**: Set to peak concurrent I/O operations
- Must be power of 2 (ring buffer)
- Default (64): Moderate I/O concurrency
- Increase for high-throughput I/O workloads

### Checking Memory Usage

After configuration, rebuild and check actual memory usage:

```bash
make clean && make all

# Check static data size (BSS + DATA segments)
size build/librt.a

# Check total binary size
size build/pingpong

# Runtime verification
# Pool exhaustion errors will be logged if limits are too low
./build/your_application
```

### Pool Exhaustion Handling

When a pool is exhausted, the runtime returns clear errors:
- **IPC send**: Returns `RT_ERR_NOMEM` ("Mailbox pool exhausted")
- **Timer creation**: Returns `RT_ERR_NOMEM` ("Timer pool exhausted")
- **Link/Monitor**: Returns `RT_ERR_NOMEM` ("Link pool exhausted")
- **Exit messages**: Dropped with `RT_LOG_ERROR` (non-critical)

If you see pool exhaustion errors, increase the relevant pool size in `rt_static_config.h` and rebuild.

### Current Limitations

- No stack overflow detection (guard patterns/MPU planned)
- Linux-only (FreeRTOS/ARM Cortex-M port planned)

These limitations are intentional for the current release. See `spec.md` for complete details and `Future Work` below for planned enhancements.

## Future Work

- Port to ARM Cortex-M with FreeRTOS integration
- Add stack overflow detection (guard patterns or MPU)

## License

See project license file.
