# Actor Runtime - Minimalistic Implementation

A minimalistic actor-based runtime for embedded systems with cooperative multitasking and message passing, inspired by Erlang's actor model.

This is a minimal x86-64 Linux implementation demonstrating the core concepts. See `spec.md` for the full specification including STM32/ARM Cortex-M support.

## Features Implemented

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
│   ├── rt_types.h       # Core types and error handling
│   ├── rt_context.h     # Context switching interface
│   ├── rt_actor.h       # Actor management
│   ├── rt_ipc.h         # Inter-process communication
│   ├── rt_link.h        # Actor linking and monitoring
│   ├── rt_scheduler.h   # Scheduler interface
│   ├── rt_runtime.h     # Runtime initialization and public API
│   ├── rt_timer.h       # Timer subsystem
│   ├── rt_file.h        # File I/O subsystem
│   ├── rt_net.h         # Network I/O subsystem
│   ├── rt_bus.h         # Bus pub-sub subsystem
│   ├── rt_spsc.h        # Lock-free SPSC queue
│   └── rt_log.h         # Logging utilities
├── src/              # Implementation
│   ├── rt_actor.c       # Actor table and lifecycle
│   ├── rt_context.c     # Context initialization
│   ├── rt_context_asm.S # x86-64 context switch assembly
│   ├── rt_ipc.c         # Mailbox and message passing
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
rt_net_listen(8080, 10, &listen_fd);

// Accept connection
int client_fd;
rt_net_accept(listen_fd, &client_fd);

// Receive data
char buffer[1024];
size_t received;
rt_net_recv(client_fd, buffer, sizeof(buffer), &received);

// Send data
size_t sent;
rt_net_send(client_fd, buffer, received, &sent);

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

- `rt_net_listen(port, backlog, out_fd)` - Create TCP listening socket
- `rt_net_accept(listen_fd, out_fd)` - Accept incoming connection
- `rt_net_connect(host, port, out_fd)` - Connect to remote host
- `rt_net_send(fd, buf, len, out_sent)` - Send data
- `rt_net_recv(fd, buf, len, out_recv)` - Receive data
- `rt_net_close(fd)` - Close socket

## Design Principles

1. **Minimalistic**: Only essential features, no bloat
2. **Predictable**: Cooperative scheduling, no surprises
3. **Modern C11**: Clean, safe, standards-compliant code
4. **No heap in hot paths**: Fixed-size stacks, pre-allocated tables
5. **Explicit control**: Actors yield explicitly, no preemption

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

### Limitations

- Simple memory management (uses malloc/free)
- No stack overflow detection
- Linux-only (no FreeRTOS/ARM Cortex-M port yet)

See `spec.md` for the full feature set planned for production use.

## Future Work

- Port to ARM Cortex-M with FreeRTOS integration
- Add memory pools for better performance
- Add stack overflow detection (guard patterns or MPU)
- Distributed actors for multi-MCU systems

## License

See project license file.
