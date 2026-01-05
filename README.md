# Actor Runtime for Embedded Systems

A complete actor-based runtime designed for **embedded and safety-critical systems**. Features cooperative multitasking with priority-based scheduling and message passing inspired by Erlang's actor model.

**Current platform:** x86-64 Linux (fully implemented)
**Future platform:** STM32/ARM Cortex-M with FreeRTOS (see `spec.md`)

The runtime uses **static memory allocation** for deterministic behavior with zero heap fragmentation—perfect for embedded systems and safety-critical applications. It features **priority-based scheduling** (4 levels: CRITICAL, HIGH, NORMAL, LOW) with fast context switching via manual assembly. Despite this minimalistic design, it provides a complete actor system with message passing (IPC with COPY/BORROW modes), linking, monitoring, timers, pub-sub messaging (bus), async networking, and async file I/O.

## Quick Links

- 📖 **[Full Specification](spec.md)** - Complete design and implementation details
- 💻 **[Examples Directory](examples/)** - Working examples (pingpong, bus, echo server, etc.)
- 🔧 **[Static Configuration](include/rt_static_config.h)** - Compile-time memory limits and pool sizes
- ⚡ **[Benchmarks](#performance)** - Performance measurements and comparison
- 🐛 **[Troubleshooting](TROUBLESHOOTING.md)** - Common issues and solutions
- ❓ **[FAQ](FAQ.md)** - Frequently asked questions
- 🤖 **[Development Guide](CLAUDE.md)** - Instructions for Claude Code when working with this codebase

## Table of Contents

- [Why Use This?](#why-use-this)
- [Features](#features)
- [Performance](#performance)
- [Prerequisites](#prerequisites)
- [Building](#building)
- [Running Examples](#running-examples)
- [Quick Start](#quick-start)
- [API Overview](#api-overview)
- [Design Principles](#design-principles)
- [Implementation Details](#implementation-details)
- [Memory Configuration](#memory-configuration)
- [Troubleshooting](TROUBLESHOOTING.md) - Common issues and solutions
- [FAQ](FAQ.md) - Frequently asked questions
- [Future Work](#future-work)

## Why Use This?

### Who Should Use This

This runtime is designed for:
- **Embedded systems developers** building autopilots, sensor networks, or robotics platforms
- **Safety-critical applications** requiring deterministic memory behavior (DO-178C, etc.)
- **MCU projects** needing structured concurrency on top of FreeRTOS (or bare-metal with pthreads on Linux)
- **Researchers** exploring actor models on resource-constrained hardware

### What Problems It Solves

- ✅ **Structured concurrency** - Actor model eliminates shared-state bugs and race conditions
- ✅ **Deterministic memory** - Static allocation means no heap fragmentation or OOM surprises
- ✅ **Type-safe IPC** - Message passing with backpressure (BORROW mode)
- ✅ **Predictable scheduling** - Priority-based cooperative multitasking, no preemption surprises
- ✅ **Async I/O without complexity** - File/network operations handled by worker threads

### When NOT to Use This

- ❌ **General-purpose servers** - Use Erlang/BEAM, Akka, or Go instead
- ❌ **UI applications** - Event loops (Qt, GTK) are better suited
- ❌ **Preemptive actor scheduling** - Actors use cooperative multitasking; use native OS threads for preemption
- ❌ **Dynamic workloads** - Requires compile-time configuration of pool sizes

### Comparison with Alternatives

| Feature | This Runtime | Erlang/BEAM | FreeRTOS | Akka | Plain C |
|---------|--------------|-------------|----------|------|---------|
| **Actor model** | ✅ | ✅ | ❌ | ✅ | ❌ |
| **Embedded-friendly** | ✅ | ❌ | ✅ | ❌ | ✅ |
| **Static memory** | ✅ | ❌ | ⚠️ | ❌ | ⚠️ |
| **Deterministic** | ✅ | ❌ | ✅ | ❌ | ✅ |
| **Priority scheduling** | ✅ | ❌ | ✅ | ❌ | ❌ |
| **Message passing** | ✅ | ✅ | ⚠️ | ✅ | ❌ |
| **MCU-ready** | ✅ (planned) | ❌ | ✅ | ❌ | ✅ |

### Real-World Use Cases

**Drone Autopilot (STM32F7)**
```
Actors:
- IMU sensor reader (CRITICAL priority) - reads gyro/accel at 1kHz
- Flight controller (CRITICAL) - PID loops for stabilization
- GPS processor (HIGH) - position updates at 10Hz
- Radio receiver (HIGH) - pilot commands
- Telemetry sender (NORMAL) - ground station updates
- LED status (LOW) - visual indicators

Communication: Sensor actors publish to bus, controller subscribes
Memory: ~150 KB static pools + 10 actors × 32KB stacks = ~470 KB total
```

**Industrial Sensor Network (ARM Cortex-M4)**
```
Actors:
- Temperature sensors (HIGH) - 4 actors, one per sensor
- Pressure monitor (HIGH) - analog sensor reader
- Data aggregator (NORMAL) - collects and processes readings
- ModBus interface (NORMAL) - communicates with PLC
- Alarm monitor (CRITICAL) - safety threshold checks
- LED display (LOW) - local status display

Communication: Sensors publish to shared bus with retention policy
Memory: ~80 KB static pools + 7 actors × 16KB stacks = ~192 KB total
```

**Robotics Control System (x86-64 Linux)**
```
Actors:
- Vision processor (HIGH) - camera input processing
- Path planner (HIGH) - navigation and obstacle avoidance
- Motor controllers (CRITICAL) - 6 actors, one per motor
- Lidar processor (HIGH) - distance sensing
- Web interface (LOW) - remote monitoring/control
- Data logger (NORMAL) - record telemetry

Communication: Mix of IPC (COPY for small messages) and bus (sensor data)
Memory: ~231 KB static pools + 12 actors × 64KB stacks = ~1 MB total
```

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

## Performance

Benchmarks measured on a vanilla Dell XPS 13 (Intel Core i7, x86-64 Linux):

| Operation | Latency | Throughput | Notes |
|-----------|---------|------------|-------|
| **Context switch** | ~1.1 µs/switch | 0.87 M switches/sec | Manual assembly, cooperative |
| **IPC (COPY mode)** | ~2.3 µs/msg | 0.43 M msgs/sec | 8-256 byte messages |
| **Pool allocation** | ~10 ns/op | 100 M ops/sec | 1.1x faster than malloc |
| **Actor spawn** | ~2.7 µs/actor | 365K actors/sec | Includes stack allocation |
| **Bus pub/sub** | ~281 ns/msg | 3.55 M msgs/sec | With cooperative yields |

Run benchmarks yourself:
```bash
make bench
```

**Key insights:**
- Pool allocation is faster and more predictable than malloc/free
- Context switching overhead is minimal (~1 µs)
- Message passing is fast enough for high-frequency communication
- Bus pub/sub demonstrates proper cooperative actor behavior

## Memory Model

The runtime uses **compile-time configuration** for predictable memory allocation.

### Compile-Time Configuration (`include/rt_static_config.h`)

All resource limits are defined at compile time. Edit and recompile to change:

```c
#define RT_MAX_ACTORS 64                // Maximum concurrent actors
#define RT_MAILBOX_ENTRY_POOL_SIZE 256  // Mailbox pool size
#define RT_MESSAGE_DATA_POOL_SIZE 256   // Message pool size
#define RT_COMPLETION_QUEUE_SIZE 64     // I/O completion queue size
#define RT_MAX_BUSES 32                 // Maximum concurrent buses
// ... and more (see rt_static_config.h for full list)
```

**Memory characteristics:**
- All structures (except actor stacks) are **statically allocated**
- No malloc in hot paths (IPC, scheduling, I/O completions)
- Memory footprint is **calculable at link time**
- No heap fragmentation
- Perfect for embedded/safety-critical systems

## Prerequisites

### Required

- **Linux** (kernel 2.6.25+ for timerfd support)
- **GCC** 4.7+ or **Clang** 3.1+ (C11 support required)
- **GNU Make**
- **pthread** library (typically included with glibc)

### Platform-Specific Requirements

**Current platform (x86-64 Linux):**
- x86-64 architecture (uses manual assembly for context switching)
- POSIX.1-2008 compliance (`timerfd_create`, `epoll`, pthreads)

**Future platform (ARM Cortex-M):**
- ARM Cortex-M4/M7 with FreeRTOS (planned)

### Verification

Check your environment:

```bash
# Check GCC version (need 4.7+)
gcc --version

# Check if pthread is available
gcc -pthread -o /tmp/test -x c - <<< 'int main(){return 0;}'

# Verify you're on x86-64
uname -m  # Should show: x86_64
```

## Building

```bash
make
```

That's it! Then try `./build/pingpong` to see actors in action.

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
    rt_init();

    rt_spawn(my_actor, NULL);

    rt_run();
    rt_cleanup();

    return 0;
}
```

### Advanced Actor Spawning

```c
// Passing arguments to actors
typedef struct {
    int worker_id;
    const char *task_name;
} worker_args;

void worker_actor(void *arg) {
    worker_args *args = (worker_args *)arg;
    printf("Worker %d starting task: %s\n", args->worker_id, args->task_name);

    // Do work...

    rt_exit();
}

int main(void) {
    rt_init();

    // Spawn with custom configuration
    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.name = "high_priority_worker";
    cfg.priority = 0;           // CRITICAL priority (0-3, lower is higher)
    cfg.stack_size = 128 * 1024; // 128 KB stack

    worker_args args = {.worker_id = 1, .task_name = "sensor_processing"};
    actor_id worker = rt_spawn_ex(worker_actor, &args, &cfg);

    printf("Spawned worker actor (ID: %u, priority: %d)\n", worker, cfg.priority);

    rt_run();
    rt_cleanup();
    return 0;
}
```

### Sending Messages

```c
// Send a COPY message (async - data is copied, sender continues immediately)
int data = 42;
rt_ipc_send(target_actor, &data, sizeof(data), IPC_COPY);

// Send a BORROW message (zero-copy - sender blocks until receiver releases)
int data = 42;
rt_ipc_send(target_actor, &data, sizeof(data), IPC_BORROW);
// Sender blocked here until receiver calls rt_ipc_release()
```

### Receiving Messages

```c
// Blocking receive
rt_message msg;
rt_ipc_recv(&msg, -1);  // Block until message arrives

// Process COPY message (data is in msg.data)
if (msg.mode == IPC_COPY) {
    int *value = (int *)msg.data;
    printf("Received COPY: %d from actor %u\n", *value, msg.sender);
    // No release needed for COPY messages
}

// Process BORROW message (data is on sender's stack, must release!)
if (msg.mode == IPC_BORROW) {
    int *value = (int *)msg.data;
    printf("Received BORROW: %d from actor %u\n", *value, msg.sender);

    // IMPORTANT: Release borrowed message to unblock sender
    rt_ipc_release(&msg);
}

// Non-blocking receive
if (rt_ipc_recv(&msg, 0) == RT_OK) {
    // Process message (check msg.mode and release if BORROW)
}
```

**Also available:**
- `rt_ipc_pending()` - Check if messages are available without receiving
- `rt_ipc_count()` - Get number of messages in mailbox
- Timeout parameter: `-1` (block forever), `0` (non-blocking), `>0` (milliseconds)

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

**Also available:**
- All operations return `rt_status` - check with `RT_FAILED(status)` for errors
- Operations are **async** - they block the calling actor but not the runtime
- Worker thread handles actual I/O, communicates via completion queue
- See `examples/fileio.c` for complete example

### Network I/O

```c
#include "rt_net.h"

// SERVER SIDE: Create listening socket and handle connections
int listen_fd;
rt_net_listen(8080, &listen_fd);

// Accept connection (blocking, timeout in ms, -1 = infinite)
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

// CLIENT SIDE: Connect to remote server
int server_fd;
rt_status status = rt_net_connect("127.0.0.1", 8080, &server_fd, 5000); // 5 sec timeout
if (!RT_FAILED(status)) {
    // Send request
    const char *request = "GET /data";
    size_t sent;
    rt_net_send(server_fd, request, strlen(request), &sent, -1);

    // Receive response
    char response[1024];
    size_t received;
    rt_net_recv(server_fd, response, sizeof(response), &received, -1);

    rt_net_close(server_fd);
}
```

**Also available:**
- All operations return `rt_status` - check with `RT_FAILED(status)` for errors
- Timeout parameter: `-1` (block forever), `0` (non-blocking), `>0` (milliseconds)
- Operations are **async** - worker thread handles actual network I/O
- Non-blocking mode useful for polling multiple connections
- See `examples/echo.c` for complete TCP echo server example

### Bus (Pub-Sub)

```c
#include "rt_bus.h"

// Create a bus with retention policy
rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
cfg.max_entries = 16;          // Ring buffer size
cfg.max_entry_size = 256;      // Max payload size
cfg.max_subscribers = 32;      // Max concurrent subscribers
cfg.max_readers = 0;           // 0 = data persists (not removed after read)
cfg.max_age_ms = 0;            // 0 = no time-based expiry

bus_id sensor_bus;
rt_bus_create(&cfg, &sensor_bus);

// Publisher actor - publish sensor data
typedef struct {
    uint32_t timestamp;
    float temperature;
} sensor_data;

sensor_data data = {.timestamp = 123, .temperature = 25.5f};
rt_bus_publish(sensor_bus, &data, sizeof(data));

// Subscriber actor - subscribe and read data
rt_bus_subscribe(sensor_bus);

// Blocking read - wait for next message
size_t actual_len;
sensor_data received;
rt_bus_read_wait(sensor_bus, &received, sizeof(received), &actual_len, -1);

// Non-blocking read - check if data available
rt_status status = rt_bus_read(sensor_bus, &received, sizeof(received), &actual_len);
if (!RT_FAILED(status)) {
    printf("Temperature: %.1f\n", received.temperature);
}

// Unsubscribe when done
rt_bus_unsubscribe(sensor_bus);

// Destroy bus
rt_bus_destroy(sensor_bus);
```

**Retention policy explained:**
- `max_readers = 0` - Entries persist indefinitely (removed only when buffer wraps)
- `max_readers = N` - Entry auto-removed after N subscribers read it
- `max_age_ms = 0` - No time-based expiry
- `max_age_ms = T` - Entries older than T milliseconds are auto-removed
- Multiple subscribers can read the same data (pub-sub pattern)
- See `examples/bus.c` for complete multi-subscriber example

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

**Link vs Monitor:**
- **Link** (`rt_link`) - Bidirectional: both actors get exit notification if either dies
- **Monitor** (`rt_monitor`) - Unidirectional: only monitor gets notification when target dies
- Use **link** for peer relationships (both actors depend on each other)
- Use **monitor** for supervisor patterns (supervisor watches workers)
- Exit reasons: `RT_EXIT_NORMAL` (clean exit), `RT_EXIT_CRASH` (error), `RT_EXIT_KILLED` (terminated)
- See `examples/link_demo.c` and `examples/supervisor.c` for complete examples

## API Overview

### Runtime Initialization

- `rt_init()` - Initialize the runtime
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

### Bus (Pub-Sub)

- `rt_bus_create(config, out_id)` - Create a new bus with retention policy
- `rt_bus_destroy(bus)` - Destroy a bus
- `rt_bus_subscribe(bus)` - Subscribe current actor to bus
- `rt_bus_unsubscribe(bus)` - Unsubscribe current actor from bus
- `rt_bus_publish(bus, data, len)` - Publish data to bus (non-blocking)
- `rt_bus_read(bus, buf, len, out_len)` - Read next message (non-blocking)
- `rt_bus_read_wait(bus, buf, len, out_len, timeout_ms)` - Read next message (blocking)

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

The runtime uses **static memory allocation** for deterministic behavior:

- **Deterministic footprint**: Total memory usage known at compile/link time
- **Zero fragmentation**: No malloc in hot paths (message passing, timers, etc.)
- **Predictable allocation**: Pool exhaustion returns clear errors
- **Safety-critical ready**: Suitable for DO-178C and similar certifications

**Only one malloc**: Actor stacks are allocated at spawn time (~64KB default each).

### Quick Configuration

Edit `include/rt_static_config.h` to adjust pool sizes:

```c
#define RT_MAX_ACTORS 64                // Maximum concurrent actors
#define RT_MAILBOX_ENTRY_POOL_SIZE 256  // Mailbox pool
#define RT_MESSAGE_DATA_POOL_SIZE  256  // Message pool
#define RT_MAX_MESSAGE_SIZE        256  // Max bytes per message
// ... see rt_static_config.h for complete list
```

Rebuild after changes: `make clean && make all`

### Memory Footprint

**Total memory** = Static pools + Actor stacks

- **Static pools**: ~231 KB (default config, measured)
- **Actor stacks**: N actors × stack size (e.g., 20 × 64KB = 1.3 MB)
- **Example total**: ~1.5 MB for 20 actors with default config

📖 **For detailed memory calculations, pool sizing guidelines, and configuration examples**, see [spec.md](spec.md#memory-model).

## Troubleshooting

See **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** for detailed solutions to common issues:
- Pool exhaustion errors
- Examples hanging or timing out
- Segmentation faults
- Build errors
- Performance issues
- Debugging tips and workflow

## FAQ

See **[FAQ.md](FAQ.md)** for frequently asked questions about:
- Production readiness, thread safety, comparison with FreeRTOS
- Performance (cooperative vs preemptive, overhead, COPY vs BORROW)
- Configuration (pool sizes, priorities, memory usage, platforms)
- Debugging (deadlocks, GDB usage, common issues)

## Future Work

- Port to ARM Cortex-M with FreeRTOS integration
- Add stack overflow detection (guard patterns or MPU)

## License

See project license file.
