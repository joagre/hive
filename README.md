# Actor Runtime for Embedded Systems

> **Lightweight actors for embedded systems. Deterministic memory, no GC.**

A complete actor-based runtime designed for **embedded and safety-critical systems**. Features cooperative multitasking with priority-based scheduling and message passing using the actor model.

**Current platform:** x86-64 Linux (fully implemented)
**Future platform:** STM32/ARM Cortex-M bare metal (see `SPEC.md`)

**Safety-critical caveat:** File I/O stalls the entire scheduler. Restrict file I/O to initialization, shutdown, or non–time-critical phases.

The runtime uses **statically bounded memory** for deterministic behavior with zero heap fragmentation (heap allocation optional only for actor stacks). It features **priority-based scheduling** (4 levels: CRITICAL, HIGH, NORMAL, LOW) with fast context switching. Provides message passing (IPC with selective receive and request/reply), linking, monitoring, timers, pub-sub messaging (bus), network I/O, and file I/O.

## Quick Links

- **[Full Specification](SPEC.md)** - Complete design and implementation details
- **[Examples Directory](examples/)** - Working examples (pingpong, bus, echo server, etc.)
- **[Static Configuration](include/acrt_static_config.h)** - Compile-time memory limits and pool sizes

## Features

- Statically bounded memory - Deterministic footprint, zero fragmentation, heap optional only for actor stacks
- Cooperative multitasking with manual x86-64 context switching
- Priority-based round-robin scheduler (4 priority levels)
- Stack overflow detection with guard patterns (16-byte overhead per actor)
- Actor lifecycle management (spawn, exit)
- IPC with selective receive and request/reply
- Message classes: NOTIFY (async), REQUEST/REPLY, TIMER, SYSTEM
- Actor linking and monitoring (bidirectional links, unidirectional monitors)
- Exit notifications with exit reasons (normal, crash, stack overflow, killed)
- Timers (one-shot and periodic with timerfd/epoll)
- Network I/O (non-blocking TCP via event loop)
- File I/O (synchronous, stalls scheduler)
- Bus (pub-sub with retention policies)

## Performance

Benchmarks measured on a vanilla Dell XPS 13 (Intel Core i7, x86-64 Linux):

| Operation | Latency | Throughput | Notes |
|-----------|---------|------------|-------|
| **Context switch** | ~1.2 µs/switch | 0.85 M switches/sec | Manual assembly, cooperative |
| **IPC send/recv** | ~2.2-2.5 µs/msg | 0.40-0.46 M msgs/sec | 8-252 byte messages |
| **Pool allocation** | ~9 ns/op | 104 M ops/sec | 1.2x faster than malloc |
| **Actor spawn** | ~370 ns/actor | 2.7 M actors/sec | Includes stack allocation (arena) |
| **Bus pub/sub** | ~265 ns/msg | 3.76 M msgs/sec | With cooperative yields |

Run benchmarks yourself:
```bash
make bench
```

## Memory Model

The runtime uses **compile-time configuration** for predictable memory allocation.

### Compile-Time Configuration (`include/acrt_static_config.h`)

All resource limits are defined at compile time. Edit and recompile to change:

```c
#define ACRT_MAX_ACTORS 64                // Maximum concurrent actors
#define ACRT_STACK_ARENA_SIZE (1*1024*1024) // Stack arena size (1 MB default)
#define ACRT_MAILBOX_ENTRY_POOL_SIZE 256  // Mailbox pool size
#define ACRT_MESSAGE_DATA_POOL_SIZE 256   // Message pool size
#define ACRT_MAX_MESSAGE_SIZE 256         // Max message size (4-byte header + 252 payload)
#define ACRT_MAX_BUSES 32                 // Maximum concurrent buses
// ... see acrt_static_config.h for full list
```

All structures are statically allocated. Actor stacks use a static arena allocator by default (configurable size), with optional malloc via `actor_config.malloc_stack = true`. Stack sizes are configurable per actor, allowing different actors to use different stack sizes. Arena memory is automatically reclaimed and reused when actors exit. No malloc in hot paths. Memory footprint calculable at link time when using arena allocator (default); optional malloc'd stacks add runtime-dependent heap usage.

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

# Request/reply example (with acrt_ipc_request)
./build/sync_ipc

# Priority scheduling example (4 levels, starvation demo)
./build/priority
```

## Quick Start

### Basic Actor Example

```c
#include "acrt_runtime.h"
#include "acrt_ipc.h"
#include <stdio.h>

void my_actor(void *arg) {
    printf("Hello from actor %u\n", acrt_self());
    acrt_exit();
}

int main(void) {
    acrt_init();

    actor_id id;
    acrt_spawn(my_actor, NULL, &id);

    acrt_run();
    acrt_cleanup();

    return 0;
}
```

### IPC and Configuration

```c
// Configure actor
actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
cfg.priority = 0;             // 0-3, lower is higher
cfg.stack_size = 128 * 1024;
cfg.malloc_stack = false;     // false=arena (default), true=malloc
actor_id worker;
acrt_spawn_ex(worker_actor, &args, &cfg, &worker);

// Notify (async message)
int data = 42;
acrt_status status = acrt_ipc_notify(target, &data, sizeof(data));
if (ACRT_FAILED(status)) {
    // Pool exhausted: ACRT_MAILBOX_ENTRY_POOL_SIZE or ACRT_MESSAGE_DATA_POOL_SIZE
    // Notify does NOT block or drop - caller must handle ACRT_ERR_NOMEM

    // Backoff and retry pattern:
    acrt_message msg;
    acrt_ipc_recv(&msg, 10);  // Backoff 10ms (returns timeout or message)
    // Retry notify...
}

// Request/reply pattern: Send request and wait for reply
acrt_message reply;
status = acrt_ipc_request(target, &data, sizeof(data), &reply, 5000);  // 5s timeout
if (ACRT_FAILED(status)) {
    // ACRT_ERR_TIMEOUT if no reply, ACRT_ERR_NOMEM if pool exhausted
}

// Reply to a REQUEST message (in receiver actor)
acrt_ipc_reply(&msg, &response, sizeof(response));

// Receive messages
acrt_message msg;
acrt_ipc_recv(&msg, -1);   // -1=block forever
acrt_ipc_recv(&msg, 0);    // 0=non-blocking (returns ACRT_ERR_WOULDBLOCK if empty)
acrt_ipc_recv(&msg, 100);  // 100=timeout after 100ms (returns ACRT_ERR_TIMEOUT if no message)

// Decode message header
acrt_msg_class class;
uint32_t tag;
const void *payload;
size_t payload_len;
acrt_msg_decode(&msg, &class, &tag, &payload, &payload_len);
```

### Timers

```c
timer_id timer;
acrt_timer_after(500000, &timer);    // One-shot, 500ms
acrt_timer_every(200000, &periodic); // Periodic, 200ms

acrt_message msg;
acrt_ipc_recv(&msg, -1);
if (acrt_msg_is_timer(&msg)) {
    // Handle timer tick - tag contains timer_id
    uint32_t tag;
    acrt_msg_decode(&msg, NULL, &tag, NULL, NULL);
    printf("Timer %u fired\n", tag);
}
acrt_timer_cancel(periodic);
```

### File and Network I/O

```c
// File operations (block until complete)
int fd;
acrt_file_open("test.txt", O_RDWR | O_CREAT, 0644, &fd);
acrt_file_write(fd, data, len, &written);
acrt_file_read(fd, buffer, sizeof(buffer), &nread);
acrt_file_close(fd);

// Network server
int listen_fd, client_fd;
acrt_net_listen(8080, &listen_fd);
acrt_net_accept(listen_fd, &client_fd, -1);
acrt_net_recv(client_fd, buffer, sizeof(buffer), &received, -1);
acrt_net_send(client_fd, buffer, received, &sent, -1);
acrt_net_close(client_fd);

// Network client
int server_fd;
acrt_net_connect("127.0.0.1", 8080, &server_fd, 5000);
```

### Bus (Pub-Sub)

```c
acrt_bus_config cfg = ACRT_BUS_CONFIG_DEFAULT;
cfg.consume_after_reads = 0;   // 0=persist, N=remove after N reads
cfg.max_age_ms = 0;    // 0=no expiry, T=expire after T ms
// Note: Maximum 32 subscribers per bus (architectural limit)

bus_id bus;
acrt_bus_create(&cfg, &bus);
acrt_bus_subscribe(bus);

sensor_data data = {.temperature = 25.5f};
acrt_status status = acrt_bus_publish(bus, &data, sizeof(data));
if (ACRT_FAILED(status)) {
    // Message pool exhausted (shares ACRT_MESSAGE_DATA_POOL_SIZE with IPC)
    // Note: Ring buffer full automatically drops oldest entry
}

sensor_data received;
acrt_bus_read_wait(bus, &received, sizeof(received), &actual_len, -1);
```

### Linking and Monitoring

```c
actor_id other;
acrt_spawn(other_actor, NULL, &other);
acrt_link(other);     // Bidirectional - both get exit notifications
uint32_t mon_id;
acrt_monitor(other, &mon_id);  // Unidirectional - only monitor gets notifications

acrt_message msg;
acrt_ipc_recv(&msg, -1);
if (acrt_is_exit_msg(&msg)) {
    acrt_exit_msg exit_info;
    acrt_decode_exit(&msg, &exit_info);
    // exit_info.reason: ACRT_EXIT_NORMAL, ACRT_EXIT_CRASH, ACRT_EXIT_CRASH_STACK, ACRT_EXIT_KILLED
}
```

## API Overview

### Runtime Initialization

- `acrt_init()` - Initialize the runtime
- `acrt_run()` - Run the scheduler (blocks until all actors exit)
- `acrt_cleanup()` - Cleanup and free resources
- `acrt_shutdown()` - Request graceful shutdown
- `acrt_actor_alive(id)` - Check if actor is still alive

### Actor Management

- `acrt_spawn(fn, arg, out)` - Spawn actor with default config
- `acrt_spawn_ex(fn, arg, config, out)` - Spawn actor with custom config
- `acrt_exit()` - Terminate current actor
- `acrt_self()` - Get current actor's ID
- `acrt_yield()` - Voluntarily yield to scheduler

### IPC

- `acrt_ipc_notify(to, data, len)` - Async message (fire-and-forget)
- `acrt_ipc_recv(msg, timeout)` - Receive any message
- `acrt_ipc_recv_match(from, class, tag, msg, timeout)` - Selective receive with filtering
- `acrt_ipc_request(to, req, len, reply, timeout)` - Blocking request/reply
- `acrt_ipc_reply(request, data, len)` - Reply to a REQUEST message
- `acrt_msg_decode(msg, class, tag, payload, len)` - Decode message header
- `acrt_msg_is_timer(msg)` - Check if message is a timer tick
- `acrt_ipc_pending()` - Check if messages are available
- `acrt_ipc_count()` - Get number of pending messages

### Linking and Monitoring

- `acrt_link(target)` - Create bidirectional link
- `acrt_link_remove(target)` - Remove bidirectional link
- `acrt_monitor(target, out)` - Create unidirectional monitor
- `acrt_monitor_cancel(id)` - Cancel monitor
- `acrt_is_exit_msg(msg)` - Check if message is exit notification
- `acrt_decode_exit(msg, out)` - Extract exit information

### Timers

- `acrt_timer_after(delay_us, out)` - Create one-shot timer
- `acrt_timer_every(interval_us, out)` - Create periodic timer
- `acrt_timer_cancel(id)` - Cancel a timer
- `acrt_msg_is_timer(msg)` - Check if message is a timer tick (also in IPC)

### File I/O

- `acrt_file_open(path, flags, mode, out_fd)` - Open file
- `acrt_file_close(fd)` - Close file
- `acrt_file_read(fd, buf, count, out_bytes)` - Read from file
- `acrt_file_pread(fd, buf, count, offset, out_bytes)` - Read from file at offset
- `acrt_file_write(fd, buf, count, out_bytes)` - Write to file
- `acrt_file_pwrite(fd, buf, count, offset, out_bytes)` - Write to file at offset
- `acrt_file_sync(fd)` - Sync file to disk

**Note:** File operations block until complete. No timeout parameter.

### Network I/O

- `acrt_net_listen(port, out_fd)` - Create TCP listening socket (backlog hardcoded to 5)
- `acrt_net_accept(listen_fd, out_fd, timeout_ms)` - Accept incoming connection
- `acrt_net_connect(ip, port, out_fd, timeout_ms)` - Connect to remote server (numeric IPv4 only)
- `acrt_net_send(fd, buf, len, out_sent, timeout_ms)` - Send data
- `acrt_net_recv(fd, buf, len, out_recv, timeout_ms)` - Receive data
- `acrt_net_close(fd)` - Close socket

### Bus (Pub-Sub)

- `acrt_bus_create(config, out_id)` - Create a new bus with retention policy
- `acrt_bus_destroy(bus)` - Destroy a bus
- `acrt_bus_subscribe(bus)` - Subscribe current actor to bus
- `acrt_bus_unsubscribe(bus)` - Unsubscribe current actor from bus
- `acrt_bus_publish(bus, data, len)` - Publish data to bus (non-blocking)
- `acrt_bus_read(bus, buf, len, out_len)` - Read next message (non-blocking)
- `acrt_bus_read_wait(bus, buf, len, out_len, timeout_ms)` - Read next message (blocking)
- `acrt_bus_entry_count(bus)` - Get number of entries in bus

## Implementation Details

### Event Loop Architecture

The runtime is **completely single-threaded** with an event loop architecture. All actors run cooperatively in a single scheduler thread. There are no I/O worker threads - all I/O operations are integrated into the scheduler's event loop.

**Linux (epoll)**:
- Timers: `timerfd` registered in `epoll`
- Network: Non-blocking sockets registered in `epoll`
- File: Direct synchronous I/O (regular files don't work with epoll)
- Event loop: `epoll_wait()` with bounded timeout (10ms) for defensive wakeup

**STM32 (bare metal)**:
- Timers: Hardware timers (SysTick or TIM peripherals)
- Network: lwIP in NO_SYS mode (polling or interrupt-driven)
- File: Direct synchronous I/O (FATFS/littlefs are fast, <1ms typically)
- Event loop: WFI (Wait For Interrupt) when no actors runnable

This single-threaded model provides:
- Maximum simplicity (no lock ordering, no deadlock)
- Conditional determinism (no lock contention, no priority inversion; see SPEC.md for epoll ordering caveats)
- Maximum performance (zero lock overhead, no cache line bouncing)
- Safety-critical compliance (deterministic memory, predictable scheduling)

### Thread Safety

All runtime APIs must be called from actor context (the scheduler thread). The runtime uses **zero synchronization primitives** in the core event loop:

- No mutexes (single thread, no contention)
- No C11 atomics (single writer/reader per data structure)
- No condition variables (event loop uses epoll/select for waiting)
- No locks (mailboxes, actor state, bus state accessed only by scheduler thread)

**Note:** STM32 uses `volatile bool` flags with interrupt disable for ISR-to-scheduler communication. This is a synchronization protocol but not C11 atomics.

**Reentrancy constraint:** Runtime APIs are **not reentrant**. Actors **must not** call runtime APIs from signal handlers or interrupt service routines (ISRs). Violating this results in undefined behavior.

**External thread communication:** External threads cannot call runtime APIs directly. Use platform-specific IPC (sockets/pipes) with dedicated reader actors to bridge external threads into the actor system. See `SPEC.md` "Thread Safety" section for complete details.

## Testing

```bash
# Build and run all tests
make test

# Run with valgrind (memory error detection)
valgrind --leak-check=full ./build/actor_test
valgrind --leak-check=full ./build/ipc_test
# ... etc for each test

# Note: stack_overflow_test intentionally corrupts memory to test
# overflow detection - valgrind errors are expected for this test
```

The test suite includes 18 test programs covering actors, IPC, timers, bus, networking, file I/O, linking, monitoring, and edge cases like pool exhaustion and stack overflow detection.

## Future Work

- Port to ARM Cortex-M bare metal (STM32)
