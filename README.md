# Actor Runtime for Embedded Systems

> **Erlang's actor model for embedded systems, with deterministic memory and no GC.**

A complete actor-based runtime designed for **embedded and safety-critical systems**. Features cooperative multitasking with priority-based scheduling and message passing inspired by Erlang's actor model.

**Current platform:** x86-64 Linux (fully implemented)
**Future platform:** STM32/ARM Cortex-M bare metal (see `SPEC.md`)

The runtime uses **static memory allocation** for deterministic behavior with zero heap fragmentation. It features **priority-based scheduling** (4 levels: CRITICAL, HIGH, NORMAL, LOW) with fast context switching. Provides message passing (IPC with ASYNC/SYNC modes), linking, monitoring, timers, pub-sub messaging (bus), network I/O, and file I/O.

## Quick Links

- **[Full Specification](SPEC.md)** - Complete design and implementation details
- **[Examples Directory](examples/)** - Working examples (pingpong, bus, echo server, etc.)
- **[Static Configuration](include/rt_static_config.h)** - Compile-time memory limits and pool sizes

## Features

- Static memory allocation - Deterministic footprint, zero fragmentation, safety-critical ready
- Cooperative multitasking with manual x86-64 context switching
- Priority-based round-robin scheduler (4 priority levels)
- Stack overflow detection with guard patterns (16-byte overhead per actor)
- Actor lifecycle management (spawn, exit)
- IPC with ASYNC and SYNC modes
- Blocking and non-blocking message receive
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
| **Context switch** | ~1.1 µs/switch | 0.90 M switches/sec | Manual assembly, cooperative |
| **IPC (ASYNC mode)** | ~2.1-2.3 µs/msg | 0.44-0.47 M msgs/sec | 8-256 byte messages |
| **Pool allocation** | ~9 ns/op | 110 M ops/sec | 1.2x faster than malloc |
| **Actor spawn** | ~290 ns/actor | 3.4 M actors/sec | Includes stack allocation (arena) |
| **Bus pub/sub** | ~278 ns/msg | 3.59 M msgs/sec | With cooperative yields |

Run benchmarks yourself:
```bash
make bench
```

## Memory Model

The runtime uses **compile-time configuration** for predictable memory allocation.

### Compile-Time Configuration (`include/rt_static_config.h`)

All resource limits are defined at compile time. Edit and recompile to change:

```c
#define RT_MAX_ACTORS 64                // Maximum concurrent actors
#define RT_STACK_ARENA_SIZE (1*1024*1024) // Stack arena size (1 MB default)
#define RT_MAILBOX_ENTRY_POOL_SIZE 256  // Mailbox pool size
#define RT_MESSAGE_DATA_POOL_SIZE 256   // Message pool size (IPC_ASYNC)
#define RT_SYNC_BUFFER_POOL_SIZE 64   // Sync buffer pool (IPC_SYNC)
#define RT_MAX_BUSES 32                 // Maximum concurrent buses
// ... see rt_static_config.h for full list
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

### IPC and Configuration

```c
// Configure actor
actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
cfg.priority = 0;             // 0-3, lower is higher
cfg.stack_size = 128 * 1024;
cfg.malloc_stack = false;     // false=arena (default), true=malloc
actor_id worker = rt_spawn_ex(worker_actor, &args, &cfg);

// Send messages
int data = 42;
rt_status status = rt_ipc_send(target, &data, sizeof(data), IPC_ASYNC);
if (RT_FAILED(status)) {
    // Pool exhausted: RT_MAILBOX_ENTRY_POOL_SIZE or RT_MESSAGE_DATA_POOL_SIZE (ASYNC)
    // For IPC_SYNC: RT_MAILBOX_ENTRY_POOL_SIZE or RT_SYNC_BUFFER_POOL_SIZE
    // Send does NOT block or drop - caller must handle RT_ERR_NOMEM

    // Backoff and retry pattern:
    rt_message msg;
    rt_ipc_recv(&msg, 10);  // Backoff 10ms (returns timeout or message)
    // Retry send...
}

// IPC_SYNC: Payload copied to sync buffer pool, sender blocks until receiver releases
// WARNING: Use with care - sender blocks, risk of deadlock
// See SPEC.md for safety considerations and deadlock scenarios
rt_ipc_send(target, &data, sizeof(data), IPC_SYNC);

// Receive messages
rt_message msg;
rt_ipc_recv(&msg, -1);   // -1=block forever
rt_ipc_recv(&msg, 0);    // 0=non-blocking (returns RT_ERR_WOULDBLOCK if empty)
rt_ipc_recv(&msg, 100);  // 100=timeout after 100ms (returns RT_ERR_TIMEOUT if no message)

// Process message data
process(&msg);

// Release message (required for SYNC, no-op for ASYNC)
rt_ipc_release(&msg);
```

### Timers

```c
timer_id timer;
rt_timer_after(500000, &timer);    // One-shot, 500ms
rt_timer_every(200000, &periodic); // Periodic, 200ms

rt_message msg;
rt_ipc_recv(&msg, -1);
if (rt_timer_is_tick(&msg)) {
    // Handle timer tick
}
rt_timer_cancel(periodic);
```

### File and Network I/O

```c
// File operations (block until complete)
int fd;
rt_file_open("test.txt", O_RDWR | O_CREAT, 0644, &fd);
rt_file_write(fd, data, len, &written);
rt_file_read(fd, buffer, sizeof(buffer), &nread);
rt_file_close(fd);

// Network server
int listen_fd, client_fd;
rt_net_listen(8080, &listen_fd);
rt_net_accept(listen_fd, &client_fd, -1);
rt_net_recv(client_fd, buffer, sizeof(buffer), &received, -1);
rt_net_send(client_fd, buffer, received, &sent, -1);
rt_net_close(client_fd);

// Network client
int server_fd;
rt_net_connect("127.0.0.1", 8080, &server_fd, 5000);
```

### Bus (Pub-Sub)

```c
rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
cfg.max_readers = 0;   // 0=persist, N=remove after N reads
cfg.max_age_ms = 0;    // 0=no expiry, T=expire after T ms
// Note: Maximum 32 subscribers per bus (architectural limit)

bus_id bus;
rt_bus_create(&cfg, &bus);
rt_bus_subscribe(bus);

sensor_data data = {.temperature = 25.5f};
rt_status status = rt_bus_publish(bus, &data, sizeof(data));
if (RT_FAILED(status)) {
    // Message pool exhausted (shares RT_MESSAGE_DATA_POOL_SIZE with IPC)
    // Note: Ring buffer full automatically drops oldest entry
}

sensor_data received;
rt_bus_read_wait(bus, &received, sizeof(received), &actual_len, -1);
```

### Linking and Monitoring

```c
actor_id other = rt_spawn(other_actor, NULL);
rt_link(other);     // Bidirectional - both get exit notifications
rt_monitor(other, &ref);  // Unidirectional - only monitor gets notifications

rt_message msg;
rt_ipc_recv(&msg, -1);
if (rt_is_exit_msg(&msg)) {
    rt_exit_msg exit_info;
    rt_decode_exit(&msg, &exit_info);
    // exit_info.reason: RT_EXIT_NORMAL, RT_EXIT_CRASH, RT_EXIT_CRASH_STACK, RT_EXIT_KILLED
}
```

## API Overview

### Runtime Initialization

- `rt_init()` - Initialize the runtime
- `rt_run()` - Run the scheduler (blocks until all actors exit)
- `rt_cleanup()` - Cleanup and free resources
- `rt_shutdown()` - Request graceful shutdown
- `rt_actor_alive(id)` - Check if actor is still alive

### Actor Management

- `rt_spawn(fn, arg)` - Spawn actor with default config
- `rt_spawn_ex(fn, arg, config)` - Spawn actor with custom config
- `rt_exit()` - Terminate current actor
- `rt_self()` - Get current actor's ID
- `rt_yield()` - Voluntarily yield to scheduler

### IPC

- `rt_ipc_send(to, data, len, mode)` - Send message
- `rt_ipc_recv(msg, timeout)` - Receive message
- `rt_ipc_release(msg)` - Release synced message
- `rt_ipc_pending()` - Check if messages are available
- `rt_ipc_count()` - Get number of pending messages

### Linking and Monitoring

- `rt_link(target)` - Create bidirectional link
- `rt_unlink(target)` - Remove bidirectional link
- `rt_monitor(target, monitor_ref)` - Create unidirectional monitor
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
- `rt_file_pread(fd, buf, count, offset, out_bytes)` - Read from file at offset
- `rt_file_write(fd, buf, count, out_bytes)` - Write to file
- `rt_file_pwrite(fd, buf, count, offset, out_bytes)` - Write to file at offset
- `rt_file_sync(fd)` - Sync file to disk

**Note:** File operations block until complete. No timeout parameter.

### Network I/O

- `rt_net_listen(port, out_fd)` - Create TCP listening socket (backlog hardcoded to 5)
- `rt_net_accept(listen_fd, out_fd, timeout_ms)` - Accept incoming connection
- `rt_net_connect(ip, port, out_fd, timeout_ms)` - Connect to remote server (numeric IPv4 only)
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
- `rt_bus_entry_count(bus)` - Get number of entries in bus

## Implementation Details

### Event Loop Architecture

The runtime is **completely single-threaded** with an event loop architecture. All actors run cooperatively in a single scheduler thread. There are no I/O worker threads - all I/O operations are integrated into the scheduler's event loop.

**Linux (epoll)**:
- Timers: `timerfd` registered in `epoll`
- Network: Non-blocking sockets registered in `epoll`
- File: Direct synchronous I/O (regular files don't work with epoll)
- Event loop: `epoll_wait()` blocks when no actors runnable

**STM32 (bare metal)**:
- Timers: Hardware timers (SysTick or TIM peripherals)
- Network: lwIP in NO_SYS mode (polling or interrupt-driven)
- File: Direct synchronous I/O (FATFS/littlefs are fast, <1ms typically)
- Event loop: WFI (Wait For Interrupt) when no actors runnable

This single-threaded model provides:
- Maximum simplicity (no lock ordering, no deadlock)
- Maximum determinism (no lock contention, no priority inversion)
- Maximum performance (zero lock overhead, no cache line bouncing)
- Safety-critical compliance (fully deterministic behavior)

### Thread Safety

All runtime APIs must be called from actor context (the scheduler thread). The runtime uses **zero synchronization primitives** in the core event loop:

- No mutexes (single thread, no contention)
- No atomics (single writer/reader per data structure)
- No condition variables (event loop uses epoll/select for waiting)
- No locks (mailboxes, actor state, bus state accessed only by scheduler thread)

**Reentrancy constraint:** Runtime APIs are **not reentrant**. Actors **must not** call runtime APIs from signal handlers or interrupt service routines (ISRs). Violating this results in undefined behavior.

**External thread communication:** External threads cannot call runtime APIs directly. Use platform-specific IPC (sockets/pipes) with dedicated reader actors to bridge external threads into the actor system. See `SPEC.md` "Thread Safety" section for complete details.

## Future Work

- Port to ARM Cortex-M bare metal (STM32)
