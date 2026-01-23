# Hive - Actor Runtime for Embedded Systems

> **Erlang-style actors in C for embedded systems. Deterministic memory, cooperative scheduling, no GC.**

A complete actor-based runtime designed for **embedded and safety-critical systems**. Features cooperative multitasking with priority-based scheduling and message passing using the actor model.

**Platforms:** x86-64 Linux (fully implemented), STM32/ARM Cortex-M bare metal (core runtime implemented)

The runtime uses **statically bounded memory** for predictable behavior with no heap fragmentation in hot paths (heap allocation optional only for actor stacks). It features **priority-based scheduling** (4 levels: CRITICAL, HIGH, NORMAL, LOW) with fast context switching. Provides message passing (IPC with selective receive and request/reply), linking, monitoring, timers, pub-sub messaging (bus), network I/O, and file I/O.

**Hive is:**
- A single-threaded, cooperative actor runtime
- With Erlang-inspired semantics (mailboxes, selective receive, links, monitors, supervisors)
- Designed for MCUs and safety-conscious embedded systems
- Emphasizing bounded memory, explicit failure handling, and predictable scheduling

**Hive is not:**
- An RTOS (no preemption, no priority inversion protection)
- A hard real-time guarantee engine (no deadline enforcement)
- A distributed actor framework (single process only)
- A statechart or FSM framework

**Hive favors boundedness and inspectability over fairness and throughput.**

## Quick Links

- **[Full Specification](SPEC.md)** - Complete design and implementation details
- **[Examples Directory](examples/)** - Working examples (pingpong, bus, echo server, etc.)
- **[Static Configuration](include/hive_static_config.h)** - Compile-time memory limits and pool sizes
- **[Man Pages](man/man3/)** - API reference documentation

## Man Pages

Comprehensive API documentation is available as Unix man pages:

```bash
# Install man pages
sudo make install-man                    # Install to /usr/local/share/man/man3/
make install-man PREFIX=~/.local         # Install to custom prefix

# View man pages (after install)
man hive_init      # Runtime initialization
man hive_spawn     # Actor lifecycle
man hive_ipc       # Message passing
man hive_link      # Linking and monitoring
man hive_timer     # Timers
man hive_bus       # Pub-sub bus
man hive_select    # Unified event waiting
man hive_net       # Network I/O
man hive_file       # File I/O
man hive_supervisor # Supervision
man hive_types      # Types and compile-time configuration

# View without installing
man man/man3/hive_ipc.3
```

## Features

- Statically bounded memory - Predictable footprint, no fragmentation in hot paths, heap optional only for actor stacks
- Cooperative multitasking with manual assembly context switching (x86-64 and ARM Cortex-M)
- Priority-based round-robin scheduler (4 priority levels)
- Configurable per-actor stack sizes with arena allocator
- Actor lifecycle management (spawn, exit)
- IPC with selective receive and request/reply
- Message classes: NOTIFY (async), REQUEST/REPLY, TIMER, EXIT
- Actor linking and monitoring (bidirectional links, unidirectional monitors)
- Supervision (restart strategies, intensity limiting, child specs)
- Exit notifications with exit reasons (normal, crash, stack overflow, killed)
- Timers (one-shot and periodic with timerfd/epoll)
- Network I/O (non-blocking TCP via event loop)
- File I/O (synchronous, stalls scheduler)
- Logging (compile-time filtering, dual output: console + binary file)
- Bus (pub-sub with retention policies)

## Hive vs QP/C

[QP/C](https://github.com/QuantumLeaps/qpc) is the most comparable embedded actor framework. Both target ARM Cortex-M with static memory allocation.

| Aspect | Hive | QP/C |
|--------|------|------|
| **Core Model** | Actors with mailboxes | Active Objects + Hierarchical State Machines |
| **State Management** | Implicit (actor code) | Explicit UML statecharts |
| **Message Handling** | Selective receive with pattern matching | Event dispatch to state handlers |
| **Blocking** | Actors can block on receive with timeout | Run-to-completion (no blocking in handlers) |
| **Supervision** | Supervisors (restart strategies, intensity) | Less emphasis on fault supervision |
| **Error Philosophy** | "Let it crash" + restart | Defensive, state machine guards |
| **API Style** | Minimalist C functions | Object-oriented C macros |
| **Learning Curve** | Lower (if familiar with actors) | Steeper (requires statechart knowledge) |

**Choose Hive if:** You prefer message-passing with blocking receive, want fault-tolerant supervision, or find state machines overkill for your use case.

**Choose QP/C if:** You need formal state machine modeling, want UML tooling integration, or require safety certification with established track record.

## Cooperative Scheduling

Actors run until they **yield** - there is no preemption. Operations that yield:

| Yields (other actors run) | Never yields |
|---------------------------|--------------|
| `hive_yield()` | `hive_ipc_recv(..., 0)` (timeout=0) |
| `hive_ipc_recv()` (timeout ≠ 0) | `hive_bus_read()` |
| `hive_ipc_recv_match()` | `hive_ipc_notify()` |
| `hive_ipc_request()` | `hive_bus_publish()` |
| `hive_bus_read_wait()` | `hive_select(..., 0)` (timeout=0) |
| `hive_select()` (timeout ≠ 0) | |
| `hive_net_*()` | |
| `hive_exit()` | |

**File I/O** (`hive_file_*`) is different - it stalls the entire scheduler (no actors run). See [SPEC.md](SPEC.md#scheduler-stalling-calls) for details.

## Performance

Benchmarks measured on Intel Core i7-8565U (x86-64 Linux, performance governor):

| Operation | Latency | Throughput | Notes |
|-----------|---------|------------|-------|
| **Context switch** | ~1.1 µs/switch | 0.94 M switches/sec | Manual assembly, cooperative |
| **IPC send/recv** | ~2.0-2.2 µs/msg | 0.46-0.49 M msgs/sec | 8-252 byte messages |
| **Pool allocation** | ~9 ns/op | 111 M ops/sec | 1.1x faster than malloc |
| **Actor spawn** | ~300 ns/actor | 3.3 M actors/sec | Includes stack allocation (arena) |
| **Bus pub/sub** | ~260 ns/msg | 3.84 M msgs/sec | With cooperative yields |

Run benchmarks yourself:
```bash
make bench
```

## Memory Model

The runtime uses **compile-time configuration** for predictable memory allocation.

### Compile-Time Configuration (`include/hive_static_config.h`)

All resource limits are defined at compile time. Edit and recompile to change:

```c
#define HIVE_MAX_ACTORS 64                // Maximum concurrent actors
#define HIVE_STACK_ARENA_SIZE (1*1024*1024) // Stack arena size (1 MB default)
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 256  // Mailbox pool size
#define HIVE_MESSAGE_DATA_POOL_SIZE 256   // Message pool size
#define HIVE_MAX_MESSAGE_SIZE 256         // Max message size (4-byte header + 252 payload)
#define HIVE_MAX_BUSES 32                 // Maximum concurrent buses
// ... see hive_static_config.h for full list
```

All structures are statically allocated. Actor stacks use a static arena allocator by default (configurable size), with optional malloc via `actor_config.malloc_stack = true`. Stack sizes are configurable per actor, allowing different actors to use different stack sizes. Arena memory is automatically reclaimed and reused when actors exit. No malloc in hot paths. Memory footprint calculable at link time when using arena allocator (default); optional malloc'd stacks add runtime-dependent heap usage.

**Embedded footprint:** The defaults above are generous for Linux development. The `examples/pilot/` quadcopter application demonstrates a minimal embedded configuration: 10 actors, 40KB stack arena, 16-entry mailbox pool. The complete pilot firmware (9 actors, flight control, sensor fusion) compiles to ~60KB flash and ~58KB RAM—suitable for microcontrollers like STM32F4.

## Running Examples

```bash
# Basic IPC example
./build/pingpong

# Actor linking example (bidirectional links)
./build/link_demo

# Supervisor (auto-restart workers)
./build/supervisor

# Manual supervisor (link/monitor pattern without hive_supervisor)
./build/supervisor_manual

# File I/O example
./build/fileio

# Network echo server (listens on port 8080)
./build/echo

# Timer example (one-shot and periodic)
./build/timer

# Bus pub-sub example
./build/bus

# Request/reply example (with hive_ipc_request)
./build/request_reply

# Unified event waiting (hive_select)
./build/select

# Priority scheduling example (4 levels, starvation demo)
./build/priority

# Logging example (HIVE_LOG_* macros, binary log file)
./build/logging

# Name registry example (service discovery pattern)
./build/registry
```

## Quick Start

### Basic Actor Example

```c
#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>

void my_actor(void *args, const hive_spawn_info *siblings, size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("Hello from actor %u\n", hive_self());
    hive_exit();
}

int main(void) {
    if (HIVE_FAILED(hive_init())) {
        fprintf(stderr, "Failed to initialize runtime\n");
        return 1;
    }

    actor_id id;
    if (HIVE_FAILED(hive_spawn(my_actor, NULL, NULL, NULL, &id))) {
        fprintf(stderr, "Failed to spawn actor\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    return 0;
}
```

### IPC and Configuration

```c
// Configure actor
actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
cfg.priority = 0;             // 0-3, lower is higher
cfg.stack_size = 128 * 1024;
cfg.malloc_stack = false;     // false=arena (default), true=malloc
cfg.auto_register = false;    // true = auto-register name in registry
actor_id worker;
hive_status status = hive_spawn(worker_actor, NULL, &args, &cfg, &worker);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if actor table or stack arena full
}

// Notify (async message with tag for selective receive)
int data = 42;
status = hive_ipc_notify(target, 0, &data, sizeof(data));  // tag=0
if (HIVE_FAILED(status)) {
    // Pool exhausted: HIVE_MAILBOX_ENTRY_POOL_SIZE or HIVE_MESSAGE_DATA_POOL_SIZE
    // Notify does NOT block or drop - caller must handle HIVE_ERR_NOMEM

    // Backoff and retry pattern:
    hive_message msg;
    hive_ipc_recv(&msg, 10);  // Backoff 10ms (returns timeout or message)
    // Retry notify...
}

// Request/reply pattern: Send request and wait for reply
hive_message reply;
status = hive_ipc_request(target, &data, sizeof(data), &reply, 5000);  // 5s timeout
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if no reply (or target died), HIVE_ERR_NOMEM if pool exhausted
    // If linked/monitoring target, check for EXIT message to distinguish death from timeout
}

// Reply to a REQUEST message (in receiver actor)
hive_ipc_reply(&msg, &response, sizeof(response));

// Receive messages
hive_message msg;
hive_ipc_recv(&msg, -1);   // -1=block forever
hive_ipc_recv(&msg, 0);    // 0=non-blocking (returns HIVE_ERR_WOULDBLOCK if empty)
hive_ipc_recv(&msg, 100);  // 100=timeout after 100ms (returns HIVE_ERR_TIMEOUT if no message)

// Direct field access
my_data *data = (my_data *)msg.data;  // Direct payload access
if (msg.class == HIVE_MSG_REQUEST) {
    // Handle request...
}
```

### Timers

```c
timer_id timer, periodic;
hive_status status = hive_timer_after(500000, &timer);    // One-shot, 500ms
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if timer pool exhausted (HIVE_TIMER_ENTRY_POOL_SIZE)
}
status = hive_timer_every(200000, &periodic); // Periodic, 200ms
if (HIVE_FAILED(status)) {
    // Handle error...
}

// Wait for specific timer using selective receive (recommended)
hive_message msg;
status = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if timeout expires (not possible with -1)
}
// Other messages stay in mailbox, only this timer is consumed

// Or receive any message and check timer_id in msg.tag
hive_ipc_recv(&msg, -1);
if (hive_msg_is_timer(&msg) && msg.tag == periodic) {
    printf("Periodic timer %u fired\n", msg.tag);
}
hive_timer_cancel(periodic);
```

### File and Network I/O

```c
// File operations (block until complete)
int fd;
size_t bytes_written, bytes_read;
hive_status status = hive_file_open("test.txt", HIVE_O_RDWR | HIVE_O_CREAT, 0644, &fd);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_IO on open failure, HIVE_ERR_INVALID for bad flags
}
status = hive_file_write(fd, data, len, &bytes_written);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_IO on write failure
}
status = hive_file_read(fd, buffer, sizeof(buffer), &bytes_read);
// bytes_read == 0 indicates EOF (not an error)
hive_file_close(fd);

// Network server
int listen_fd, client_fd;
status = hive_net_listen(8080, &listen_fd);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_IO if bind/listen fails (port in use, permission denied)
}
status = hive_net_accept(listen_fd, &client_fd, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if timeout expires
}
size_t received, sent;
status = hive_net_recv(client_fd, buffer, sizeof(buffer), &received, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_CLOSED if peer closed connection
}
hive_net_send(client_fd, buffer, received, &sent, -1);
hive_net_close(client_fd);

// Network client
int server_fd;
status = hive_net_connect("127.0.0.1", 8080, &server_fd, 5000);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if connection times out
    // HIVE_ERR_IO if connection refused
}
```

### Bus (Pub-Sub)

```c
hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
cfg.consume_after_reads = 0;   // 0=persist, N=remove after N reads
cfg.max_age_ms = 0;    // 0=no expiry, T=expire after T ms
// Note: Maximum 32 subscribers per bus (architectural limit)

bus_id bus;
hive_status status = hive_bus_create(&cfg, &bus);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if bus table full (HIVE_MAX_BUSES)
}
status = hive_bus_subscribe(bus);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if subscriber table full (HIVE_MAX_BUS_SUBSCRIBERS)
}

sensor_data data = {.temperature = 25.5f};
status = hive_bus_publish(bus, &data, sizeof(data));
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if message pool exhausted (shares HIVE_MESSAGE_DATA_POOL_SIZE with IPC)
    // Note: Ring buffer full automatically drops oldest entry (not an error)
}

sensor_data received;
size_t bytes_read;
status = hive_bus_read_wait(bus, &received, sizeof(received), &bytes_read, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if timeout expires, HIVE_ERR_WOULDBLOCK if no data (timeout=0)
}
```

### Linking and Monitoring

```c
actor_id other;
hive_status status = hive_spawn(other_actor, NULL, NULL, NULL, &other);
if (HIVE_FAILED(status)) {
    // Handle spawn failure...
}
status = hive_link(other);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_INVALID if target doesn't exist
    // HIVE_ERR_NOMEM if link pool exhausted (HIVE_LINK_ENTRY_POOL_SIZE)
}
uint32_t mon_id;
status = hive_monitor(other, &mon_id);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_INVALID if target doesn't exist
    // HIVE_ERR_NOMEM if monitor pool exhausted (HIVE_MONITOR_ENTRY_POOL_SIZE)
}

hive_message msg;
hive_ipc_recv(&msg, -1);
if (hive_is_exit_msg(&msg)) {
    hive_exit_msg exit_info;
    hive_decode_exit(&msg, &exit_info);
    printf("Actor %u died: %s\n", exit_info.actor, hive_exit_reason_str(exit_info.reason));
    // exit_info.reason: HIVE_EXIT_NORMAL, HIVE_EXIT_CRASH, HIVE_EXIT_CRASH_STACK, HIVE_EXIT_KILLED
}
```

### Supervision

```c
#include "hive_supervisor.h"

// Define child specifications
static int worker_ids[2] = {1, 2};
hive_child_spec children[] = {
    {
        .start = worker_actor,            // Actor entry point
        .init = NULL,                     // Init function (NULL = skip)
        .init_args = &worker_ids[0],      // Arguments to actor
        .init_args_size = sizeof(int),    // Copy arg (0 = pass pointer)
        .name = "worker-1",               // For debugging + supervisor tracking
        .auto_register = false,           // Auto-register in name registry
        .restart = HIVE_CHILD_PERMANENT,  // Always restart
        .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
    },
    {
        .start = worker_actor,
        .init = NULL,
        .init_args = &worker_ids[1],
        .init_args_size = sizeof(int),
        .name = "worker-2",
        .auto_register = false,
        .restart = HIVE_CHILD_TRANSIENT,  // Restart only on crash
        .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
    },
};

// Configure supervisor
hive_supervisor_config config = {
    .strategy = HIVE_STRATEGY_ONE_FOR_ONE,  // Restart only failed child
    .max_restarts = 5,                      // Max 5 restarts...
    .restart_period_ms = 10000,             // ...within 10 seconds
    .children = children,
    .num_children = 2,
    .on_shutdown = my_shutdown_callback,    // Optional callback
    .shutdown_ctx = NULL,
};

// Start supervisor
actor_id supervisor;
hive_status status = hive_supervisor_start(&config, NULL, &supervisor);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if supervisor table full (HIVE_MAX_SUPERVISORS)
    // HIVE_ERR_INVALID if config is invalid
}

// Monitor supervisor for shutdown (optional)
uint32_t monitor_id;
status = hive_monitor(supervisor, &monitor_id);
if (HIVE_FAILED(status)) {
    // Handle error...
}

// Stop supervisor gracefully
hive_supervisor_stop(supervisor);

// Restart strategies:
//   HIVE_STRATEGY_ONE_FOR_ONE  - Restart only the failed child
//   HIVE_STRATEGY_ONE_FOR_ALL  - Restart all children if one fails
//   HIVE_STRATEGY_REST_FOR_ONE - Restart failed child and all started after it

// Child restart types:
//   HIVE_CHILD_PERMANENT - Always restart (default)
//   HIVE_CHILD_TRANSIENT - Restart only on crash (not normal exit)
//   HIVE_CHILD_TEMPORARY - Never restart
```

## API Overview

### Runtime Initialization

- `hive_init()` - Initialize the runtime
- `hive_run()` - Run the scheduler (blocks until all actors exit)
- `hive_run_until_blocked()` - Run actors until all are blocked (for external event loop integration)
- `hive_advance_time(delta_us)` - Advance simulation time and fire due timers
- `hive_cleanup()` - Cleanup and free resources
- `hive_shutdown()` - Request graceful shutdown
- `hive_actor_alive(id)` - Check if actor is still alive

### Actor Management

- `hive_spawn(fn, init, init_args, cfg, out)` - Spawn actor (cfg=NULL for defaults)
  - `fn`: Actor function with signature `void fn(void *args, const hive_spawn_info *siblings, size_t sibling_count)`
  - `init`: Optional init function called in spawner context (NULL to skip)
  - `init_args`: Arguments passed to init (or directly to actor if init is NULL)
  - `cfg`: Actor configuration (NULL = defaults), includes `auto_register` for name registry
- `hive_exit()` - Terminate current actor
- `hive_self()` - Get current actor's ID
- `hive_yield()` - Voluntarily yield to scheduler
- `hive_find_sibling(siblings, count, name)` - Find sibling by name in spawn info array

### Name Registry

- `hive_register(name)` - Register calling actor with a name (must be unique)
- `hive_whereis(name, out)` - Look up actor ID by name
- `hive_unregister(name)` - Unregister a name (auto on actor exit)

### IPC

- `hive_ipc_notify(to, tag, data, len)` - Fire-and-forget notification with tag
- `hive_ipc_notify_ex(to, class, tag, data, len)` - Send with explicit class and tag
- `hive_ipc_recv(msg, timeout)` - Receive any message (`msg.class`, `msg.tag`, `msg.data`)
- `hive_ipc_recv_match(from, class, tag, msg, timeout)` - Selective receive with filtering
- `hive_ipc_recv_matches(filters, n, msg, timeout, matched_idx)` - Multi-pattern selective receive
- `hive_ipc_request(to, req, len, reply, timeout)` - Blocking request/reply
- `hive_ipc_reply(request, data, len)` - Reply to a REQUEST message
- `hive_msg_is_timer(msg)` - Check if message is a timer tick
- `hive_ipc_pending()` - Check if messages are available
- `hive_ipc_count()` - Get number of pending messages

### Linking and Monitoring

- `hive_link(target)` - Create bidirectional link
- `hive_link_remove(target)` - Remove bidirectional link
- `hive_monitor(target, out)` - Create unidirectional monitor
- `hive_monitor_cancel(id)` - Cancel monitor
- `hive_is_exit_msg(msg)` - Check if message is exit notification
- `hive_decode_exit(msg, out)` - Decode exit message into `hive_exit_msg` struct
- `hive_exit_reason_str(reason)` - Convert exit reason to string ("NORMAL", "CRASH", etc.)
- `hive_kill(target)` - Kill an actor externally (for supervisor use)

### Supervision

- `hive_supervisor_start(config, actor_cfg, out)` - Start supervisor with child specs
- `hive_supervisor_stop(supervisor)` - Stop supervisor gracefully (terminates all children)
- `hive_restart_strategy_str(strategy)` - Convert strategy to string
- `hive_child_restart_str(restart)` - Convert restart type to string

### Timers

- `hive_timer_after(delay_us, out)` - Create one-shot timer
- `hive_timer_every(interval_us, out)` - Create periodic timer
- `hive_timer_cancel(id)` - Cancel a timer
- `hive_sleep(delay_us)` - Sleep without losing messages (uses selective receive)
- `hive_get_time()` - Get current monotonic time in microseconds
- `hive_msg_is_timer(msg)` - Check if message is a timer tick (also in IPC)

### File I/O

- `hive_file_open(path, flags, mode, fd_out)` - Open file
- `hive_file_close(fd)` - Close file
- `hive_file_read(fd, buf, len, bytes_read)` - Read from file
- `hive_file_pread(fd, buf, len, offset, bytes_read)` - Read from file at offset
- `hive_file_write(fd, buf, len, bytes_written)` - Write to file
- `hive_file_pwrite(fd, buf, len, offset, bytes_written)` - Write to file at offset
- `hive_file_sync(fd)` - Sync file to disk

**Use `HIVE_O_*` flags** for cross-platform compatibility:
- `HIVE_O_RDONLY`, `HIVE_O_WRONLY`, `HIVE_O_RDWR`
- `HIVE_O_CREAT`, `HIVE_O_TRUNC`, `HIVE_O_APPEND`

**Linux:** File operations block until complete. No timeout parameter.

**STM32:** Uses flash-backed virtual files with a ring buffer for efficiency.
Most writes complete immediately. When the buffer fills up, `write()` blocks to flush data to
flash before continuing. Virtual file paths are hardcoded (`/log`, `/config`), enabled by defining their flash layout:
```c
-DHIVE_VFILE_LOG_BASE=0x08020000   // Enables "/log" at this flash address
-DHIVE_VFILE_LOG_SIZE=131072       // Size in bytes (128KB)
-DHIVE_VFILE_LOG_SECTOR=5          // Flash sector number (for erase)
-DHIVE_FILE_RING_SIZE=4096         // RAM ring buffer size
// Optional: -DHIVE_VFILE_CONFIG_BASE/SIZE/SECTOR enables "/config"
```
See `examples/pilot/Makefile.crazyflie-2.1+` for a complete example and SPEC.md for full platform differences.

### Logging

- `HIVE_LOG_TRACE(fmt, ...)` - Verbose tracing (compile out with `-DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_DEBUG`)
- `HIVE_LOG_DEBUG(fmt, ...)` - Debug information
- `HIVE_LOG_INFO(fmt, ...)` - General information (default level)
- `HIVE_LOG_WARN(fmt, ...)` - Warnings
- `HIVE_LOG_ERROR(fmt, ...)` - Errors

**Logging lifecycle** (managed by application):
- `hive_log_init()` - Initialize logging subsystem (called by `hive_init()`)
- `hive_log_file_open(path)` - Open log file (on STM32, erases flash sector)
- `hive_log_file_sync()` - Flush to storage (call periodically)
- `hive_log_file_close()` - Close log file
- `hive_log_cleanup()` - Cleanup logging subsystem

**Compile-time configuration** (`-D` flags or `hive_static_config.h`):
- `HIVE_LOG_LEVEL` - Minimum level to compile (default: `HIVE_LOG_LEVEL_INFO`)
- `HIVE_LOG_TO_STDOUT` - Console output (default: 1 on Linux, 0 on STM32)
- `HIVE_LOG_TO_FILE` - File logging (default: 1 on both)
- `HIVE_LOG_FILE_PATH` - Log file path (default: `/var/tmp/hive.log` on Linux, `/log` on STM32)

**Binary log format:** 12-byte header + text payload. Use `tools/decode_log.py` to decode.

### Network I/O

- `hive_net_listen(port, out_fd)` - Create TCP listening socket (backlog hardcoded to 5)
- `hive_net_accept(listen_fd, out_fd, timeout_ms)` - Accept incoming connection
- `hive_net_connect(ip, port, out_fd, timeout_ms)` - Connect to remote server (numeric IPv4 only)
- `hive_net_send(fd, buf, len, sent, timeout_ms)` - Send data
- `hive_net_recv(fd, buf, len, received, timeout_ms)` - Receive data
- `hive_net_close(fd)` - Close socket

### Bus (Pub-Sub)

- `hive_bus_create(config, out_id)` - Create a new bus with retention policy
- `hive_bus_destroy(bus)` - Destroy a bus
- `hive_bus_subscribe(bus)` - Subscribe current actor to bus
- `hive_bus_unsubscribe(bus)` - Unsubscribe current actor from bus
- `hive_bus_publish(bus, data, len)` - Publish data to bus (non-blocking)
- `hive_bus_read(bus, buf, len, bytes_read)` - Read next message (non-blocking)
- `hive_bus_read_wait(bus, buf, len, bytes_read, timeout_ms)` - Read next message (blocking)
- `hive_bus_entry_count(bus)` - Get number of entries in bus

### Unified Event Waiting

- `hive_select(sources, num_sources, result, timeout_ms)` - Wait on multiple event sources (IPC + bus)

`hive_select()` provides unified waiting on heterogeneous sources:
```c
hive_select_source sources[] = {
    {HIVE_SEL_BUS, .bus = sensor_bus},
    {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, heartbeat}},
    {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_SHUTDOWN}},
};
hive_select_result result;
hive_status status = hive_select(sources, 3, &result, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if timeout expires
    // HIVE_ERR_WOULDBLOCK if no data ready (timeout=0)
}

switch (result.index) {
case 0: /* Bus data: result.bus.data, result.bus.len */ break;
case 1: /* Timer: result.ipc */ break;
case 2: /* Shutdown: result.ipc */ break;
}
```

**Priority:** Strict array order. When multiple sources are ready, the first one in the array wins.

## Implementation Details

### Event Loop and Thread Safety

The runtime is **completely single-threaded**. All actors run cooperatively in a single scheduler thread with zero synchronization primitives (no mutexes, atomics, or locks).

- **Linux:** `epoll` for timers and network; synchronous file I/O
- **STM32:** Hardware timers, WFI for idle, flash-backed virtual files

All runtime APIs must be called from actor context. External threads must use platform IPC (sockets/pipes) with dedicated reader actors.

See [SPEC.md](SPEC.md#thread-safety) for complete thread safety contract and external thread communication patterns.

### Hardware Abstraction Layer (HAL)

The runtime uses a HAL to isolate platform-specific code. Porters implement HAL functions without needing to understand scheduler or timer internals.

```
include/hal/
  hive_hal_time.h      - Time + critical sections (3 functions)
  hive_hal_event.h     - Event loop primitives (5 functions)
  hive_hal_timer.h     - Timer operations (6 functions)
  hive_hal_context.h   - Context switching (1 function + struct)
  hive_hal_file.h      - File I/O (8 functions, optional)
  hive_hal_net.h       - Network I/O (10 functions, optional)

src/hal/
  linux/               - Linux implementation (epoll, POSIX)
  stm32/               - STM32 implementation (WFI, flash)
  template/            - Documented templates for new ports
```

**Platform-independent wrappers** (in `src/`):
- `hive_timer.c` - Timer wrapper (calls HAL timer functions)
- `hive_file.c` - File I/O wrapper (calls HAL file functions)
- `hive_net.c` - Network I/O wrapper (calls HAL network functions)

**Minimum port**: ~15 C functions + 1 assembly function + 1 struct definition

See `src/hal/template/README.md` for the complete porting guide.

## Testing

```bash
# Build and run all tests
make test

# Run with valgrind (memory error detection)
valgrind --leak-check=full ./build/actor_test
valgrind --leak-check=full ./build/ipc_test
# ... etc for each test

```

The test suite includes 22 test programs covering actors, IPC, timers, bus, networking, file I/O, linking, monitoring, supervision, logging, name registry, and edge cases like pool exhaustion.

## Building

```bash
# Linux (default)
make                           # Build for x86-64 Linux

# STM32 (requires ARM cross-compiler)
make PLATFORM=stm32 CC=arm-none-eabi-gcc

# Disable optional subsystems
make ENABLE_NET=0 ENABLE_FILE=0

# STM32 defaults to ENABLE_NET=0 ENABLE_FILE=1
```

## Code Style

The project uses **One True Style (1TBS)** enforced by clang-format.

### Style Summary

- 4-space indent, no tabs
- 80 column limit
- K&R braces (opening brace on same line)
- Pointer on variable: `int *ptr` not `int* ptr`
- No single-line control bodies (always use braces)

### Formatting Code

```bash
# Format specific files
clang-format -i src/*.c include/*.h

# Check without modifying (dry run)
clang-format --dry-run --Werror src/hive_actor.c

# Do NOT run on assembly files (*.S) - will corrupt them
```

### Pre-commit Hook

A pre-commit hook checks formatting before each commit:

```bash
# Install (one-time setup after clone)
git config core.hooksPath scripts

# Bypass if needed
git commit --no-verify
```

### Protecting ASCII Diagrams

Use `// clang-format off` / `// clang-format on` around ASCII art:

```c
// clang-format off
//   Sensor → Bus → Motor
//      ↓
//   Logger
// clang-format on
```

## QEMU Testing

The runtime can be tested on ARM Cortex-M via QEMU emulation:

```bash
# Install prerequisites
sudo apt install gcc-arm-none-eabi qemu-system-arm

# Build and run tests on QEMU
make qemu-test-suite           # Run all compatible tests (19 tests)
make qemu-run-actor_test       # Run specific test

# Build and run examples on QEMU
make qemu-example-suite        # Run all compatible examples (10 examples)
make qemu-example-pingpong     # Run specific example
```

Compatible tests exclude `net_test`, `file_test`, and `logging_test` (require ENABLE_NET/ENABLE_FILE).
Compatible examples exclude `echo`, `fileio`, and `logging` (same reason).

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

Third-party components included in this project have their own licenses documented in
[THIRD_PARTY_LICENSES](THIRD_PARTY_LICENSES).

## Future Work

- STM32: Network I/O (lwIP integration)
- MPU-based stack guard pages for hardware-guaranteed overflow detection
