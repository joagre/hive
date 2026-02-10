# Hive - Actor Runtime for Embedded Systems

> **Erlang-style actors in C for embedded systems. Deterministic memory, cooperative scheduling, no GC.**

## Why Hive?

**No shared state** - Each actor owns its data. No locks, no races. Without shared mutable state, accidental cross-actor corruption is eliminated (though C still allows intentional pointer misuse).

**Predictable scheduling** - Cooperative multitasking means you know exactly when context switches happen only at blocking operations (IPC receive, bus read, select, yield). No preemption surprises, no priority inversion, no mysterious timing bugs.

**Bounded memory** - All memory comes from static pools. No heap allocation. Memory footprint is known at link time.

**Supervision** - Supervisors can restart crashed actors with clean state. Fault isolation without taking down the system.

**Portable** - Develop and debug on Linux, deploy to STM32. Same actor code, same behavior. The HAL is ~16 functions to port to new hardware.

**No RTOS dependency** - Hive is the runtime. No FreeRTOS, no Zephyr, no vendor SDK lock-in.

**Real-world example** - The [pilot example](examples/pilot/) is a quadcopter flight controller running on a Crazyflie 2.1: 12 actors, sensor fusion, cascaded PID, 60KB flash.

See [docs/spec/](docs/spec/) for design details.

---

## Table of Contents

**Overview**
- [Features](#features)
- [Hive vs QP/C](#hive-vs-qpc)
- [Hive vs RTOS](#hive-vs-rtos-freertos-zephyr-etc)

**Core Concepts**
- [Cooperative Scheduling](#cooperative-scheduling)
- [Performance](#performance)
- [Memory Model](#memory-model)

**Getting Started**
- [Quick Start](#quick-start)
- [Running Examples](#running-examples)

**API Reference**
- [API Overview](#api-overview)
- [Implementation Details](#implementation-details)

**Development**
- [Building](#building)
- [Testing](#testing)
- [Code Style](#code-style)
- [QEMU Testing](#qemu-testing)

**Resources**
- [Quick Links](#quick-links)
- [Man Pages](#man-pages)
- [License](#license)
- [Future Work](#future-work)

---

## What Hive Is (and Isn't)

**Hive is** a single-threaded, cooperative actor runtime with Erlang-inspired semantics: mailboxes, selective receive, links, monitors, and supervisors. It's designed for MCUs and safety-conscious embedded systems, emphasizing bounded memory, explicit failure handling, and predictable scheduling.

**Hive is not** an RTOS (no preemption), a hard real-time engine (no deadline enforcement), a distributed framework (single process only), or a statechart library.

**Hive favors boundedness and inspectability over fairness and throughput** (single-threaded, fixed pools, no time-slicing).

**Platforms** - x86-64 Linux (fully implemented), STM32/ARM Cortex-M bare metal (TCP not yet supported)

## Features

- Statically bounded memory - Predictable footprint, no heap allocation
- Cooperative multitasking with manual assembly context switching (x86-64 and ARM Cortex-M)
- Priority-based round-robin scheduler (4 priority levels)
- Configurable per-actor stack sizes with arena allocator
- Actor lifecycle management (spawn, exit)
- IPC with selective receive and request/reply
- Message classes: NOTIFY (async), REQUEST/REPLY, TIMER, EXIT
- Message `id` field for type dispatch, separate from correlation `tag`
- Actor linking and monitoring (bidirectional links, unidirectional monitors)
- Supervision (restart strategies, intensity limiting, child specs)
- Exit notifications with exit reasons (normal, crash, killed)
- Timers (one-shot and periodic with timerfd/epoll)
- TCP (non-blocking via event loop)
- File I/O (POSIX on Linux, flash-backed on STM32 with optional SD card)
- Logging (compile-time filtering, dual output: console + plain text file)
- Bus (pub-sub with retention policies)
- HAL events (ISR-safe signaling to wake actors from hardware interrupts)
- Pool exhaustion handling (optional blocking with priority-ordered wakeup)

## Hive vs QP/C

[QP/C](https://github.com/QuantumLeaps/qpc) is the most comparable embedded actor framework. Both target ARM Cortex-M with static memory allocation.

| Aspect | Hive | QP/C |
|--------|------|------|
| **Core Model** | Actors with mailboxes | Active Objects + Hierarchical State Machines |
| **State Management** | Implicit (actor code) | Explicit UML statecharts |
| **Message Handling** | Selective receive with pattern matching | Event dispatch to state handlers |
| **Execution Model** | Actors yield on wait (IPC, bus, timers); scheduler runs others | Run-to-completion (handlers must not block) |
| **Supervision** | Supervisors (restart strategies, intensity) | Less emphasis on fault supervision |
| **Error Philosophy** | Handle errors locally, supervision as safety net | Defensive, state machine guards |
| **API Style** | Minimalist C functions | Object-oriented C macros |
| **Learning Curve** | Lower (if familiar with actors) | Steeper (requires statechart knowledge) |

**Choose Hive if** - You prefer sequential actor code with blocking operations (IPC, bus, timers, I/O) over event-driven state machines, want fault-tolerant supervision, or find explicit statecharts overkill for your use case.

**Choose QP/C if** - You need formal state machine modeling, want UML tooling integration, or require safety certification with established track record.

## Hive vs RTOS (FreeRTOS, Zephyr, etc.)

Hive is designed for **bare-metal** - it *is* the runtime, not a layer on top of one.

**Why not run Hive on FreeRTOS?**

- **Redundant scheduling** - FreeRTOS has tasks, priorities, preemption. Adding Hive's cooperative scheduler on top means two schedulers.
- **Redundant IPC** - FreeRTOS has queues, semaphores, event groups. Hive has mailboxes, bus. Duplication.
- **Wasted resources** - FreeRTOS task stacks + Hive actor stacks = double memory overhead.

**If you're already on an RTOS** (e.g., ESP32 with ESP-IDF), you're better off using RTOS primitives directly or building lightweight actor patterns on top of them, rather than running Hive inside an RTOS task.

**Hive makes sense when** - You want the full runtime on bare-metal (STM32 and other ARM Cortex-M) where there's no OS, and you want Erlang-style actors, supervision, and message passing without pulling in an RTOS.

## Cooperative Scheduling

Actors run until they **yield** - there is no preemption. Operations that yield:

| Yields (other actors run) | Returns immediately (no yield) |
|---------------------------|-------------------------------|
| `hive_yield()` | `hive_ipc_recv(..., 0)` (timeout=0) |
| `hive_ipc_recv()` (timeout != 0) | `hive_bus_read(..., 0)` (timeout=0) |
| `hive_ipc_recv_match()` | `hive_ipc_notify()` |
| `hive_ipc_request()` | `hive_bus_publish()` |
| `hive_bus_read()` (timeout != 0) | `hive_select(..., 0)` (timeout=0) |
| `hive_select()` (timeout != 0) | |
| `hive_tcp_*()` | |
| `hive_exit()` | |

**File I/O** (`hive_file_*`) is synchronous and briefly stalls the scheduler. This is fine for short, bursty operations; use LOW priority actors for file work. See [docs/spec/design.md](docs/spec/design.md#scheduler-stalling-calls) for details.

## Performance

Benchmarks measured on Intel Core i7-8565U (x86-64 Linux, performance governor):

| Operation | Latency | Throughput | Notes |
|-----------|---------|------------|-------|
| **Context switch** | ~1.1 us/switch | 0.94 M switches/sec | Manual assembly, cooperative |
| **IPC notify/recv** | ~2.0-2.2 us/msg | 0.46-0.49 M msgs/sec | 8-252 byte messages |
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
#define HIVE_MAX_MESSAGE_SIZE 256         // Max message size (6-byte header + 250 payload)
#define HIVE_MAX_BUSES 32                 // Maximum concurrent buses
// ... see include/hive_static_config.h for full list
```

See [`include/hive_static_config.h`](include/hive_static_config.h) for all compile-time configuration options.

All structures are statically allocated. Actor stacks use a static arena allocator by default (configurable size), with optional malloc via `actor_config_t.malloc_stack = true`. Stack sizes are configurable per actor, allowing different actors to use different stack sizes. Arena memory is automatically reclaimed when actors exit, with adjacent free blocks coalesced to prevent fragmentation. No malloc in hot paths. Memory footprint calculable at link time when using arena allocator (default); optional malloc'd stacks add runtime-dependent heap usage.

**Embedded footprint** - The defaults above are generous for Linux development. See the [Pilot Example](#pilot-example-quadcopter-flight-controller) for a minimal embedded configuration (~60KB flash, ~58KB RAM on STM32F4).

### Configuration Hierarchy

Compile-time parameters flow through a hierarchy where later levels override earlier ones:

```
┌─────────────────────────────────────────────────────────────────────┐
│  Level 1: Library Defaults                                         │
│  include/hive_static_config.h                                      │
│  All #ifndef guarded - generous defaults for Linux development     │
│  Example: HIVE_MAX_ACTORS=64, HIVE_STACK_ARENA_SIZE=1MB            │
├─────────────────────────────────────────────────────────────────────┤
│  Level 2: Application Config                                       │
│  Your app's hive_config.mk (included by Makefile)                  │
│  Overrides for your specific application's needs                   │
│  Example: -DHIVE_MAX_ACTORS=16 -DHIVE_DEFAULT_STACK_SIZE=4096      │
├─────────────────────────────────────────────────────────────────────┤
│  Level 3: Board Config                                             │
│  hal/<board>/hive_board_config.mk                                  │
│  Hardware-specific: flash addresses, SD pins, peripherals          │
│  Example: -DHIVE_VFILE_LOG_BASE=0x08080000 -DHIVE_ENABLE_SD=1      │
├─────────────────────────────────────────────────────────────────────┤
│  Level 4: Command Line                                             │
│  make CFLAGS+='-DHIVE_MAX_ACTORS=32'                               │
│  Highest priority - overrides everything above                     │
└─────────────────────────────────────────────────────────────────────┘
```

**How it works:**
- `hive_static_config.h` uses `#ifndef` guards so any `-D` flag takes precedence
- Application Makefiles `-include hive_config.mk` to inherit shared settings
- Board config includes add hardware-specific settings
- Command-line `-D` flags can override any level for testing

**Example flow** (see `examples/pilot/`):
```makefile
# Makefile.crazyflie-2.1plus
include hive_config.mk                      # Level 2: app config
include hal/crazyflie-2.1plus/hive_board_config.mk  # Level 3: board config

# hive_config.mk sets: HIVE_MAX_ACTORS=16, pools, stack sizes
# hive_board_config.mk sets: flash addresses, SD config
# Command line: make ENABLE_SD=1  (overrides Level 3 default)
```

This hierarchy ensures:
- Library has generous defaults for Linux development (works out of the box)
- Applications customize memory for their actor count
- Boards customize hardware-specific addresses
- Users can override anything from command line

## Quick Start

### Basic Actor Example

```c
#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>

void my_actor(void *args, const hive_spawn_info_t *siblings, size_t sibling_count) {
    printf("Hello from actor %u\n", hive_self());
    return;
}

int main(void) {
    if (HIVE_FAILED(hive_init())) {
        fprintf(stderr, "Failed to initialize runtime\n");
        return 1;
    }

    hive_actor_id_t id;
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
// Configure and spawn an actor
hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
cfg.priority = HIVE_PRIORITY_NORMAL;  // 0=CRITICAL, 1=HIGH, 2=NORMAL, 3=LOW
cfg.stack_size = 32 * 1024;           // 32KB stack
cfg.malloc_stack = false;             // false=arena (default), true=malloc
cfg.auto_register = false;            // true = auto-register name in registry
cfg.pool_block = false;               // true = block on pool exhaustion

int worker_id = 1;
hive_actor_id_t worker;
hive_status_t status = hive_spawn(worker_actor, NULL, &worker_id, &cfg, &worker);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if actor table or stack arena full
}

// Notify (fire-and-forget)
// hive_ipc_notify(to, id, data, len) - id is the message type for dispatch
int data = 42;
#define MSG_WORK 1  // Application-defined message type
status = hive_ipc_notify(worker, MSG_WORK, &data, sizeof(data));
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if pool exhausted (when pool_block=false)
    // Notify does NOT block or drop - caller must handle error

    // Backoff and retry pattern:
    hive_message_t msg;
    hive_ipc_recv(&msg, 10);  // Backoff 10ms
    status = hive_ipc_notify(worker, MSG_WORK, &data, sizeof(data));  // Retry
}

// Alternative: Use pool_block=true to yield on pool exhaustion
// (notify/request always succeeds eventually, but can deadlock if no one consumes)

// Request/reply pattern (blocks until reply or timeout)
// hive_ipc_request(to, id, request, len, reply, timeout)
hive_message_t reply;
status = hive_ipc_request(worker, MSG_WORK, &data, sizeof(data), &reply, 5000);  // 5s timeout
if (status.code == HIVE_ERR_CLOSED) {
    // Target died before replying (detected via internal monitor)
} else if (status.code == HIVE_ERR_TIMEOUT) {
    // No reply within timeout
}

// Reply to a REQUEST message (in the receiving actor)
void worker_actor(void *args, const hive_spawn_info_t *siblings, size_t sibling_count) {
    hive_message_t msg;
    hive_ipc_recv(&msg, -1);
    if (msg.class == HIVE_MSG_REQUEST) {
        int result = *(int *)msg.data * 2;  // Process request
        hive_ipc_reply(&msg, &result, sizeof(result));
    }
    return;
}

// Receive with different timeout behaviors
hive_message_t msg;
hive_ipc_recv(&msg, -1);   // Block forever until message arrives
hive_ipc_recv(&msg, 0);    // Non-blocking: HIVE_ERR_WOULDBLOCK if empty
hive_ipc_recv(&msg, 100);  // Wait up to 100ms: HIVE_ERR_TIMEOUT if no message

// Access message fields directly
// msg.id = message type, msg.tag = correlation (for request/reply)
if (msg.class == HIVE_MSG_NOTIFY && msg.id == MSG_WORK) {
    int *value = (int *)msg.data;
    printf("Received work %d from actor %u\n", *value, msg.sender);
}
```

### Timers

```c
hive_timer_id_t timer, periodic;
hive_status_t status = hive_timer_after(500000, &timer);    // One-shot, 500ms
if (HIVE_FAILED(status)) {
    // HIVE_ERR_NOMEM if timer pool exhausted (HIVE_TIMER_ENTRY_POOL_SIZE)
}
status = hive_timer_every(200000, &periodic); // Periodic, 200ms
if (HIVE_FAILED(status)) {
    // Handle error...
}

// Wait for specific timer using hive_timer_recv (recommended)
// hive_timer_recv(timer, msg, timeout)
hive_message_t msg;
status = hive_timer_recv(timer, &msg, -1);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_TIMEOUT if timeout expires (not possible with -1)
}
// Other messages stay in mailbox, only this timer is consumed

// Or receive any message and check hive_timer_id_t in msg.tag
hive_ipc_recv(&msg, -1);
if (hive_msg_is_timer(&msg) && msg.tag == periodic) {
    printf("Periodic timer %u fired\n", msg.tag);
}
hive_timer_cancel(periodic);
```

### File and TCP

```c
// File I/O (blocks until complete - use LOW priority actors for file work)
int fd;
hive_status_t status = hive_file_open("test.txt", HIVE_O_RDWR | HIVE_O_CREAT, 0644, &fd);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_IO on open failure
}

const char *message = "Hello, file!";
size_t bytes_written;
hive_file_write(fd, message, strlen(message), &bytes_written);

char buffer[256];
size_t bytes_read;
hive_file_pread(fd, buffer, sizeof(buffer), 0, &bytes_read);  // Read from offset 0
// bytes_read == 0 indicates EOF (not an error)

hive_file_close(fd);

// TCP echo server (non-blocking via event loop)
int listen_fd, client_fd;
status = hive_tcp_listen(8080, &listen_fd);
if (HIVE_FAILED(status)) {
    // HIVE_ERR_IO if port in use or permission denied
}

status = hive_tcp_accept(listen_fd, &client_fd, -1);  // Block until connection
if (HIVE_SUCCEEDED(status)) {
    size_t received, sent;
    char buf[1024];

    // Echo loop
    while (HIVE_SUCCEEDED(hive_tcp_recv(client_fd, buf, sizeof(buf), &received, -1))) {
        if (received == 0) {
            break;  // Client closed connection
        }
        hive_tcp_send(client_fd, buf, received, &sent, -1);
    }
    hive_tcp_close(client_fd);
}

// TCP client
int server_fd;
status = hive_tcp_connect("127.0.0.1", 8080, &server_fd, 5000);  // 5s timeout
if (status.code == HIVE_ERR_TIMEOUT) {
    // Connection timed out
} else if (status.code == HIVE_ERR_IO) {
    // Connection refused
}
```

### Bus (Pub-Sub)

```c
// Define shared data type
typedef struct {
    float temperature;
    float humidity;
} sensor_reading;

// Create bus with retention policy
hive_bus_config_t cfg = HIVE_BUS_CONFIG_DEFAULT;
cfg.consume_after_reads = 0;  // 0 = persist until buffer wraps
cfg.max_age_ms = 0;           // 0 = no time-based expiry
// Note: Maximum 32 subscribers per bus (architectural limit)

hive_bus_id_t sensor_bus;
hive_status_t status = hive_bus_create(&cfg, &sensor_bus);

// Subscribe to receive data (each actor must subscribe)
hive_bus_subscribe(sensor_bus);

// Publisher: sensor actor publishes readings
sensor_reading reading = {.temperature = 25.5f, .humidity = 60.0f};
status = hive_bus_publish(sensor_bus, &reading, sizeof(reading));
if (status.code == HIVE_ERR_NOMEM) {
    // Message pool exhausted (shares with IPC)
}
// Note: If ring buffer is full, oldest entry is dropped automatically

// Subscriber: consumer actor reads data
sensor_reading received;
size_t len;
status = hive_bus_read(sensor_bus, &received, sizeof(received), &len,
                       HIVE_TIMEOUT_INFINITE);
if (HIVE_SUCCEEDED(status)) {
    printf("Temperature: %.1f\n", received.temperature);
}
```

### Linking and Monitoring

```c
// Links are BIDIRECTIONAL: if either actor dies, the other gets an EXIT message
hive_actor_id_t worker;
hive_spawn(worker_actor, NULL, NULL, NULL, &worker);
hive_link(worker);  // Now linked both ways

// Monitors are UNIDIRECTIONAL: only the monitoring actor gets notified
uint32_t mon_id;
hive_monitor(worker, &mon_id);  // We watch worker, but worker doesn't watch us

// Wait for exit notification (from link or monitor)
hive_message_t msg;
hive_ipc_recv(&msg, -1);
if (hive_msg_is_exit(&msg)) {
    hive_exit_msg_t info;
    hive_decode_exit(&msg, &info);
    printf("Actor %u died (reason=%u)\n", info.actor, (unsigned)info.reason);

    // Distinguish link from monitor notification
    if (info.monitor_id == 0) {
        // From link (bidirectional)
    } else {
        // From monitor (info.monitor_id matches mon_id)
    }
}

// Exit reasons: HIVE_EXIT_REASON_NORMAL, HIVE_EXIT_REASON_CRASH, HIVE_EXIT_REASON_KILLED
```

### Supervision

```c
#include "hive_supervisor.h"

// Worker actor
void worker_actor(void *args, const hive_spawn_info_t *siblings, size_t sibling_count) {
    int id = *(int *)args;
    printf("Worker %d started\n", id);
    // ... do work ...
    // return = NORMAL exit, hive_exit(HIVE_EXIT_REASON_CRASH) = signal failure
}

// Define child specifications
static int worker_ids[2] = {1, 2};
hive_child_spec_t children[] = {
    {
        .start = worker_actor,
        .init_args = &worker_ids[0],
        .init_args_size = sizeof(int),    // Copy arg (supervisor owns the data)
        .name = "worker-1",
        .restart = HIVE_CHILD_PERMANENT,  // Always restart on exit
    },
    {
        .start = worker_actor,
        .init_args = &worker_ids[1],
        .init_args_size = sizeof(int),
        .name = "worker-2",
        .restart = HIVE_CHILD_TRANSIENT,  // Restart only on crash (not normal exit)
    },
};

// Configure and start supervisor
hive_supervisor_config_t config = {
    .strategy = HIVE_STRATEGY_ONE_FOR_ONE,  // Restart only the failed child
    .max_restarts = 5,                      // Max 5 restarts...
    .restart_period_ms = 10000,             // ...within 10 seconds
    .children = children,
    .num_children = 2,
};

hive_actor_id_t supervisor;
hive_supervisor_start(&config, NULL, &supervisor);

// Supervisor runs until: intensity exceeded, all children exit, or hive_supervisor_stop()
hive_supervisor_stop(supervisor);

// Restart strategies:
//   HIVE_STRATEGY_ONE_FOR_ONE  - Restart only the failed child
//   HIVE_STRATEGY_ONE_FOR_ALL  - Restart all children if one fails
//   HIVE_STRATEGY_REST_FOR_ONE - Restart failed child and all started after it

// Child restart types:
//   HIVE_CHILD_PERMANENT - Always restart (default)
//   HIVE_CHILD_TRANSIENT - Restart only on crash (not normal exit)
//   HIVE_CHILD_TEMPORARY - Never restart

// Stable actor IDs: children with .auto_register = true keep the same
// hive_actor_id_t across restarts, so cached IDs and sender filters stay valid.
```

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

# TCP echo server (listens on port 8080)
./build/echo

# Timer example (one-shot and periodic)
./build/timer

# Bus pub-sub example
./build/bus

# Request/reply example (with hive_ipc_request)
./build/request_reply

# Unified event waiting (hive_select)
./build/select

# HAL event example (interrupt-driven actor wakeup)
./build/hal_event

# Priority scheduling example (4 levels, starvation demo)
./build/priority

# Logging example (HIVE_LOG_* macros, log file)
./build/logging

# Name registry example (service discovery pattern)
./build/registry

# Pool exhaustion handling (backpressure demo)
./build/pool_exhaustion
```

### Pilot Example (Quadcopter Flight Controller)

The `examples/pilot/` directory contains a complete quadcopter autopilot targeting real hardware (Crazyflie 2.1+). Not a toy demo: 12 actors, cascaded PID control, sensor fusion, and fail-safe supervision. Compiles to ~60KB flash and ~58KB RAM with a 52KB stack arena.

See [examples/pilot/README.md](examples/pilot/README.md) for build instructions and architecture details.

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
  - `fn`: Actor function with signature `void fn(void *args, const hive_spawn_info_t *siblings, size_t sibling_count)`
  - `init`: Optional init function called in spawner context (NULL to skip)
  - `init_args`: Arguments passed to init (or directly to actor if init is NULL)
  - `cfg`: Actor configuration (NULL = defaults), includes `auto_register` for name registry
- `hive_exit(reason)` - Terminate current actor (or just return for NORMAL exit)
- `hive_self()` - Get current actor's ID
- `hive_yield()` - Voluntarily yield to scheduler
- `hive_find_sibling(siblings, count, name)` - Find sibling by name in spawn info array

### Name Registry

- `hive_register(name)` - Register calling actor with a name (must be unique)
- `hive_whereis(name, out)` - Look up actor ID by name
- `hive_unregister(name)` - Unregister a name (auto on actor exit)

### IPC

Messages have two identifiers: `id` (message type for dispatch) and `tag` (correlation for request/reply).

- `hive_ipc_notify(to, id, data, len)` - Fire-and-forget notification with message type
- `hive_ipc_notify_ex(to, class, id, data, len)` - Notify with explicit class and id
- `hive_ipc_recv(msg, timeout)` - Receive any message (`msg.class`, `msg.id`, `msg.tag`, `msg.data`)
- `hive_ipc_recv_match(from, class, id, msg, timeout)` - Selective receive with filtering
- `hive_ipc_recv_matches(filters, n, msg, timeout, matched_idx)` - Multi-pattern selective receive
- `hive_ipc_request(to, id, req, len, reply, timeout)` - Blocking request/reply
- `hive_ipc_reply(request, data, len)` - Reply to a REQUEST message
- `hive_ipc_named_notify(name, id, data, len)` - Notify by actor name
- `hive_ipc_named_request(name, id, req, len, reply, timeout)` - Request/reply by actor name
- `hive_msg_is_timer(msg)` - Check if message is a timer tick
- `hive_ipc_pending()` - Check if messages are available
- `hive_ipc_count()` - Get number of pending messages

### Linking and Monitoring

- `hive_link(target)` - Create bidirectional link
- `hive_unlink(target)` - Remove bidirectional link
- `hive_monitor(target, out)` - Create unidirectional monitor
- `hive_demonitor(id)` - Cancel monitor
- `hive_msg_is_exit(msg)` - Check if message is exit notification
- `hive_decode_exit(msg, out)` - Decode exit message into `hive_exit_msg_t` struct
- `hive_actor_kill(target)` - Kill an actor externally (for supervisor use)

### Supervision

- `hive_supervisor_start(config, sup_actor_cfg, out)` - Start supervisor with child specs
- `hive_supervisor_stop(supervisor)` - Stop supervisor gracefully (terminates all children)
- `hive_restart_strategy_str(strategy)` - Convert strategy to string
- `hive_child_restart_str(restart)` - Convert restart type to string

### Timers

- `hive_timer_after(delay_us, out)` - Create one-shot timer
- `hive_timer_every(interval_us, out)` - Create periodic timer
- `hive_timer_cancel(id)` - Cancel a timer
- `hive_timer_recv(timer, msg, timeout)` - Wait for a specific timer message
- `hive_sleep(delay_us)` - Sleep without losing messages (uses selective receive)
- `hive_get_time()` - Get current monotonic time in microseconds
- `hive_msg_is_timer(msg)` - Check if message is a timer tick (also in IPC)

### File I/O

Intentionally minimal file interface - cross-platform abstraction for embedded constraints.
Omits seek, stat, readdir, mkdir, unlink by design (lowest common denominator across platforms).

- `hive_file_open(path, flags, mode, out)` - Open file
- `hive_file_close(fd)` - Close file
- `hive_file_read(fd, buf, len, bytes_read)` - Read from file
- `hive_file_pread(fd, buf, len, offset, bytes_read)` - Read from file at offset
- `hive_file_write(fd, buf, len, bytes_written)` - Write to file
- `hive_file_pwrite(fd, buf, len, offset, bytes_written)` - Write to file at offset
- `hive_file_sync(fd)` - Sync file to disk
- `hive_file_mount_available(path)` - Check if mount point is ready (useful for SD card detection)

**Use `HIVE_O_*` flags** for cross-platform compatibility:
- `HIVE_O_RDONLY`, `HIVE_O_WRONLY`, `HIVE_O_RDWR`
- `HIVE_O_CREAT`, `HIVE_O_TRUNC`, `HIVE_O_APPEND`

**Linux** - File operations block until complete. No timeout parameter.

**STM32** - Uses flash-backed virtual files with a ring buffer for efficiency. Optional SD card support via SPI.
Most writes complete immediately. When the buffer fills up, `write()` blocks to flush data to
flash before continuing. Virtual file paths are hardcoded (`/log`, `/config`), enabled by defining their flash layout:
```c
-DHIVE_VFILE_LOG_BASE=0x08020000   // Enables "/log" at this flash address
-DHIVE_VFILE_LOG_SIZE=131072       // Size in bytes (128KB)
-DHIVE_VFILE_LOG_SECTOR=5          // Flash sector number (for erase)
-DHIVE_FILE_RING_SIZE=4096         // RAM ring buffer size
// Optional: -DHIVE_VFILE_CONFIG_BASE/SIZE/SECTOR enables "/config"
```

**STM32 SD Card** (optional) - Build with `ENABLE_SD=1` to enable SD card support via SPI using FatFS.
SD card files are accessed via the `/sd` mount point. Use `hive_file_mount_available("/sd")` to check
if the card is present before opening files. See [docs/spec/api.md](docs/spec/api.md#stm32-sd-card-support-optional)
for details and limitations.

See `examples/pilot/Makefile.crazyflie-2.1plus` for a complete example and [docs/spec/api.md](docs/spec/api.md#file-api) for full platform differences.

### Logging

Intentionally minimal logging optimized for embedded. No log rotation, no remote/syslog,
no JSON - just compile-time filtered levels with dual output (console + file).

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

**Log file format** - Plain text, `[MM:SS.mmm] LEVEL message`. View with `cat`, `tail`, etc.

### TCP

Intentionally minimal TCP interface for actor-based networking. Not a BSD socket replacement.

- `hive_tcp_listen(port, out)` - Create TCP listening socket (backlog hardcoded to 5)
- `hive_tcp_accept(listen_fd, out, timeout_ms)` - Accept incoming connection
- `hive_tcp_connect(ip, port, out, timeout_ms)` - Connect to remote server (numeric IPv4 only)
- `hive_tcp_send(fd, buf, len, bytes_written, timeout_ms)` - Send data
- `hive_tcp_recv(fd, buf, len, bytes_read, timeout_ms)` - Receive data
- `hive_tcp_close(fd)` - Close socket

Omits SSL/TLS, UDP, socket options by design. See `man hive_tcp` for rationale.

### Bus (Pub-Sub)

- `hive_bus_create(config, out_id)` - Create a new bus with retention policy
- `hive_bus_destroy(bus)` - Destroy a bus
- `hive_bus_subscribe(bus)` - Subscribe current actor to bus
- `hive_bus_unsubscribe(bus)` - Unsubscribe current actor from bus
- `hive_bus_publish(bus, data, len)` - Publish data to bus (non-blocking)
- `hive_bus_read(bus, buf, len, bytes_read, timeout_ms)` - Read next message (timeout=0 non-blocking, timeout=-1 infinite)
- `hive_bus_entry_count(bus)` - Get number of entries in bus

### Unified Event Waiting

Embedded code often needs to wait for multiple things: a hardware interrupt, a timer, and a command from another actor. Without unified waiting, you poll in a loop or build a state machine. `hive_select()` handles all sources in one blocking call - the actor sleeps until something is ready.

```c
// IPC filter: {.sender, .class, .id, .tag} - omitted fields default to wildcard
hive_select_source_t sources[] = {
    {.type = HIVE_SEL_HAL_EVENT, .event = uart_rx_event},  // UART RX interrupt
    {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_TIMER, .tag = timeout}},
    {.type = HIVE_SEL_BUS, .bus = sensor_bus},
};
hive_select_result_t result;
hive_select(sources, 3, &result, -1);  // Block until any source ready

switch (result.index) {
case 0: hal_uart_read(buf, len); break;  // HAL event - read the data
case 1: handle_timeout(); break;          // Timer fired
case 2: process(result.bus.data); break;  // Bus data available
}
```

**Source types** - `HIVE_SEL_IPC` (messages), `HIVE_SEL_BUS` (pub-sub), `HIVE_SEL_HAL_EVENT` (hardware interrupts). HAL events are ISR-safe: the ISR calls `hive_hal_event_signal(id)` to wake the actor.

**Priority** - Strict array order. First ready source wins.

**Thin wrappers** - `hive_ipc_recv()`, `hive_bus_read()`, and `hive_event_wait()` are wrappers around `hive_select()` for single-source cases. Use `hive_select()` directly when waiting on multiple sources.

See `man hive_select` for details.

## Implementation Details

### Event Loop and Thread Safety

The runtime is **completely single-threaded**. All actors run cooperatively in a single scheduler thread with zero synchronization primitives (no mutexes, atomics, or locks).

- **Linux** - `epoll` for timers and TCP; synchronous file I/O
- **STM32** - Hardware timers, WFI for idle, flash-backed virtual files, optional SD card

All runtime APIs must be called from actor context. External threads must use platform IPC (sockets/pipes) with dedicated reader actors.

See [docs/spec/design.md](docs/spec/design.md#thread-safety) for complete thread safety contract and external thread communication patterns.

### Hardware Abstraction Layer (HAL)

The runtime uses a HAL to isolate platform-specific code. Porters implement HAL functions without needing to understand scheduler or timer internals.

```
include/hal/
  hive_hal_time.h      - Time + critical sections (3 functions)
  hive_hal_event.h     - Event loop primitives (6 functions)
  hive_hal_timer.h     - Timer operations (6 functions)
  hive_hal_context.h   - Context switching (2 functions + struct)
  hive_hal_file.h      - File I/O (9 functions, optional)
  hive_hal_tcp.h       - TCP (11 functions, optional)

src/hal/
  linux/               - Linux implementation (epoll, POSIX)
  stm32/               - STM32 implementation (WFI, flash)
  template/            - Documented templates for new ports
```

**Platform-independent wrappers** (in `src/`):
- `hive_timer.c` - Timer wrapper (calls HAL timer functions)
- `hive_file.c` - File I/O wrapper (calls HAL file functions)
- `hive_tcp.c` - TCP wrapper (calls HAL TCP functions)

**Minimum port** - ~16 C functions + 1 assembly function + 1 struct definition

See `src/hal/template/README.md` for the complete porting guide.

## Building

```bash
# Linux (default)
make                           # Build for x86-64 Linux

# STM32 (requires ARM cross-compiler)
make PLATFORM=stm32 CC=arm-none-eabi-gcc

# Disable optional subsystems
make ENABLE_TCP=0 ENABLE_FILE=0

# STM32 with SD card support (requires FatFS library)
make PLATFORM=stm32 ENABLE_SD=1

# STM32 defaults to ENABLE_TCP=0 ENABLE_FILE=1 ENABLE_SD=0
```

## Testing

```bash
# Build and run all tests
make test

# Run with valgrind (memory error detection)
valgrind --leak-check=full ./build/actor_test
valgrind --leak-check=full ./build/ipc_test
# ... etc for each test

```

The test suite includes 24 test programs covering actors, IPC, timers, bus, TCP, file I/O, linking, monitoring, supervision, logging, name registry, and edge cases like pool exhaustion.

## Code Style

The project uses **One True Style (1TBS)** enforced by clang-format.

### Style Summary

- 4-space indent, no tabs
- 80 column limit
- K&R braces (opening brace on same line)
- Pointer on variable: `int *ptr` not `int* ptr`
- No single-line control bodies (always use braces)
- Designated initializers only (`{.field = val}`, never positional `{val1, val2}`)
- Explicit boolean checks for non-bool types (`ptr != NULL`, `count > 0`, never `if (ptr)`)
- Use `stdbool.h` and `while (true)` for infinite loops

### Formatting Code

```bash
# Format specific files
clang-format -i src/*.c include/*.h

# Check without modifying (dry run)
clang-format --dry-run --Werror src/hive_actor.c

# Do NOT run on assembly files (*.S) - will corrupt them
```

### Pre-commit Hook

A pre-commit hook auto-formats code and checks Unicode policy before each commit:

```bash
# Install (one-time setup after clone)
git config core.hooksPath scripts

# Bypass if needed
git commit --no-verify
```

The hook enforces:
- Auto-formatting with clang-format
- ASCII-only in prose comments
- Unicode allowed only for diagrams: box-drawing (`─│┌┐└┘├┤┬┴┼`) and arrows (`→←↑↓`)

### Protecting ASCII Diagrams

Use `// clang-format off` / `// clang-format on` around ASCII art:

```c
// clang-format off
//   Sensor -> Bus -> Motor
//      |
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

Compatible tests exclude `tcp_test`, `file_test`, and `logging_test` (require ENABLE_TCP/ENABLE_FILE).
Compatible examples exclude `echo`, `fileio`, and `logging` (same reason).

## Quick Links

- **[Full Specification](docs/spec/)** - Complete design and implementation details
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
man hive_tcp       # TCP
man hive_file       # File I/O
man hive_supervisor # Supervision
man hive_types      # Types and compile-time configuration

# View without installing
man man/man3/hive_ipc.3
```

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.

Third-party components included in this project have their own licenses documented in
[THIRD_PARTY_LICENSES](THIRD_PARTY_LICENSES).

## Future Work

- STM32: TCP (lwIP integration)
- MPU-based stack guard pages for hardware-guaranteed overflow detection
