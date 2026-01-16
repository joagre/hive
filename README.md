# Actor Runtime for Embedded Systems

> **Lightweight actors for embedded systems. Deterministic memory, no GC.**

A complete actor-based runtime designed for **embedded and safety-critical systems**. Features cooperative multitasking with priority-based scheduling and message passing using the actor model.

**Platforms:** x86-64 Linux (fully implemented), STM32/ARM Cortex-M bare metal (core runtime implemented)

The runtime uses **statically bounded memory** for deterministic behavior with zero heap fragmentation (heap allocation optional only for actor stacks). It features **priority-based scheduling** (4 levels: CRITICAL, HIGH, NORMAL, LOW) with fast context switching. Provides message passing (IPC with selective receive and request/reply), linking, monitoring, timers, pub-sub messaging (bus), network I/O, and file I/O.

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
man hive_net       # Network I/O
man hive_file      # File I/O
man hive_types     # Types and compile-time configuration

# View without installing
man man/man3/hive_ipc.3
```

## Features

- Statically bounded memory - Deterministic footprint, zero fragmentation, heap optional only for actor stacks
- Cooperative multitasking with manual assembly context switching (x86-64 and ARM Cortex-M)
- Priority-based round-robin scheduler (4 priority levels)
- Configurable per-actor stack sizes with arena allocator
- Actor lifecycle management (spawn, exit)
- IPC with selective receive and request/reply
- Message classes: NOTIFY (async), REQUEST/REPLY, TIMER, EXIT
- Actor linking and monitoring (bidirectional links, unidirectional monitors)
- Exit notifications with exit reasons (normal, crash, stack overflow, killed)
- Timers (one-shot and periodic with timerfd/epoll)
- Network I/O (non-blocking TCP via event loop)
- File I/O (synchronous, stalls scheduler)
- Logging (compile-time filtering, dual output: console + binary file)
- Bus (pub-sub with retention policies)

## Cooperative Scheduling

Actors run until they **yield** - there is no preemption. Operations that yield:

| Yields (other actors run) | Never yields |
|---------------------------|--------------|
| `hive_yield()` | `hive_ipc_recv(..., 0)` (timeout=0) |
| `hive_ipc_recv()` (timeout ≠ 0) | `hive_bus_read()` |
| `hive_ipc_recv_match()` | `hive_ipc_notify()` |
| `hive_ipc_request()` | `hive_bus_publish()` |
| `hive_bus_read_wait()` | |
| `hive_net_*()` | |
| `hive_exit()` | |

**File I/O** (`hive_file_*`) is different - it stalls the entire scheduler (no actors run). See [SPEC.md](SPEC.md#scheduler-stalling-calls) for details.

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

# Worker monitoring example
./build/supervisor

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

# Priority scheduling example (4 levels, starvation demo)
./build/priority
```

## Quick Start

### Basic Actor Example

```c
#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>

void my_actor(void *arg) {
    printf("Hello from actor %u\n", hive_self());
    hive_exit();
}

int main(void) {
    hive_init();

    actor_id id;
    hive_spawn(my_actor, NULL, &id);

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
actor_id worker;
hive_spawn_ex(worker_actor, &args, &cfg, &worker);

// Notify (async message with tag for selective receive)
int data = 42;
hive_status status = hive_ipc_notify(target, 0, &data, sizeof(data));  // tag=0
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
timer_id timer;
hive_timer_after(500000, &timer);    // One-shot, 500ms
hive_timer_every(200000, &periodic); // Periodic, 200ms

// Wait for specific timer using selective receive (recommended)
hive_message msg;
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
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
hive_file_open("test.txt", HIVE_O_RDWR | HIVE_O_CREAT, 0644, &fd);
hive_file_write(fd, data, len, &bytes_written);
hive_file_read(fd, buffer, sizeof(buffer), &bytes_read);
hive_file_close(fd);

// Network server
int listen_fd, client_fd;
hive_net_listen(8080, &listen_fd);
hive_net_accept(listen_fd, &client_fd, -1);
hive_net_recv(client_fd, buffer, sizeof(buffer), &received, -1);
hive_net_send(client_fd, buffer, received, &sent, -1);
hive_net_close(client_fd);

// Network client
int server_fd;
hive_net_connect("127.0.0.1", 8080, &server_fd, 5000);
```

### Bus (Pub-Sub)

```c
hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
cfg.consume_after_reads = 0;   // 0=persist, N=remove after N reads
cfg.max_age_ms = 0;    // 0=no expiry, T=expire after T ms
// Note: Maximum 32 subscribers per bus (architectural limit)

bus_id bus;
hive_bus_create(&cfg, &bus);
hive_bus_subscribe(bus);

sensor_data data = {.temperature = 25.5f};
hive_status status = hive_bus_publish(bus, &data, sizeof(data));
if (HIVE_FAILED(status)) {
    // Message pool exhausted (shares HIVE_MESSAGE_DATA_POOL_SIZE with IPC)
    // Note: Ring buffer full automatically drops oldest entry
}

sensor_data received;
size_t bytes_read;
hive_bus_read_wait(bus, &received, sizeof(received), &bytes_read, -1);
```

### Linking and Monitoring

```c
actor_id other;
hive_spawn(other_actor, NULL, &other);
hive_link(other);     // Bidirectional - both get exit notifications
uint32_t mon_id;
hive_monitor(other, &mon_id);  // Unidirectional - only monitor gets notifications

hive_message msg;
hive_ipc_recv(&msg, -1);
if (hive_is_exit_msg(&msg)) {
    hive_exit_msg exit_info;
    hive_decode_exit(&msg, &exit_info);
    printf("Actor %u died: %s\n", exit_info.actor, hive_exit_reason_str(exit_info.reason));
    // exit_info.reason: HIVE_EXIT_NORMAL, HIVE_EXIT_CRASH, HIVE_EXIT_CRASH_STACK, HIVE_EXIT_KILLED
}
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

- `hive_spawn(fn, arg, out)` - Spawn actor with default config
- `hive_spawn_ex(fn, arg, config, out)` - Spawn actor with custom config
- `hive_exit()` - Terminate current actor
- `hive_self()` - Get current actor's ID
- `hive_yield()` - Voluntarily yield to scheduler

### IPC

- `hive_ipc_notify(to, tag, data, len)` - Fire-and-forget notification with tag
- `hive_ipc_notify_ex(to, class, tag, data, len)` - Send with explicit class and tag
- `hive_ipc_recv(msg, timeout)` - Receive any message (`msg.class`, `msg.tag`, `msg.data`)
- `hive_ipc_recv_match(from, class, tag, msg, timeout)` - Selective receive with filtering
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
See `examples/pilot/Makefile.STEVAL-DRONE01` for a complete example and SPEC.md for full platform differences.

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
- Network: Not yet implemented (planned: lwIP in NO_SYS mode)
- File: Flash-backed virtual files (e.g., `/log`) with ring buffer
  - Board config via -D flags: `HIVE_VFILE_LOG_BASE`, `HIVE_VFILE_LOG_SIZE`, `HIVE_VFILE_LOG_SECTOR`
  - `write()` pushes to ring buffer (O(1), never blocks)
  - `sync()` drains ring buffer to flash (blocking)
  - `HIVE_O_TRUNC` erases flash sector (blocks 1-4 seconds)
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

```

The test suite includes 16 test programs covering actors, IPC, timers, bus, networking, file I/O, linking, monitoring, and edge cases like pool exhaustion.

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
# Install (symlink to tracked script)
ln -sf ../../scripts/pre-commit .git/hooks/pre-commit

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
make qemu-test-suite           # Run all compatible tests (14 tests)
make qemu-run-actor_test       # Run specific test

# Build and run examples on QEMU
make qemu-example-suite        # Run all compatible examples (7 examples)
make qemu-example-pingpong     # Run specific example
```

Compatible tests exclude `net_test` and `file_test` (require ENABLE_NET/ENABLE_FILE).
Compatible examples exclude `echo` and `fileio` (same reason).

## Future Work

- STM32: Network I/O (lwIP integration)
- MPU-based stack guard pages for hardware-guaranteed overflow detection
