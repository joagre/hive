# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an actor-based runtime for embedded systems, targeting STM32 (ARM Cortex-M) autopilot applications. The runtime implements cooperative multitasking with message passing using the actor model.

**Language:** Modern C (C11 or later)

**Target Platforms:**
- Development: Linux x86-64
- Production: STM32 (ARM Cortex-M) bare metal

## Documentation

- **SPEC.md** - Complete design specification
- **README.md** - Quick start and API overview
- **man/man3/** - Unix man pages for all API functions

Man pages available:
- `hive_init(3)` - Runtime initialization, run loop, shutdown
- `hive_spawn(3)` - Actor lifecycle, priority, stack allocation
- `hive_ipc(3)` - Message passing, request/reply, selective receive
- `hive_link(3)` - Linking and monitoring for death notifications
- `hive_supervisor(3)` - Supervision (restart strategies, child specs)
- `hive_timer(3)` - One-shot and periodic timers
- `hive_bus(3)` - Publish-subscribe bus
- `hive_net(3)` - Non-blocking TCP network I/O
- `hive_file(3)` - Synchronous file I/O
- `hive_types(3)` - Types, constants, compile-time configuration

View without installing: `man man/man3/hive_ipc.3`
Install: `make install-man` (or `sudo make install-man`)

## Design Principles

- Minimalistic and predictable behavior
- Statically bounded memory: All runtime memory is statically bounded. Heap allocation forbidden in hot paths, optional only for actor stacks (`malloc_stack = true`)
- Pool-based allocation: O(1) for hot paths (IPC, scheduling, I/O); O(n) bounded arena for cold paths (spawn/exit)
- Least surprise API design
- Fast context switching via manual assembly (no setjmp/longjmp or ucontext)
- Cooperative multitasking only (no preemption within actor runtime)

### Heap Usage Policy

**Allowed malloc**:
- Actor stack allocation: Only if `actor_config.malloc_stack = true` (default is arena allocator)
- Actor cleanup: Corresponding free for malloc'd stacks

**Forbidden malloc** (exhaustive):
- Scheduler, IPC, timers, bus, network I/O, file I/O, linking/monitoring, I/O event processing
- Hot path operations use static pools with O(1) allocation

### Unicode in Comments

- **Allowed**: Unicode box-drawing characters (`┌─┐│└┘├┤┬┴┼`) for diagrams (state machines, memory layouts, protocol formats)
- **Forbidden**: Unicode in regular prose comments — use ASCII only

## Architecture

The runtime is **completely single-threaded** with an event loop architecture. All actors run cooperatively in a single scheduler thread. There are no I/O worker threads.

The runtime consists of:

1. **Actors**: Cooperative tasks with individual stacks and mailboxes
2. **Scheduler**: Priority-based round-robin scheduler with 4 priority levels (0=CRITICAL to 3=LOW), integrated event loop (epoll on Linux, WFI on STM32)
3. **IPC**: Inter-process communication via mailboxes with selective receive and request/reply support
4. **Bus**: Publish-subscribe system with configurable retention policies (consume_after_reads, max_age_ms)
5. **Timers**: timerfd registered in epoll (Linux), hardware timers on STM32 (SysTick/TIM)
6. **Network**: Non-blocking sockets registered in epoll (Linux only; STM32 not yet implemented)
7. **File**: Platform-specific implementation
   - Linux: Synchronous POSIX I/O (stalls scheduler briefly)
   - STM32: Flash-backed virtual files (`/log`, `/config`) with ring buffer for fast writes (blocks when buffer full)
   - Use `HIVE_O_*` flags for cross-platform compatibility
   - **Safety-critical caveat:** Restrict file I/O to initialization, shutdown, or non–time-critical phases
8. **Logging**: Structured logging with compile-time filtering
   - Log levels: TRACE, DEBUG, INFO, WARN, ERROR, NONE
   - Dual output: console (stderr) + binary file
   - Platform defaults: Linux (stdout + file), STM32 (file only, disabled by default)
   - File logging managed by application (open/sync/close lifecycle)
9. **Supervisor**: Supervision for automatic child restart
   - Restart strategies: one_for_one, one_for_all, rest_for_one
   - Child restart types: permanent, transient, temporary
   - Restart intensity limiting (max restarts within time period)
10. **Name Registry**: Actor naming (`hive_register`, `hive_whereis`, `hive_unregister`)
    - Actors register with symbolic names, others look up by name
    - Names auto-cleaned on actor exit
    - Use with supervisors: `whereis()` returns fresh actor ID after restart

## Key Concepts

### Scheduling Model
- Actors run until they explicitly yield (via blocking I/O, `hive_yield()`, or `hive_exit()`)
- No preemption or time slicing
- Priority levels: 0 (CRITICAL) to 3 (LOW), with round-robin within each level

### Reentrancy Constraint
- **Runtime APIs are NOT reentrant**
- **Actors MUST NOT call runtime APIs from signal handlers or interrupt service routines (ISRs)**
- All runtime API calls must occur from actor context (the scheduler thread)
- Violating this constraint results in undefined behavior (data corruption, crashes)

### Context Switching
- **x86-64**: Save/restore rbx, rbp, r12-r15, and stack pointer
- **ARM Cortex-M**: Save/restore r4-r11 and stack pointer; on Cortex-M4F with FPU, also saves s16-s31 (conditional on `__ARM_FP`)
- Implemented in manual assembly for performance

### Memory Management
- Actor stacks: Hybrid allocation strategy with variable sizes per actor
  - Default: Static arena allocator (HIVE_STACK_ARENA_SIZE = 1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
  - Optional: malloc via `actor_config.malloc_stack = true`
- Actor table: Static array (HIVE_MAX_ACTORS), configured at compile time
- Hot path structures: Static pools with O(1) allocation
  - IPC: Mailbox entry pool (256), message data pool (256)
  - Links/Monitors: Dedicated pools (128 each)
  - Timers: Timer entry pool (64)
  - Bus: Uses message pool for entry data
- Cold path structures: Arena allocator with O(n) first-fit (bounded by free blocks)
  - Actor stacks: HIVE_STACK_ARENA_SIZE (1 MB default)
- Deterministic memory: ~1.2 MB static (incl. stack arena) + optional malloc'd stacks
- Zero heap allocation in runtime operations (see Heap Usage Policy above)

### Error Handling
All runtime functions return `hive_status` with a code and optional string literal message. The message field is never heap-allocated.

Convenience macros:
- `HIVE_SUCCESS` - Success status
- `HIVE_SUCCEEDED(s)` - Check if status indicates success
- `HIVE_FAILED(s)` - Check if status indicates failure
- `HIVE_ERROR(code, msg)` - Create error status
- `HIVE_ERR_STR(s)` - Get error message string (handles NULL msg)

### Actor Lifecycle
- Spawn actors with `hive_spawn()` or `hive_spawn_ex()`
- Actors can link (bidirectional) or monitor (unidirectional) other actors for death notifications
- When an actor dies: mailbox cleared, links/monitors notified, bus subscriptions removed, timers cancelled, resources freed

### Exit Message Handling
Exit messages are received when linked/monitored actors die:
```c
if (hive_is_exit_msg(&msg)) {
    hive_exit_msg exit_info;
    hive_decode_exit(&msg, &exit_info);
    printf("Actor %u died: %s\n", exit_info.actor, hive_exit_reason_str(exit_info.reason));
}
```
- `hive_is_exit_msg(msg)` checks if message is an exit notification
- `hive_decode_exit(msg, out)` decodes exit message into `hive_exit_msg` struct
- `hive_exit_reason_str(reason)` returns "NORMAL", "CRASH", "STACK_OVERFLOW", or "KILLED"
- Exit reasons: `HIVE_EXIT_NORMAL`, `HIVE_EXIT_CRASH`, `HIVE_EXIT_CRASH_STACK`, `HIVE_EXIT_KILLED`

### IPC Message Format
All messages have a 4-byte header prepended to payload:
- **class** (4 bits): Message type (NOTIFY, REQUEST, REPLY, TIMER, EXIT)
- **gen** (1 bit): Generated tag flag (1 = runtime-generated, 0 = user-provided)
- **tag** (27 bits): Correlation identifier for request/reply

### IPC API
- **`hive_ipc_notify(to, tag, data, len)`**: Fire-and-forget notification (class=NOTIFY) with tag for selective receive
- **`hive_ipc_notify_ex(to, class, tag, data, len)`**: Send with explicit class and tag
- **`hive_ipc_recv(msg, timeout)`**: Receive any message
- **`hive_ipc_recv_match(from, class, tag, msg, timeout)`**: Selective receive with filtering
- **`hive_ipc_request(to, req, len, reply, timeout)`**: Blocking request/reply (send REQUEST, wait for REPLY)
- **`hive_ipc_reply(request, data, len)`**: Reply to a REQUEST message

**`hive_ipc_request()` errors**: Returns `HIVE_ERR_TIMEOUT` if no reply (including when target died), `HIVE_ERR_NOMEM` if pool exhausted, `HIVE_ERR_INVALID` for bad arguments. If linked/monitoring target, check for EXIT message after timeout to distinguish death from timeout.

### Message Structure
The `hive_message` struct provides direct access to all fields:
```c
hive_message msg;
hive_ipc_recv(&msg, -1);
my_data *data = (my_data *)msg.data;  // Direct payload access
if (msg.class == HIVE_MSG_REQUEST) { ... }
// msg.tag also available directly
```

### Selective Receive
- `hive_ipc_recv_match()` scans mailbox for messages matching filter criteria
- Non-matching messages are **skipped but not dropped** - they remain in mailbox
- Filter on sender (`HIVE_SENDER_ANY` = wildcard), class (`HIVE_MSG_ANY`), tag (`HIVE_TAG_ANY`)
- Enables request/reply pattern: send REQUEST with generated tag, wait for REPLY with matching tag

### IPC Pool Exhaustion
IPC uses global pools shared by all actors:
- **Mailbox entry pool**: `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `HIVE_MESSAGE_DATA_POOL_SIZE` (256 default)

**When pools are exhausted:**
- `hive_ipc_notify()` and `hive_ipc_notify_ex()` return `HIVE_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure (retry, backoff, or discard)

**Notes:**
- No per-actor mailbox limit - pools are shared globally
- Use `hive_ipc_request()` for natural backpressure (sender waits for reply)
- Pool exhaustion indicates system overload - increase pool sizes or add backpressure

### Bus Retention
- **consume_after_reads**: Remove entry after N actors read it (0 = persist until aged out or buffer wraps)
- **max_age_ms**: Remove entry after time expires (0 = no time-based expiry)
- Buffer full: Oldest entry evicted on publish

### Bus Pool Exhaustion
Bus shares the message data pool with IPC and has per-bus buffer limits:

**Message Pool Exhaustion** (shared with IPC):
- Bus uses the global `HIVE_MESSAGE_DATA_POOL_SIZE` pool (same as IPC)
- When pool is exhausted, `hive_bus_publish()` returns `HIVE_ERR_NOMEM` immediately
- Does NOT block waiting for space
- Does NOT drop messages automatically in this case
- Caller must check return value and handle failure

**Bus Ring Buffer Full** (per-bus limit):
- Each bus has its own ring buffer sized via `max_entries` config
- When ring buffer is full, `hive_bus_publish()` **automatically evicts oldest entry**
- This is different from IPC - bus has automatic message dropping
- Publish succeeds (unless message pool also exhausted)
- Slow readers may miss messages if buffer wraps

**Subscriber Table Full**:
- Each bus has subscriber limit via `max_subscribers` config (up to `HIVE_MAX_BUS_SUBSCRIBERS`)
- When full, `hive_bus_subscribe()` returns `HIVE_ERR_NOMEM`

**Key Differences from IPC:**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`HIVE_MESSAGE_DATA_POOL_SIZE`)

## Important Implementation Details

### Memory Allocation - Compile-Time Configuration

The runtime uses **compile-time configuration** for deterministic memory allocation:

**Compile-Time Limits (`hive_static_config.h`)** - All resource limits defined at compile time:
- `HIVE_MAX_ACTORS`: Maximum concurrent actors (64)
- `HIVE_MAX_BUSES`: Maximum concurrent buses (32)
- `HIVE_MAILBOX_ENTRY_POOL_SIZE`: Mailbox entry pool (256)
- `HIVE_MESSAGE_DATA_POOL_SIZE`: Message data pool (256)
- `HIVE_LINK_ENTRY_POOL_SIZE`: Link entry pool (128)
- `HIVE_MONITOR_ENTRY_POOL_SIZE`: Monitor entry pool (128)
- `HIVE_TIMER_ENTRY_POOL_SIZE`: Timer entry pool (64)
- `HIVE_MAX_MESSAGE_SIZE`: Maximum message size in bytes (256, with 4-byte header = 252 payload)
- `HIVE_STACK_ARENA_SIZE`: Stack arena size (1*1024*1024) // 1 MB default
- `HIVE_DEFAULT_STACK_SIZE`: Default actor stack size (65536)
- `HIVE_MAX_SUPERVISORS`: Maximum concurrent supervisors (8)
- `HIVE_MAX_SUPERVISOR_CHILDREN`: Maximum children per supervisor (16)
- `HIVE_MAX_REGISTERED_NAMES`: Maximum registered actor names (32)

**Logging Configuration** (also in `hive_static_config.h`):
- `HIVE_LOG_LEVEL`: Minimum log level to compile (default: INFO on Linux, NONE on STM32)
- `HIVE_LOG_TO_STDOUT`: Enable console output (default: 1 on Linux, 0 on STM32)
- `HIVE_LOG_TO_FILE`: Enable file logging code (default: 1 on both)
- `HIVE_LOG_FILE_PATH`: Log file path (default: `/var/tmp/hive.log` on Linux, `/log` on STM32)
- `HIVE_LOG_MAX_ENTRY_SIZE`: Maximum log message size (128)

**STM32 File I/O Configuration** (via -D flags in board Makefile):
- `HIVE_VFILE_LOG_BASE`: Flash base address for `/log`
- `HIVE_VFILE_LOG_SIZE`: Flash size for `/log`
- `HIVE_VFILE_LOG_SECTOR`: Flash sector number for `/log`
- `HIVE_FILE_RING_SIZE`: Ring buffer size (8 KB default)
- `HIVE_FILE_BLOCK_SIZE`: Flash write block size (256 bytes default)

To change these limits, edit `hive_static_config.h` or pass -D flags and recompile.

**Memory characteristics:**
- All runtime structures are **statically allocated** based on compile-time limits
- Actor stacks use static arena by default, with optional malloc via `actor_config.malloc_stack`
- No malloc in hot paths (IPC, scheduling, I/O operations)
- Memory footprint calculable at link time
- Zero heap fragmentation in message passing
- Ideal for embedded/safety-critical systems

After `hive_run()` completes, call `hive_cleanup()` to free actor stacks.

### Event Loop
When all actors are blocked on I/O, the scheduler efficiently waits for I/O events using platform-specific mechanisms:
- **Linux**: `epoll_wait()` blocks until timer fires or socket becomes ready
- **STM32**: `WFI` (Wait For Interrupt) until hardware interrupt occurs

This eliminates busy-polling and CPU waste while providing immediate response to I/O events.

### Thread Safety

The runtime is **completely single-threaded**. All runtime APIs must be called from actor context (the scheduler thread).

**Zero synchronization primitives** in the core event loop:
- No mutexes (single thread, no contention)
- No C11 atomics (single writer/reader per data structure)
- No condition variables (event loop uses epoll/select for waiting)
- No locks (mailboxes, actor state, bus state accessed only by scheduler thread)

**STM32 exception:** ISR-to-scheduler communication uses `volatile bool` flags with interrupt disable/enable. This is a synchronization protocol but not C11 atomics or lock-based synchronization.

**External threads (forbidden):**
- CANNOT call runtime APIs (hive_ipc_notify NOT THREAD-SAFE - no locking/atomics)
- Must use platform-specific IPC (sockets, pipes) with dedicated reader actors

See SPEC.md "Thread Safety" section for full details.

### Platform Abstraction
Different implementations for Linux (dev) vs STM32 bare metal (prod):
- Context switch: x86-64 asm vs ARM Cortex-M asm
- Event notification: epoll vs WFI + interrupt flags
- Timer: timerfd + epoll vs software timer wheel (SysTick/TIM)
- Network: Non-blocking BSD sockets + epoll (Linux only; STM32 not yet implemented)
- File: Synchronous POSIX (`hive_file_linux.c`) vs flash-backed ring buffer (`hive_file_stm32.c`)

Platform-specific source files:
- Scheduler: `hive_scheduler_linux.c` / `hive_scheduler_stm32.c`
- Timer: `hive_timer_linux.c` / `hive_timer_stm32.c`
- Context: `hive_context_x86_64.S` / `hive_context_arm_cm.S`
- File: `hive_file_linux.c` / `hive_file_stm32.c`

**STM32 File I/O Differences:**

The STM32 implementation uses flash-backed virtual files with a ring buffer for efficiency.
Most writes complete immediately (fast path). When the buffer fills up, `write()` blocks
to flush data to flash before continuing. This ensures the same no-data-loss semantics
as Linux while still providing fast writes in the common case.

STM32 restrictions:
- Only virtual paths work (`/log`, `/config`) - arbitrary paths rejected
- `HIVE_O_RDWR` rejected - use `HIVE_O_RDONLY` or `HIVE_O_WRONLY`
- `HIVE_O_WRONLY` requires `HIVE_O_TRUNC` (flash must be erased first)
- `hive_file_read()` returns error - use `hive_file_pread()` instead
- `hive_file_pwrite()` returns error (ring buffer doesn't support random writes)
- Single writer at a time

See SPEC.md "File I/O" section for full platform differences table.

Build commands:
- `make` or `make PLATFORM=linux` - Build for x86-64 Linux
- `make PLATFORM=stm32 CC=arm-none-eabi-gcc` - Build for STM32
- `make ENABLE_NET=0 ENABLE_FILE=0` - Disable optional subsystems

### Message Classes
Messages are identified by class (accessible directly via `msg.class`):
- `HIVE_MSG_NOTIFY`: Fire-and-forget notification
- `HIVE_MSG_REQUEST`: Request expecting a reply
- `HIVE_MSG_REPLY`: Response to a REQUEST
- `HIVE_MSG_TIMER`: Timer tick (`msg.tag` contains timer_id)
- `HIVE_MSG_EXIT`: System notification (e.g., actor death)
- `HIVE_MSG_ANY`: Wildcard for selective receive filtering

Check message type directly: `if (msg.class == HIVE_MSG_TIMER) { ... }` or use `hive_msg_is_timer(&msg)`.

### Waiting for Timer Messages
When waiting for a timer, use selective receive with the specific timer_id:
```c
timer_id my_timer;
hive_timer_after(500000, &my_timer);
hive_message msg;
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, my_timer, &msg, -1);
```
Do NOT use `HIVE_TAG_ANY` for timer messages - always use the timer_id to avoid consuming the wrong timer's message.

### Logging

Structured logging with compile-time level filtering and dual output (console + binary file).

**Log Macros** (compile out based on `HIVE_LOG_LEVEL`):
```c
HIVE_LOG_TRACE(fmt, ...)  // Level 0 - verbose tracing
HIVE_LOG_DEBUG(fmt, ...)  // Level 1 - debug info
HIVE_LOG_INFO(fmt, ...)   // Level 2 - general info (default)
HIVE_LOG_WARN(fmt, ...)   // Level 3 - warnings
HIVE_LOG_ERROR(fmt, ...)  // Level 4 - errors
// HIVE_LOG_LEVEL_NONE (5) disables all logging
```

**File Logging API** (lifecycle managed by application):
```c
hive_log_file_open(path);   // Open log file (on STM32, erases flash sector)
hive_log_file_sync();       // Flush to storage (call periodically)
hive_log_file_close();      // Close log file
```

**Binary Log Format** (12-byte header + text payload):
- Magic: `0x4C47` ("LG" little-endian)
- Sequence number, timestamp (µs), length, level
- Use `tools/decode_log.py` to decode

**Platform Defaults:**
| Config | Linux | STM32 |
|--------|-------|-------|
| `HIVE_LOG_TO_STDOUT` | 1 (enabled) | 0 (disabled) |
| `HIVE_LOG_TO_FILE` | 1 (enabled) | 1 (enabled) |
| `HIVE_LOG_FILE_PATH` | `/var/tmp/hive.log` | `/log` |
| `HIVE_LOG_LEVEL` | INFO | NONE (disabled) |
