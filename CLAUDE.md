# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an actor-based runtime for embedded systems, targeting STM32 (ARM Cortex-M) autopilot applications. The runtime implements cooperative multitasking with message passing using the actor model.

**Language:** Modern C (C11 or later)

**Target Platforms:**
- Development: Linux x86-64
- Production: STM32 (ARM Cortex-M) bare metal

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

## Architecture

The runtime is **completely single-threaded** with an event loop architecture. All actors run cooperatively in a single scheduler thread. There are no I/O worker threads.

The runtime consists of:

1. **Actors**: Cooperative tasks with individual stacks and mailboxes
2. **Scheduler**: Priority-based round-robin scheduler with 4 priority levels (0=CRITICAL to 3=LOW), integrated event loop (epoll on Linux, WFI on STM32)
3. **IPC**: Inter-process communication via mailboxes with selective receive and request/reply support
4. **Bus**: Publish-subscribe system with configurable retention policies (consume_after_reads, max_age_ms)
5. **Timers**: timerfd registered in epoll (Linux), hardware timers on STM32 (SysTick/TIM)
6. **Network**: Non-blocking sockets registered in epoll (Linux), lwIP NO_SYS mode on STM32
7. **File**: Synchronous I/O (stalls scheduler; regular files don't work with epoll; embedded filesystems are fast). **Safety-critical caveat:** Restrict file I/O to initialization, shutdown, or non–time-critical phases

## Key Concepts

### Scheduling Model
- Actors run until they explicitly yield (via blocking I/O, `acrt_yield()`, or `acrt_exit()`)
- No preemption or time slicing
- Priority levels: 0 (CRITICAL) to 3 (LOW), with round-robin within each level

### Reentrancy Constraint
- **Runtime APIs are NOT reentrant**
- **Actors MUST NOT call runtime APIs from signal handlers or interrupt service routines (ISRs)**
- All runtime API calls must occur from actor context (the scheduler thread)
- Violating this constraint results in undefined behavior (data corruption, crashes)

### Context Switching
- **x86-64**: Save/restore rbx, rbp, r12-r15, and stack pointer
- **ARM Cortex-M**: Save/restore r4-r11 and stack pointer
- Implemented in manual assembly for performance

### Memory Management
- Actor stacks: Hybrid allocation strategy with variable sizes per actor
  - Default: Static arena allocator (ACRT_STACK_ARENA_SIZE = 1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
  - Optional: malloc via `actor_config.malloc_stack = true`
- Actor table: Static array (ACRT_MAX_ACTORS), configured at compile time
- Hot path structures: Static pools with O(1) allocation
  - IPC: Mailbox entry pool (256), message data pool (256)
  - Links/Monitors: Dedicated pools (128 each)
  - Timers: Timer entry pool (64)
  - Bus: Uses message pool for entry data
- Cold path structures: Arena allocator with O(n) first-fit (bounded by free blocks)
  - Actor stacks: ACRT_STACK_ARENA_SIZE (1 MB default)
- Deterministic memory: ~1.2 MB static (incl. stack arena) + optional malloc'd stacks
- Zero heap allocation in runtime operations (see Heap Usage Policy above)

### Error Handling
All runtime functions return `acrt_status` with a code and optional string literal message. The message field is never heap-allocated.

Convenience macros:
- `ACRT_SUCCESS` - Success status
- `ACRT_FAILED(s)` - Check if status indicates failure
- `ACRT_ERROR(code, msg)` - Create error status
- `ACRT_ERR_STR(s)` - Get error message string (handles NULL msg)

### Actor Lifecycle
- Spawn actors with `acrt_spawn()` or `acrt_spawn_ex()`
- Actors can link (bidirectional) or monitor (unidirectional) other actors for death notifications
- When an actor dies: mailbox cleared, links/monitors notified, bus subscriptions removed, timers cancelled, resources freed

### Exit Message Handling
Exit messages are received when linked/monitored actors die:
```c
if (msg.class == ACRT_MSG_EXIT) {
    acrt_exit_msg *exit_info = (acrt_exit_msg *)msg.data;
    printf("Actor %u died: %s\n", exit_info->actor, acrt_exit_reason_str(exit_info->reason));
}
```
- `acrt_exit_reason_str(reason)` returns "NORMAL", "CRASH", "STACK_OVERFLOW", or "KILLED"
- Exit reasons: `ACRT_EXIT_NORMAL`, `ACRT_EXIT_CRASH`, `ACRT_EXIT_CRASH_STACK`, `ACRT_EXIT_KILLED`

### IPC Message Format
All messages have a 4-byte header prepended to payload:
- **class** (4 bits): Message type (NOTIFY, REQUEST, REPLY, TIMER, EXIT)
- **gen** (1 bit): Generated tag flag (1 = runtime-generated, 0 = user-provided)
- **tag** (27 bits): Correlation identifier for request/reply

### IPC API
- **`acrt_ipc_notify(to, data, len)`**: Fire-and-forget notification (class=NOTIFY)
- **`acrt_ipc_recv(msg, timeout)`**: Receive any message
- **`acrt_ipc_recv_match(from, class, tag, msg, timeout)`**: Selective receive with filtering
- **`acrt_ipc_request(to, req, len, reply, timeout)`**: Blocking request/reply (send REQUEST, wait for REPLY)
- **`acrt_ipc_reply(request, data, len)`**: Reply to a REQUEST message

### Message Structure (Pre-decoded)
The `acrt_message` struct provides direct access to all fields - no `acrt_msg_decode()` needed:
```c
acrt_message msg;
acrt_ipc_recv(&msg, -1);
my_data *data = (my_data *)msg.data;  // Direct payload access
if (msg.class == ACRT_MSG_REQUEST) { ... }  // Pre-decoded class
// msg.tag also available directly
```

### Selective Receive
- `acrt_ipc_recv_match()` scans mailbox for messages matching filter criteria
- Non-matching messages are **skipped but not dropped** - they remain in mailbox
- Filter on sender (`ACRT_SENDER_ANY` = wildcard), class (`ACRT_MSG_ANY`), tag (`ACRT_TAG_ANY`)
- Enables request/reply pattern: send REQUEST with generated tag, wait for REPLY with matching tag

### IPC Pool Exhaustion
IPC uses global pools shared by all actors:
- **Mailbox entry pool**: `ACRT_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `ACRT_MESSAGE_DATA_POOL_SIZE` (256 default)

**When pools are exhausted:**
- `acrt_ipc_notify()` returns `ACRT_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure (retry, backoff, or discard)

**Notes:**
- No per-actor mailbox limit - pools are shared globally
- Use `acrt_ipc_request()` for natural backpressure (sender waits for reply)
- Pool exhaustion indicates system overload - increase pool sizes or add backpressure

### Bus Retention
- **consume_after_reads**: Remove entry after N actors read it (0 = persist until aged out or buffer wraps)
- **max_age_ms**: Remove entry after time expires (0 = no time-based expiry)
- Buffer full: Oldest entry evicted on publish

### Bus Pool Exhaustion
Bus shares the message data pool with IPC and has per-bus buffer limits:

**Message Pool Exhaustion** (shared with IPC):
- Bus uses the global `ACRT_MESSAGE_DATA_POOL_SIZE` pool (same as IPC)
- When pool is exhausted, `acrt_bus_publish()` returns `ACRT_ERR_NOMEM` immediately
- Does NOT block waiting for space
- Does NOT drop messages automatically in this case
- Caller must check return value and handle failure

**Bus Ring Buffer Full** (per-bus limit):
- Each bus has its own ring buffer sized via `max_entries` config
- When ring buffer is full, `acrt_bus_publish()` **automatically evicts oldest entry**
- This is different from IPC - bus has automatic message dropping
- Publish succeeds (unless message pool also exhausted)
- Slow readers may miss messages if buffer wraps

**Subscriber Table Full**:
- Each bus has subscriber limit via `max_subscribers` config (up to `ACRT_MAX_BUS_SUBSCRIBERS`)
- When full, `acrt_bus_subscribe()` returns `ACRT_ERR_NOMEM`

**Key Differences from IPC:**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`ACRT_MESSAGE_DATA_POOL_SIZE`)

## Important Implementation Details

### Memory Allocation - Compile-Time Configuration

The runtime uses **compile-time configuration** for deterministic memory allocation:

**Compile-Time Limits (`acrt_static_config.h`)** - All resource limits defined at compile time:
- `ACRT_MAX_ACTORS`: Maximum concurrent actors (64)
- `ACRT_MAX_BUSES`: Maximum concurrent buses (32)
- `ACRT_MAILBOX_ENTRY_POOL_SIZE`: Mailbox entry pool (256)
- `ACRT_MESSAGE_DATA_POOL_SIZE`: Message data pool (256)
- `ACRT_LINK_ENTRY_POOL_SIZE`: Link entry pool (128)
- `ACRT_MONITOR_ENTRY_POOL_SIZE`: Monitor entry pool (128)
- `ACRT_TIMER_ENTRY_POOL_SIZE`: Timer entry pool (64)
- `ACRT_MAX_MESSAGE_SIZE`: Maximum message size in bytes (256, with 4-byte header = 252 payload)
- `ACRT_STACK_ARENA_SIZE`: Stack arena size (1*1024*1024) // 1 MB default
- `ACRT_DEFAULT_STACK_SIZE`: Default actor stack size (65536)

To change these limits, edit `acrt_static_config.h` and recompile.

**Memory characteristics:**
- All runtime structures are **statically allocated** based on compile-time limits
- Actor stacks use static arena by default, with optional malloc via `actor_config.malloc_stack`
- No malloc in hot paths (IPC, scheduling, I/O operations)
- Memory footprint calculable at link time
- Zero heap fragmentation in message passing
- Ideal for embedded/safety-critical systems

After `acrt_run()` completes, call `acrt_cleanup()` to free actor stacks.

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
- CANNOT call runtime APIs (acrt_ipc_notify NOT THREAD-SAFE - no locking/atomics)
- Must use platform-specific IPC (sockets, pipes) with dedicated reader actors

See SPEC.md "Thread Safety" section for full details.

### Platform Abstraction
Different implementations for Linux (dev) vs STM32 bare metal (prod):
- Context switch: x86-64 asm vs ARM Cortex-M asm
- Event notification: epoll vs WFI + interrupt flags
- Timer: timerfd + epoll vs hardware timers (SysTick/TIM)
- Network: Non-blocking BSD sockets + epoll vs lwIP NO_SYS mode
- File: Synchronous POSIX vs synchronous FATFS/littlefs

### Message Classes
Messages are identified by class (accessible directly via `msg.class`):
- `ACRT_MSG_NOTIFY`: Fire-and-forget notification
- `ACRT_MSG_REQUEST`: Request expecting a reply
- `ACRT_MSG_REPLY`: Response to a REQUEST
- `ACRT_MSG_TIMER`: Timer tick (`msg.tag` contains timer_id)
- `ACRT_MSG_EXIT`: System notification (e.g., actor death)
- `ACRT_MSG_ANY`: Wildcard for selective receive filtering

Check message type directly: `if (msg.class == ACRT_MSG_TIMER) { ... }` or use `acrt_msg_is_timer(&msg)`.
