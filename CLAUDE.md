# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an actor-based runtime for embedded systems, targeting STM32 (ARM Cortex-M) autopilot applications. The runtime implements cooperative multitasking with message passing, inspired by Erlang's actor model.

**Language:** Modern C (C11 or later)

**Target Platforms:**
- Development: Linux x86-64
- Production: STM32 (ARM Cortex-M) with FreeRTOS

## Design Principles

- Minimalistic and predictable behavior
- Pool-based allocation: O(1) for all runtime operations, zero malloc after initialization
- Least surprise API design
- Fast context switching via manual assembly (no setjmp/longjmp or ucontext)
- Cooperative multitasking only (no preemption within actor runtime)

### Heap Usage Policy

**Allowed malloc**:
- Actor stack allocation: Only if `actor_config.malloc_stack = true` (default is arena allocator)
- Actor cleanup: Corresponding free for malloc'd stacks

**Forbidden malloc** (exhaustive):
- Scheduler, IPC, timers, bus, network I/O, file I/O, linking/monitoring, completion processing
- All runtime operations use static pools with O(1) allocation

## Architecture

The runtime consists of:

1. **Actors**: Cooperative tasks with individual stacks and mailboxes
2. **Scheduler**: Priority-based round-robin scheduler with 4 priority levels (0=CRITICAL to 3=LOW)
3. **IPC**: Inter-process communication via mailboxes with COPY (async) and BORROW (zero-copy with backpressure) modes
4. **Bus**: Publish-subscribe system with configurable retention policies (max_readers, max_age_ms)
5. **I/O Subsystems**: Network, file, and timer operations handled by separate threads/tasks with lock-free completion queues
6. **Completion Queues**: Lock-free SPSC (Single Producer Single Consumer) queues for cross-thread communication

On FreeRTOS, the entire actor runtime runs as a single task. Blocking I/O is delegated to separate FreeRTOS tasks that communicate via lock-free queues.

## Key Concepts

### Scheduling Model
- Actors run until they explicitly yield (via blocking I/O, `rt_yield()`, or `rt_exit()`)
- No preemption or time slicing
- Priority levels: 0 (CRITICAL) to 3 (LOW), with round-robin within each level

### Context Switching
- **x86-64**: Save/restore rbx, rbp, r12-r15, and stack pointer
- **ARM Cortex-M**: Save/restore r4-r11 and stack pointer
- Implemented in manual assembly for performance

### Memory Management
- Actor stacks: Hybrid allocation strategy with variable sizes per actor
  - Default: Static arena allocator (RT_STACK_ARENA_SIZE = 1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
  - Optional: malloc via `actor_config.malloc_stack = true`
- Actor table: Static array (RT_MAX_ACTORS), configured at compile time
- All runtime structures: Static pools with O(1) allocation
  - IPC: Mailbox entry pool (256) and message data pool (256)
  - Links/Monitors: Dedicated pools (128 each)
  - Timers: Timer entry pool (64)
  - Bus: Uses message pool for entry data
- Deterministic memory: ~231KB static + variable actor stacks
- Zero heap allocation in runtime operations (see Heap Usage Policy above)

### Error Handling
All runtime functions return `rt_status` with a code and optional string literal message. The message field is never heap-allocated.

### Actor Lifecycle
- Spawn actors with `rt_spawn()` or `rt_spawn_ex()`
- Actors can link (bidirectional) or monitor (unidirectional) other actors for death notifications
- When an actor dies: mailbox cleared, links/monitors notified, bus subscriptions removed, timers cancelled, resources freed

### IPC Modes
- **IPC_COPY**: Payload copied to receiver's mailbox, sender continues immediately
  - Use for: Small messages, general communication, fire-and-forget
  - Safe default for most use cases

- **IPC_BORROW**: Zero-copy, payload on sender's stack, sender blocks until receiver calls `rt_ipc_release()`
  - Use for: Large data (>1KB), performance-critical paths, trusted cooperating actors
  - WARNING: Requires careful use - actor-only, stack-based, blocking, deadlock risk
  - Preconditions: Data on sender's stack, sender cannot process other messages while blocked
  - Avoid: Circular borrows, nested borrows, untrusted receivers
  - See spec.md "IPC_BORROW Safety Considerations" for full details

### IPC Pool Exhaustion
IPC uses global pools shared by all actors:
- **Mailbox entry pool**: `RT_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `RT_MESSAGE_DATA_POOL_SIZE` (256 default, COPY mode only)

**When pools are exhausted:**
- `rt_ipc_send()` returns `RT_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure (retry, backoff, or discard)

**Notes:**
- No per-actor mailbox limit - pools are shared globally
- BORROW mode does NOT use message data pool (zero-copy)
- Pool exhaustion indicates system overload - increase pool sizes or add backpressure

### Bus Retention
- **max_readers**: Remove entry after N actors read it (0 = persist until aged out or buffer wraps)
- **max_age_ms**: Remove entry after time expires (0 = no time-based expiry)
- Buffer full: Oldest entry evicted on publish

### Bus Pool Exhaustion
Bus shares the message data pool with IPC and has per-bus buffer limits:

**Message Pool Exhaustion** (shared with IPC):
- Bus uses the global `RT_MESSAGE_DATA_POOL_SIZE` pool (same as IPC)
- When pool is exhausted, `rt_bus_publish()` returns `RT_ERR_NOMEM` immediately
- Does NOT block waiting for space
- Does NOT drop messages automatically in this case
- Caller must check return value and handle failure

**Bus Ring Buffer Full** (per-bus limit):
- Each bus has its own ring buffer sized via `max_entries` config
- When ring buffer is full, `rt_bus_publish()` **automatically evicts oldest entry**
- This is different from IPC - bus has automatic message dropping
- Publish succeeds (unless message pool also exhausted)
- Slow readers may miss messages if buffer wraps

**Subscriber Table Full**:
- Each bus has subscriber limit via `max_subscribers` config (up to `RT_MAX_BUS_SUBSCRIBERS`)
- When full, `rt_bus_subscribe()` returns `RT_ERR_NOMEM`

**Key Differences from IPC:**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`RT_MESSAGE_DATA_POOL_SIZE`)

## Important Implementation Details

### Memory Allocation - Compile-Time Configuration

The runtime uses **compile-time configuration** for deterministic memory allocation:

**Compile-Time Limits (`rt_static_config.h`)** - All resource limits defined at compile time:
- `RT_MAX_ACTORS`: Maximum concurrent actors (64)
- `RT_MAX_BUSES`: Maximum concurrent buses (32)
- `RT_MAILBOX_ENTRY_POOL_SIZE`: Mailbox entry pool (256)
- `RT_MESSAGE_DATA_POOL_SIZE`: Message data pool (256)
- `RT_LINK_ENTRY_POOL_SIZE`: Link entry pool (128)
- `RT_MONITOR_ENTRY_POOL_SIZE`: Monitor entry pool (128)
- `RT_TIMER_ENTRY_POOL_SIZE`: Timer entry pool (64)
- `RT_COMPLETION_QUEUE_SIZE`: I/O completion queue size (64)
- `RT_MAX_MESSAGE_SIZE`: Maximum message size in bytes (256)
- `RT_STACK_ARENA_SIZE`: Stack arena size (1*1024*1024) // 1 MB default
- `RT_DEFAULT_STACK_SIZE`: Default actor stack size (65536)

To change these limits, edit `rt_static_config.h` and recompile.

**Memory characteristics:**
- All runtime structures are **statically allocated** based on compile-time limits
- Actor stacks use static arena by default, with optional malloc via `actor_config.malloc_stack`
- No malloc in hot paths (IPC, scheduling, I/O completions)
- Memory footprint calculable at link time
- Zero heap fragmentation in message passing
- Ideal for embedded/safety-critical systems

After `rt_run()` completes, call `rt_cleanup()` to free actor stacks.

### Completion Queue
Lock-free SPSC queue with atomic head/tail pointers. Capacity must be power of 2. Producer (I/O thread) pushes, consumer (scheduler) pops.

### Scheduler Wakeup
When all actors are blocked on I/O, the scheduler efficiently waits on a single shared wakeup primitive (eventfd on Linux, binary semaphore on FreeRTOS) instead of busy-polling. All I/O threads (file, network, timer) signal this primitive after posting completions. This eliminates CPU waste and provides immediate wakeup on I/O completion.

### Thread Safety
Single-threaded runtime model. Only the scheduler thread may call runtime APIs (rt_ipc_send, rt_spawn, etc.). I/O threads may only push to SPSC completion queues and signal scheduler wakeup. External threads cannot call runtime APIs - use platform-specific mechanisms (sockets, pipes) with dedicated reader actors instead. No locks in hot paths (mailboxes, IPC, scheduling) for deterministic behavior. See spec.md "Thread Safety" section for full details.

### Platform Abstraction
Different implementations for Linux (dev) vs FreeRTOS (prod):
- Context switch: x86-64 asm vs ARM Cortex-M asm
- I/O threads: pthreads vs FreeRTOS tasks
- Timer: timerfd vs FreeRTOS timer/hardware timer
- Network: BSD sockets vs lwIP
- File: POSIX vs FATFS/littlefs

### Special Sender IDs
- `RT_SENDER_TIMER` (0xFFFFFFFF): Timer tick messages
- `RT_SENDER_SYSTEM` (0xFFFFFFFE): System messages (e.g., actor death notifications)
