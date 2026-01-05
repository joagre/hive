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
- No heap allocation in hot paths
- Least surprise API design
- Fast context switching via manual assembly (no setjmp/longjmp or ucontext)
- Cooperative multitasking only (no preemption within actor runtime)

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
- No heap fragmentation: Zero malloc in hot paths (message passing, timers, etc.)

### Error Handling
All runtime functions return `rt_status` with a code and optional string literal message. The message field is never heap-allocated.

### Actor Lifecycle
- Spawn actors with `rt_spawn()` or `rt_spawn_ex()`
- Actors can link (bidirectional) or monitor (unidirectional) other actors for death notifications
- When an actor dies: mailbox cleared, links/monitors notified, bus subscriptions removed, timers cancelled, resources freed

### IPC Modes
- **IPC_COPY**: Payload copied to receiver's mailbox, sender continues immediately (suitable for small messages)
- **IPC_BORROW**: Zero-copy, payload on sender's stack, sender blocks until receiver calls `rt_ipc_release()` (provides backpressure)

### Bus Retention
- **max_readers**: Remove entry after N actors read it (0 = persist until aged out or buffer wraps)
- **max_age_ms**: Remove entry after time expires (0 = no time-based expiry)
- Buffer full: Oldest entry evicted on publish

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

## Future Considerations
- Stack overflow detection (guard patterns or MPU)
- Distributed actors for multi-MCU systems
- Hot code reload
