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
- Actor stacks: Fixed-size, allocated at spawn, no reallocation
- Actor table: Static array configured at runtime init
- Mailbox/bus entries: Simple linked lists or ring buffers from pools
- First version uses malloc/free; future versions may use dedicated pools

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

### Runtime Configuration
The runtime is configured via `rt_config` structure passed to `rt_init()`:
- `default_stack_size`: Default actor stack size in bytes (default: 65536)
- `max_actors`: Maximum number of concurrent actors (default: 64)
- `completion_queue_size`: Size of I/O completion queues for each subsystem (default: 64)

After `rt_run()` completes, call `rt_cleanup()` to free all runtime resources.

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

## Future Considerations (Not in First Version)
- Throttling on IPC send (D-language style backpressure)
- Dedicated memory pools
- Stack overflow detection (guard patterns or MPU)
