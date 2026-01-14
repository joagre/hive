# Actor Runtime Specification

## Overview

A minimalistic actor-based runtime designed for **embedded and safety-critical systems**. The runtime implements cooperative multitasking with priority-based scheduling and message passing using the actor model.

**Target use cases:** Drone autopilots, industrial sensor networks, robotics control systems, and other resource-constrained embedded applications requiring structured concurrency.

**Safety-critical caveat:** File I/O operations stall the entire scheduler (see "File I/O"). Safety-critical deployments should restrict file I/O to initialization, shutdown, or non–time-critical phases.

**Design principles:**

1. **Minimalistic**: Only essential features, no bloat
2. **Predictable**: Cooperative scheduling, no surprises
3. **Modern C11**: Clean, safe, standards-compliant code
4. **Statically bounded memory**: All runtime memory is statically bounded and deterministic. Heap allocation is forbidden in hot paths and optional only for actor stacks (`malloc_stack = true`)
5. **Pool-based allocation**: O(1) pools for hot paths; stack arena allocation is bounded and occurs only on spawn/exit
6. **Explicit control**: Actors yield explicitly, no preemption

### Heap Usage Policy

**Allowed heap use** (malloc/free):
- `hive_init()`: None (uses only static/BSS)
- `hive_spawn()` / `hive_spawn_ex()`: Actor stack allocation **only if** `actor_config.malloc_stack = true`
  - Default: Arena allocator (static memory, no malloc)
  - Optional: Explicit malloc via config flag
- Actor exit/cleanup: Corresponding free for malloc'd stacks

**Forbidden heap use** (exhaustive):
- Scheduler loop (`hive_run()`, `hive_yield()`, context switching)
- IPC (`hive_ipc_notify()`, `hive_ipc_notify_ex()`, `hive_ipc_recv()`, `hive_ipc_recv_match()`, `hive_ipc_request()`, `hive_ipc_reply()`)
- Timers (`hive_timer_after()`, `hive_timer_every()`, timer delivery)
- Bus (`hive_bus_publish()`, `hive_bus_read()`, `hive_bus_read_wait()`)
- Network I/O (`hive_net_*()` functions, event loop dispatch)
- File I/O (`hive_file_*()` functions, synchronous execution)
- Linking/monitoring (`hive_link()`, `hive_monitor()`, death notifications)
- All I/O event processing (epoll event dispatch)

**Consequence**: All "hot path" operations (scheduling, IPC, I/O) use **static pools** with **O(1) allocation** and return `HIVE_ERR_NOMEM` on pool exhaustion. Stack allocation (spawn/exit, cold path) uses arena allocator with O(n) first-fit search bounded by number of free blocks. No malloc in hot paths, no heap fragmentation, predictable allocation latency.

**Linux verification**: Run with `LD_PRELOAD` malloc wrapper to assert no malloc calls after `hive_init()` (except explicit `malloc_stack = true` spawns).

## Target Platforms

**Development:** Linux x86-64

**Production:** STM32 (ARM Cortex-M) bare metal

On both platforms, the actor runtime is **single-threaded** with an event loop architecture. All actors run cooperatively in a single scheduler thread. I/O operations use platform-specific non-blocking mechanisms integrated directly into the scheduler's event loop.

## Architecture

```
┌──────────────────────────────────────────────────────────┐
│              Actor Runtime (single-threaded)             │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐         │
│  │ Actor 1 │ │ Actor 2 │ │ Actor 3 │ │   ...   │         │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘         │
│       │           │           │           │              │
│       └───────────┴─────┬─────┴───────────┘              │
│                         │                                │
│              ┌──────────▼──────────┐                     │
│              │     Scheduler       │                     │
│              │  (event loop with   │                     │
│              │       epoll)        │                     │
│              └──────────┬──────────┘                     │
│                         │                                │
│       ┌─────────────────┼─────────────────┐              │
│       │                 │                 │              │
│       ▼                 ▼                 ▼              │
│   ┌──────┐          ┌──────┐          ┌──────┐           │
│   │ IPC  │          │ Bus  │          │Timers│           │
│   │      │          │      │          │(tfd) │           │
│   └──────┘          └──────┘          └──┬───┘           │
│                                          │               │
│                    ┌─────────────────────┼──────┐        │
│                    │                     │      │        │
│                    ▼                     ▼      ▼        │
│               ┌─────────┐         ┌──────────────────┐   │
│               │ Network │         │   File I/O       │   │
│               │(sockets)│         │ (synchronous)    │   │
│               └────┬────┘         └──────────────────┘   │
│                    │                                     │
└────────────────────┼─────────────────────────────────────┘
                     │
            ┌────────▼─────────┐
            │  epoll (Linux)   │
            │  WFI (STM32)     │
            └──────────────────┘
```

## Scheduling

### Cooperative Model

Actors run until they explicitly yield control. The scheduler reschedules an actor only when:

- The actor calls a blocking I/O primitive (net, file, IPC receive)
- The actor explicitly yields via `hive_yield()`
- The actor exits via `hive_exit()`

There is no preemptive scheduling or time slicing within the actor runtime.

**Reentrancy constraint:**
- Runtime APIs are **not reentrant**
- Actors **must not** call runtime APIs from signal handlers or interrupt service routines (ISRs)
- All runtime API calls must occur from actor context (the scheduler thread)
- Violating this constraint results in undefined behavior (data corruption, crashes)

### Actor Blocking Semantics

When an actor calls a blocking API, the following contract applies:

**State transition:**
- Actor transitions to `ACTOR_STATE_WAITING` and yields to scheduler
- Actor is removed from run queue (not schedulable until unblocked)
- Scheduler saves actor context and switches to next runnable actor

**Operations that block (actor yields, other actors run):**
- `hive_ipc_recv()` with timeout > 0 or timeout < 0 (block until message or timeout)
- `hive_ipc_recv_match()` (block until matching message or timeout)
- `hive_ipc_request()` (block until reply or timeout)
- `hive_net_send()`, `hive_net_recv()` (block until at least 1 byte transferred or timeout)
- `hive_bus_read_wait()` (block until data available or timeout)

**Note:** File I/O (`hive_file_*`) is NOT in this list. See "Scheduler-Stalling Calls" below.

**Mailbox availability while blocked:**
- Blocked actors **can** receive mailbox messages
- Message enqueues are scheduler-owned operations (do not require actor to be runnable)
- Enqueued messages are available when actor unblocks

**Unblock conditions:**
- I/O readiness signaled (network socket becomes readable/writable)
- Timer expires (for APIs with timeout)
- Message arrives in mailbox (for `hive_ipc_recv()`, `hive_ipc_recv_match()`, `hive_ipc_request()`)
- Bus data published (for `hive_bus_read_wait()`)
- **Important**: Mailbox arrival only unblocks actors blocked in IPC receive operations, not actors blocked on network I/O or bus read

**Scheduling phase definition:**

A scheduling phase consists of one iteration of the scheduler loop:

1. **Find next runnable actor**: Select highest-priority ready actor (round-robin within priority)
2. **Execute actor**: Run until yield, block, or exit
3. **If no runnable actors**: Call `epoll_wait()` to block until I/O events arrive
4. **Drain epoll events**: Process all returned events before returning to step 1

**Event drain order within a phase:**

When `epoll_wait()` returns multiple ready events (timers and network I/O), they are processed in **array index order** as returned by the kernel. This order is deterministic for a given set of ready file descriptors but is not controllable by the runtime.

For each event:
- **Timer event (timerfd)**: Read timerfd, send timer tick message to actor's mailbox, wake actor
- **Network event (socket)**: Perform I/O operation, store result in actor's `io_status`, wake actor

**Event drain timing (Option A semantics):**
- If runnable actors exist, they run immediately (no `epoll_wait` call)
- `epoll_wait` is only called when the run queue is empty
- All events from a single `epoll_wait` call are drained before selecting the next actor
- This minimizes latency for already-runnable actors at the cost of potentially delayed I/O event processing

**Timeout vs I/O readiness - request state machine:**

Each blocking network request has an implicit state: `PENDING` → `COMPLETED` or `TIMED_OUT`.

**State transitions:**
- Request starts in `PENDING` when actor blocks on network I/O with timeout
- First event processed transitions state out of `PENDING`; subsequent events for same request are **ignored without side effects**

**Tie-break rule (deadline check at wake time):**
- When actor wakes, check: `now >= deadline`?
- If yes: state = `TIMED_OUT`, return `HIVE_ERR_TIMEOUT`, **do not perform I/O even if socket is ready**
- If no: state = `COMPLETED`, perform I/O operation, return result

**Rationale:** This avoids the "read data then discard" oddity. The deadline check is deterministic and independent of epoll event ordering.

**Concrete behavior:**
- If I/O ready **before** timeout: Actor wakes, deadline not reached, I/O performed, success
- If timeout **before** I/O ready: Actor wakes, deadline reached, return timeout (no I/O attempted)
- If both fire in **same** `epoll_wait()`: Actor wakes, deadline check determines outcome (I/O not performed if timed out)

**Request serialization (network I/O only):**
- Applies to: `hive_net_accept()`, `hive_net_connect()`, `hive_net_recv()`, `hive_net_send()` with timeouts
- Constraint: **One outstanding network request per actor** (enforced by actor blocking)
- When timeout occurs: epoll registration cleaned up, any late readiness signal is ignored
- New request from same actor gets fresh epoll state

**Note:** This serialization model does NOT apply to:
- **File I/O**: Stalls scheduler synchronously (no event loop involvement)

**Determinism guarantee (qualified):**

> **Summary:** The runtime is deterministic given a fixed event order, but event order itself is kernel-dependent (epoll). Scheduling policy is deterministic; event arrival order is not.

The runtime provides **conditional determinism**, not absolute determinism:

- **Deterministic policy**: Given identical epoll event arrays in identical order, scheduling decisions are deterministic
- **Source of nondeterminism**: epoll_wait returns ready file descriptors in kernel-determined order (not sorted, not stable)
- **Consequence**: If multiple FDs become ready simultaneously, their processing order is kernel-dependent
- **What we guarantee**:
  - No phantom wakeups (actor only unblocks on specified conditions)
  - Consistent policy (same input → same output)
  - **Concrete wake ordering rule**: When processing I/O events from a single `epoll_wait` call, actors made ready are appended to their priority run queues in event processing order. Within a priority level, the run queue is FIFO.
- **What we do NOT guarantee**:
  - Deterministic ordering when multiple timers/sockets fire in the same epoll_wait call
  - Reproducible event dispatch order across kernel versions or system load conditions

**Design choice**: Sorting epoll events by fd/source-id before dispatch would provide determinism but adds O(n log n) overhead per epoll_wait. For embedded systems prioritizing latency over reproducibility, this overhead is not justified. Applications requiring deterministic replay should use external event logging.

### Scheduler-Stalling Calls

File I/O operations (`hive_file_read()`, `hive_file_write()`, `hive_file_sync()`) are **synchronous** and stall the entire runtime:

**Behavior:**
- Calling actor does NOT transition to `ACTOR_STATE_WAITING`
- The scheduler event loop is paused during the syscall
- All actors are stalled (no actor runs while file I/O executes)
- Timer delivery is suspended during the stall (timerfd expirations accumulate in kernel)
- Network events are not processed during the stall
- After stall resumes: accumulated timer expirations are observed and delivered per tick-coalescing rules (one tick message regardless of expiration count)

**Rationale:**
- Regular files do not work with `epoll` on Linux (always report ready)
- True async file I/O would require `io_uring` (Linux 5.1+) or a thread pool
- For embedded (STM32): FATFS/littlefs operations are typically fast (<1ms)
- Simplicity: no additional complexity for a rarely-needed feature

**Consequences:**
- File I/O breaks real-time latency bounds
- Long file operations delay all actors, timers, and network processing
- Use file I/O sparingly and with small buffers
- For latency-sensitive systems: perform file I/O only during initialization or shutdown

**Design alternatives not implemented:**
- `io_uring`: Would add Linux 5.1+ dependency and significant complexity
- Thread pool: Contradicts single-threaded design; removed in event loop migration
- DMA with completion interrupt (STM32): Could be added for specific flash/SD drivers

### Priority Levels

Four priority levels, lower value means higher priority:

| Level | Name | Typical Use |
|-------|------|-------------|
| 0 | `HIVE_PRIORITY_CRITICAL` | Flight control loop |
| 1 | `HIVE_PRIORITY_HIGH` | Sensor fusion |
| 2 | `HIVE_PRIORITY_NORMAL` | Telemetry |
| 3 | `HIVE_PRIORITY_LOW` | Logging |

The scheduler always picks the highest-priority runnable actor. Within the same priority level, actors are scheduled round-robin.

**Fairness guarantees:**
- Round-robin scheduling within a priority level ensures fairness among actors of equal priority
- The scheduler does **not** guarantee starvation freedom across priority levels
- A continuously runnable high-priority actor can starve lower-priority actors indefinitely
- Applications must design priority hierarchies to avoid starvation scenarios

### Context Switching

Context switching is implemented via manual assembly for performance:

- **x86-64 (Linux):** Save/restore callee-saved registers (rbx, rbp, r12-r15) and stack pointer
- **ARM Cortex-M (STM32):** Save/restore callee-saved registers (r4-r11) and stack pointer. On Cortex-M4F and similar with hardware FPU, also saves/restores FPU registers (s16-s31) when `__ARM_FP` is defined.

No use of setjmp/longjmp or ucontext for performance reasons.

## Thread Safety

### Single-Threaded Event Loop Model

The runtime is **completely single-threaded**. All runtime operations execute in a single scheduler thread with an event loop architecture. There are **no I/O worker threads** - all I/O operations are integrated into the scheduler's event loop using platform-specific non-blocking mechanisms.

**Key architectural invariant:** Only one actor executes at any given time. Actors run cooperatively until they explicitly yield (via `hive_yield()`, blocking I/O, or `hive_exit()`). The scheduler then switches to the next runnable actor or waits for I/O events.

### Runtime API Thread Safety Contract

**All runtime APIs must be called from actor context (the scheduler thread):**

| API Category | Thread Safety | Enforcement |
|-------------|---------------|-------------|
| **Actor APIs** (`hive_spawn`, `hive_exit`, etc.) | Single-threaded only | Must call from actor context |
| **IPC APIs** (`hive_ipc_notify`, `hive_ipc_recv`) | Single-threaded only | Must call from actor context |
| **Bus APIs** (`hive_bus_publish`, `hive_bus_read`) | Single-threaded only | Must call from actor context |
| **Timer APIs** (`hive_timer_after`, `hive_timer_every`) | Single-threaded only | Must call from actor context |
| **File APIs** (`hive_file_read`, `hive_file_write`) | Single-threaded only | Must call from actor context; stalls scheduler |
| **Network APIs** (`hive_net_recv`, `hive_net_send`) | Single-threaded only | Must call from actor context |

**Forbidden from:**
- Signal handlers (not reentrant)
- Interrupt service routines (ISRs on embedded systems)
- External threads (no locking/atomics implemented)

### External Thread Communication Pattern

**Problem:** External threads cannot call runtime APIs directly (no thread safety layer).

**Solution:** Use platform-specific IPC mechanisms with a dedicated reader actor:

```c
// External thread writes to socket/pipe
void external_producer(void) {
    int sock = connect_to_actor_socket();
    write(sock, data, len);  // Platform IPC, not runtime API
}

// Dedicated reader actor bridges external thread -> actors
void socket_reader_actor(void *arg) {
    int sock = listen_and_accept();
    while (1) {
        size_t received;
        hive_net_recv(sock, buf, len, &received, -1);  // Blocks in event loop
        hive_ipc_notify(worker, 0, buf, received);  // Forward to actors
    }
}
```

This pattern is safe because:
- External thread only touches OS-level socket (thread-safe by OS)
- `hive_net_recv()` executes in scheduler thread (actor context)
- `hive_ipc_notify()` executes in scheduler thread (actor context)
- No direct runtime state access from external thread

### Synchronization Primitives

The runtime uses **zero synchronization primitives** in the core event loop:

- **No mutexes** - single thread, no contention
- **No C11 atomics** - single writer/reader per data structure
- **No condition variables** - event loop uses epoll for waiting
- **No locks** - mailboxes, actor state, bus state accessed only by scheduler thread

**STM32 exception:** ISR-to-scheduler communication uses `volatile bool` flags with interrupt disable/enable for safe flag clearing. This is a synchronization protocol but not C11 atomics or lock-based synchronization.

**Optional synchronization** (not part of hot path):
- **Logging:** No synchronization needed (single-threaded)

### Rationale

This pure single-threaded model provides:

- **Maximum simplicity:** No lock ordering, no deadlock, trivial to reason about
- **No threading nondeterminism:** No lock contention, no priority inversion, no data races
- **Maximum performance:** Zero lock overhead, no cache line bouncing, no atomic operations
- **Safety-critical friendly:** No threading edge cases (event dispatch order is kernel-dependent, see "Determinism guarantee")
- **Portability:** Works on platforms without pthread support

**Trade-off:** Cannot leverage multiple CPU cores for I/O parallelism. This is acceptable for embedded systems (typically single-core) and simplifies the implementation dramatically.

### Event Loop Architecture

*Terminology: "Event loop" and "scheduler loop" refer to the same construct - the main loop that dispatches I/O events and schedules actors. This document uses "event loop" as the canonical term.*

**Linux:**
- Timers: `timerfd` registered in `epoll`
- Network: Non-blocking sockets registered in `epoll`
- File: Direct synchronous I/O (regular files don't work with epoll anyway)
- Event loop: `epoll_wait()` with bounded timeout (10ms) for defensive wakeup

**Defensive timeout rationale:** The 10ms bounded timeout guards against lost wakeups, misconfigured epoll registrations, or unexpected platform behavior. It is not required for correctness under ideal conditions but provides a safety net against programming errors or kernel edge cases.

**STM32 (bare metal):**
- Timers: Hardware timers (SysTick or TIM peripherals)
- Network: lwIP in NO_SYS mode with ISR flag protocol (see below)
- File: Direct synchronous I/O (FATFS/littlefs are fast, <1ms typically)
- Event loop: WFI (Wait For Interrupt) when no actors runnable

**lwIP NO_SYS ISR contract:**

In NO_SYS mode, lwIP callbacks can occur from interrupt context. The runtime enforces strict separation:

- **ISR responsibility:** Set flag only (`pending_events |= EVENT_NETWORK`), then return
- **ISR prohibition:** Never call runtime APIs, never call `lwip_input()` or heavy lwIP paths
- **Scheduler responsibility:** Check flags, call `lwip_input()` and process packets from scheduler context
- **Rationale:** Runtime APIs are not reentrant; lwIP in NO_SYS mode is not ISR-safe for most operations

```c
// Separate flags per interrupt source (avoids RMW atomicity issues)
volatile bool net_event_pending = false;
volatile bool timer_event_pending = false;

// ISR: minimal, single-word write (atomic on Cortex-M for aligned bool)
void ETH_IRQHandler(void) {
    ETH_DMAClearITPendingBit(ETH_DMA_IT_R);  // Clear interrupt
    net_event_pending = true;                 // Single store, no RMW
    // DO NOT call lwip_input() or runtime APIs here
}

// Scheduler: processes flags from non-ISR context
if (net_event_pending) {
    __disable_irq();
    net_event_pending = false;
    __enable_irq();
    ethernetif_input(&netif);  // Safe: called from scheduler context
}
```

**Atomicity note:** Using separate `volatile bool` flags per interrupt source avoids the read-modify-write (`|=`) atomicity issue. On Cortex-M, aligned word writes are atomic, but `|=` is not atomic (it's load-or-store). Separate flags ensure the ISR performs only a single store.

**Key insight:** Modern OSes provide non-blocking I/O mechanisms (epoll, kqueue, IOCP). On bare metal, hardware interrupts and WFI provide equivalent functionality. The event loop pattern is standard in async runtimes (Node.js, Tokio, libuv, asyncio).

## Memory Model

### Actor Stacks

Each actor has a fixed-size stack allocated at spawn time. Stack size is configurable per actor via `actor_config.stack_size`, with a system-wide default (`HIVE_DEFAULT_STACK_SIZE`). Different actors can use different stack sizes to optimize memory usage.

Stack growth/reallocation is not supported. Stack overflow results in undefined behavior - proper stack sizing is required (see "Stack Overflow" section).

### Memory Allocation

The runtime uses static allocation for deterministic behavior and suitability for MCU deployment:

**Design Principle:** All memory regions are statically reserved at compile time; allocation within those regions (e.g., stack arena, message pools) occurs at runtime via deterministic algorithms. No heap allocation occurs in hot paths (message passing, scheduling, I/O). Optional malloc for actor stacks when explicitly enabled via `actor_config.malloc_stack = true`.

**Allocation Strategy:**

- **Actor table:** Static array of `HIVE_MAX_ACTORS` (64), configured at compile time
- **Actor stacks:** Hybrid allocation (configurable per actor)
  - Default: Static arena allocator with `HIVE_STACK_ARENA_SIZE` (1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
    - Supports different stack sizes for different actors
  - Optional: malloc via `actor_config.malloc_stack = true`
- **IPC pools:** Static pools with O(1) allocation (hot path)
  - Mailbox entry pool: `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256)
  - Message data pool: `HIVE_MESSAGE_DATA_POOL_SIZE` (256), fixed-size entries of `HIVE_MAX_MESSAGE_SIZE` bytes
- **Link/Monitor pools:** Static pools for actor relationships
  - Link entry pool: `HIVE_LINK_ENTRY_POOL_SIZE` (128)
  - Monitor entry pool: `HIVE_MONITOR_ENTRY_POOL_SIZE` (128)
- **Timer pool:** Static pool of `HIVE_TIMER_ENTRY_POOL_SIZE` (64)
- **Bus storage:** Static arrays per bus
  - Bus entries: Pre-allocated array of `HIVE_MAX_BUS_ENTRIES` (64) per bus
  - Bus subscribers: Pre-allocated array of `HIVE_MAX_BUS_SUBSCRIBERS` (32) per bus
  - Entry data: Uses shared message pool
- **I/O sources:** Pool of `io_source` structures for tracking pending I/O operations in the event loop

**Memory Footprint (estimated, 64-bit Linux build, default configuration):**

*Note: Exact sizes are toolchain-dependent. Run `size build/libhive.a` for precise numbers. Estimates below are for GCC on x86-64 Linux.*

- Static data (BSS): ~1.2 MB total (includes 1 MB stack arena)
  - Stack arena: 1 MB (configurable via `HIVE_STACK_ARENA_SIZE`)
  - Actor table: ~10–15 KB
  - Mailbox pool: ~10–15 KB
  - Message pool: 64 KB (256 × 256 bytes, configurable)
  - Link/monitor pools: ~5 KB
  - Timer pool: ~5 KB
  - Bus tables: ~90 KB
  - I/O source pool: ~5 KB
- Without stack arena: ~190 KB

**Total:** ~1.2 MB static (verify with `size` command; no heap allocation with default arena)

**Benefits:**

- Deterministic memory: Footprint calculable at link time
- Zero heap fragmentation: No malloc after initialization (except explicit `malloc_stack` flag)
- Predictable allocation: Pool exhaustion returns clear errors (`HIVE_ERR_NOMEM`)
- Suitable for safety-critical certification
- Predictable timing: O(1) pool allocation for hot paths, bounded arena allocation for cold paths (spawn/exit)

## Error Handling

All runtime functions return `hive_status`:

```c
typedef enum {
    HIVE_OK = 0,
    HIVE_ERR_NOMEM,
    HIVE_ERR_INVALID,
    HIVE_ERR_TIMEOUT,
    HIVE_ERR_CLOSED,
    HIVE_ERR_WOULDBLOCK,
    HIVE_ERR_IO,
} hive_error_code;

typedef struct {
    hive_error_code code;
    const char    *msg;   // string literal or NULL, never heap-allocated
} hive_status;
```

The `msg` field always points to a string literal or is NULL. It is never dynamically allocated, ensuring safe use across concurrent actors.

Convenience macros:

```c
#define HIVE_SUCCESS ((hive_status){HIVE_OK, NULL})
#define HIVE_SUCCEEDED(s) ((s).code == HIVE_OK)
#define HIVE_FAILED(s) ((s).code != HIVE_OK)
#define HIVE_ERROR(code, msg) ((hive_status){(code), (msg)})
#define HIVE_ERR_STR(s) ((s).msg ? (s).msg : "unknown error")
```

Usage example:
```c
if (len > HIVE_MAX_MESSAGE_SIZE) {
    return HIVE_ERROR(HIVE_ERR_INVALID, "Message too large");
}
```

## Design Trade-offs and Sharp Edges

This runtime makes deliberate design choices that favor **determinism, performance, and simplicity** over **ergonomics and safety**. These are not bugs - they are conscious trade-offs suitable for embedded/safety-critical systems with trusted code.

**Accept these consciously before using this runtime:**

### 1. Message Lifetime Rule is Sharp and Error-Prone

**Trade-off:** Message payload pointer valid only until next `hive_ipc_recv()` call.

**Why this design:**
- Pool reuse: Deterministic O(1) allocation, no fragmentation
- Performance: Single pool slot per actor, no reference counting
- Simplicity: No hidden malloc, no garbage collection

**Consequence:** Easy to misuse - storing `msg.data` across recv iterations causes use-after-free.

**This is not beginner-friendly.** Code must copy data immediately if needed beyond current iteration.

**Mitigation:** Documented with WARNING box and correct/incorrect examples in IPC section. But developers will still make mistakes.

**Acceptable if:** You optimize for determinism over ergonomics, and code reviews catch pointer misuse.

---

### 2. No Per-Actor Mailbox Quota (Global Starvation Possible)

**Trade-off:** One slow/malicious actor can consume all mailbox entries, starving the system.

**Why this design:**
- Simplicity: No per-actor accounting, no quota enforcement
- Flexibility: Bursty actors can use available pool space
- Performance: No quota checks on send path

**Consequence:** A single bad actor can cause global `HIVE_ERR_NOMEM` failures for all IPC sends.

**Supervisors are not optional.** You must design actor hierarchies with supervision and monitoring.

**Mitigation:** Application-level quotas, supervisor actors, monitoring. Runtime provides primitives, not policies.

**Acceptable if:** You deploy trusted code in embedded systems, not untrusted actors in general-purpose systems.

---

### 3. Selective Receive is O(n) per Mailbox Scan

**Trade-off:** Selective receive scans mailbox linearly, which is O(n) where n = mailbox depth.

**Why this design:**
- Battle-tested: Proven pattern for building complex protocols
- Simplicity: No index structures, no additional memory overhead
- Flexibility: Any filter criteria supported without pre-registration

**Consequence:** Deep mailboxes slow down selective receive. If 100 messages are queued and you're waiting for a specific tag, each wake scans all 100.

**Keep mailboxes shallow.** The request/reply pattern naturally does this (block waiting for reply).

**Mitigation:** Process messages promptly. Don't let mailbox grow deep. Use `hive_ipc_request()` which blocks until reply.

**Acceptable if:** Typical mailbox depth is small (< 20 messages) and request/reply pattern is followed.

---

### 4. Bus and IPC Share Message Pool (Resource Contention)

**Trade-off:** High-rate bus publishing can starve IPC globally.

**Why this design:**
- Simplicity: Single message pool, no subsystem isolation
- Flexibility: Pool space shared dynamically based on actual usage
- Memory efficiency: No wasted dedicated pools

**Consequence:** Misconfigured bus can cause all IPC sends to fail with `HIVE_ERR_NOMEM`.

**Mitigation:** WARNING box in Bus section, size pool for combined load, monitor exhaustion.

**Acceptable if:** You size pools correctly and monitor resource usage in production.

---

### Summary: This Runtime is a Sharp Tool

These design choices make the runtime:
- **Deterministic:** Bounded memory, predictable timing, no hidden allocations
- **Fast:** O(1) hot paths, minimal overhead, zero-copy options
- **Simple:** Minimal code, easy to audit, no complex features
- **Not beginner-friendly:** Sharp edges require careful use
- **Not fault-tolerant:** No automatic isolation, quotas, or recovery

**This is intentional.** The runtime provides primitives for building robust systems, not a complete safe environment.

This runtime provides primitives for deterministic embedded performance with full control.

## Core Types

```c
// Handles (opaque to user)
typedef uint32_t actor_id;
typedef uint32_t bus_id;
typedef uint32_t timer_id;

#define ACTOR_ID_INVALID  ((actor_id)0)
#define BUS_ID_INVALID    ((bus_id)0)
#define TIMER_ID_INVALID  ((timer_id)0)

// Wildcard for selective receive filtering
#define HIVE_SENDER_ANY     ((actor_id)0xFFFFFFFF)

// Actor entry point
typedef void (*actor_fn)(void *arg);

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    hive_priority_level priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
} actor_config;
```

## Actor API

### Lifecycle

```c
// Spawn a new actor with default configuration
hive_status hive_spawn(actor_fn fn, void *arg, actor_id *out);

// Spawn with explicit configuration
hive_status hive_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg, actor_id *out);

// Terminate current actor
_Noreturn void hive_exit(void);
```

**Actor function return behavior:**

Actors **must** call `hive_exit()` to terminate cleanly. If an actor function returns without calling `hive_exit()`, the runtime detects this as a crash:

1. Exit reason is set to `HIVE_EXIT_CRASH`
2. Linked/monitoring actors receive exit notification with `HIVE_EXIT_CRASH` reason
3. Normal cleanup proceeds (mailbox cleared, resources freed)
4. An ERROR log is emitted: "Actor N returned without calling hive_exit()"

This crash detection prevents infinite loops and ensures linked actors are notified of the improper termination.

```c
// Get current actor's ID
actor_id hive_self(void);

// Yield to scheduler
void hive_yield(void);

// Check if actor is alive
bool hive_actor_alive(actor_id id);
```

### Linking and Monitoring

Actors can link to other actors to receive notification when they die:

```c
// Bidirectional link: if either dies, the other receives exit message
hive_status hive_link(actor_id target);
hive_status hive_link_remove(actor_id target);

// Unidirectional monitor: receive notification when target dies
hive_status hive_monitor(actor_id target, uint32_t *out);
hive_status hive_monitor_cancel(uint32_t id);
```

Exit message structure:

```c
typedef enum {
    HIVE_EXIT_NORMAL,       // Actor called hive_exit()
    HIVE_EXIT_CRASH,        // Actor function returned without calling hive_exit()
    HIVE_EXIT_CRASH_STACK,  // Stack overflow detected
    HIVE_EXIT_KILLED,       // Actor was killed externally
} hive_exit_reason;

typedef struct {
    actor_id       actor;   // who died
    hive_exit_reason reason;
} hive_exit_msg;

// Check if message is exit notification
bool hive_is_exit_msg(const hive_message *msg);

// Decode exit message into struct
hive_status hive_decode_exit(const hive_message *msg, hive_exit_msg *out);

// Convert exit reason to string (for logging/debugging)
const char *hive_exit_reason_str(hive_exit_reason reason);
```

**Handling exit messages:**

Exit messages should be decoded using `hive_decode_exit()`:

```c
hive_message msg;
hive_ipc_recv(&msg, -1);

if (hive_is_exit_msg(&msg)) {
    hive_exit_msg exit_info;
    hive_decode_exit(&msg, &exit_info);
    printf("Actor %u died: %s\n", exit_info.actor, hive_exit_reason_str(exit_info.reason));
}
```

## IPC API

Inter-process communication via mailboxes. Each actor has one mailbox. All messages are asynchronous (sender doesn't wait for response). Request/reply is built on top using message tags for correlation.

### Message Header Format

All messages have a 4-byte header prepended to the payload:

```
┌──────────────────────────────────────────────────────────────┐
│ class (4 bits) │ gen (1 bit) │ tag (27 bits) │ payload (len) │
└──────────────────────────────────────────────────────────────┘
```

- **class**: Message type (4 bits, 16 possible values)
- **gen**: Generated flag (1 = runtime-generated tag, 0 = user-provided tag)
- **tag**: Correlation identifier (27 bits, 134M unique values)
- **payload**: Application data (up to `HIVE_MAX_MESSAGE_SIZE - 4` = 252 bytes)

**Header overhead:** 4 bytes per message.

### Message Classes

```c
typedef enum {
    HIVE_MSG_NOTIFY = 0,   // Fire-and-forget message
    HIVE_MSG_REQUEST,       // Request expecting a reply
    HIVE_MSG_REPLY,      // Response to a REQUEST
    HIVE_MSG_TIMER,      // Timer tick
    HIVE_MSG_EXIT,     // System notifications (actor death)
    // 5-14 reserved for future use
    HIVE_MSG_ANY = 15,   // Wildcard for selective receive filtering
} hive_msg_class;
```

### Tag System

```c
#define HIVE_TAG_NONE        0            // No tag (for simple NOTIFY messages)
#define HIVE_TAG_ANY         0x0FFFFFFF   // Wildcard for selective receive filtering

// Note: HIVE_TAG_GEN_BIT and HIVE_TAG_VALUE_MASK are internal implementation
// details and not part of the public API
```

**Tag semantics:**
- **HIVE_TAG_NONE**: Used for simple NOTIFY messages where no correlation is needed
- **HIVE_TAG_ANY**: Used in `hive_ipc_recv_match()` to match any tag
- **Generated tags**: Created automatically by `hive_ipc_request()` for request/reply correlation

**Tag generation:** Internal to `hive_ipc_request()`. Global counter increments on each call. Generated tags have `HIVE_TAG_GEN_BIT` set. Wraps at 2^27 (134M values).

**Namespace separation:** Generated tags (gen=1) and user tags (gen=0) can never collide.

### Message Structure

```c
typedef struct {
    actor_id       sender;       // Sender actor ID
    hive_msg_class class;        // Message class (pre-decoded)
    uint32_t       tag;          // Message tag (pre-decoded)
    size_t         len;          // Payload length (excludes 4-byte header)
    const void    *data;         // Payload pointer (past header)
} hive_message;
```

The message struct provides **direct access to all fields** - no `hive_msg_decode()` call required:

```c
hive_message msg;
hive_ipc_recv(&msg, -1);

// Direct access - no boilerplate
my_data *payload = (my_data *)msg.data;
if (msg.class == HIVE_MSG_REQUEST) {
    hive_ipc_reply(&msg, &response, sizeof(response));
}
```

**Lifetime rule:** Data is valid until the next successful `hive_ipc_recv()` or `hive_ipc_recv_match()` call. Copy immediately if needed beyond current iteration.

### Functions

#### Basic Messaging

```c
// Fire-and-forget message (class=NOTIFY)
// Tag enables selective receive filtering on the receiver side
hive_status hive_ipc_notify(actor_id to, uint32_t tag, const void *data, size_t len);

// Send with explicit class and tag (sender is current actor)
hive_status hive_ipc_notify_ex(actor_id to, hive_msg_class class,
                               uint32_t tag, const void *data, size_t len);

// Receive any message (no filtering)
// timeout_ms == 0:  non-blocking, returns HIVE_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns HIVE_ERR_TIMEOUT if exceeded
hive_status hive_ipc_recv(hive_message *msg, int32_t timeout_ms);
```

#### Selective Receive

```c
// Receive with filtering on sender, class, and/or tag
// Blocks until message matches ALL non-wildcard criteria, or timeout
// Use HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY as wildcards
hive_status hive_ipc_recv_match(actor_id from, hive_msg_class class,
                            uint32_t tag, hive_message *msg, int32_t timeout_ms);
```

**Filter semantics:**
- `from == HIVE_SENDER_ANY` → match any sender
- `class == HIVE_MSG_ANY` → match any class
- `tag == HIVE_TAG_ANY` → match any tag
- Non-wildcard values must match exactly

**Usage examples:**
```c
// Match any message (equivalent to hive_ipc_recv)
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY, &msg, -1);

// Match only from specific sender
hive_ipc_recv_match(some_actor, HIVE_MSG_ANY, HIVE_TAG_ANY, &msg, -1);

// Match REQUEST messages from any sender
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_REQUEST, HIVE_TAG_ANY, &msg, -1);

// Match REPLY with specific tag (used internally by hive_ipc_request)
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_REPLY, expected_tag, &msg, 5000);
```

#### Request/Reply

```c
// Send REQUEST, block until REPLY with matching tag, or timeout
hive_status hive_ipc_request(actor_id to, const void *request, size_t req_len,
                      hive_message *reply, int32_t timeout_ms);

// Reply to a received REQUEST (extracts sender and tag from request automatically)
hive_status hive_ipc_reply(const hive_message *request, const void *data, size_t len);
```

**Request/reply implementation:**
```c
// hive_ipc_request internally does:
// 1. Generate unique tag
// 2. Send message with class=REQUEST
// 3. Block on hive_ipc_recv_match() for REPLY with matching tag
// 4. Return reply or timeout error

// hive_ipc_reply internally does:
// 1. Decode sender and tag from request
// 2. Send message with class=REPLY and same tag
```

**Error conditions for `hive_ipc_request()`:**
- `HIVE_ERR_TIMEOUT`: No reply received within timeout (including when target died)
- `HIVE_ERR_NOMEM`: Pool exhausted when sending request
- `HIVE_ERR_INVALID`: Invalid target actor ID or NULL request with non-zero length

**Target death handling:** If the caller has linked to or is monitoring the target actor, and the target dies before replying, an EXIT message arrives in the caller's mailbox. Since `hive_ipc_request()` internally uses `hive_ipc_recv_match()` waiting for a REPLY, the EXIT message does not match and remains queued. The function returns `HIVE_ERR_TIMEOUT` when the timeout expires. The caller should check for pending EXIT messages to determine if the timeout was due to target death:

```c
hive_message reply;
hive_status status = hive_ipc_request(target, &req, sizeof(req), &reply, 5000);
if (status.code == HIVE_ERR_TIMEOUT) {
    // Check if target died
    hive_message msg;
    if (hive_ipc_recv(&msg, 0).code == HIVE_OK && msg.class == HIVE_MSG_EXIT) {
        hive_exit_msg *exit_info = (hive_exit_msg *)msg.data;
        if (exit_info->actor == target) {
            // Target died - handle accordingly
        }
    }
}
```

#### Message Decoding (Optional)

Since `hive_message` now provides pre-decoded `class`, `tag`, and direct `data` pointer, `hive_msg_decode()` is **rarely needed**. It remains available for backwards compatibility:

```c
// Returns pre-decoded values from hive_message struct
// All output parameters are optional (may be NULL)
hive_status hive_msg_decode(const hive_message *msg,
                        hive_msg_class *class,    // out: msg->class
                        uint32_t *tag,            // out: msg->tag
                        const void **payload,     // out: msg->data
                        size_t *payload_len);     // out: msg->len
```

**Preferred pattern (direct access):**
```c
hive_message msg;
hive_ipc_recv(&msg, -1);
my_data *data = (my_data *)msg.data;  // Direct payload access
if (msg.class == HIVE_MSG_REQUEST) { ... }
```

### API Contract: hive_ipc_notify()

**Parameter validation:**
- If `data == NULL && len > 0`: Returns `HIVE_ERR_INVALID`
- If `len > HIVE_MAX_MESSAGE_SIZE - 4` (252 bytes payload): Returns `HIVE_ERR_INVALID`
- Oversized messages are rejected immediately, not truncated

**Behavior when pools are exhausted:**

`hive_ipc_notify()` uses two global pools:
1. **Mailbox entry pool** (`HIVE_MAILBOX_ENTRY_POOL_SIZE` = 256)
2. **Message data pool** (`HIVE_MESSAGE_DATA_POOL_SIZE` = 256)

**Fail-fast semantics:**

- **Returns `HIVE_ERR_NOMEM` immediately** if either pool is exhausted
- **Does NOT block** waiting for pool space
- **Does NOT drop** messages silently
- **Atomic operation:** Either succeeds completely or fails

**Caller responsibilities:**
- **MUST** check return value and handle `HIVE_ERR_NOMEM`
- **MUST** implement backpressure/retry logic if needed
- **MUST NOT** assume message was delivered if `HIVE_FAILED(status)`

**Example: Handling pool exhaustion**

```c
// Bad: Ignoring return value
hive_ipc_notify(target, 0, &data, sizeof(data));  // WRONG - message may be lost

// Good: Check and handle with backoff-retry
hive_status status = hive_ipc_notify(target, 0, &data, sizeof(data));
if (status.code == HIVE_ERR_NOMEM) {
    // Pool exhausted - backoff and retry
    hive_message msg;
    hive_ipc_recv(&msg, 10);  // Wait 10ms, process any incoming messages
    // Retry the send
    status = hive_ipc_notify(target, 0, &data, sizeof(data));
    if (HIVE_FAILED(status)) {
        // Still failing - drop message or take other action
    }
}
```

### Message Data Lifetime

**CRITICAL LIFETIME RULE:**
- **Data is ONLY valid until the next successful `hive_ipc_recv()` or `hive_ipc_recv_match()` call**
- **Per actor: only ONE message payload pointer is valid at a time**
- **Storing `msg.data` across receive iterations causes use-after-free**
- **If you need the data later, COPY IT IMMEDIATELY**

**Failed recv does NOT invalidate:**
- If recv returns `HIVE_ERR_TIMEOUT` or `HIVE_ERR_WOULDBLOCK`, previous buffer remains valid
- Only a successful recv invalidates the previous pointer

**Correct pattern:**
```c
hive_message msg;
hive_ipc_recv(&msg, -1);

// SAFE: Direct access and copy immediately
char local_copy[256];
memcpy(local_copy, msg.data, msg.len);
// local_copy is safe to use indefinitely
// msg.class and msg.tag also available directly

// UNSAFE: Storing pointer across recv calls
const char *ptr = msg.data;   // DANGER
hive_ipc_recv(&msg, -1);       // ptr now INVALID
```

### Mailbox Semantics

**Capacity model:**

Per-actor mailbox limits: **No per-actor quota** - capacity is constrained by global pools

Global pool limits: **Yes** - all actors share:
- `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256 default) - mailbox entries
- `HIVE_MESSAGE_DATA_POOL_SIZE` (256 default) - message data

**Important:** One slow receiver can consume all mailbox entries, starving other actors.

**Fairness guarantees:** The runtime does not provide per-actor fairness guarantees; resource exhaustion caused by a misbehaving actor is considered an application-level fault. Applications requiring protection against resource starvation should implement supervisor actors or application-level quotas.

### Selective Receive Semantics

`hive_ipc_recv_match()` implements selective receive. This is the key mechanism for building complex protocols like request/reply.

**Blocking behavior:**

1. Scan mailbox from head for message matching all filter criteria
2. If found → remove from mailbox, return immediately
3. If not found → block, yield to scheduler
4. When any message arrives → wake, rescan from head
5. If match → return
6. If no match → go back to sleep
7. Repeat until match or timeout

**Key properties:**

- **Non-matching messages are NOT dropped** — they stay in mailbox
- **Order preserved** — messages remain in FIFO order
- **Later retrieval** — non-matching messages retrieved by subsequent `hive_ipc_recv()` calls
- **Scan complexity** — O(n) where n = mailbox depth

**Example: Waiting for specific reply**

```c
// Using hive_ipc_request() for request/reply (recommended - handles tag generation internally)
hive_message reply;
hive_ipc_request(server, &request, sizeof(request), &reply, 5000);

// Or manually using selective receive:
uint32_t expected_tag = 42;  // Known tag from earlier call
hive_ipc_recv_match(server, HIVE_MSG_REPLY, expected_tag, &reply, 5000);

// During the wait:
// - NOTIFY messages from other actors: skipped, stay in mailbox
// - TIMER messages: skipped, stay in mailbox
// - REPLY from server with wrong tag: skipped
// - REPLY from server with matching tag: returned!

// After returning, skipped messages can be retrieved:
hive_ipc_recv(&msg, 0);  // Gets first skipped message
```

**When selective receive is efficient:**

- Typical request/reply: mailbox is empty or near-empty while waiting for reply
- Shallow mailbox: O(n) scan is fast when n is small

**When selective receive is less efficient:**

- Deep mailbox with many non-matching messages
- Example: 100 pending NOTIFYs while waiting for specific REPLY → scans 100 messages

**Mitigation:** Process messages promptly. Don't let mailbox grow deep. The request/reply pattern naturally keeps mailbox shallow because you block waiting for reply.

### Message Ordering

**Mailbox implementation:** FIFO linked list (enqueue at tail, dequeue from head)

**Ordering guarantees:**

**Single sender to single receiver:**
- Messages are received in the order they were sent
- **FIFO guaranteed**
- Example: If actor A sends M1, M2, M3 to actor B, B receives them in order M1 → M2 → M3

**Multiple senders to single receiver:**
- Message order depends on scheduling (which sender runs first)
- **Arrival order is scheduling-dependent**
- Example: If actor A sends M1 and actor B sends M2, receiver may get M1→M2 or M2→M1

**Selective receive and ordering:**
- Selective receive can retrieve messages out of FIFO order
- Non-matching messages are skipped but not reordered
- Example: Mailbox has [NOTIFY, TIMER, REPLY]. Filtering for REPLY returns REPLY first.

**Consequences:**

```c
// Single sender - FIFO guaranteed
void sender_actor(void *arg) {
    hive_ipc_notify(receiver, 0, &msg1, sizeof(msg1));  // Sent first
    hive_ipc_notify(receiver, 0, &msg2, sizeof(msg2));  // Sent second
    // Receiver will see: msg1, then msg2 (guaranteed)
}

// Multiple senders - order depends on scheduling
void sender_A(void *arg) {
    hive_ipc_notify(receiver, 0, &msgA, sizeof(msgA));
}
void sender_B(void *arg) {
    hive_ipc_notify(receiver, 0, &msgB, sizeof(msgB));
}
// Receiver may see: msgA then msgB, OR msgB then msgA

// Selective receive - can retrieve out of order
void request_reply_actor(void *arg) {
    // Start timer
    timer_id t;
    hive_timer_after(1000000, &t);

    // Do request/reply
    hive_message reply;
    hive_ipc_request(server, &req, sizeof(req), &reply, 5000);
    // Timer tick arrived during request/reply wait - it's in mailbox

    // Now process timer using selective receive with timer_id
    hive_message timer_msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, t, &timer_msg, 0);
}
```

### Timer and System Messages

Timer and system messages use the same mailbox and message format as IPC.

**Timer messages:**
- Class: `HIVE_MSG_TIMER`
- Sender: Actor that owns the timer
- Tag: `timer_id`
- Payload: Empty (len = 0 after decoding header)

**System messages (exit notifications):**
- Class: `HIVE_MSG_EXIT`
- Sender: Actor that died
- Tag: `HIVE_TAG_NONE`
- Payload: `hive_exit_msg` struct

**Checking message type:**

```c
hive_message msg;
hive_ipc_recv(&msg, -1);

switch (msg.class) {
    case HIVE_MSG_NOTIFY:
        handle_cast(msg.sender, msg.data, msg.len);
        break;
    case HIVE_MSG_REQUEST:
        handle_call_and_reply(&msg, msg.data, msg.len);
        break;
    case HIVE_MSG_TIMER:
        handle_timer_tick(msg.tag);  // tag is timer_id
        break;
    case HIVE_MSG_EXIT:
        handle_exit_notification(msg.sender, msg.data);
        break;
    default:
        break;
}
```

**Convenience function for timer checking:**

```c
// Returns true if message is a timer tick
bool hive_msg_is_timer(const hive_message *msg);
```

### Query Functions

```c
// Check if any message is pending in current actor's mailbox
bool hive_ipc_pending(void);

// Count messages in current actor's mailbox
size_t hive_ipc_count(void);
```

**Semantics:**
- Query the **current actor's** mailbox only
- Cannot query another actor's mailbox
- Return `false` / `0` if called outside actor context

### Pool Exhaustion Behavior

IPC uses two global pools shared by all actors:
- **Mailbox entry pool**: `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `HIVE_MESSAGE_DATA_POOL_SIZE` (256 default)

**When pools are exhausted:**
- `hive_ipc_notify()` returns `HIVE_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure

**No per-actor mailbox limit**: All actors share the global pools. A single actor can consume all available entries if receivers don't process messages.

**Mitigation strategies:**
- Size pools appropriately: `1.5× peak concurrent messages`
- Check return values and implement retry logic or backpressure
- Use `hive_ipc_request()` for natural backpressure (sender waits for reply)
- Ensure receivers process messages promptly

**Backoff-retry example:**
```c
hive_status status = hive_ipc_notify(target, 0, data, len);
if (status.code == HIVE_ERR_NOMEM) {
    // Pool exhausted - backoff before retry
    hive_message msg;
    status = hive_ipc_recv(&msg, 10);  // Backoff 10ms

    if (HIVE_SUCCEEDED(status)) {
        // Got message during backoff, handle it first
        handle_message(&msg);
    }
    // Retry send...
}
```

## Bus API

Publish-subscribe communication with configurable retention policy.

### Configuration

```c
typedef struct {
    uint8_t  max_subscribers; // max concurrent subscribers (1..HIVE_MAX_BUS_SUBSCRIBERS)
    uint8_t  consume_after_reads;     // consume after N reads, 0 = unlimited (0..max_subscribers)
    uint32_t max_age_ms;      // expire entries after ms, 0 = no expiry
    size_t   max_entries;     // ring buffer capacity
    size_t   max_entry_size;  // max payload bytes per entry
} hive_bus_config;
```

**Configuration constraints (normative):**
- `max_subscribers`: **Architectural limit of 32 subscribers** (enforced by 32-bit `readers_mask`)
  - Valid range: 1..32
  - `HIVE_MAX_BUS_SUBSCRIBERS = 32` is a hard architectural invariant, not a tunable parameter
  - Attempts to configure `max_subscribers > 32` return `HIVE_ERR_INVALID`
- `consume_after_reads`: Valid range: 0..max_subscribers
- Subscriber index assignment is stable for the lifetime of a subscription (affects `readers_mask` bit position)

### Functions

```c
// Create bus
hive_status hive_bus_create(const hive_bus_config *cfg, bus_id *out);

// Destroy bus (fails if subscribers exist)
hive_status hive_bus_destroy(bus_id bus);

// Publish data
hive_status hive_bus_publish(bus_id bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
hive_status hive_bus_subscribe(bus_id bus);
hive_status hive_bus_unsubscribe(bus_id bus);

// Read entry (non-blocking)
// Returns HIVE_ERR_WOULDBLOCK if no data available
hive_status hive_bus_read(bus_id bus, void *buf, size_t max_len, size_t *bytes_read);

// Read with blocking
hive_status hive_bus_read_wait(bus_id bus, void *buf, size_t max_len,
                               size_t *bytes_read, int32_t timeout_ms);

// Query bus state
size_t hive_bus_entry_count(bus_id bus);
```

**Message size validation:**

`hive_bus_create()`:
- If `cfg->max_entry_size > HIVE_MAX_MESSAGE_SIZE` (256 bytes): Returns `HIVE_ERR_INVALID`
- Bus entries share the message data pool, which uses fixed-size `HIVE_MAX_MESSAGE_SIZE` entries
- This constraint ensures bus entries fit in pool slots

`hive_bus_publish()`:
- If `len > cfg.max_entry_size`: Returns `HIVE_ERR_INVALID`
- Oversized messages are rejected immediately, not truncated
- The `max_entry_size` was validated at bus creation time

`hive_bus_read()` / `hive_bus_read_wait()`:
- If message size > `max_len`: Data is **truncated** to fit in buffer
- `*bytes_read` returns the **actual bytes copied** (truncated length), NOT the original message size
- This is safe buffer overflow protection - caller gets as much data as fits
- No error is returned for truncation (caller can compare `bytes_read < max_entry_size` to detect)

### Bus Consumption Model (Semantic Contract)

The bus implements **per-subscriber read cursors** with the following **three contractual rules**:

---

#### **RULE 1: Subscription Start Position**

**Contract:** `hive_bus_subscribe()` initializes the subscriber's read cursor to **"next publish"** (current `bus->head` write position).

**Guaranteed semantics:**
- Subscriber **CANNOT** read retained entries published before subscription
- Subscriber **ONLY** sees entries published **after** `hive_bus_subscribe()` returns
- First `hive_bus_read()` call returns `HIVE_ERR_WOULDBLOCK` if no new entries published since subscription
- Implementation: `subscriber.next_read_idx = bus->head`

**Implications:**
- New subscribers do NOT see history
- If you need to read retained entries, subscribe **before** publishing starts
- Late subscribers will miss all prior messages

**Example:**
```c
// Bus has retained entries [E1, E2, E3] with head=3
hive_bus_subscribe(bus);
//   -> subscriber.next_read_idx = 3 (next write position)

hive_bus_read(bus, buf, len, &bytes_read);
//   -> Returns HIVE_ERR_WOULDBLOCK (no new data)
//   -> E1, E2, E3 are invisible (behind cursor)

// Publisher publishes E4 (head advances to 4)
hive_bus_read(bus, buf, len, &bytes_read);
//   -> Returns E4 (first entry after subscription)
```

---

#### **RULE 2: Per-Subscriber Cursor Storage and Eviction Behavior**

**Contract:** Each subscriber has an independent read cursor. Slow subscribers may miss entries due to buffer wraparound; no error or notification is generated. The bus implementation supports a maximum of 32 concurrent subscribers per bus, enforced by the 32-bit `readers_mask`.

**Guaranteed semantics:**
1. **Storage per subscriber:**
   - `bus_subscriber` struct with `next_read_idx` field (tracks next entry to read)
   - Each subscriber reads at their own pace independently
   - Storage cost: **O(max_subscribers)** fixed overhead

2. **Storage per entry:**
   - 32-bit `readers_mask` bitmask (max 32 subscribers per bus)
   - Bit N set -> subscriber N has read this entry
   - Storage cost: **O(1)** per entry (4 bytes bitmask + 1 byte read_count)

3. **Eviction behavior (buffer full):**
   - When `hive_bus_publish()` finds buffer full (`count >= max_entries`):
     - Oldest entry at `bus->tail` is **evicted immediately** (freed from message pool)
     - Tail advances: `bus->tail = (bus->tail + 1) % max_entries`
     - **No check if subscribers have read the evicted entry**
   - If slow subscriber's cursor pointed to evicted entry:
     - On next `hive_bus_read()`, search starts from **current `bus->tail`** (oldest surviving entry)
     - Subscriber **silently skips** to next available unread entry
     - **No error** returned (appears as normal read)
     - **No signal** of data loss (by design)

**Implications:**
- Slow subscribers lose data without notification
- Fast subscribers never lose data (assuming buffer sized for publish rate)
- No backpressure mechanism (unlike `hive_ipc_request()` request/reply pattern)
- Real-time principle: Prefer fresh data over old data

**Example (data loss):**
```c
// Bus: max_entries=3, entries=[E1, E2, E3] (full), tail=0, head=0
// Fast subscriber: next_read_idx=0 (read all, awaiting E4)
// Slow subscriber: next_read_idx=0 (still at E1, hasn't read any)

hive_bus_publish(bus, &E4, sizeof(E4));
//   -> Buffer full: Evict E1 at tail=0, free from pool
//   -> Write E4 at index 0: entries=[E4, E2, E3]
//   -> tail=1 (E2 is now oldest), head=1 (next write)

// Slow subscriber calls hive_bus_read():
//   -> Search from tail=1: E2 (unread), E3 (unread), E4 (unread)
//   -> Returns E2 (first unread)
//   -> E1 is LOST (no error, silent skip)
```

**Eviction does NOT notify slow subscribers:**
- No `HIVE_ERR_OVERFLOW` or similar
- No special message indicating data loss
- Application must detect via message sequence numbers if needed

---

#### **RULE 3: consume_after_reads Counting Semantics**

**Contract:** `consume_after_reads` counts **unique subscribers** who have read an entry, **NOT** total reads.

**Guaranteed semantics:**
- Each entry has a `readers_mask` bitmask (32 bits, max 32 subscribers per bus)
- When subscriber N reads an entry:
  1. Check if bit N is set in `readers_mask` -> if yes, skip (already read)
  2. If no, set bit N, increment `read_count`, return entry
- Entry is removed when `read_count >= consume_after_reads` (N unique subscribers have read)
- Same subscriber CANNOT read the same entry twice (deduplication)

**Implementation mechanism:**
```c
// Check if already read
if (entry->readers_mask & (1u << subscriber_idx)) {
    continue;  // Skip, already read by this subscriber
}

// Mark as read
entry->readers_mask |= (1u << subscriber_idx);
entry->read_count++;

// Remove if consume_after_reads reached
if (config.consume_after_reads > 0 && entry->read_count >= config.consume_after_reads) {
    // Free entry from pool, invalidate
}
```

**Implications:**
- `consume_after_reads=3` means "remove after 3 **different** subscribers read it"
- If subscriber A reads entry twice (e.g., re-subscribes), that's still 1 read
- If 3 subscribers each read entry once, that's 3 reads -> entry removed
- Set `consume_after_reads=0` to disable (entry persists until aged out or evicted)

**Example (unique counting):**
```c
// Bus with consume_after_reads=2
// Subscribers: A, B, C

hive_bus_publish(bus, &E1, sizeof(E1));
//   -> E1: readers_mask=0b000, read_count=0

// Subscriber A reads E1
hive_bus_read(bus, ...);
//   -> E1: readers_mask=0b001, read_count=1 (A's bit set)

// Subscriber A reads again (tries to read E1)
hive_bus_read(bus, ...);
//   -> E1 skipped (bit already set), returns HIVE_ERR_WOULDBLOCK
//   -> E1: readers_mask=0b001, read_count=1 (unchanged)

// Subscriber B reads E1
hive_bus_read(bus, ...);
//   -> E1: readers_mask=0b011, read_count=2 (B's bit set)
//   -> read_count >= consume_after_reads (2) -> E1 REMOVED, freed from pool
```

---

### Summary: The Three Rules

| Rule | Contract |
|------|----------|
| **1. Subscription start position** | New subscribers start at "next publish" (cannot read history) |
| **2. Cursor storage & eviction** | Per-subscriber cursors; slow readers may miss entries on wraparound (no notification) |
| **3. consume_after_reads counting** | Counts UNIQUE subscribers (deduplication), not total reads |

---

### Retention Policy Configuration

Entries can be removed by **three mechanisms** (whichever occurs first):

1. **consume_after_reads (unique subscriber counting):**
   - Entry removed when `read_count >= consume_after_reads` (N unique subscribers have read it)
   - Value `0` = disabled (entry persists until aged out or evicted)
   - See **RULE 3** above for exact counting semantics

2. **max_age_ms (time-based expiry):**
   - Entry removed when `(current_time_ms - entry.timestamp_ms) >= max_age_ms`
   - Value `0` = disabled (no time-based expiry)
   - Checked on every `hive_bus_read()` and `hive_bus_publish()` call

3. **Buffer full (forced eviction):**
   - Oldest entry at `bus->tail` evicted when `count >= max_entries` on publish
   - ALWAYS enabled (cannot be disabled)
   - See **RULE 2** above for eviction semantics

**Typical configurations:**

| Use Case | consume_after_reads | max_age_ms | Behavior |
|----------|-------------|------------|----------|
| **Sensor data** | `0` | `100` | Multiple readers, data stale after 100ms |
| **Configuration** | `0` | `0` | Persistent until buffer wraps |
| **Events** | `N` | `0` | Consumed after N subscribers read, no timeout |
| **Recent history** | `0` | `5000` | Keep 5 seconds of history, multiple readers |

**Interaction of mechanisms:**
- Entry is removed when **FIRST** condition is met (OR, not AND)
- Example: `consume_after_reads=3, max_age_ms=1000`
  - Entry removed after 3 subscribers read it, **OR**
  - Entry removed after 1 second, **OR**
  - Entry removed when buffer full (forced eviction)

### Pool Exhaustion and Buffer Full Behavior

**WARNING: Resource Contention Between IPC and Bus**

Bus publishing consumes the same message data pool as IPC (`HIVE_MESSAGE_DATA_POOL_SIZE`). A misconfigured or high-rate bus can exhaust the message pool and cause **all** IPC sends to fail with `HIVE_ERR_NOMEM`, potentially starving critical actor communication.

**Architectural consequences:**
- Bus auto-evicts oldest entries when its ring buffer fills (graceful degradation)
- IPC never auto-drops (fails immediately with `HIVE_ERR_NOMEM`)
- A single high-rate bus publisher can starve IPC globally
- No per-subsystem quotas or fairness guarantees

**Design implications:**
- Size `HIVE_MESSAGE_DATA_POOL_SIZE` for combined IPC + bus peak load
- Use bus retention policies to limit memory consumption
- Monitor pool exhaustion in critical systems
- Consider separate message pools if isolation is required (requires code modification)

---

The bus can encounter two types of resource limits:

**1. Message Pool Exhaustion** (shared with IPC):
- Bus uses the global `HIVE_MESSAGE_DATA_POOL_SIZE` pool (same as IPC)
- When pool is exhausted, `hive_bus_publish()` returns `HIVE_ERR_NOMEM` immediately
- Does NOT block waiting for space
- Does NOT drop messages automatically in this case
- Caller must check return value and handle failure

**2. Bus Ring Buffer Full** (per-bus limit):
- Each bus has its own ring buffer sized via `max_entries` config
- When ring buffer is full, `hive_bus_publish()` **automatically evicts oldest entry**
- This is different from IPC - bus has automatic message dropping
- Publish succeeds (unless message pool also exhausted)
- Slow readers may miss messages if buffer wraps

**3. Subscriber Table Full**:
- Each bus has subscriber limit via `max_subscribers` config (up to `HIVE_MAX_BUS_SUBSCRIBERS`)
- When full, `hive_bus_subscribe()` returns `HIVE_ERR_NOMEM`

**Key Differences from IPC:**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`HIVE_MESSAGE_DATA_POOL_SIZE`)

**Mitigation strategies:**
- Size message pool appropriately for combined IPC + bus load
- Configure per-bus `max_entries` based on publish rate vs read rate
- Use retention policies (`consume_after_reads`, `max_age_ms`) to prevent accumulation
- Monitor `hive_bus_entry_count()` to detect slow readers

## Timer API

Timers for periodic and one-shot wake-ups.

```c
// One-shot: wake current actor after delay
hive_status hive_timer_after(uint32_t delay_us, timer_id *out);

// Periodic: wake current actor every interval
hive_status hive_timer_every(uint32_t interval_us, timer_id *out);

// Cancel timer
hive_status hive_timer_cancel(timer_id id);

// Sleep for specified duration (microseconds)
// Uses selective receive - other messages remain in mailbox
hive_status hive_sleep(uint32_t delay_us);

// Get current time in microseconds (monotonic)
// Returns monotonic time suitable for measuring elapsed durations.
// In simulation mode, returns simulated time.
uint64_t hive_get_time(void);

// Check if message is a timer tick (convenience wrapper for hive_msg_decode)
bool hive_msg_is_timer(const hive_message *msg);
```

Timer wake-ups are delivered as messages with `class == HIVE_MSG_TIMER`. The tag contains the `timer_id`. The actor receives these in its normal `hive_ipc_recv()` loop and can use `hive_msg_is_timer()` or `hive_msg_decode()` to identify timer messages.

### Timer Tick Coalescing (Periodic Timers)

**Behavior:** When the scheduler reads a timerfd, it obtains an expiration count (how many intervals elapsed since last read). The runtime sends **exactly one tick message** regardless of the expiration count.

**Rationale:**
- Simplicity: Actor receives predictable single-message notification
- Real-time principle: Current state matters more than history
- Memory efficiency: No risk of mailbox flooding from fast timers

**Implications:**
- If scheduler is delayed (file I/O stall, long actor computation), periodic timer ticks are **coalesced**
- Actor cannot determine how many intervals actually elapsed
- For precise tick counting, use `hive_get_time()` to measure elapsed time

**Example:**
```c
// 10ms periodic timer, but actor takes 35ms to process
hive_timer_every(10000, &timer);  // 10ms = 10000us

while (1) {
    hive_ipc_recv(&msg, -1);
    if (hive_msg_is_timer(&msg)) {
        // Even if 35ms passed (3-4 intervals), actor receives ONE tick
        // timerfd read returned expirations=3 or 4, but only one message sent
        do_work();  // Takes 35ms
    }
}
```

**Alternative not implemented:** Enqueuing N tick messages for N expirations was rejected because:
- Risk of mailbox overflow for fast timers
- Most embedded use cases don't need tick counting
- Actors needing precise counts can use `hive_get_time()` to measure elapsed time

### Timer Precision and Monotonicity

**Unit mismatch by design:**
- Timer API uses **microseconds** (`uint32_t delay_us`, `uint32_t interval_us`)
- Rest of system (IPC, file, network) uses **milliseconds** (`int32_t timeout_ms`)
- Rationale: Timers often need sub-millisecond precision; I/O timeouts rarely do

**Resolution and precision:**

Platform | Clock Source | API Precision | Actual Precision | Notes
---------|-------------|---------------|------------------|------
**Linux (x86-64)** | `CLOCK_MONOTONIC` via `timerfd` | Nanosecond (`itimerspec`) | ~1 ms typical | Kernel-limited, non-realtime scheduler
**STM32 (ARM)** | Hardware timer (SysTick/TIM) | Microsecond | ~1-10 us typical | Depends on timer configuration

- On Linux, timers use `CLOCK_MONOTONIC` clock source via `timerfd_create()`
- On Linux, requests < 1ms may still fire with ~1ms precision due to kernel scheduling
- On STM32, hardware timers provide microsecond-level precision

**Monotonic clock guarantee:**
- Uses **monotonic clock** on both platforms (CLOCK_MONOTONIC on Linux, hardware timer on STM32)
- **NOT affected by**:
  - System time changes (NTP adjustments, user setting clock)
  - Timezone changes
  - Daylight saving time
- **Guaranteed to**:
  - Never go backwards
  - Count elapsed time accurately (subject to clock drift, typically <100 ppm)
- **Use case**: Timers measure **elapsed time**, not wall-clock time

**hive_get_time() precision:**

`hive_get_time()` returns the current monotonic time in microseconds:

Platform | Implementation | Resolution | Notes
---------|----------------|------------|------
**Linux** | `clock_gettime(CLOCK_MONOTONIC)` | Nanosecond | vDSO, minimal overhead
**STM32** | `tick_count * HIVE_TIMER_TICK_US` | 1 ms default | Limited by tick rate

- Linux: Uses vDSO for low-overhead system call (no kernel trap in most cases)
- STM32: Resolution limited by `HIVE_TIMER_TICK_US` (default 1000μs = 1ms)
- For sub-millisecond precision on STM32, reduce `HIVE_TIMER_TICK_US` or use DWT cycle counter
- In simulation mode, returns simulated time (advanced by `hive_timer_advance_time()`)

**Delivery guarantee:**
- Timer callbacks are delivered **at or after** the requested time; early delivery never occurs
- Delays may occur due to scheduling (higher priority actors, blocked scheduler)
- In safety-critical systems, timers provide lower bounds on timing, not exact timing

**Wraparound behavior:**

1. **Timer interval wraparound** (`uint32_t interval_us`):
   - Type: 32-bit unsigned integer
   - Max value: 4,294,967,295 microseconds = **~4295 seconds** = **~71.6 minutes**
   - Wraparound: Values > 71.6 minutes wrap around (e.g., 72 minutes becomes 24 seconds)
   - **Mitigation**: Use multiple timers or external tick counting for intervals > 1 hour

2. **Timer ID wraparound** (`timer_id` = `uint32_t`):
   - Global counter `g_timer.next_id` increments on each timer creation
   - Wraps at 4,294,967,295 timers
   - Potential collision: If timer ID wraps and old timer still active, `hive_timer_cancel()` may cancel wrong timer
   - **Likelihood**: Extremely rare (requires 4 billion timer creations without runtime restart)
   - **Mitigation**: None needed in practice; runtime typically restarts long before wraparound

**Example: Maximum timer interval**

```c
// Maximum safe one-shot timer: ~71 minutes
uint32_t max_interval = UINT32_MAX;  // 4,294,967,295 us
hive_timer_after(max_interval, &timer);  // OK, fires after ~71.6 minutes

// For longer intervals, use periodic timer with counter
uint32_t seventy_two_min_us = 72 * 60 * 1000000;  // 4.32 billion us
// ERROR: Exceeds UINT32_MAX (4.29 billion), wraps to ~25 seconds

// Correct approach for long intervals:
uint32_t tick_interval = 60 * 1000000;  // 1 minute
hive_timer_every(tick_interval, &timer);
// Count ticks in actor to reach 60 minutes
```

**Comparison with rest of system:**

Feature | Timer API | IPC/File/Network API
--------|-----------|---------------------
**Units** | Microseconds (`uint32_t`) | Milliseconds (`int32_t`)
**Max value** | ~71 minutes | ~24.8 days (2^31 ms)
**Signed/Unsigned** | Unsigned (always positive) | Signed (negative = block forever)
**Wraparound** | At ~71 minutes | At ~24.8 days (unlikely in practice)

**Design rationale:**
- Microseconds for timers: Sub-millisecond intervals common in embedded systems (sensor sampling, PWM, etc.)
- Milliseconds for I/O: Network/file timeouts rarely need microsecond precision
- 32-bit limit: Embedded systems prefer fixed-size types; 64-bit would waste memory
- Trade-off: Accept wraparound at ~71 minutes for memory efficiency

## Network API

Non-blocking network I/O with blocking wrappers.

```c
// Socket management
hive_status hive_net_listen(uint16_t port, int *fd_out);
hive_status hive_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms);
hive_status hive_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms);
hive_status hive_net_close(int fd);

// Data transfer
hive_status hive_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms);
hive_status hive_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms);
```

**DNS resolution is out of scope.** The `ip` parameter must be a numeric IPv4 address (e.g., "192.168.1.1"). Hostnames are not supported. Rationale:
- DNS resolution (`getaddrinfo`, `gethostbyname`) is blocking and would stall the scheduler
- On STM32 bare metal, DNS typically unavailable or requires complex async plumbing
- Callers needing DNS should resolve externally before calling `hive_net_connect()`

All functions with `timeout_ms` parameter support **timeout enforcement**:

- `timeout_ms == 0`: Non-blocking, returns `HIVE_ERR_WOULDBLOCK` if would block
- `timeout_ms < 0`: Block forever until I/O completes
- `timeout_ms > 0`: Block up to timeout, returns `HIVE_ERR_TIMEOUT` if exceeded

**Timeout implementation:** Uses timer-based enforcement (consistent with `hive_ipc_recv`). When timeout expires, a timer message wakes the actor and `HIVE_ERR_TIMEOUT` is returned. This is essential for handling unreachable hosts, slow connections, and implementing application-level keepalives.

On blocking calls, the actor yields to the scheduler. The scheduler's event loop registers the I/O operation with the platform's event notification mechanism (epoll on Linux, interrupt flags on STM32) and dispatches the operation when the socket becomes ready.

### Completion Semantics

**`hive_net_recv()` - Partial completion:**
- Returns successfully when **at least 1 byte** is read (or 0 for EOF/peer closed)
- Does NOT loop until `len` bytes are received
- `*received` contains actual bytes read (may be less than `len`)
- Caller must loop if full message is required:
```c
size_t total = 0;
while (total < expected_len) {
    size_t n;
    hive_status s = hive_net_recv(fd, buf + total, expected_len - total, &n, timeout);
    if (HIVE_FAILED(s)) return s;
    if (n == 0) return HIVE_ERROR(HIVE_ERR_IO, "Connection closed");
    total += n;
}
```

**`hive_net_send()` - Partial completion:**
- Returns successfully when **at least 1 byte** is written
- Does NOT loop until `len` bytes are sent
- `*sent` contains actual bytes written (may be less than `len`)
- Caller must loop if full buffer must be sent:
```c
size_t total = 0;
while (total < len) {
    size_t n;
    hive_status s = hive_net_send(fd, buf + total, len - total, &n, timeout);
    if (HIVE_FAILED(s)) return s;
    total += n;
}
```

**`hive_net_connect()` - Async connect completion:**
- If `connect()` returns `EINPROGRESS`: registers for `EPOLLOUT`, actor yields
- When socket becomes writable: checks `getsockopt(fd, SOL_SOCKET, SO_ERROR, ...)`
- If `SO_ERROR == 0`: success, returns `HIVE_SUCCESS` with connected socket in `*fd_out`
- If `SO_ERROR != 0`: returns `HIVE_ERR_IO` with error message, socket is closed
- If timeout expires before writable: returns `HIVE_ERR_TIMEOUT`, socket is closed

**`hive_net_accept()` - Connection ready:**
- Waits for `EPOLLIN` on listen socket (incoming connection ready)
- Calls `accept()` to obtain connected socket
- Returns `HIVE_SUCCESS` with new socket in `*conn_fd_out`

**Rationale for partial completion:**
- Matches POSIX socket semantics (recv/send may return partial)
- Avoids hidden loops that could block indefinitely
- Caller controls retry policy and timeout behavior
- Simpler implementation, more predictable behavior

## File API

File I/O operations.

```c
hive_status hive_file_open(const char *path, int flags, int mode, int *fd_out);
hive_status hive_file_close(int fd);

hive_status hive_file_read(int fd, void *buf, size_t len, size_t *bytes_read);
hive_status hive_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read);

hive_status hive_file_write(int fd, const void *buf, size_t len, size_t *bytes_written);
hive_status hive_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written);

hive_status hive_file_sync(int fd);
```

### Platform-Independent Flags

Use `HIVE_O_*` flags instead of POSIX `O_*` flags for cross-platform compatibility:

```c
#define HIVE_O_RDONLY   0x0001
#define HIVE_O_WRONLY   0x0002
#define HIVE_O_RDWR     0x0003
#define HIVE_O_CREAT    0x0100
#define HIVE_O_TRUNC    0x0200
#define HIVE_O_APPEND   0x0400
```

On Linux, these map directly to POSIX equivalents. On STM32, they're interpreted by the flash file implementation.

### Platform Differences

**Linux:**
- Standard filesystem paths (e.g., `/tmp/log.bin`)
- Uses POSIX `open()`, `read()`, `write()`, `fsync()`
- Synchronous blocking I/O
- All flags and functions fully supported

**STM32:**
- Virtual file paths mapped to flash sectors (e.g., `/log`, `/config`)
- Board configuration via -D flags:
  ```makefile
  CFLAGS += -DHIVE_VFILE_LOG_BASE=0x08020000
  CFLAGS += -DHIVE_VFILE_LOG_SIZE=131072
  CFLAGS += -DHIVE_VFILE_LOG_SECTOR=5
  ```
- Ring buffer for efficient writes (most are O(1), blocks to flush when full)
- `HIVE_O_TRUNC` triggers flash sector erase (blocks 1-4 seconds)
- `hive_file_sync()` drains ring buffer to flash (blocking)
- No data loss - writes block when buffer is full to ensure delivery

**STM32 Write Behavior:**

The STM32 implementation uses a ring buffer for efficiency. Most writes complete
immediately. When the buffer fills up, `write()` blocks to flush data to flash
before continuing. This ensures the same no-data-loss semantics as Linux, while
still providing fast writes in the common case.

| Platform | Behavior | Data Loss |
|----------|----------|-----------|
| Linux | Blocking | Never (or error) |
| STM32 | Fast (ring buffer), blocks when full | Never (or error) |

**STM32 API Restrictions:**

| Feature | Linux | STM32 |
|---------|-------|-------|
| Arbitrary paths | Yes | No - only virtual paths (`/log`, `/config`) |
| `hive_file_read()` | Works | **Returns error** - use `pread()` |
| `hive_file_pread()` | Works | Works (direct flash read) |
| `hive_file_write()` | Blocking | Ring buffer (fast, blocks when full) |
| `hive_file_pwrite()` | Works | **Returns error** |
| Multiple writers | Yes | No - single writer at a time |

**STM32 Flag Restrictions:**

| Flag | Linux | STM32 |
|------|-------|-------|
| `HIVE_O_RDONLY` | Supported | Supported |
| `HIVE_O_WRONLY` | Supported | **Requires `HIVE_O_TRUNC`** |
| `HIVE_O_RDWR` | Supported | **Rejected** (read() doesn't work) |
| `HIVE_O_CREAT` | Creates file | Ignored (virtual files always exist) |
| `HIVE_O_TRUNC` | Truncates file | **Required for writes** (erases flash sector) |
| `HIVE_O_APPEND` | Appends | Ignored (always appends via ring buffer) |

**STM32 Ring Buffer Defaults** (`hive_static_config.h`):
```c
#define HIVE_FILE_RING_SIZE     (8 * 1024)  // 8 KB
#define HIVE_FILE_BLOCK_SIZE    256         // Flash write block size
```

**STM32 Usage Example:**
```c
int log_fd;
// Erase flash sector and open for writing
hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &log_fd);

// Fast writes go to ring buffer (rarely blocks, only when buffer full)
hive_file_write(log_fd, &sensor_data, sizeof(sensor_data), &written);

// Periodically flush to flash (call from low-priority logger actor)
hive_file_sync(log_fd);

hive_file_close(log_fd);
```

## Logging API

Structured logging with compile-time level filtering and dual output (console + file).

### Log Levels

```c
#define HIVE_LOG_LEVEL_TRACE 0  // Verbose tracing
#define HIVE_LOG_LEVEL_DEBUG 1  // Debug information
#define HIVE_LOG_LEVEL_INFO  2  // General information (default)
#define HIVE_LOG_LEVEL_WARN  3  // Warnings
#define HIVE_LOG_LEVEL_ERROR 4  // Errors
#define HIVE_LOG_LEVEL_NONE  5  // Disable all logging
```

### Logging Macros

```c
HIVE_LOG_TRACE(fmt, ...)  // Compile out if HIVE_LOG_LEVEL > TRACE
HIVE_LOG_DEBUG(fmt, ...)  // Compile out if HIVE_LOG_LEVEL > DEBUG
HIVE_LOG_INFO(fmt, ...)   // Compile out if HIVE_LOG_LEVEL > INFO
HIVE_LOG_WARN(fmt, ...)   // Compile out if HIVE_LOG_LEVEL > WARN
HIVE_LOG_ERROR(fmt, ...)  // Compile out if HIVE_LOG_LEVEL > ERROR
```

### File Logging API

```c
hive_status hive_log_init(void);              // Initialize (call once at startup)
hive_status hive_log_file_open(const char *path);  // Open log file
hive_status hive_log_file_sync(void);         // Flush to storage
hive_status hive_log_file_close(void);        // Close log file
void hive_log_cleanup(void);                  // Cleanup (call at shutdown)
```

### Binary Log Format

Log files use a binary format with 12-byte headers for efficient storage and crash recovery:

```
Offset  Size  Field
0       2     magic       0x4C47 ("LG" little-endian)
2       2     seq         Monotonic sequence number
4       4     timestamp   Microseconds since boot
8       2     len         Payload length
10      1     level       Log level (0-4)
11      1     reserved    Always 0
12      len   payload     Log message text (no null terminator)
```

### Compile-Time Configuration (`hive_static_config.h`)

```c
// Maximum log entry size (excluding header)
#define HIVE_LOG_MAX_ENTRY_SIZE 128

// Enable console output (default: 1 on Linux, 0 on STM32)
#define HIVE_LOG_TO_STDOUT 1

// Enable file logging (default: 1 on both platforms)
#define HIVE_LOG_TO_FILE 1

// Log file path (default: "/var/tmp/hive.log" on Linux, "/log" on STM32)
#define HIVE_LOG_FILE_PATH "/var/tmp/hive.log"

// Log level (default: INFO)
#define HIVE_LOG_LEVEL HIVE_LOG_LEVEL_INFO
```

### Platform Differences

| Feature | Linux | STM32 |
|---------|-------|-------|
| Console output | Enabled by default | Disabled by default |
| File logging | Regular file | Flash-backed virtual file |
| Default log path | `/var/tmp/hive.log` | `/log` |
| Default log level | INFO | NONE (disabled) |

### Usage Pattern

```c
// Supervisor actor manages log lifecycle
void supervisor_actor(void *arg) {
    // ARM phase: open log file (on STM32, erases flash sector)
    hive_log_file_open(HIVE_LOG_FILE_PATH);

    // Start periodic sync timer (every 4 seconds)
    timer_id sync_timer;
    hive_timer_every(4000000, &sync_timer);

    while (flying) {
        hive_message msg;
        hive_ipc_recv(&msg, -1);
        if (msg.class == HIVE_MSG_TIMER && msg.tag == sync_timer) {
            hive_log_file_sync();  // Flush logs to storage
        }
    }

    // DISARM phase: close log file
    hive_timer_cancel(sync_timer);
    hive_log_file_close();
}
```

## Memory Allocation Architecture

The runtime uses **compile-time configuration** for deterministic memory allocation.

### Compile-Time Configuration (`hive_static_config.h`)

All resource limits are defined at compile-time and require recompilation to change:

```c
// Feature toggles (set to 0 to disable)
#define HIVE_ENABLE_NET 1                     // Network I/O subsystem
#define HIVE_ENABLE_FILE 1                    // File I/O subsystem

// Resource limits
#define HIVE_MAX_ACTORS 64                    // Maximum concurrent actors
#define HIVE_STACK_ARENA_SIZE (1*1024*1024)   // Stack arena size (1 MB)
#define HIVE_MAX_BUSES 32                     // Maximum concurrent buses
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 256      // Mailbox entry pool
#define HIVE_MESSAGE_DATA_POOL_SIZE 256       // Message data pool
#define HIVE_MAX_MESSAGE_SIZE 256             // Maximum message size
#define HIVE_LINK_ENTRY_POOL_SIZE 128         // Link entry pool
#define HIVE_MONITOR_ENTRY_POOL_SIZE 128      // Monitor entry pool
#define HIVE_TIMER_ENTRY_POOL_SIZE 64         // Timer entry pool
#define HIVE_DEFAULT_STACK_SIZE 65536         // Default actor stack size
```

Feature toggles can also be set via Makefile: `make ENABLE_NET=0 ENABLE_FILE=0`.

All runtime structures are **statically allocated** based on these limits. Actor stacks use a static arena allocator by default (configurable via `actor_config.malloc_stack` for malloc). This ensures:
- Deterministic memory footprint (calculable at link time)
- Zero heap allocation in runtime operations (see Heap Usage Policy)
- O(1) pool allocation for hot paths (scheduling, IPC); O(n) bounded arena allocation for cold paths (spawn/exit)
- Suitable for embedded/MCU deployment

### Runtime API

```c
// Initialize runtime (call once from main)
hive_status hive_init(void);

// Run scheduler (blocks until all actors exit or hive_shutdown called)
void hive_run(void);

// Request graceful shutdown
void hive_shutdown(void);

// Cleanup runtime resources (call after hive_run completes)
void hive_cleanup(void);
```

## Actor Death Handling

When an actor dies (via `hive_exit()`, crash, or external kill):

**Note:** Stack overflow results in undefined behavior since there is no runtime detection. The cleanup below assumes normal actor death. If stack overflow corrupts runtime metadata, the system may crash before cleanup completes.

**Normal death cleanup:**

1. **Mailbox cleared:** All pending messages are discarded.

2. **Links notified:** All linked actors receive an exit message (class=`HIVE_MSG_EXIT`, sender=dying actor).

3. **Monitors notified:** All monitoring actors receive an exit message (class=`HIVE_MSG_EXIT`).

4. **Bus subscriptions removed:** Actor is unsubscribed from all buses.

5. **Timers cancelled:** All timers owned by the actor are cancelled.

6. **Resources freed:** Stack and actor table entry are released.

### Exit Notification Ordering

**When exit notifications are sent:**

Exit notifications (steps 2-3 above) are enqueued in recipient mailboxes **during death processing**, following standard FIFO mailbox semantics.

**Ordering guarantees:**

1. **Tail-append semantics:**
   - Exit notifications are enqueued at the **tail** of recipient mailboxes at the moment death processing occurs
   - Any messages already in that mailbox before that point remain ahead of the exit notification
   - Example: If B's mailbox contains [M1, M2] when A's death is processed, B receives: M1 -> M2 -> EXIT(A)

2. **No global ordering guarantee:**
   - The runtime does NOT guarantee ordering relative to concurrent events (timers, network I/O)
   - If death processing and other enqueues occur in the same scheduling phase, ordering depends on processing order
   - Example: If A dies and a timer fires for B in the same phase, the order of EXIT(A) vs timer tick in B's mailbox depends on event dispatch order

3. **Messages in dying actor's mailbox:**
   - Dying actor's mailbox is **cleared** (step 1)
   - All pending messages are **discarded**
   - Senders are **not** notified of message loss

4. **Multiple recipients:**
   - Each recipient's exit notification is enqueued independently
   - No ordering guarantee across different recipients
   - Example: A linked to B and C, dies. B and C both receive EXIT(A), but no guarantee which processes it first.

**Consequences:**

```c
// Dying actor sends messages before death
void actor_A(void *arg) {
    hive_ipc_notify(B, 0, &msg1, sizeof(msg1));  // Enqueued in B's mailbox
    hive_ipc_notify(C, 0, &msg2, sizeof(msg2));  // Enqueued in C's mailbox
    hive_exit();  // Exit notifications sent to links/monitors
}
// Linked actor B will receive: msg1, then EXIT(A) (class=HIVE_MSG_EXIT)
// Actor C will receive: msg2 (no exit notification, not linked)

// Messages sent TO dying actor are lost
void actor_B(void *arg) {
    hive_ipc_notify(A, 0, &msg, sizeof(msg));  // Enqueued in A's mailbox
}
void actor_A(void *arg) {
    // ... does some work ...
    hive_exit();  // Mailbox cleared, msg from B is discarded
}
```

**Design rationale:**
- FIFO enqueuing = predictable, testable behavior
- Messages before death delivered before exit = "happens-before" relationship
- Mailbox clearing = simple cleanup, no orphaned messages
- No sender notification = simpler implementation, sender must handle recipient death via links/monitors

**Design choice:** Exit notifications follow standard FIFO mailbox ordering, enqueued at tail when death is processed. This provides simpler, more predictable semantics.

## Scheduler Main Loop

Pseudocode for the scheduler:

```
procedure hive_run():
    epoll_fd = epoll_create1(0)

    while not shutdown_requested and actors_alive > 0:
        # 1. Pick highest-priority runnable actor
        actor = pick_next_runnable()

        if actor exists:
            # 2. Context switch to actor
            current_actor = actor
            context_switch(scheduler_ctx, actor.ctx)
            # Returns here when actor yields/blocks

        else:
            # 3. No runnable actors, wait for I/O events
            events = epoll_wait(epoll_fd, timeout=10ms)  # Bounded timeout for defensive wakeup

            # 4. Dispatch I/O events
            for event in events:
                source = event.data.ptr
                if source.type == TIMER:
                    read(timerfd, &expirations, 8)  # Clear level-triggered, get count
                    send_timer_tick(source.owner)   # One tick regardless of count
                    wake_actor(source.owner)
                elif source.type == NETWORK:
                    perform_io_operation(source)  # recv/send partial, connect checks SO_ERROR
                    wake_actor(source.owner)
```

### Simulation Time Integration

For integration with external event loops (e.g., Webots simulator), use `hive_advance_time()` and `hive_run_until_blocked()`:

```
procedure hive_advance_time(delta_us):
    # Enable simulation time mode on first call
    if not sim_mode:
        sim_mode = true

    # Advance simulation time
    sim_time_us += delta_us

    # Fire all timers that are now due
    for timer in active_timers:
        while timer.expiry_us <= sim_time_us:
            send_timer_message(timer.owner, timer.id)
            if timer.periodic:
                timer.expiry_us += timer.interval_us
            else:
                remove_timer(timer)
                break

procedure hive_run_until_blocked():
    # Run actors until all are blocked (WAITING) or dead
    while not shutdown_requested and num_actors > 0:
        # Poll for I/O events (non-blocking)
        events = epoll_wait(epoll_fd, timeout=0)
        dispatch_io_events(events)

        # Find next ready actor (priority-based round-robin)
        actor = find_next_runnable()
        if actor is None:
            break  # All actors blocked or dead

        context_switch(scheduler_ctx, actor.ctx)

    return HIVE_OK
```

**Use case:** Simulation integration where an external loop controls time:
```c
// All actors use timers (same code as production)
void sensor_actor(void *arg) {
    timer_id timer;
    hive_timer_every(TIME_STEP_MS * 1000, &timer);  // Timer-driven
    while (1) {
        hive_message msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
        read_sensors();
        publish_to_bus();
    }
}

// Main loop advances simulation time
while (wb_robot_step(TIME_STEP_MS) != -1) {
    hive_advance_time(TIME_STEP_MS * 1000);  // Fire due timers
    hive_run_until_blocked();                 // Run until all blocked
}
```

**Benefits of simulation time mode:**
- Same actor code runs in simulation and production
- Deterministic, reproducible behavior
- Timer granularity matches simulation time step
- No wall-clock dependencies in actors

## Event Loop Architecture

When all actors are blocked on I/O, the scheduler waits efficiently for I/O events using platform-specific event notification mechanisms.

### Implementation

**Linux (epoll):**
```c
// Scheduler event loop
int epoll_fd = epoll_create1(0);

// Timer creation - add timerfd to epoll
int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
timerfd_settime(tfd, 0, &its, NULL);
struct epoll_event ev = {.events = EPOLLIN, .data.ptr = timer_source};
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, tfd, &ev);

// Network I/O - add socket to epoll when would block
int sock = socket(AF_INET, SOCK_STREAM, 0);
fcntl(sock, F_SETFL, O_NONBLOCK);
struct epoll_event ev = {.events = EPOLLIN, .data.ptr = net_source};
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sock, &ev);

// Scheduler waits when no actors are ready
if (no_runnable_actors) {
    struct epoll_event events[64];
    // Bounded timeout provides defensive wakeup (guards against lost events)
    int n = epoll_wait(epoll_fd, events, 64, 10);  // 10ms timeout
    if (n < 0 && errno == EINTR) continue;  // Signal interrupted, retry
    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;
        if (source->type == TIMER) {
            uint64_t expirations;
            read(source->fd, &expirations, sizeof(expirations));  // Clear timerfd
            // expirations may be > 1 for periodic timers (coalesced)
            // We send ONE tick message regardless of count
        }
        dispatch_io_event(source);  // Handle timer tick or network I/O
    }
}
```

**STM32 (bare metal with WFI):**
```c
// Interrupt handlers set flags
volatile uint32_t pending_events = 0;
#define EVENT_TIMER   (1 << 0)
#define EVENT_NETWORK (1 << 1)

void SysTick_Handler(void) {
    pending_events |= EVENT_TIMER;
}

void ETH_IRQHandler(void) {
    pending_events |= EVENT_NETWORK;
}

// Scheduler waits when no actors are ready
if (no_runnable_actors) {
    __disable_irq();
    if (pending_events == 0) {
        __WFI();  // Wait For Interrupt - CPU sleeps until interrupt
    }
    __enable_irq();

    // Process pending events
    if (pending_events & EVENT_TIMER) {
        pending_events &= ~EVENT_TIMER;
        handle_timer_event();
    }
    if (pending_events & EVENT_NETWORK) {
        pending_events &= ~EVENT_NETWORK;
        handle_network_event();
    }
}
```

### Semantics

- **Single-threaded event loop**: All I/O is multiplexed in the scheduler thread
- **Non-blocking I/O registration**: Timers create timerfds, network operations register sockets with epoll
- **Event dispatching**: epoll_wait returns when any I/O source becomes ready
- **Immediate handling**: Timer ticks and network I/O are processed immediately when ready

### Event Loop Guarantees

**Event processing order:**
- epoll returns events in **kernel order** (arrival-dependent, not guaranteed FIFO)
- All returned events are processed before next epoll_wait
- No I/O events are lost

**epoll_wait return handling:**
- Returns > 0: Events ready, process them
- Returns 0: Timeout expired (10ms), no events - scheduler retries (defensive wakeup)
- Returns -1 with `errno=EINTR`: Signal interrupted syscall - scheduler retries
- Note: Runtime APIs are not reentrant - signal handlers must not call runtime APIs

**Lost event prevention:**
- Timerfds are level-triggered (epoll reports ready until `read()` clears the event)
- Scheduler **must** read timerfd to clear level-triggered state (otherwise epoll spins)
- Sockets are level-triggered by default (readable until data consumed)
- epoll guarantees: if I/O is ready, epoll_wait will return it

**File I/O stalling:**
- File operations (read, write, fsync) are **synchronous** and stall the entire scheduler
- See "Scheduler-Stalling Calls" section for detailed semantics and consequences
- On embedded systems: FATFS/littlefs operations are fast (< 1ms typical)
- On Linux dev: Acceptable for development workloads

### Advantages

- **Standard pattern**: Used by Node.js, Tokio, libuv, asyncio
- **Single-threaded**: No synchronization, no race conditions
- **Efficient**: Kernel/hardware wakes scheduler only when I/O is ready
- **Portable**: epoll (Linux), kqueue (BSD/macOS), WFI + interrupts (bare metal)
- **Low overhead**: No context switches between threads, no queue copies

## Platform Abstraction

The runtime abstracts platform-specific functionality:

| Component | Linux (dev) | STM32 bare metal (prod) |
|-----------|-------------|-------------------------|
| Context switch | x86-64 asm | ARM Cortex-M asm |
| Event notification | epoll | WFI + interrupt flags |
| Timer | timerfd + epoll | Software timer wheel (SysTick/TIM) |
| Network | Non-blocking BSD sockets + epoll | lwIP NO_SYS mode |
| File | Synchronous POSIX | Synchronous FATFS or littlefs |

### Platform-Specific Source Files

Platform-specific implementations use naming convention `*_linux.c` / `*_stm32.c`:

| Component | Linux | STM32 |
|-----------|-------|-------|
| Scheduler | `hive_scheduler_linux.c` | `hive_scheduler_stm32.c` |
| Timer | `hive_timer_linux.c` | `hive_timer_stm32.c` |
| Context switch | `hive_context_x86_64.S` | `hive_context_arm_cm.S` |

Shared code in `hive_context.c` handles platform-specific context initialization.

### Building for Different Platforms

```bash
# Linux (default)
make                           # Build for x86-64 Linux
make PLATFORM=linux            # Explicit

# STM32 (requires ARM cross-compiler)
make PLATFORM=stm32 CC=arm-none-eabi-gcc

# Feature toggles (disable network and file I/O)
make ENABLE_NET=0 ENABLE_FILE=0

# STM32 defaults to ENABLE_NET=0 ENABLE_FILE=0
```

Platform selection sets `HIVE_PLATFORM_LINUX` or `HIVE_PLATFORM_STM32` preprocessor defines.

## Stack Overflow

### No Runtime Detection

The runtime does **not** perform stack overflow detection. Stack overflow results in **undefined behavior**.

**Rationale:** Pattern-based guard detection (checking magic values on context switch) was removed because:
1. Detection occurred too late - after memory corruption had already happened
2. Severe overflows corrupt memory beyond the guard, causing crashes before detection
3. The mechanism gave a false sense of security while not providing reliable protection
4. Proper stack sizing is the only reliable solution

### Behavior on Overflow

**Undefined behavior.** Possible outcomes:
- **Segfault** (most likely on Linux)
- **Corruption of adjacent actor stacks** (arena allocator places stacks contiguously)
- **Corruption of runtime state** (if overflow is severe)
- **Silent data corruption** (worst case - no immediate crash)

**Links/monitors are NOT notified.** The `HIVE_EXIT_CRASH_STACK` exit reason exists but is not currently triggered.

### Required Mitigation

Stack overflow prevention is the **application's responsibility**:

1. **Size stacks conservatively** - Use 2-3x safety margin over measured worst-case
2. **Profile stack usage** - Measure actual usage under worst-case conditions
3. **Use static analysis** - Tools like `gcc -fstack-usage` report per-function stack use
4. **Test thoroughly** - Include stress tests with deep call stacks
5. **Use AddressSanitizer** - `-fsanitize=address` catches stack issues during development

### System-Level Protection

For production systems:
- **Watchdog timer:** Detect hung system, trigger reboot/failsafe
- **Supervisor architecture:** Monitor actors, restart on anomaly
- **Memory isolation:** Hardware MPU (ARM Cortex-M) can provide hardware-guaranteed protection

### Future Considerations

Hardware-based protection may be added in future versions:
- **Linux:** `mprotect()` guard pages (immediate SIGSEGV on overflow)
- **ARM Cortex-M:** MPU guard regions (immediate HardFault on overflow)

These would provide immediate, hardware-guaranteed detection with zero runtime overhead.

## Future Considerations

Not in scope for first version, but noted for future:
