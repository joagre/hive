# Actor Runtime Specification

## Overview

A minimalistic actor-based runtime designed for **embedded and safety-critical systems**. The runtime implements cooperative multitasking with priority-based scheduling and message passing, inspired by Erlang's actor model.

**Target use cases:** Drone autopilots, industrial sensor networks, robotics control systems, and other resource-constrained embedded applications requiring structured concurrency.

**Design principles:**

1. **Minimalistic**: Only essential features, no bloat
2. **Predictable**: Cooperative scheduling, no surprises
3. **Modern C11**: Clean, safe, standards-compliant code
4. **Static allocation**: Statically-bounded memory regions, zero heap fragmentation, deterministic footprint
5. **Pool-based allocation**: O(1) pools for hot paths; stack arena allocation is bounded and occurs only on spawn/exit
6. **Explicit control**: Actors yield explicitly, no preemption

### Heap Usage Policy

**Allowed heap use** (malloc/free):
- `rt_init()`: None (uses only static/BSS)
- `rt_spawn()` / `rt_spawn_ex()`: Actor stack allocation **only if** `actor_config.malloc_stack = true`
  - Default: Arena allocator (static memory, no malloc)
  - Optional: Explicit malloc via config flag
- Actor exit/cleanup: Corresponding free for malloc'd stacks

**Forbidden heap use** (exhaustive):
- Scheduler loop (`rt_run()`, `rt_yield()`, context switching)
- IPC (`rt_ipc_send()`, `rt_ipc_recv()`, `rt_ipc_release()`)
- Timers (`rt_timer_after()`, `rt_timer_every()`, timer delivery)
- Bus (`rt_bus_publish()`, `rt_bus_read()`, `rt_bus_read_wait()`)
- Network I/O (`rt_net_*()` functions, event loop dispatch)
- File I/O (`rt_file_*()` functions, synchronous execution)
- Linking/monitoring (`rt_link()`, `rt_monitor()`, death notifications)
- All I/O event processing (epoll event dispatch)

**Consequence**: All "hot path" operations (scheduling, IPC, I/O) use **static pools** with **O(1) allocation** and return `RT_ERR_NOMEM` on pool exhaustion. Stack allocation (spawn/exit, cold path) uses arena allocator with O(n) first-fit search bounded by number of free blocks. No malloc in hot paths, no heap fragmentation, predictable allocation latency.

**Linux verification**: Run with `LD_PRELOAD` malloc wrapper to assert no malloc calls after `rt_init()` (except explicit `malloc_stack = true` spawns).

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
│              │   epoll/select)     │                     │
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
- The actor explicitly yields via `rt_yield()`
- The actor exits via `rt_exit()`

There is no preemptive scheduling or time slicing within the actor runtime.

**Reentrancy constraint:**
- Runtime APIs are **not reentrant**
- Actors **must not** call runtime APIs from signal handlers or interrupt service routines (ISRs)
- All runtime API calls must occur from actor context (the scheduler thread)
- Violating this constraint results in undefined behavior (data corruption, crashes)

### Actor Blocking Semantics

When an actor calls a blocking API, the following contract applies:

**State transition:**
- Actor transitions to `ACTOR_STATE_BLOCKED` and yields to scheduler
- Actor is removed from run queue (not schedulable until unblocked)
- Scheduler saves actor context and switches to next runnable actor

**Operations that block (actor yields, other actors run):**
- `rt_ipc_recv()` with timeout > 0 or timeout < 0 (block until message or timeout)
- `rt_ipc_send()` with `IPC_SYNC` (block until receiver releases)
- `rt_net_send()`, `rt_net_recv()` (block until I/O completes or timeout)
- `rt_bus_read_wait()` (block until data available or timeout)

**Note:** File I/O (`rt_file_*`) is NOT in this list. See "Scheduler-Stalling Calls" below.

**Mailbox availability while blocked:**
- Blocked actors **can** receive mailbox messages
- Message enqueues are scheduler-owned operations (do not require actor to be runnable)
- Enqueued messages are available when actor unblocks

**Unblock conditions:**
- I/O readiness signaled (network socket becomes readable/writable)
- Timer expires (for APIs with timeout)
- Message arrives in mailbox (for `rt_ipc_recv()`)
- Bus data published (for `rt_bus_read_wait()`)
- Explicit release occurs (for `IPC_SYNC` sender)
- **Important**: Mailbox arrival only unblocks actors blocked in `rt_ipc_recv()`, not actors blocked on network I/O, bus read, or `IPC_SYNC` send

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

All events are processed before any actor runs. This ensures the actor sees a consistent state.

**Timeout vs I/O readiness - no race:**

Timeouts and I/O readiness do not "race" in the traditional sense. They are **sequenced** by the event loop:

- If I/O becomes ready **before** timeout expires: Only network event fires, actor receives data
- If timeout expires **before** I/O ready: Only timer event fires, actor receives timeout error
- If both fire in the **same** `epoll_wait()` call: Both are processed, actor sees both I/O result and timer message

In the simultaneous case, the actor's wakeup handler checks for timeout by examining the mailbox head. If the timer message is present, timeout is reported. The I/O result (if any) is discarded.

**Note:** This is "first-processed wins" semantics based on epoll array order. For most practical scenarios (I/O ready well before or well after timeout), this distinction is irrelevant.

**Request serialization:**
- Constraint: **One outstanding blocking request per actor per I/O subsystem**
- Enforced by scheduler: blocked actor cannot issue new requests
- When timeout occurs: I/O registration cleaned up, any late I/O result is ignored
- New request from same actor gets fresh state

**Determinism guarantee:**
- **Deterministic policy**: Given the same sequence of epoll events in the same order, scheduling decisions are deterministic
- Runtime does not introduce nondeterminism beyond external event arrival order
- No phantom wakeups (actor only unblocks on specified conditions)
- FIFO ordering among actors enqueued in the same scheduling phase

### Scheduler-Stalling Calls

File I/O operations (`rt_file_read()`, `rt_file_write()`, `rt_file_sync()`) are **synchronous** and stall the entire runtime:

**Behavior:**
- Calling actor does NOT transition to `ACTOR_STATE_BLOCKED`
- The scheduler event loop is paused during the syscall
- All actors are stalled (no actor runs while file I/O executes)
- Timers do not fire during the stall
- Network events are not processed during the stall

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
| 0 | `RT_PRIO_CRITICAL` | Flight control loop |
| 1 | `RT_PRIO_HIGH` | Sensor fusion |
| 2 | `RT_PRIO_NORMAL` | Telemetry |
| 3 | `RT_PRIO_LOW` | Logging |

The scheduler always picks the highest-priority runnable actor. Within the same priority level, actors are scheduled round-robin.

**Fairness guarantees:**
- Round-robin scheduling within a priority level ensures fairness among actors of equal priority
- The scheduler does **not** guarantee starvation freedom across priority levels
- A continuously runnable high-priority actor can starve lower-priority actors indefinitely
- Applications must design priority hierarchies to avoid starvation scenarios

### Context Switching

Context switching is implemented via manual assembly for performance:

- **x86-64 (Linux):** Save/restore callee-saved registers (rbx, rbp, r12-r15) and stack pointer
- **ARM Cortex-M (STM32):** Save/restore callee-saved registers (r4-r11) and stack pointer

No use of setjmp/longjmp or ucontext for performance reasons.

## Thread Safety

### Single-Threaded Event Loop Model

The runtime is **completely single-threaded**. All runtime operations execute in a single scheduler thread with an event loop architecture. There are **no I/O worker threads** - all I/O operations are integrated into the scheduler's event loop using platform-specific non-blocking mechanisms.

**Key architectural invariant:** Only one actor executes at any given time. Actors run cooperatively until they explicitly yield (via `rt_yield()`, blocking I/O, or `rt_exit()`). The scheduler then switches to the next runnable actor or waits for I/O events.

### Runtime API Thread Safety Contract

**All runtime APIs must be called from actor context (the scheduler thread):**

| API Category | Thread Safety | Enforcement |
|-------------|---------------|-------------|
| **Actor APIs** (`rt_spawn`, `rt_exit`, etc.) | Single-threaded only | Must call from actor context |
| **IPC APIs** (`rt_ipc_send`, `rt_ipc_recv`) | Single-threaded only | Must call from actor context |
| **Bus APIs** (`rt_bus_publish`, `rt_bus_read`) | Single-threaded only | Must call from actor context |
| **Timer APIs** (`rt_timer_after`, `rt_timer_every`) | Single-threaded only | Must call from actor context |
| **File APIs** (`rt_file_read`, `rt_file_write`) | Single-threaded only | Must call from actor context; stalls scheduler |
| **Network APIs** (`rt_net_recv`, `rt_net_send`) | Single-threaded only | Must call from actor context |

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
        rt_net_recv(sock, buf, len, &received, -1);  // Blocks in event loop
        rt_ipc_send(worker, buf, received, IPC_ASYNC);  // Forward to actors
    }
}
```

This pattern is safe because:
- External thread only touches OS-level socket (thread-safe by OS)
- `rt_net_recv()` executes in scheduler thread (actor context)
- `rt_ipc_send()` executes in scheduler thread (actor context)
- No direct runtime state access from external thread

### Synchronization Primitives

The runtime uses **zero synchronization primitives** in the core event loop:

- **No mutexes** - single thread, no contention
- **No atomics** - single writer/reader per data structure
- **No condition variables** - event loop uses epoll/select for waiting
- **No locks** - mailboxes, actor state, bus state accessed only by scheduler thread

**Optional synchronization** (not part of hot path):
- **Logging:** No synchronization needed (single-threaded)

### Rationale

This pure single-threaded model provides:

- **Maximum simplicity:** No lock ordering, no deadlock, trivial to reason about
- **Maximum determinism:** No lock contention, no priority inversion, reproducible execution
- **Maximum performance:** Zero lock overhead, no cache line bouncing, no atomic operations
- **Safety-critical compliance:** Fully deterministic behavior, no threading edge cases
- **Portability:** Works on platforms without pthread support

**Trade-off:** Cannot leverage multiple CPU cores for I/O parallelism. This is acceptable for embedded systems (typically single-core) and simplifies the implementation dramatically.

### Event Loop Architecture

**Linux:**
- Timers: `timerfd` registered in `epoll`
- Network: Non-blocking sockets registered in `epoll`
- File: Direct synchronous I/O (regular files don't work with epoll anyway)
- Event loop: `epoll_wait()` blocks when no actors runnable

**STM32 (bare metal):**
- Timers: Hardware timers (SysTick or TIM peripherals)
- Network: lwIP in NO_SYS mode (polling or interrupt-driven)
- File: Direct synchronous I/O (FATFS/littlefs are fast, <1ms typically)
- Event loop: WFI (Wait For Interrupt) when no actors runnable

**Key insight:** Modern OSes provide non-blocking I/O mechanisms (epoll, kqueue, IOCP). On bare metal, hardware interrupts and WFI provide equivalent functionality. The event loop pattern is standard in async runtimes (Node.js, Tokio, libuv, asyncio).

## Memory Model

### Actor Stacks

Each actor has a fixed-size stack allocated at spawn time. Stack size is configurable per actor via `actor_config.stack_size`, with a system-wide default (`RT_DEFAULT_STACK_SIZE`). Different actors can use different stack sizes to optimize memory usage.

Stack growth/reallocation is not supported. Stack overflow is detected via guard patterns (see "Stack Overflow Detection" section below) with defined, safe behavior.

### Memory Allocation

The runtime uses static allocation for deterministic behavior and suitability for MCU deployment:

**Design Principle:** All memory regions are statically reserved at compile time; allocation within those regions (e.g., stack arena, message pools) occurs at runtime via deterministic algorithms. No heap allocation occurs in hot paths (message passing, scheduling, I/O). Optional malloc for actor stacks when explicitly enabled via `actor_config.malloc_stack = true`.

**Allocation Strategy:**

- **Actor table:** Static array of `RT_MAX_ACTORS` (64), configured at compile time
- **Actor stacks:** Hybrid allocation (configurable per actor)
  - Default: Static arena allocator with `RT_STACK_ARENA_SIZE` (1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
    - Supports different stack sizes for different actors
  - Optional: malloc via `actor_config.malloc_stack = true`
- **IPC pools:** Static pools with O(1) allocation (hot path)
  - Mailbox entry pool: `RT_MAILBOX_ENTRY_POOL_SIZE` (256)
  - Message data pool: `RT_MESSAGE_DATA_POOL_SIZE` (256), fixed-size entries of `RT_MAX_MESSAGE_SIZE` (256 bytes)
- **Link/Monitor pools:** Static pools for actor relationships
  - Link entry pool: `RT_LINK_ENTRY_POOL_SIZE` (128)
  - Monitor entry pool: `RT_MONITOR_ENTRY_POOL_SIZE` (128)
- **Timer pool:** Static pool of `RT_TIMER_ENTRY_POOL_SIZE` (64)
- **Bus storage:** Static arrays per bus
  - Bus entries: Pre-allocated array of `RT_MAX_BUS_ENTRIES` (64) per bus
  - Bus subscribers: Pre-allocated array of `RT_MAX_BUS_SUBSCRIBERS` (32) per bus
  - Entry data: Uses shared message pool
- **I/O sources:** Pool of `io_source` structures for tracking pending I/O operations in the event loop

**Memory Footprint (measured with default configuration):**
- Static data (BSS): ~1.2 MB total (includes 1 MB stack arena)
  - Stack arena: 1,048,576 bytes (1 MB, configurable via `RT_STACK_ARENA_SIZE`)
  - Actor table: 64 actors × ~200 bytes = 12.8 KB
  - Mailbox pool: 256 entries × ~43 bytes = 11 KB
  - Message pool: 256 entries × 256 bytes = 64 KB
  - Sync buffer pool: 64 entries × 256 bytes = 16 KB
  - Link/monitor pools: 256 entries × ~18 bytes = 4.5 KB
  - Timer pool: 64 entries × ~66 bytes = 4.2 KB
  - Bus tables: 32 buses × ~2.9 KB each = 91 KB
  - I/O source pool: 128 entries × ~42 bytes = 5.3 KB
- Without stack arena: ~208 KB

**Total:** ~1.2 MB static (calculable at link time, no heap allocation with default arena)

**Benefits:**

- Deterministic memory: Footprint calculable at link time
- Zero heap fragmentation: No malloc after initialization (except explicit `malloc_stack` flag)
- Predictable allocation: Pool exhaustion returns clear errors (`RT_ERR_NOMEM`)
- Suitable for safety-critical certification
- Predictable timing: O(1) pool allocation for hot paths, bounded arena allocation for cold paths (spawn/exit)

## Error Handling

All runtime functions return `rt_status`:

```c
typedef enum {
    RT_OK = 0,
    RT_ERR_NOMEM,
    RT_ERR_INVALID,
    RT_ERR_TIMEOUT,
    RT_ERR_CLOSED,
    RT_ERR_WOULDBLOCK,
    RT_ERR_IO,
} rt_status_code;

typedef struct {
    rt_status_code code;
    const char    *msg;   // string literal or NULL, never heap-allocated
} rt_status;
```

The `msg` field always points to a string literal or is NULL. It is never dynamically allocated, ensuring safe use across concurrent actors.

Convenience macros:

```c
#define RT_SUCCESS ((rt_status){RT_OK, NULL})
#define RT_FAILED(s) ((s).code != RT_OK)
```

## Design Trade-offs and Sharp Edges

This runtime makes deliberate design choices that favor **determinism, performance, and simplicity** over **ergonomics and safety**. These are not bugs - they are conscious trade-offs suitable for embedded/safety-critical systems with trusted code.

**Accept these consciously before using this runtime:**

### 1. IPC_ASYNC Lifetime Rule is Sharp and Error-Prone

**Trade-off:** Message payload pointer valid only until next `rt_ipc_recv()` call.

**Why this design:**
- Pool reuse: Deterministic O(1) allocation, no fragmentation
- Performance: Single pool slot per actor, no reference counting
- Simplicity: No hidden malloc, no garbage collection

**Consequence:** Easy to misuse - storing `msg.data` across recv iterations causes use-after-free.

**This is not beginner-friendly.** Code must copy data immediately if needed beyond current iteration.

**Mitigation:** Documented with WARNING box and correct/incorrect examples in IPC_ASYNC section. But developers will still make mistakes.

**Acceptable if:** You optimize for determinism over ergonomics, and code reviews catch pointer misuse.

---

### 2. No Per-Actor Mailbox Quota (Global Starvation Possible)

**Trade-off:** One slow/malicious actor can consume all mailbox entries, starving the system.

**Why this design:**
- Simplicity: No per-actor accounting, no quota enforcement
- Flexibility: Bursty actors can use available pool space
- Performance: No quota checks on send path

**Consequence:** A single bad actor can cause global `RT_ERR_NOMEM` failures for all IPC sends.

**Supervisors are not optional.** You must design actor hierarchies with supervision and monitoring.

**Mitigation:** Application-level quotas, supervisor actors, monitoring. Runtime provides primitives, not policies.

**Acceptable if:** You deploy trusted code in embedded systems, not untrusted actors in general-purpose systems.

---

### 3. IPC_SYNC is Deadlock-Prone by Design

**Trade-off:** Synchronous IPC provides backpressure but enables deadlock.

**Why this design:**
- Flow control: Natural rate limiting via sender blocking
- Memory safety: Pinned buffers prevent UAF if sender dies
- Simplicity: No complex async state machines needed

**Consequence:** Circular dependencies, self-send, nested calls all deadlock immediately.

**IPC_SYNC must be used rarely.** Code reviews must treat SYNC usage as suspicious.

**Mitigation:** Documented deadlock scenarios in IPC_SYNC section, runtime detects self-send, auto-release prevents some deadlocks.

**Acceptable if:** You use SYNC sparingly for validated request-response patterns, not general communication.

---

### 4. Bus and IPC Share Message Pool (Resource Contention)

**Trade-off:** High-rate bus publishing can starve IPC globally.

**Why this design:**
- Simplicity: Single message pool, no subsystem isolation
- Flexibility: Pool space shared dynamically based on actual usage
- Memory efficiency: No wasted dedicated pools

**Consequence:** Misconfigured bus can cause all `IPC_ASYNC` sends to fail with `RT_ERR_NOMEM`.

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

If you want automatic safety, use Erlang. If you want deterministic embedded performance with full control, use this.

## Core Types

```c
// Handles (opaque to user)
typedef uint32_t actor_id;
typedef uint32_t bus_id;
typedef uint32_t timer_id;

#define ACTOR_ID_INVALID  ((actor_id)0)
#define BUS_ID_INVALID    ((bus_id)0)
#define TIMER_ID_INVALID  ((timer_id)0)

// Special sender IDs
#define RT_SENDER_TIMER   ((actor_id)0xFFFFFFFF)
#define RT_SENDER_SYSTEM  ((actor_id)0xFFFFFFFE)

// Actor entry point
typedef void (*actor_fn)(void *arg);

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    rt_priority priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
} actor_config;
```

## Actor API

### Lifecycle

```c
// Spawn a new actor with default configuration
actor_id rt_spawn(actor_fn fn, void *arg);

// Spawn with explicit configuration
actor_id rt_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg);

// Terminate current actor
_Noreturn void rt_exit(void);

// Get current actor's ID
actor_id rt_self(void);

// Yield to scheduler
void rt_yield(void);

// Check if actor is alive
bool rt_actor_alive(actor_id id);
```

### Linking and Monitoring

Actors can link to other actors to receive notification when they die:

```c
// Bidirectional link: if either dies, the other receives exit message
rt_status rt_link(actor_id target);
rt_status rt_unlink(actor_id target);

// Unidirectional monitor: receive notification when target dies
rt_status rt_monitor(actor_id target, uint32_t *monitor_ref);
rt_status rt_demonitor(uint32_t monitor_ref);
```

Exit message structure:

```c
typedef enum {
    RT_EXIT_NORMAL,       // Actor called rt_exit()
    RT_EXIT_CRASH,        // Actor function returned without calling rt_exit()
    RT_EXIT_CRASH_STACK,  // Stack overflow detected
    RT_EXIT_KILLED,       // Actor was killed externally
} rt_exit_reason;

typedef struct {
    actor_id       actor;   // who died
    rt_exit_reason reason;
} rt_exit_msg;

// Check if message is exit notification
bool rt_is_exit_msg(const rt_message *msg);
rt_status rt_decode_exit(const rt_message *msg, rt_exit_msg *out);
```

## IPC API

Inter-process communication via mailboxes. Each actor has one mailbox.

### Message Structure

```c
typedef struct {
    actor_id    sender;
    size_t      len;
    const void *data;   // ASYNC: valid until next rt_ipc_recv()
                        // SYNC: valid until rt_ipc_release() (next recv auto-releases)
} rt_message;
```

### Send Modes

```c
typedef enum {
    IPC_ASYNC,    // copy to message pool, sender continues immediately
    IPC_SYNC,     // copy to sync buffer pool, sender blocks until receiver releases
} rt_ipc_mode;
```

**IPC_ASYNC:** Payload copied to message data pool (RT_MESSAGE_DATA_POOL_SIZE). Sender continues immediately. Suitable for small messages and general-purpose communication.

**IPC_SYNC:** Payload copied to sync buffer pool (RT_SYNC_BUFFER_POOL_SIZE). Sender blocks until receiver calls `rt_ipc_release()`. Provides implicit backpressure and memory safety (no UAF if sender dies).

### API Contract: rt_ipc_send()

**Signature:**
```c
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);
```

**Behavior when pools are exhausted:**

`rt_ipc_send()` uses three global pools (mode-dependent):
1. **Mailbox entry pool** (`RT_MAILBOX_ENTRY_POOL_SIZE` = 256) - **all modes** (ASYNC and SYNC)
2. **Message data pool** (`RT_MESSAGE_DATA_POOL_SIZE` = 256) - **IPC_ASYNC only**
3. **Sync buffer pool** (`RT_SYNC_BUFFER_POOL_SIZE` = 64) - **IPC_SYNC only**

Each mode uses mailbox entry pool plus one mode-specific data pool.

**Fail-fast semantics** (chosen for deterministic embedded behavior):

- **Returns `RT_ERR_NOMEM` immediately** if either required pool is exhausted
- **Does NOT block** waiting for pool space to become available
- **Does NOT drop** the message silently
- **Does NOT enqueue** the message (atomic operation: either succeeds completely or fails)

**Specific cases:**

| Mode | Mailbox Pool Full | Message Pool Full | Result |
|------|-------------------|-------------------|--------|
| **IPC_ASYNC** | Returns `RT_ERR_NOMEM` | Returns `RT_ERR_NOMEM` | Fail immediately |
| **IPC_SYNC** | Returns `RT_ERR_NOMEM` | Returns `RT_ERR_NOMEM` (sync buffer pool) | Fail immediately |

**Caller responsibilities:**
- **MUST** check return value and handle `RT_ERR_NOMEM`
- **MUST** implement backpressure/retry logic if needed
- **MUST NOT** assume message was delivered if `RT_FAILED(status)`

**Design rationale:**
- **Deterministic**: Fail-fast behavior is predictable and testable
- **No deadlock**: No blocking on pool exhaustion (see "Deadlock Freedom" below)
- **No silent drops**: Caller knows immediately if send failed
- **Explicit backpressure**: Caller controls retry policy (backoff, drop, etc.)
- **Safety-critical friendly**: No hidden surprises, all failures explicit

**Example: Handling pool exhaustion**

```c
// Bad: Ignoring RT_ERR_NOMEM (message lost, caller unaware)
rt_ipc_send(target, &data, sizeof(data), IPC_ASYNC);  // WRONG

// Good: Check and handle pool exhaustion
rt_status status = rt_ipc_send(target, &data, sizeof(data), IPC_ASYNC);
if (status.code == RT_ERR_NOMEM) {
    // Pool exhausted - implement backpressure
    // Option 1: Drop with telemetry
    log_dropped_message(target);

    // Option 2: Retry with backoff
    rt_message msg;
    rt_ipc_recv(&msg, 10);  // Backoff 10ms, process messages
    rt_ipc_release(&msg);
    // Retry send...

    // Option 3: Fail-fast to caller
    return RT_ERR_NOMEM;
}
```

### Message Data Lifetime

The lifetime of `rt_message.data` depends on the send mode:

**IPC_ASYNC:**

**CRITICAL LIFETIME RULE:**
- **Data is ONLY valid until the next `rt_ipc_recv()` call**
- **Per actor: only ONE message payload pointer is valid at a time**
- **Storing `msg.data` across receive iterations causes use-after-free**
- **If you need the data later, COPY IT IMMEDIATELY**

**Correct pattern (copy immediately if needed):**
```c
rt_message msg;
rt_ipc_recv(&msg, -1);

// SAFE: Copy data immediately
char local_copy[256];
memcpy(local_copy, msg.data, msg.len);
// local_copy is safe to use indefinitely

// UNSAFE: Storing pointer
const char *ptr = msg.data;  // DANGER
rt_ipc_recv(&msg, -1);       // ptr now INVALID (use-after-free)
```

**Implementation details:**
- Data lives in a pool-allocated buffer
- Next recv frees the previous message's buffer and reuses the pool entry
- Calling `rt_ipc_release()` is optional (no-op for ASYNC)

**IPC_SYNC:**
- Data is **valid until `rt_ipc_release()`** is called
- Data lives in pinned runtime buffer (persists even if sender dies)
- Sender is blocked until `rt_ipc_release()` is called
- **Auto-release:** If `rt_ipc_recv()` is called without releasing the previous SYNC message, the sender is automatically unblocked (prevents deadlock)

**Auto-release behavior:**

```c
// Safe: explicit release (recommended)
rt_message msg;
rt_ipc_recv(&msg, -1);
if (msg.data) {
    process(msg.data);
}
rt_ipc_release(&msg);  // Unblocks sender (if SYNC)

// Also safe: auto-release on next recv (forgiving)
rt_message msg1, msg2;
rt_ipc_recv(&msg1, -1);  // msg1 is SYNC
process(msg1.data);
rt_ipc_recv(&msg2, -1);  // msg1 auto-released, sender unblocked
// No explicit release needed, but calling it is harmless
```

**Design rationale:**
- Auto-release prevents deadlock if receiver forgets to call `rt_ipc_release()`
- Forgiving API reduces cognitive load (one code path handles both ASYNC and SYNC)
- Explicit release is still recommended for clarity and to minimize sender blocking time

### Mailbox Semantics

**Capacity model:**

Per-actor mailbox limits: **No per-actor quota** - capacity is constrained by global pools

Global pool limits: **Yes** - all actors share:
- `RT_MAILBOX_ENTRY_POOL_SIZE` (256 default) - mailbox entries
- `RT_MESSAGE_DATA_POOL_SIZE` (256 default) - message data (ASYNC mode only)

**Important:** One slow receiver can consume all mailbox entries, starving other actors.

**Fairness guarantees:** The runtime does not provide per-actor fairness guarantees; resource exhaustion caused by a misbehaving actor is considered an application-level fault. Applications requiring protection against resource starvation should implement supervisor actors or application-level quotas.

**Behavior when pools are exhausted:**

| Mode        | Pool Exhausted Behavior | Blocks Waiting for Pool? | Drops Messages? |
|-------------|-------------------------|--------------------------|-----------------|
| **IPC_ASYNC**    | Returns `RT_ERR_NOMEM` immediately | No | No |
| **IPC_SYNC**  | Returns `RT_ERR_NOMEM` immediately | No | No |

**IPC_SYNC blocking semantics:**
- Blocks **after** message is in mailbox, waiting for receiver to call `rt_ipc_release()`
- Does **not** block waiting for pool availability (fails immediately if pool exhausted)
- Sender blocks after successful mailbox insertion (not during pool allocation)

**Design rationale:**
- Explicit failure (`RT_ERR_NOMEM`) enables testable, predictable behavior
- No implicit message drops (principle of least surprise)
- No blocking on pool exhaustion (prevents deadlock)
- Caller controls backpressure via retry logic

See "Pool Exhaustion Behavior" section below for mitigation strategies and examples.

### Message Ordering

**Mailbox implementation:** FIFO linked list (enqueue at tail, dequeue from head)

**Ordering guarantees:**

**Single sender to single receiver:**
- Messages are received in the order they were sent
- **FIFO guaranteed**
- Example: If actor A sends M1, M2, M3 to actor B, B receives them in order M1 -> M2 -> M3

**Multiple senders to single receiver:**
- Message order depends on scheduling (which sender runs first)
- **Arrival order is scheduling-dependent**
- No fairness guarantee (one sender can monopolize if scheduled more often)
- Example: If actor A sends M1 and actor B sends M2, receiver may get M1->M2 or M2->M1 depending on scheduler

**Timer messages interleaving with IPC:**
- Timer messages use the **same mailbox** as regular IPC messages
- Timer messages are enqueued to tail when the event loop processes timer events (epoll returns timerfd ready)
- **No bypass, no special priority** - timers follow FIFO with other messages
- Interleave based on when the event loop dispatches timer events relative to other sends
- Example: If actor receives IPC message M1, then timer fires, then IPC message M2, mailbox order is M1 -> timer_tick -> M2

**System messages (actor death notifications):**
- Exit messages (sender == RT_SENDER_SYSTEM) also use the same mailbox
- Follow FIFO ordering like all other messages
- No special delivery priority

**Consequences:**

```c
// Single sender - FIFO guaranteed
void sender_actor(void *arg) {
    rt_ipc_send(receiver, &msg1, sizeof(msg1), IPC_ASYNC);  // Sent first
    rt_ipc_send(receiver, &msg2, sizeof(msg2), IPC_ASYNC);  // Sent second
    // Receiver will see: msg1, then msg2 (guaranteed)
}

// Multiple senders - order depends on scheduling
void sender_A(void *arg) {
    rt_ipc_send(receiver, &msgA, sizeof(msgA), IPC_ASYNC);
}
void sender_B(void *arg) {
    rt_ipc_send(receiver, &msgB, sizeof(msgB), IPC_ASYNC);
}
// Receiver may see: msgA then msgB, OR msgB then msgA (depends on scheduler)

// Timer + IPC interleaving
void actor_with_timer(void *arg) {
    timer_id timer;
    rt_timer_every(100000, &timer);  // 100ms periodic

    rt_message msg;
    while (1) {
        rt_ipc_recv(&msg, -1);

        if (rt_timer_is_tick(&msg)) {
            // Timer tick received in FIFO order with other messages
        } else {
            // Regular IPC message
        }
        // No guarantee which arrives first - depends on timing
    }
}
```

**Design rationale:**
- Single FIFO mailbox = simple, predictable within single sender
- Scheduling-dependent ordering across senders = unavoidable in cooperative scheduler
- No message priorities = simpler implementation, deterministic behavior

### IPC_SYNC Safety Considerations

**WARNING: IPC_SYNC requires careful use.** It trades simplicity and performance for strict constraints.

**Design rationale:**
- Separate sync buffer pool (RT_SYNC_BUFFER_POOL_SIZE) isolates ASYNC/SYNC resource contention
- Pinned runtime buffers for memory safety (prevents UAF if sender dies)
- Blocking provides implicit backpressure (sender cannot proceed until receiver consumes)
- Static pool allocation for deterministic memory
- Simple implementation suitable for embedded/safety-critical systems

**Mandatory preconditions:**

1. **Actor context only**: SYNC can ONLY be used from actor context
   - OK: From actor's main function
   - FORBIDDEN: From event loop callbacks (timer/network event handlers)
   - FORBIDDEN: From interrupt contexts (signal handlers, ISRs)

2. **Sender blocks until release**: Sender cannot process other messages while waiting
   - Sender's state = ACTOR_STATE_BLOCKED
   - Sender cannot receive messages
   - Sender cannot send other SYNC messages

3. **Data copied to pinned buffer**: Sender's data is copied once to a runtime buffer
   - Buffer persists even if sender dies (memory safe)
   - Pool size: RT_SYNC_BUFFER_POOL_SIZE (64 default)
   - Exhaustion: Returns RT_ERR_NOMEM (caller must retry or use IPC_ASYNC)

**Deadlock scenarios (avoid these):**

```c
// DEADLOCK: Circular synchronous send
Actor A: rt_ipc_send(B, &data, len, IPC_SYNC);  // A blocks
Actor B: rt_ipc_send(A, &data, len, IPC_SYNC);  // B blocks -> DEADLOCK

// DEADLOCK: Nested synchronous send
Actor A: rt_ipc_send(B, &data, len, IPC_SYNC);  // A blocks
         // A cannot receive release notification!

// DEADLOCK: Sync send then receive
Actor A: rt_ipc_send(B, &data, len, IPC_SYNC);
         rt_ipc_recv(&msg, -1);  // Can't receive - already blocked!

// CORRECT: Sync send and wait for completion
Actor A: rt_ipc_send(B, &data, len, IPC_SYNC);  // Blocks until B releases
         // Automatically unblocks when B calls rt_ipc_release()
```

**When to use each mode:**

| Scenario | Use Mode | Reason |
|----------|----------|--------|
| Small messages (< 256 bytes) | IPC_ASYNC | Simple, no blocking |
| Fire-and-forget messaging | IPC_ASYNC | Sender doesn't wait |
| Untrusted receivers | IPC_ASYNC | Sender not vulnerable to deadlock |
| General communication | IPC_ASYNC | Safest default |
| Implicit backpressure needed | IPC_SYNC | Sender blocks until consumed |
| Trusted cooperating actors | IPC_SYNC | Both sides understand protocol |
| Flow control required | IPC_SYNC | Natural rate limiting via blocking |

**Failure handling:**

If receiver crashes or exits without releasing:
- Sender is automatically unblocked during receiver's actor cleanup
- `rt_ipc_send()` returns `RT_ERR_CLOSED` (receiver death is not success)
- If sender is unblocked because receiver released normally: returns `RT_SUCCESS`
- Message data is no longer referenced by receiver
- Principle of least surprise: sender is not stuck forever
- **Important:** Sender does NOT receive notification of receiver death (unless explicitly linked/monitoring)

**Semantic note:** `RT_SUCCESS` means receiver explicitly released (via `rt_ipc_release()` or auto-release), but does NOT guarantee the receiver processed the message successfully. `RT_ERR_CLOSED` means receiver died before releasing. If the sender requires confirmation that the message was processed correctly, it must use an explicit acknowledgment protocol (e.g., receiver sends reply after processing).

Best practice: Design actors to always release sync messages, but receiver crashes are handled gracefully.

**Edge cases (explicit semantics):**

1. **Calling `rt_ipc_recv()` again without releasing previous SYNC:**
   - **Legal:** Auto-release semantics apply
   - Behavior: Previous SYNC sender is automatically unblocked
   - Implementation: Auto-release logic in `rt_ipc_recv()`
   - Rationale: Prevents deadlock if receiver forgets to release
   - Example:
   ```c
   rt_message msg1, msg2;
   rt_ipc_recv(&msg1, -1);  // msg1 is SYNC from actor A
   // ... process msg1.data ...
   rt_ipc_recv(&msg2, -1);  // msg1 AUTO-RELEASED, actor A unblocked
   // msg1.data now invalid, msg2.data valid
   ```

2. **Receiver death while holding sync message:**
   - **Auto-release:** Sender is automatically unblocked during receiver's actor cleanup
   - Sender's `rt_ipc_send()` returns `RT_ERR_CLOSED` (receiver died, not normal release)
   - Message data no longer referenced by dead receiver
   - **Sender CAN distinguish** receiver crash (RT_ERR_CLOSED) from normal release (RT_SUCCESS)
   - Rationale: Honest failure reporting - don't lie to caller about receiver death
   - Consistent with file/net I/O semantics (connection closed = error)

3. **Sender death while blocked in SYNC:**
   - **Memory safety:** Data in pinned runtime buffer, NOT sender's stack
   - **Receiver sees:** Normal message with `msg.data` pointing to valid runtime buffer
   - **Data validity:** VALID - buffer persists even after sender dies
   - **Receiver behavior:** Can safely read `msg.data` as normal
   - **Cleanup:** Buffer freed when receiver calls `rt_ipc_release()` or dies
   - **Rationale:** Eliminates use-after-free vulnerability for safety-critical systems
   - **Note:** This is a key difference from stack-based approach (no UAF possible)

4. **Self-send with SYNC (`rt_ipc_send(rt_self(), ..., IPC_SYNC)`):**
   - **Forbidden:** Immediate deadlock
   - **Behavior:** Sender enqueues message to own mailbox, then blocks waiting for self to release
   - **Result:** Actor blocks forever (cannot receive while blocked)
   - **Detection:** Implementation returns `RT_ERR_INVALID` for self-send with SYNC
   - **Enforcement:** Self-send detection in `rt_ipc_send()` for SYNC mode
   - **Rationale:** Prevent guaranteed deadlock
   - Example:
   ```c
   // DEADLOCK: Self-send with SYNC
   rt_status status = rt_ipc_send(rt_self(), &data, len, IPC_SYNC);
   // Returns RT_ERR_INVALID, prevents deadlock
   assert(status.code == RT_ERR_INVALID);

   // OK: Self-send with ASYNC
   rt_ipc_send(rt_self(), &data, len, IPC_ASYNC);  // Works fine
   ```

**Lifetime rule (strict definition):**

For `IPC_SYNC`, `rt_message.data` is valid:
- **From:** When `rt_ipc_recv()` returns the message
- **Until:** EARLIEST of:
  1. `rt_ipc_release(&msg)` called explicitly, OR
  2. Next `rt_ipc_recv()` called (auto-release), OR
  3. Receiver actor dies (cleanup frees buffer)
- **After:** Data pointer is INVALID, use triggers undefined behavior
- **Note:** Sender death does NOT invalidate data (pinned buffer persists)

**Safety-critical recommendation:**

In safety-critical systems:
- **IPC_ASYNC** remains the safest default (no blocking, simpler semantics)
- **IPC_SYNC** is now memory-safe (pinned buffers eliminate UAF)
- Use SYNC when:
  - Implicit backpressure is desired (flow control)
  - Between trusted, cooperating actors (deadlock awareness)
  - Validated communication patterns
- Document all SYNC usage in code reviews (deadlock risk remains)
- Test deadlock scenarios explicitly

### Functions

```c
// Send message to actor
// Blocks if mode == IPC_SYNC (until receiver consumes)
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);

// Receive message
// timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded
rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms);

// Release sync message (must call after consuming IPC_SYNC message)
void rt_ipc_release(const rt_message *msg);

// Query mailbox state
bool rt_ipc_pending(void);
size_t rt_ipc_count(void);
```

### Pool Exhaustion Behavior

IPC uses global pools shared by all actors:
- **Mailbox entry pool**: `RT_MAILBOX_ENTRY_POOL_SIZE` (256 default, shared by ASYNC and SYNC)
- **Message data pool**: `RT_MESSAGE_DATA_POOL_SIZE` (256 default, ASYNC mode only)
- **Sync buffer pool**: `RT_SYNC_BUFFER_POOL_SIZE` (64 default, SYNC mode only)

**When pools are exhausted:**
- `rt_ipc_send()` returns `RT_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure
- Both IPC_ASYNC and IPC_SYNC share the mailbox entry pool
- IPC_ASYNC additionally requires message data pool space

**No per-actor mailbox limit**: All actors share the global pools. A single actor can consume all available mailbox entries if receivers don't process messages.

**Mitigation strategies:**
- Size pools appropriately: `1.5× peak concurrent messages`
- Check return values and implement retry logic or backpressure
- Use IPC_SYNC for large messages to avoid data pool exhaustion
- Ensure receivers process messages promptly

**Backoff-retry example:**
```c
rt_status status = rt_ipc_send(target, data, len, IPC_ASYNC);
if (status.code == RT_ERR_NOMEM) {
    // Pool exhausted - backoff before retry
    rt_message msg;
    status = rt_ipc_recv(&msg, 10);  // Backoff 10ms

    if (status.code == RT_ERR_TIMEOUT) {
        // No messages during backoff, retry send
        rt_ipc_send(target, data, len, IPC_ASYNC);
    } else if (!RT_FAILED(status)) {
        // Got message during backoff, handle it first
        handle_message(&msg);
        // Then retry send
    }
}
```

## Bus API

Publish-subscribe communication with configurable retention policy.

### Configuration

```c
typedef struct {
    uint8_t  max_subscribers; // max concurrent subscribers (1..RT_MAX_BUS_SUBSCRIBERS)
    uint8_t  max_readers;     // consume after N reads, 0 = unlimited (0..max_subscribers)
    uint32_t max_age_ms;      // expire entries after ms, 0 = no expiry
    size_t   max_entries;     // ring buffer capacity
    size_t   max_entry_size;  // max payload bytes per entry
} rt_bus_config;
```

**Configuration constraints (normative):**
- `max_subscribers`: **Architectural limit of 32 subscribers** (enforced by 32-bit `readers_mask`)
  - Valid range: 1..32
  - `RT_MAX_BUS_SUBSCRIBERS = 32` is a hard architectural invariant, not a tunable parameter
  - Attempts to configure `max_subscribers > 32` return `RT_ERR_INVALID`
- `max_readers`: Valid range: 0..max_subscribers
- Subscriber index assignment is stable for the lifetime of a subscription (affects `readers_mask` bit position)

### Functions

```c
// Create bus
rt_status rt_bus_create(const rt_bus_config *cfg, bus_id *out);

// Destroy bus (fails if subscribers exist)
rt_status rt_bus_destroy(bus_id bus);

// Publish data
rt_status rt_bus_publish(bus_id bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
rt_status rt_bus_subscribe(bus_id bus);
rt_status rt_bus_unsubscribe(bus_id bus);

// Read entry (non-blocking)
// Returns RT_ERR_WOULDBLOCK if no data available
rt_status rt_bus_read(bus_id bus, void *buf, size_t max_len, size_t *actual_len);

// Read with blocking
rt_status rt_bus_read_wait(bus_id bus, void *buf, size_t max_len,
                           size_t *actual_len, int32_t timeout_ms);

// Query bus state
size_t rt_bus_entry_count(bus_id bus);
```

### Bus Consumption Model (Semantic Contract)

The bus implements **per-subscriber read cursors** with the following **three contractual rules**:

---

#### **RULE 1: Subscription Start Position**

**Contract:** `rt_bus_subscribe()` initializes the subscriber's read cursor to **"next publish"** (current `bus->head` write position).

**Guaranteed semantics:**
- Subscriber **CANNOT** read retained entries published before subscription
- Subscriber **ONLY** sees entries published **after** `rt_bus_subscribe()` returns
- First `rt_bus_read()` call returns `RT_ERR_WOULDBLOCK` if no new entries published since subscription
- Implementation: `subscriber.next_read_idx = bus->head`

**Implications:**
- New subscribers do NOT see history
- If you need to read retained entries, subscribe **before** publishing starts
- Late subscribers will miss all prior messages

**Example:**
```c
// Bus has retained entries [E1, E2, E3] with head=3
rt_bus_subscribe(bus);
//   -> subscriber.next_read_idx = 3 (next write position)

rt_bus_read(bus, buf, len, &actual);
//   -> Returns RT_ERR_WOULDBLOCK (no new data)
//   -> E1, E2, E3 are invisible (behind cursor)

// Publisher publishes E4 (head advances to 4)
rt_bus_read(bus, buf, len, &actual);
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
   - When `rt_bus_publish()` finds buffer full (`count >= max_entries`):
     - Oldest entry at `bus->tail` is **evicted immediately** (freed from message pool)
     - Tail advances: `bus->tail = (bus->tail + 1) % max_entries`
     - **No check if subscribers have read the evicted entry**
   - If slow subscriber's cursor pointed to evicted entry:
     - On next `rt_bus_read()`, search starts from **current `bus->tail`** (oldest surviving entry)
     - Subscriber **silently skips** to next available unread entry
     - **No error** returned (appears as normal read)
     - **No signal** of data loss (by design)

**Implications:**
- Slow subscribers lose data without notification
- Fast subscribers never lose data (assuming buffer sized for publish rate)
- No backpressure mechanism (unlike IPC_SYNC)
- Real-time principle: Prefer fresh data over old data

**Example (data loss):**
```c
// Bus: max_entries=3, entries=[E1, E2, E3] (full), tail=0, head=0
// Fast subscriber: next_read_idx=0 (read all, awaiting E4)
// Slow subscriber: next_read_idx=0 (still at E1, hasn't read any)

rt_bus_publish(bus, &E4, sizeof(E4));
//   -> Buffer full: Evict E1 at tail=0, free from pool
//   -> Write E4 at index 0: entries=[E4, E2, E3]
//   -> tail=1 (E2 is now oldest), head=1 (next write)

// Slow subscriber calls rt_bus_read():
//   -> Search from tail=1: E2 (unread), E3 (unread), E4 (unread)
//   -> Returns E2 (first unread)
//   -> E1 is LOST (no error, silent skip)
```

**Eviction does NOT notify slow subscribers:**
- No `RT_ERR_OVERFLOW` or similar
- No special message indicating data loss
- Application must detect via message sequence numbers if needed

---

#### **RULE 3: max_readers Counting Semantics**

**Contract:** `max_readers` counts **unique subscribers** who have read an entry, **NOT** total reads.

**Guaranteed semantics:**
- Each entry has a `readers_mask` bitmask (32 bits, max 32 subscribers per bus)
- When subscriber N reads an entry:
  1. Check if bit N is set in `readers_mask` -> if yes, skip (already read)
  2. If no, set bit N, increment `read_count`, return entry
- Entry is removed when `read_count >= max_readers` (N unique subscribers have read)
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

// Remove if max_readers reached
if (config.max_readers > 0 && entry->read_count >= config.max_readers) {
    // Free entry from pool, invalidate
}
```

**Implications:**
- `max_readers=3` means "remove after 3 **different** subscribers read it"
- If subscriber A reads entry twice (e.g., re-subscribes), that's still 1 read
- If 3 subscribers each read entry once, that's 3 reads -> entry removed
- Set `max_readers=0` to disable (entry persists until aged out or evicted)

**Example (unique counting):**
```c
// Bus with max_readers=2
// Subscribers: A, B, C

rt_bus_publish(bus, &E1, sizeof(E1));
//   -> E1: readers_mask=0b000, read_count=0

// Subscriber A reads E1
rt_bus_read(bus, ...);
//   -> E1: readers_mask=0b001, read_count=1 (A's bit set)

// Subscriber A reads again (tries to read E1)
rt_bus_read(bus, ...);
//   -> E1 skipped (bit already set), returns RT_ERR_WOULDBLOCK
//   -> E1: readers_mask=0b001, read_count=1 (unchanged)

// Subscriber B reads E1
rt_bus_read(bus, ...);
//   -> E1: readers_mask=0b011, read_count=2 (B's bit set)
//   -> read_count >= max_readers (2) -> E1 REMOVED, freed from pool
```

---

### Summary: The Three Rules

| Rule | Contract |
|------|----------|
| **1. Subscription start position** | New subscribers start at "next publish" (cannot read history) |
| **2. Cursor storage & eviction** | Per-subscriber cursors; slow readers may miss entries on wraparound (no notification) |
| **3. max_readers counting** | Counts UNIQUE subscribers (deduplication), not total reads |

---

### Retention Policy Configuration

Entries can be removed by **three mechanisms** (whichever occurs first):

1. **max_readers (unique subscriber counting):**
   - Entry removed when `read_count >= max_readers` (N unique subscribers have read it)
   - Value `0` = disabled (entry persists until aged out or evicted)
   - See **RULE 3** above for exact counting semantics

2. **max_age_ms (time-based expiry):**
   - Entry removed when `(current_time_ms - entry.timestamp_ms) >= max_age_ms`
   - Value `0` = disabled (no time-based expiry)
   - Checked on every `rt_bus_read()` and `rt_bus_publish()` call

3. **Buffer full (forced eviction):**
   - Oldest entry at `bus->tail` evicted when `count >= max_entries` on publish
   - ALWAYS enabled (cannot be disabled)
   - See **RULE 2** above for eviction semantics

**Typical configurations:**

| Use Case | max_readers | max_age_ms | Behavior |
|----------|-------------|------------|----------|
| **Sensor data** | `0` | `100` | Multiple readers, data stale after 100ms |
| **Configuration** | `0` | `0` | Persistent until buffer wraps |
| **Events** | `N` | `0` | Consumed after N subscribers read, no timeout |
| **Recent history** | `0` | `5000` | Keep 5 seconds of history, multiple readers |

**Interaction of mechanisms:**
- Entry is removed when **FIRST** condition is met (OR, not AND)
- Example: `max_readers=3, max_age_ms=1000`
  - Entry removed after 3 subscribers read it, **OR**
  - Entry removed after 1 second, **OR**
  - Entry removed when buffer full (forced eviction)

### Pool Exhaustion and Buffer Full Behavior

**WARNING: Resource Contention Between IPC and Bus**

Bus publishing consumes the same message data pool as IPC (`RT_MESSAGE_DATA_POOL_SIZE`). A misconfigured or high-rate bus can exhaust the message pool and cause **all** IPC_ASYNC sends to fail with `RT_ERR_NOMEM`, potentially starving critical actor communication.

**Architectural consequences:**
- Bus auto-evicts oldest entries when its ring buffer fills (graceful degradation)
- IPC never auto-drops (fails immediately with `RT_ERR_NOMEM`)
- A single high-rate bus publisher can starve IPC globally
- No per-subsystem quotas or fairness guarantees

**Design implications:**
- Size `RT_MESSAGE_DATA_POOL_SIZE` for combined IPC + bus peak load
- Use bus retention policies to limit memory consumption
- Monitor pool exhaustion in critical systems
- Consider separate message pools if isolation is required (requires code modification)

---

The bus can encounter two types of resource limits:

**1. Message Pool Exhaustion** (shared with IPC):
- Bus uses the global `RT_MESSAGE_DATA_POOL_SIZE` pool (same as IPC)
- When pool is exhausted, `rt_bus_publish()` returns `RT_ERR_NOMEM` immediately
- Does NOT block waiting for space
- Does NOT drop messages automatically in this case
- Caller must check return value and handle failure

**2. Bus Ring Buffer Full** (per-bus limit):
- Each bus has its own ring buffer sized via `max_entries` config
- When ring buffer is full, `rt_bus_publish()` **automatically evicts oldest entry**
- This is different from IPC - bus has automatic message dropping
- Publish succeeds (unless message pool also exhausted)
- Slow readers may miss messages if buffer wraps

**3. Subscriber Table Full**:
- Each bus has subscriber limit via `max_subscribers` config (up to `RT_MAX_BUS_SUBSCRIBERS`)
- When full, `rt_bus_subscribe()` returns `RT_ERR_NOMEM`

**Key Differences from IPC:**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`RT_MESSAGE_DATA_POOL_SIZE`)

**Mitigation strategies:**
- Size message pool appropriately for combined IPC + bus load
- Configure per-bus `max_entries` based on publish rate vs read rate
- Use retention policies (`max_readers`, `max_age_ms`) to prevent accumulation
- Monitor `rt_bus_entry_count()` to detect slow readers

## Timer API

Timers for periodic and one-shot wake-ups.

```c
// One-shot: wake current actor after delay
rt_status rt_timer_after(uint32_t delay_us, timer_id *out);

// Periodic: wake current actor every interval
rt_status rt_timer_every(uint32_t interval_us, timer_id *out);

// Cancel timer
rt_status rt_timer_cancel(timer_id id);

// Check if message is from timer
bool rt_timer_is_tick(const rt_message *msg);
```

Timer wake-ups are delivered as messages with `sender == RT_SENDER_TIMER`. The actor receives these in its normal `rt_ipc_recv()` loop.

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
   - Potential collision: If timer ID wraps and old timer still active, `rt_timer_cancel()` may cancel wrong timer
   - **Likelihood**: Extremely rare (requires 4 billion timer creations without runtime restart)
   - **Mitigation**: None needed in practice; runtime typically restarts long before wraparound

**Example: Maximum timer interval**

```c
// Maximum safe one-shot timer: ~71 minutes
uint32_t max_interval = UINT32_MAX;  // 4,294,967,295 us
rt_timer_after(max_interval, &timer);  // OK, fires after ~71.6 minutes

// For longer intervals, use periodic timer with counter
uint32_t seventy_two_min_us = 72 * 60 * 1000000;  // 4.32 billion us
// ERROR: Exceeds UINT32_MAX (4.29 billion), wraps to ~25 seconds

// Correct approach for long intervals:
uint32_t tick_interval = 60 * 1000000;  // 1 minute
rt_timer_every(tick_interval, &timer);
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
rt_status rt_net_listen(uint16_t port, int *fd_out);
rt_status rt_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms);
rt_status rt_net_connect(const char *host, uint16_t port, int *fd_out, int32_t timeout_ms);
rt_status rt_net_close(int fd);

// Data transfer
rt_status rt_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms);
rt_status rt_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms);
```

All functions with `timeout_ms` parameter support **timeout enforcement**:

- `timeout_ms == 0`: Non-blocking, returns `RT_ERR_WOULDBLOCK` if would block
- `timeout_ms < 0`: Block forever until I/O completes
- `timeout_ms > 0`: Block up to timeout, returns `RT_ERR_TIMEOUT` if exceeded

**Timeout implementation:** Uses timer-based enforcement (consistent with `rt_ipc_recv`). When timeout expires, a timer message wakes the actor and `RT_ERR_TIMEOUT` is returned. This is essential for handling unreachable hosts, slow connections, and implementing application-level keepalives.

On blocking calls, the actor yields to the scheduler. The scheduler's event loop registers the I/O operation with the platform's event notification mechanism (epoll on Linux, interrupt flags on STM32) and dispatches the operation when the socket becomes ready.

## File API

File I/O operations.

```c
rt_status rt_file_open(const char *path, int flags, int mode, int *fd_out);
rt_status rt_file_close(int fd);

rt_status rt_file_read(int fd, void *buf, size_t len, size_t *actual);
rt_status rt_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *actual);

rt_status rt_file_write(int fd, const void *buf, size_t len, size_t *actual);
rt_status rt_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *actual);

rt_status rt_file_sync(int fd);
```

The `mode` parameter in `rt_file_open()` specifies file permissions (e.g., 0644) when creating files with `O_CREAT` flag, following POSIX conventions.

**File operations block the calling actor until I/O completes.** On embedded systems with local filesystems (FATFS, littlefs), file operations complete quickly (microseconds to milliseconds) and are bounded by hardware characteristics. Timeouts are not provided as hardware failures (dead SD card, flash corruption) cannot be recovered via timeout and require physical intervention.

## Memory Allocation Architecture

The runtime uses **compile-time configuration** for deterministic memory allocation.

### Compile-Time Configuration (`rt_static_config.h`)

All resource limits are defined at compile-time and require recompilation to change:

```c
#define RT_MAX_ACTORS 64                    // Maximum concurrent actors
#define RT_STACK_ARENA_SIZE (1*1024*1024)   // Stack arena size (1 MB)
#define RT_MAX_BUSES 32                     // Maximum concurrent buses
#define RT_MAILBOX_ENTRY_POOL_SIZE 256      // Mailbox entry pool
#define RT_MESSAGE_DATA_POOL_SIZE 256       // Message data pool
#define RT_MAX_MESSAGE_SIZE 256             // Maximum message size
#define RT_LINK_ENTRY_POOL_SIZE 128         // Link entry pool
#define RT_MONITOR_ENTRY_POOL_SIZE 128      // Monitor entry pool
#define RT_TIMER_ENTRY_POOL_SIZE 64         // Timer entry pool
#define RT_DEFAULT_STACK_SIZE 65536         // Default actor stack size
```

All runtime structures are **statically allocated** based on these limits. Actor stacks use a static arena allocator by default (configurable via `actor_config.malloc_stack` for malloc). This ensures:
- Deterministic memory footprint (calculable at link time)
- Zero heap allocation in runtime operations (see Heap Usage Policy)
- O(1) pool allocation for hot paths (scheduling, IPC); O(n) bounded arena allocation for cold paths (spawn/exit)
- Suitable for embedded/MCU deployment

### Runtime API

```c
// Initialize runtime (call once from main)
rt_status rt_init(void);

// Run scheduler (blocks until all actors exit or rt_shutdown called)
void rt_run(void);

// Request graceful shutdown
void rt_shutdown(void);

// Cleanup runtime resources (call after rt_run completes)
void rt_cleanup(void);
```

## Actor Death Handling

When an actor dies (via `rt_exit()`, crash, or external kill):

**Exception:** The exception in this section applies **only when stack corruption prevents safe cleanup** (e.g., guard pattern so corrupted that detection itself is unsafe, or runtime metadata is damaged). When overflow **is detected** via guard checks (see Stack Overflow section), the **normal cleanup path below is used** — links/monitors are notified, mailbox cleared, timers cancelled, resources cleaned up. The guard pattern detection isolates the overflow to the stack area, leaving actor metadata intact for safe cleanup.

**Normal death cleanup:**

1. **Mailbox cleared:** All pending messages are discarded. Actors blocked on `IPC_SYNC` send receive `RT_ERR_CLOSED`.

2. **Links notified:** All linked actors receive an exit message with `sender == RT_SENDER_SYSTEM`.

3. **Monitors notified:** All monitoring actors receive an exit message.

4. **Bus subscriptions removed:** Actor is unsubscribed from all buses.

5. **Timers cancelled:** All timers owned by the actor are cancelled.

6. **Resources freed:** Stack and actor table entry are released.

### Exit Notification Ordering

**When exit notifications are sent:**

Exit notifications (steps 2-3 above) are enqueued in recipient mailboxes **during death processing**, following standard FIFO mailbox semantics.

**Ordering guarantees:**

1. **Messages already in recipient mailboxes:**
   - Exit notifications are enqueued at the **tail** of recipient mailboxes
   - Recipients will receive all messages sent before death **before** the exit notification
   - Example: If A sends M1, M2 to B, then A dies, B receives: M1 -> M2 -> EXIT(A)

2. **Messages sent by dying actor:**
   - Messages successfully enqueued before death remain in recipient mailboxes
   - These messages will be delivered **before** exit notifications (FIFO)
   - Example: A sends M1 to B, then A dies, B receives: M1 -> EXIT(A)

3. **Messages in dying actor's mailbox:**
   - Dying actor's mailbox is **cleared** (step 1)
   - All pending messages are **discarded**
   - Senders are **not** notified of message loss
   - Exception: IPC_SYNC senders are unblocked (see IPC_SYNC safety section)

4. **Multiple recipients:**
   - Each recipient's exit notification is enqueued independently
   - No ordering guarantee across different recipients
   - Example: A linked to B and C, dies. B and C both receive EXIT(A), but no guarantee which processes it first.

**Consequences:**

```c
// Dying actor sends messages before death
void actor_A(void *arg) {
    rt_ipc_send(B, &msg1, sizeof(msg1), IPC_ASYNC);  // Enqueued in B's mailbox
    rt_ipc_send(C, &msg2, sizeof(msg2), IPC_ASYNC);  // Enqueued in C's mailbox
    rt_exit();  // Exit notifications sent to links/monitors
}
// Linked actor B will receive: msg1, then EXIT(A)
// Actor C will receive: msg2 (no exit notification, not linked)

// Messages sent TO dying actor are lost
void actor_B(void *arg) {
    rt_ipc_send(A, &msg, sizeof(msg), IPC_ASYNC);  // Enqueued in A's mailbox
}
void actor_A(void *arg) {
    // ... does some work ...
    rt_exit();  // Mailbox cleared, msg from B is discarded
}
```

**Design rationale:**
- FIFO enqueuing = predictable, testable behavior
- Messages before death delivered before exit = "happens-before" relationship
- Mailbox clearing = simple cleanup, no orphaned messages
- No sender notification = simpler implementation, sender must handle recipient death via links/monitors

**Comparison to Erlang:**

Erlang provides stronger guarantees (signals are ordered with messages). This runtime provides simpler semantics: exit notifications follow standard FIFO mailbox ordering, enqueued at tail when death is processed.

## Scheduler Main Loop

Pseudocode for the scheduler:

```
procedure rt_run():
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
            events = epoll_wait(epoll_fd, timeout=-1)

            # 4. Dispatch I/O events
            for event in events:
                source = event.data.ptr
                if source.type == TIMER:
                    send_timer_tick(source.owner)
                    wake_actor(source.owner)
                elif source.type == NETWORK:
                    perform_io_operation(source)
                    wake_actor(source.owner)
```

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
    int n = epoll_wait(epoll_fd, events, 64, -1);
    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;
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

**Spurious wakeup handling:**
- epoll_wait may return 0 events (rare)
- Scheduler handles correctly, immediately re-enters wait
- No correctness impact

**Lost event prevention:**
- Timerfds are level-triggered (fire until read)
- Sockets are level-triggered by default (readable until read)
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
| Timer | timerfd + epoll | Hardware timers (SysTick/TIM) |
| Network | Non-blocking BSD sockets + epoll | lwIP NO_SYS mode |
| File | Synchronous POSIX | Synchronous FATFS or littlefs |

## Stack Overflow

### Behavior Contract

**Detection-dependent behavior:** Stack overflow semantics depend on whether the overflow is detected by guard pattern checks.

---

#### **When Overflow IS Detected (Guard Check Succeeds)**

**Guaranteed behavior:**
1. **Actor terminates** with exit reason `RT_EXIT_CRASH_STACK`
2. **Links/monitors are notified** (receive exit message with `RT_EXIT_CRASH_STACK`)
3. **Runtime remains stable** (other actors continue running)
4. **Actor resources cleaned up** (stack freed, mailbox cleared, timers cancelled)
5. **Error logged:** "Actor N stack overflow detected"

**Why this is safe:**
- Guard patterns are stored **outside** the actor's usable stack region
- Actor metadata (struct actor, links, monitors, mailbox) is stored in **static global arrays**, not on the actor's stack
- Overflow corrupts the actor's stack data, not the runtime's bookkeeping structures
- Therefore: Cleanup and notification can proceed safely after detection

**Implementation:** Guard patterns checked on every context switch (rt_scheduler.c)

---

#### **When Overflow is NOT Detected (Guard Check Fails to Trigger)**

**Actual behavior: UNDEFINED**

If stack overflow is severe enough to corrupt guard patterns before the next context switch, or if corruption propagates beyond stack boundaries:

- **May segfault** (most likely on Linux with default stack allocation)
- **May corrupt other actors' stacks** (if stacks are adjacent in memory)
- **May corrupt runtime state** (if overflow is extreme)
- **May go undetected indefinitely** (if guards are overwritten with valid-looking data)

**Links/monitors are NOT guaranteed to be notified.** Runtime stability is NOT guaranteed.

**System-level mitigation required:**
- **Watchdog timer:** Detect hung system, trigger reboot/failsafe
- **Stack sizing discipline:** Size stacks with 2-3x safety margin based on profiling
- **Production hardware:** Use MPU guard pages (ARM Cortex-M) for hardware-guaranteed traps
- **Supervisor architecture:** Critical subsystems monitor less-trusted actors, reboot on anomaly

**This is NOT a runtime bug** - it is the inherent limitation of best-effort guard pattern detection. Safety-critical systems must not rely solely on overflow detection; they must architect for graceful degradation or system reset on memory corruption.

---

### Detection Mechanism (Best-Effort)

**Implementation:** Guard pattern detection (Linux/STM32)

**Mechanism:**
- 8-byte guard patterns placed at both ends of each actor stack
- Guards checked on every context switch
- Pattern: `0xDEADBEEFCAFEBABE` (uint64_t)
- Overhead: 16 bytes per stack (8 bytes × 2)

**Detection quality:**
- **Best-effort:** Catches most overflows during normal operation
- **Post-facto:** Detection occurs on next context switch after overflow
- **Not guaranteed:** Severe overflows may corrupt guards before check
- **Timing:** Overflow -> corruption -> next context switch -> detection

**Comparison with alternatives:**

| Detection Method | Overhead | Reliability | Timing |
|------------------|----------|-------------|--------|
| **Guard patterns (current)** | 16 bytes/stack | Best-effort (post-facto) | Next context switch |
| **MPU guard pages (future)** | 0 bytes | Hardware-guaranteed | Immediate trap |
| **Canaries at runtime** | Function call overhead | Function-level | At function return |
| **Static analysis** | Compile-time only | Depends on analysis | Before runtime |

**Future improvements:**
- **ARM Cortex-M:** MPU guard pages for zero-overhead hardware traps
- **Debug builds:** Stack usage watermarking and high-water tracking
- **Telemetry:** Stack usage statistics per actor

**Best practices:**
- Size stacks with 2-3x safety margin
- Test worst-case call depth (including interrupts on embedded)
- Use static analysis tools to verify stack usage
- Monitor stack overflow errors in production (should be rare)

## Future Considerations

Not in scope for first version, but noted for future:
