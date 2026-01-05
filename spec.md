# Actor Runtime Specification

## Overview

A minimalistic actor-based runtime designed for **embedded and safety-critical systems**. The runtime implements cooperative multitasking with priority-based scheduling and message passing, inspired by Erlang's actor model.

**Target use cases:** Drone autopilots, industrial sensor networks, robotics control systems, and other resource-constrained embedded applications requiring structured concurrency.

**Design principles:**

1. **Minimalistic**: Only essential features, no bloat
2. **Predictable**: Cooperative scheduling, no surprises
3. **Modern C11**: Clean, safe, standards-compliant code
4. **Static allocation**: Deterministic memory, zero fragmentation, compile-time footprint
5. **Pool-based allocation**: O(1) allocation for all runtime operations
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
- Network I/O (`rt_net_*()` functions, completion handling)
- File I/O (`rt_file_*()` functions, completion handling)
- Linking/monitoring (`rt_link()`, `rt_monitor()`, death notifications)
- All I/O completion processing

**Consequence**: All "hot path" operations use **static pools** with **O(1) allocation** and return `RT_ERR_NOMEM` on pool exhaustion. No malloc, no heap fragmentation, no allocation latency.

**Verification**: Run with `LD_PRELOAD` malloc wrapper to assert no malloc calls after `rt_init()` (except explicit `malloc_stack = true` spawns).

## Target Platforms

**Development:** Linux x86-64

**Production:** STM32 (ARM Cortex-M) with FreeRTOS

On FreeRTOS, the actor runtime runs as a single task. Blocking I/O operations are handled by separate FreeRTOS tasks that communicate with the runtime via lock-free queues.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Actor Runtime (cooperative)                │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐        │
│  │ Actor 1 │ │ Actor 2 │ │ Actor 3 │ │   ...   │        │
│  └────┬────┘ └────┬────┘ └────┬────┘ └────┬────┘        │
│       │           │           │           │             │
│       └───────────┴─────┬─────┴───────────┘             │
│                         │                               │
│              ┌──────────▼──────────┐                    │
│              │     Scheduler       │                    │
│              │  (run queues by     │                    │
│              │   priority, yield)  │                    │
│              └──────────┬──────────┘                    │
│                         │                               │
│    ┌────────────────────┼────────────────────┐          │
│    │                    │                    │          │
│    ▼                    ▼                    ▼          │
│ ┌──────┐           ┌──────────┐          ┌──────┐       │
│ │ IPC  │           │Completion│          │ Bus  │       │
│ │      │           │  Queues  │          │      │       │
│ └──────┘           └────┬─────┘          └──────┘       │
│                         │                               │
└─────────────────────────┼───────────────────────────────┘
         FreeRTOS task    │
         boundary         │
                          │
    ┌─────────────────────┼─────────────────────┐
    │                     │                     │
    ▼                     ▼                     ▼
┌────────┐          ┌──────────┐          ┌────────┐
│ Net IO │          │ File IO  │          │ Timer  │
│ Task   │          │  Task    │          │ Task   │
└────────┘          └──────────┘          └────────┘
```

## Scheduling

### Cooperative Model

Actors run until they explicitly yield control. The scheduler reschedules an actor only when:

- The actor calls a blocking I/O primitive (net, file, IPC receive)
- The actor explicitly yields via `rt_yield()`
- The actor exits via `rt_exit()`

There is no preemptive scheduling or time slicing within the actor runtime.

### Priority Levels

Four priority levels, lower value means higher priority:

| Level | Name | Typical Use |
|-------|------|-------------|
| 0 | `RT_PRIO_CRITICAL` | Flight control loop |
| 1 | `RT_PRIO_HIGH` | Sensor fusion |
| 2 | `RT_PRIO_NORMAL` | Telemetry |
| 3 | `RT_PRIO_LOW` | Logging |

The scheduler always picks the highest-priority runnable actor. Within the same priority level, actors are scheduled round-robin.

### Context Switching

Context switching is implemented via manual assembly for performance:

- **x86-64 (Linux):** Save/restore callee-saved registers (rbx, rbp, r12-r15) and stack pointer
- **ARM Cortex-M (STM32):** Save/restore callee-saved registers (r4-r11) and stack pointer

No use of setjmp/longjmp or ucontext for performance reasons.

## Thread Safety

### Thread/Task Boundary Contract

The runtime uses a **single-threaded ownership model** with strict thread boundaries. All runtime state is owned by the scheduler thread; I/O threads/tasks and external threads have **no direct access** to runtime internals.

**The following three rules define the thread safety contract:**

---

#### **BOUNDARY 1: Scheduler Thread (Exclusive Runtime Owner)**

**Contract:** **ONLY** the scheduler thread may mutate runtime state.

**Allowed operations (scheduler thread ONLY):**
- Mutate actor state (spawn, exit, state transitions)
- Mutate mailboxes (enqueue/dequeue messages, link/monitor notifications)
- Mutate bus state (publish, subscribe, read)
- Call **ANY** runtime API:
  - `rt_ipc_send()`, `rt_ipc_recv()`, `rt_ipc_release()`
  - `rt_spawn()`, `rt_spawn_ex()`, `rt_exit()`, `rt_yield()`
  - `rt_bus_create()`, `rt_bus_publish()`, `rt_bus_subscribe()`, `rt_bus_read()`
  - `rt_link()`, `rt_monitor()`, `rt_unlink()`, `rt_demonitor()`
  - `rt_timer_after()`, `rt_timer_every()`, `rt_timer_cancel()`
  - `rt_file_*()`, `rt_net_*()` (request submission only, blocking handled via SPSC)
- Process I/O completions from SPSC queues
- Access/modify pools (actor table, message pool, mailbox pool, timer pool, etc.)

**Forbidden operations:**
- None (scheduler thread has full access to all runtime state)

**Implementation:** All actor code runs in the scheduler thread. When an actor calls `rt_ipc_send()`, it executes in the scheduler thread context.

---

#### **BOUNDARY 2: I/O Threads/Tasks (Completion-Only)**

**Contract:** I/O threads/tasks may **ONLY** push completion entries to SPSC queues and signal scheduler wakeup. They **CANNOT** access runtime state.

**Allowed operations (I/O threads ONLY):**
1. **Push to SPSC completion queue:**
   - `rt_spsc_push(&g_timer.completion_queue, &completion)`
   - `rt_spsc_push(&g_file_io.completion_queue, &completion)`
   - `rt_spsc_push(&g_net_io.completion_queue, &completion)`

2. **Signal scheduler wakeup primitive:**
   - `rt_scheduler_wakeup_signal()` (posts to eventfd/semaphore)

3. **Access own I/O subsystem state:**
   - Timer thread: Access timer list (via mutex)
   - File thread: Process file I/O requests
   - Network thread: Process socket operations

**Forbidden operations (NEVER from I/O threads):**
- Call runtime APIs: `rt_ipc_send()`, `rt_spawn()`, `rt_bus_publish()`, etc. (**NOT THREAD-SAFE**)
- Access actor state directly (actor table, actor structs)
- Access mailboxes directly (enqueue/dequeue)
- Access bus state directly (ring buffers, subscriber lists)
- Access pools directly (message pool, mailbox pool) except via completion queue mechanism

**Rationale:** I/O threads exist solely to offload blocking operations (file I/O, network I/O, timer waits). They communicate results back to the scheduler via lock-free SPSC queues. The scheduler thread processes completions and updates runtime state accordingly.

**Implementation:**
- Timer worker thread: Waits on `timerfd` / `epoll`, pushes timer tick completions
- File I/O worker thread: Processes file read/write requests, pushes completions
- Network I/O worker thread: Processes socket operations, pushes completions

---

#### **BOUNDARY 3: External Threads (Forbidden)**

**Contract:** External threads **CANNOT** call runtime APIs. Communication with actors requires platform-specific mechanisms.

**Allowed operations:**
- None (no direct runtime API access)

**Forbidden operations (NEVER from external threads):**
- Call **ANY** runtime API:
  - `rt_ipc_send()` (**NOT THREAD-SAFE** - no locking/atomics implemented)
  - `rt_spawn()`, `rt_bus_publish()`, etc. (all forbidden)
- Access runtime state in any way

**Workaround for external thread communication:**
- Use platform-specific IPC mechanisms (sockets, pipes, shared memory queues)
- Have a dedicated **reader actor** that blocks on the external mechanism
- External thread writes to socket/pipe → reader actor receives via `rt_net_recv()` or `rt_file_read()` → actor forwards to other actors via `rt_ipc_send()`

**Design decision:** External threads are **NOT** supported for direct message passing. Adding thread-safe `rt_ipc_send()` would require:
- Mutex/atomic protection on mailbox enqueue (contention in hot path)
- Mutex/atomic protection on message pool allocation
- Loss of deterministic behavior (priority inversion, lock contention)
- Incompatible with safety-critical requirements

**Alternative for multi-producer scenarios:** Use network/file I/O with dedicated reader actors instead of direct API calls.

---

### Summary: Thread Boundary Rules

| Thread Type | Mutate Runtime State? | Call Runtime APIs? | Push SPSC? | Access Actor/Mailbox/Bus? |
|-------------|----------------------|-------------------|-----------|--------------------------|
| **Scheduler** | ✓ YES (exclusive owner) | ✓ YES (all APIs) | ✓ YES (pop completions) | ✓ YES (exclusive access) |
| **I/O threads** | ✗ NO | ✗ NO (forbidden) | ✓ YES (push completions) | ✗ NO (forbidden) |
| **External threads** | ✗ NO | ✗ NO (forbidden) | ✗ NO | ✗ NO (forbidden) |

### Synchronization Primitives

The runtime uses minimal synchronization:

- **SPSC queues:** Lock-free atomic head/tail for I/O completion queues
- **Scheduler wakeup:** Single eventfd (Linux) or semaphore (FreeRTOS) for I/O→scheduler signaling
- **Timer list:** Mutex-protected (accessed by both timer thread and scheduler thread for timer management)
- **Logging:** Mutex-protected (optional, not part of core runtime)

**No locks in hot paths:** Mailboxes, actor state, bus state, and message passing require no locks because they are accessed only by the scheduler thread.

### Rationale

This single-threaded model provides:

- **Simplicity:** No lock ordering, no deadlock, easier to reason about
- **Determinism:** No lock contention, predictable execution
- **Performance:** No lock overhead in message passing or scheduling
- **Safety-critical compliance:** Deterministic behavior, no priority inversion from locks

The cooperative scheduling model ensures actors yield explicitly, so there are no race conditions within the runtime itself.

### Consequences and Usage Patterns

**Valid patterns (allowed by boundary contracts):**

```c
// ✓ VALID: Actor calling runtime APIs (BOUNDARY 1: scheduler thread)
void my_actor(void *arg) {
    rt_ipc_send(other_actor, &data, sizeof(data), IPC_COPY);  // ✓ OK
    actor_id new_actor = rt_spawn(worker, NULL);               // ✓ OK
    rt_bus_publish(bus, &event, sizeof(event));               // ✓ OK
}

// ✓ VALID: I/O thread posting completion (BOUNDARY 2: completion-only)
void timer_worker_thread(void) {
    // Wait for timer expiry...
    timer_completion comp = {...};
    rt_spsc_push(&g_timer.completion_queue, &comp);  // ✓ OK (BOUNDARY 2 allows)
    rt_scheduler_wakeup_signal();                    // ✓ OK (BOUNDARY 2 allows)
}

// ✓ VALID: External thread → socket → reader actor (BOUNDARY 3 workaround)
// External thread:
void external_producer(void) {
    int sock = connect_to_actor_socket();
    write(sock, data, len);  // ✓ OK (platform-specific IPC)
}

// Reader actor:
void socket_reader_actor(void *arg) {
    int sock = listen_and_accept();
    while (1) {
        rt_net_recv(sock, buf, len, &received, -1);  // ✓ OK (scheduler thread)
        rt_ipc_send(worker, buf, received, IPC_COPY); // ✓ OK (scheduler thread)
    }
}
```

**Invalid patterns (violate boundary contracts):**

```c
// ✗ INVALID: External thread calling runtime API (violates BOUNDARY 3)
void external_thread(void) {
    rt_ipc_send(actor, &data, sizeof(data), IPC_COPY);  // ✗ FORBIDDEN
    // ERROR: No locking/atomics, NOT THREAD-SAFE, will corrupt mailbox!
}

// ✗ INVALID: I/O thread calling runtime API (violates BOUNDARY 2)
void file_worker_thread(void) {
    rt_spawn(actor, NULL);           // ✗ FORBIDDEN (violates BOUNDARY 2)
    rt_ipc_send(actor, &data, len);  // ✗ FORBIDDEN (violates BOUNDARY 2)
    // ERROR: I/O threads may ONLY push SPSC completions, not mutate runtime state!
}

// ✗ INVALID: I/O thread accessing actor state directly (violates BOUNDARY 2)
void net_worker_thread(void) {
    actor *a = rt_actor_get(id);     // ✗ FORBIDDEN (direct state access)
    a->state = ACTOR_STATE_READY;    // ✗ FORBIDDEN (mutation from wrong thread)
    // ERROR: Only scheduler thread may access/mutate actor state!
}
```

**Key takeaway:** If you have external threads that need to communicate with actors, use **platform-specific IPC** (sockets, pipes, shared memory) with a **dedicated reader actor** that calls `rt_net_recv()` or `rt_file_read()` to bridge the boundary.

## Memory Model

### Actor Stacks

Each actor has a fixed-size stack allocated at spawn time. Stack size is configurable per actor via `actor_config.stack_size`, with a system-wide default (`RT_DEFAULT_STACK_SIZE`). Different actors can use different stack sizes to optimize memory usage.

Stack growth/reallocation is not supported. Stack overflow is detected via guard patterns (see "Stack Overflow Detection" section below) with defined, safe behavior.

### Memory Allocation

The runtime uses static allocation for deterministic behavior and suitability for MCU deployment:

**Design Principle:** All memory is allocated at compile time or initialization. No heap allocation occurs during runtime operation (message passing, timer events, etc.).

**Allocation Strategy:**

- **Actor table:** Static array of `RT_MAX_ACTORS` (64), configured at compile time
- **Actor stacks:** Hybrid allocation (configurable per actor)
  - Default: Static arena allocator with `RT_STACK_ARENA_SIZE` (1 MB)
    - First-fit allocation with block splitting for variable stack sizes
    - Automatic memory reclamation and reuse when actors exit (coalescing)
    - Supports different stack sizes for different actors
  - Optional: malloc via `actor_config.malloc_stack = true`
- **IPC pools:** Static pools with O(1) allocation
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
- **Completion queues:** Static buffers of `RT_COMPLETION_QUEUE_SIZE` (64) for each I/O subsystem

**Memory Footprint (typical configuration):**
- Static data (BSS): ~231KB (measured with default configuration)
  - Actor table: 64 actors × ~200 bytes = 12.8 KB
  - Mailbox pool: 256 entries × ~40 bytes = 10.2 KB
  - Message pool: 256 entries × 256 bytes = 64 KB
  - Link/monitor pools: 256 entries × ~16 bytes = 4 KB
  - Timer pool: 64 entries × ~40 bytes = 2.5 KB
  - Bus tables: 32 buses × ~3 KB each = 96 KB
  - Completion queues: 3 subsystems × ~20 KB each = 60 KB
  - Note: Actual size includes alignment padding and internal structures
- Dynamic (heap): Variable actor stacks only
  - Example: 20 actors × 32KB average = 640 KB

**Total:** ~871 KB (known at compile time)

**Benefits:**

- Deterministic memory: Footprint calculable at link time
- Zero heap fragmentation: No malloc after initialization (except explicit `malloc_stack` flag)
- Predictable allocation: Pool exhaustion returns clear errors (`RT_ERR_NOMEM`)
- Suitable for safety-critical certification
- Predictable timing: O(1) pool allocation, no malloc latency in runtime operations

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
    const void *data;   // COPY: valid until next rt_ipc_recv()
                        // BORROW: valid until rt_ipc_release() (next recv auto-releases)
} rt_message;
```

### Send Modes

```c
typedef enum {
    IPC_COPY,    // memcpy payload to receiver's mailbox
    IPC_BORROW,  // zero-copy, sender blocks until receiver consumes
} rt_ipc_mode;
```

**IPC_COPY:** Payload is copied to receiver's mailbox. Sender continues immediately. Suitable for small messages and general-purpose communication.

**IPC_BORROW:** Zero-copy transfer. Payload remains on sender's stack. Sender blocks until receiver calls `rt_ipc_release()`. Provides implicit backpressure.

### API Contract: rt_ipc_send()

**Signature:**
```c
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);
```

**Behavior when pools are exhausted:**

`rt_ipc_send()` uses two global pools:
1. **Mailbox entry pool** (`RT_MAILBOX_ENTRY_POOL_SIZE` = 256) - for all IPC modes
2. **Message data pool** (`RT_MESSAGE_DATA_POOL_SIZE` = 256) - for IPC_COPY only

**Fail-fast semantics** (chosen for deterministic embedded behavior):

- **Returns `RT_ERR_NOMEM` immediately** if either required pool is exhausted
- **Does NOT block** waiting for pool space to become available
- **Does NOT drop** the message silently
- **Does NOT enqueue** the message (atomic operation: either succeeds completely or fails)

**Specific cases:**

| Mode | Mailbox Pool Full | Message Pool Full | Result |
|------|-------------------|-------------------|--------|
| **IPC_COPY** | Returns `RT_ERR_NOMEM` | Returns `RT_ERR_NOMEM` | Fail immediately |
| **IPC_BORROW** | Returns `RT_ERR_NOMEM` | N/A (no message pool used) | Fail immediately |

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
rt_ipc_send(target, &data, sizeof(data), IPC_COPY);  // WRONG

// Good: Check and handle pool exhaustion
rt_status status = rt_ipc_send(target, &data, sizeof(data), IPC_COPY);
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

**IPC_COPY:**
- Data is **valid until next `rt_ipc_recv()`**
- Data lives in a pool-allocated buffer
- Next recv frees the previous message's buffer and reuses the pool entry
- Calling `rt_ipc_release()` is optional (no-op for COPY)

**IPC_BORROW:**
- Data is **valid until `rt_ipc_release()`** is called
- Data lives on sender's stack
- Sender is blocked until `rt_ipc_release()` is called
- **Auto-release:** If `rt_ipc_recv()` is called without releasing the previous BORROW message, the sender is automatically unblocked (prevents deadlock)

**Auto-release behavior:**

```c
// Safe: explicit release (recommended)
rt_message msg;
rt_ipc_recv(&msg, -1);
if (msg.data) {
    process(msg.data);
}
rt_ipc_release(&msg);  // Unblocks sender (if BORROW)

// Also safe: auto-release on next recv (forgiving)
rt_message msg1, msg2;
rt_ipc_recv(&msg1, -1);  // msg1 is BORROW
process(msg1.data);
rt_ipc_recv(&msg2, -1);  // msg1 auto-released, sender unblocked
// No explicit release needed, but calling it is harmless
```

**Design rationale:**
- Auto-release prevents deadlock if receiver forgets to call `rt_ipc_release()`
- Forgiving API reduces cognitive load (one code path handles both COPY and BORROW)
- Explicit release is still recommended for clarity and to minimize sender blocking time

### Mailbox Semantics

**Capacity model:**

Per-actor mailbox limits: **None** - each actor's mailbox is unbounded (linked list)

Global pool limits: **Yes** - all actors share:
- `RT_MAILBOX_ENTRY_POOL_SIZE` (256 default) - mailbox entries
- `RT_MESSAGE_DATA_POOL_SIZE` (256 default) - message data (COPY mode only)

**Important:** One slow receiver can consume all mailbox entries, starving other actors.

**Behavior when pools are exhausted:**

| Mode        | Pool Exhausted Behavior | Blocks Waiting for Pool? | Drops Messages? |
|-------------|-------------------------|--------------------------|-----------------|
| **IPC_COPY**    | Returns `RT_ERR_NOMEM` immediately | No | No |
| **IPC_BORROW**  | Returns `RT_ERR_NOMEM` immediately | No | No |

**IPC_BORROW blocking semantics:**
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
- Example: If actor A sends M1, M2, M3 to actor B, B receives them in order M1 → M2 → M3

**Multiple senders to single receiver:**
- Message order depends on scheduling (which sender runs first)
- **Arrival order is scheduling-dependent**
- No fairness guarantee (one sender can monopolize if scheduled more often)
- Example: If actor A sends M1 and actor B sends M2, receiver may get M1→M2 or M2→M1 depending on scheduler

**Timer messages interleaving with IPC:**
- Timer messages use the **same mailbox** as regular IPC messages
- Timer messages are enqueued to tail when scheduler processes timer completions
- **No bypass, no special priority** - timers follow FIFO with other messages
- Interleave based on when `rt_timer_process_completions()` runs relative to other sends
- Example: If actor receives IPC message M1, then timer fires, then IPC message M2, mailbox order is M1 → timer_tick → M2

**System messages (actor death notifications):**
- Exit messages (sender == RT_SENDER_SYSTEM) also use the same mailbox
- Follow FIFO ordering like all other messages
- No special delivery priority

**Consequences:**

```c
// Single sender - FIFO guaranteed
void sender_actor(void *arg) {
    rt_ipc_send(receiver, &msg1, sizeof(msg1), IPC_COPY);  // Sent first
    rt_ipc_send(receiver, &msg2, sizeof(msg2), IPC_COPY);  // Sent second
    // Receiver will see: msg1, then msg2 (guaranteed)
}

// Multiple senders - order depends on scheduling
void sender_A(void *arg) {
    rt_ipc_send(receiver, &msgA, sizeof(msgA), IPC_COPY);
}
void sender_B(void *arg) {
    rt_ipc_send(receiver, &msgB, sizeof(msgB), IPC_COPY);
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

### IPC_BORROW Safety Considerations

**WARNING: IPC_BORROW requires careful use.** It trades simplicity and performance for strict constraints.

**Design rationale:**
- Zero-copy for performance-critical paths
- Stack-based for deterministic memory (no hidden allocations)
- Blocking provides implicit backpressure
- Simple implementation suitable for embedded/safety-critical systems

**Mandatory preconditions:**

1. **Actor context only**: BORROW can ONLY be used from actor context
   - OK: From actor's main function
   - FORBIDDEN: From I/O worker threads
   - FORBIDDEN: From completion handlers
   - FORBIDDEN: From interrupt contexts

2. **Data must be on sender's stack**: Borrowed pointer must remain valid while sender is blocked
   - OK: Stack-allocated variables
   - OK: Function parameters
   - FORBIDDEN: Heap-allocated data (use IPC_COPY instead)
   - FORBIDDEN: Static/global data (use IPC_COPY instead)

3. **Sender blocks until release**: Sender cannot process other messages while waiting
   - Sender's state = ACTOR_STATE_BLOCKED
   - Sender cannot receive messages
   - Sender cannot send other BORROW messages

**Deadlock scenarios (avoid these):**

```c
// DEADLOCK: Circular borrow
Actor A: rt_ipc_send(B, &data, len, IPC_BORROW);  // A blocks
Actor B: rt_ipc_send(A, &data, len, IPC_BORROW);  // B blocks -> DEADLOCK

// DEADLOCK: Nested borrow
Actor A: rt_ipc_send(B, &data, len, IPC_BORROW);  // A blocks
         // A cannot receive release notification!

// DEADLOCK: Borrow then receive
Actor A: rt_ipc_send(B, &data, len, IPC_BORROW);
         rt_ipc_recv(&msg, -1);  // Can't receive - already blocked!

// CORRECT: Borrow and wait for completion
Actor A: rt_ipc_send(B, &data, len, IPC_BORROW);  // Blocks until B releases
         // Automatically unblocks when B calls rt_ipc_release()
```

**When to use each mode:**

| Scenario | Use Mode | Reason |
|----------|----------|--------|
| Small messages (< 256 bytes) | IPC_COPY | Simple, no blocking |
| Fire-and-forget messaging | IPC_COPY | Sender doesn't wait |
| Untrusted receivers | IPC_COPY | Sender not vulnerable to deadlock |
| General communication | IPC_COPY | Safest default |
| Large data (> 1 KB) | IPC_BORROW | Avoid copy overhead |
| Performance-critical path | IPC_BORROW | Zero-copy, validated actors |
| Trusted cooperating actors | IPC_BORROW | Both sides understand protocol |
| Implicit backpressure needed | IPC_BORROW | Sender waits for consumer |

**Failure handling:**

If receiver crashes or exits without releasing:
- Sender is automatically unblocked during receiver's actor cleanup
- Sender's `rt_ipc_send()` returns normally (no error)
- Borrowed data is no longer referenced by receiver
- Principle of least surprise: sender is not stuck forever

Best practice: Design actors to always release borrowed messages, but receiver crashes are handled gracefully.

**Edge cases (explicit semantics):**

1. **Calling `rt_ipc_recv()` again without releasing previous BORROW:**
   - **Legal:** Auto-release semantics apply
   - Behavior: Previous BORROW sender is automatically unblocked
   - Implementation: `src/rt_ipc.c` lines 205-224 (auto-release in rt_ipc_recv)
   - Rationale: Prevents deadlock if receiver forgets to release
   - Example:
   ```c
   rt_message msg1, msg2;
   rt_ipc_recv(&msg1, -1);  // msg1 is BORROW from actor A
   // ... process msg1.data ...
   rt_ipc_recv(&msg2, -1);  // msg1 AUTO-RELEASED, actor A unblocked
   // msg1.data now invalid, msg2.data valid
   ```

2. **Receiver death while holding borrowed message:**
   - **Auto-release:** Sender is automatically unblocked during receiver's actor cleanup
   - Sender's `rt_ipc_send()` returns `RT_SUCCESS` (no error, unblocked normally)
   - Borrowed data no longer referenced by dead receiver
   - Sender does NOT receive error or notification (already returned from send)
   - Rationale: Principle of least surprise - sender not stuck forever
   - Tested: `tests/borrow_crash_test.c`

3. **Sender death while blocked in BORROW:**
   - **Receiver sees:** Normal message with `msg.data` pointing to dead sender's stack
   - **Data validity:** UNDEFINED - stack memory reclaimed on sender death
   - **Receiver behavior:**
     - If receiver reads before sender stack reused: May see valid data (lucky)
     - If receiver reads after sender stack reused: CORRUPTED DATA (use-after-free)
   - **Consequence:** Receiver MUST NOT use `msg.data` after sender dies
   - **Detection:** Check `msg.sender` with `rt_actor_get()` before use (returns NULL if dead)
   - **Mitigation:** Use IPC_COPY for untrusted senders, or validate sender liveness
   - Example:
   ```c
   rt_message msg;
   rt_ipc_recv(&msg, -1);

   // UNSAFE: Sender may have died
   process(msg.data);  // WRONG - could be corrupted

   // SAFE: Check sender still alive
   if (rt_actor_get(msg.sender) == NULL) {
       // Sender dead, data invalid
       rt_ipc_release(&msg);  // No-op, but harmless
       return;
   }
   process(msg.data);  // OK - sender still alive
   rt_ipc_release(&msg);
   ```

4. **Self-send with BORROW (`rt_ipc_send(rt_self(), ..., IPC_BORROW)`):**
   - **Forbidden:** Immediate deadlock
   - **Behavior:** Sender enqueues message to own mailbox, then blocks waiting for self to release
   - **Result:** Actor blocks forever (cannot receive while blocked)
   - **Detection:** Implementation returns `RT_ERR_INVALID` for self-send with BORROW
   - **Enforcement:** `src/rt_ipc.c` checks `to == sender->id` for BORROW mode
   - **Rationale:** Prevent guaranteed deadlock
   - Example:
   ```c
   // DEADLOCK: Self-send with BORROW
   rt_status status = rt_ipc_send(rt_self(), &data, len, IPC_BORROW);
   // Returns RT_ERR_INVALID, prevents deadlock
   assert(status.code == RT_ERR_INVALID);

   // OK: Self-send with COPY
   rt_ipc_send(rt_self(), &data, len, IPC_COPY);  // Works fine
   ```

**Lifetime rule (strict definition):**

For `IPC_BORROW`, `rt_message.data` is valid:
- **From:** When `rt_ipc_recv()` returns the message
- **Until:** EARLIEST of:
  1. `rt_ipc_release(&msg)` called explicitly, OR
  2. Next `rt_ipc_recv()` called (auto-release), OR
  3. Sender dies (data becomes UNDEFINED)
- **After:** Data pointer is INVALID, use triggers undefined behavior

**Safety-critical recommendation:**

In safety-critical systems:
- **Prefer IPC_COPY** for most communication
- Reserve IPC_BORROW for:
  - Validated, performance-critical transfers
  - Between trusted, cooperating actors
  - Where zero-copy benefit justifies the risk
- Document all BORROW usage in code reviews
- Test deadlock scenarios explicitly

**Debug mode (future):**

```c
#ifdef RT_DEBUG
// Verify borrowed pointer is within sender's stack bounds
if (mode == IPC_BORROW && !is_in_stack_range(sender, data, len)) {
    return RT_ERROR(RT_ERR_INVALID, "BORROW data not on sender's stack");
}
#endif
```

### Functions

```c
// Send message to actor
// Blocks if mode == IPC_BORROW (until receiver consumes)
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);

// Receive message
// timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded
rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms);

// Release borrowed message (must call after consuming IPC_BORROW message)
void rt_ipc_release(const rt_message *msg);

// Query mailbox state
bool rt_ipc_pending(void);
size_t rt_ipc_count(void);
```

### Pool Exhaustion Behavior

IPC uses global pools shared by all actors:
- **Mailbox entry pool**: `RT_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `RT_MESSAGE_DATA_POOL_SIZE` (256 default, COPY mode only)

**When pools are exhausted:**
- `rt_ipc_send()` returns `RT_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure
- Both IPC_COPY and IPC_BORROW share the mailbox entry pool
- IPC_COPY additionally requires message data pool space

**No per-actor mailbox limit**: All actors share the global pools. A single actor can consume all available mailbox entries if receivers don't process messages.

**Mitigation strategies:**
- Size pools appropriately: `1.5× peak concurrent messages`
- Check return values and implement retry logic or backpressure
- Use IPC_BORROW for large messages to avoid data pool exhaustion
- Ensure receivers process messages promptly

**Backoff-retry example:**
```c
rt_status status = rt_ipc_send(target, data, len, IPC_COPY);
if (status.code == RT_ERR_NOMEM) {
    // Pool exhausted - backoff before retry
    rt_message msg;
    status = rt_ipc_recv(&msg, 10);  // Backoff 10ms

    if (status.code == RT_ERR_TIMEOUT) {
        // No messages during backoff, retry send
        rt_ipc_send(target, data, len, IPC_COPY);
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
    uint8_t  max_readers;     // consume after N reads, 0 = unlimited
    uint32_t max_age_ms;      // expire entries after ms, 0 = no expiry
    size_t   max_entries;     // ring buffer capacity
    size_t   max_entry_size;  // max payload bytes per entry
} rt_bus_config;
```

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
//   → subscriber.next_read_idx = 3 (next write position)

rt_bus_read(bus, buf, len, &actual);
//   → Returns RT_ERR_WOULDBLOCK (no new data)
//   → E1, E2, E3 are invisible (behind cursor)

// Publisher publishes E4 (head advances to 4)
rt_bus_read(bus, buf, len, &actual);
//   → Returns E4 (first entry after subscription)
```

---

#### **RULE 2: Per-Subscriber Cursor Storage and Eviction Behavior**

**Contract:** Each subscriber has an independent read cursor; slow subscribers experience **SILENT DATA LOSS** on buffer wraparound.

**Guaranteed semantics:**
1. **Storage per subscriber:**
   - `bus_subscriber` struct with `next_read_idx` field (tracks next entry to read)
   - Each subscriber reads at their own pace independently
   - Storage cost: **O(max_subscribers)** fixed overhead

2. **Storage per entry:**
   - 32-bit `readers_mask` bitmask (max 32 subscribers per bus)
   - Bit N set → subscriber N has read this entry
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
- No backpressure mechanism (unlike IPC_BORROW)
- Real-time principle: Prefer fresh data over old data

**Example (data loss):**
```c
// Bus: max_entries=3, entries=[E1, E2, E3] (full), tail=0, head=0
// Fast subscriber: next_read_idx=0 (read all, awaiting E4)
// Slow subscriber: next_read_idx=0 (still at E1, hasn't read any)

rt_bus_publish(bus, &E4, sizeof(E4));
//   → Buffer full: Evict E1 at tail=0, free from pool
//   → Write E4 at index 0: entries=[E4, E2, E3]
//   → tail=1 (E2 is now oldest), head=1 (next write)

// Slow subscriber calls rt_bus_read():
//   → Search from tail=1: E2 (unread), E3 (unread), E4 (unread)
//   → Returns E2 (first unread)
//   → E1 is LOST (no error, silent skip)
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
  1. Check if bit N is set in `readers_mask` → if yes, skip (already read)
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
- If 3 subscribers each read entry once, that's 3 reads → entry removed
- Set `max_readers=0` to disable (entry persists until aged out or evicted)

**Example (unique counting):**
```c
// Bus with max_readers=2
// Subscribers: A, B, C

rt_bus_publish(bus, &E1, sizeof(E1));
//   → E1: readers_mask=0b000, read_count=0

// Subscriber A reads E1
rt_bus_read(bus, ...);
//   → E1: readers_mask=0b001, read_count=1 (A's bit set)

// Subscriber A reads again (tries to read E1)
rt_bus_read(bus, ...);
//   → E1 skipped (bit already set), returns RT_ERR_WOULDBLOCK
//   → E1: readers_mask=0b001, read_count=1 (unchanged)

// Subscriber B reads E1
rt_bus_read(bus, ...);
//   → E1: readers_mask=0b011, read_count=2 (B's bit set)
//   → read_count >= max_readers (2) → E1 REMOVED, freed from pool
```

---

### Summary: The Three Rules

| Rule | Contract |
|------|----------|
| **1. Subscription start position** | New subscribers start at "next publish" (cannot read history) |
| **2. Cursor storage & eviction** | Per-subscriber cursors; slow readers experience SILENT DATA LOSS on wraparound |
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
**FreeRTOS (ARM)** | Hardware timer + tick interrupt | Tick-based | 1-10 ms typical | Depends on `configTICK_RATE_HZ` (e.g., 1000 Hz = 1 ms)

- On Linux, timers use `CLOCK_MONOTONIC` clock source via `timerfd_create()`
- On Linux, requests < 1ms may still fire with ~1ms precision due to kernel scheduling
- On FreeRTOS, timers round up to next tick boundary

**Monotonic clock guarantee:**
- Uses **monotonic clock** on both platforms (CLOCK_MONOTONIC on Linux, hardware timer on FreeRTOS)
- **NOT affected by**:
  - System time changes (NTP adjustments, user setting clock)
  - Timezone changes
  - Daylight saving time
- **Guaranteed to**:
  - Never go backwards
  - Count elapsed time accurately (subject to clock drift, typically <100 ppm)
- **Use case**: Timers measure **elapsed time**, not wall-clock time

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
uint32_t one_hour_us = 3600 * 1000000;  // 3.6 billion us
// ERROR: Wraps to 705,032,704 us (~11.75 minutes)

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

All functions with `timeout_ms` parameter:

- `timeout_ms == 0`: Non-blocking, returns `RT_ERR_WOULDBLOCK` if would block
- `timeout_ms < 0`: Block forever
- `timeout_ms > 0`: Block up to timeout

On blocking calls, the actor yields to the scheduler. The I/O thread handles the actual operation and posts completion to the scheduler's completion queue.

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

File operations block the calling actor and yield to the scheduler.

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
#define RT_COMPLETION_QUEUE_SIZE 64         // I/O completion queue size
#define RT_DEFAULT_STACK_SIZE 65536         // Default actor stack size
```

All runtime structures are **statically allocated** based on these limits. Actor stacks use a static arena allocator by default (configurable via `actor_config.malloc_stack` for malloc). This ensures:
- Deterministic memory footprint (calculable at link time)
- Zero heap allocation in runtime operations (see Heap Usage Policy)
- O(1) pool allocation for all operations (scheduling, IPC, I/O completions)
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

## Completion Queue

Lock-free SPSC (Single Producer Single Consumer) queue for I/O completions.

Each I/O subsystem (net, file, timer) has its own completion queue. The I/O thread is the producer, the scheduler is the consumer.

```c
typedef struct {
    void           *buffer;
    size_t          entry_size;
    size_t          capacity;     // must be power of 2
    atomic_size_t   head;         // written by producer
    atomic_size_t   tail;         // written by consumer
} rt_spsc_queue;

rt_status rt_spsc_init(rt_spsc_queue *q, size_t entry_size, size_t capacity);
void      rt_spsc_destroy(rt_spsc_queue *q);

// Producer (I/O thread)
bool rt_spsc_push(rt_spsc_queue *q, const void *entry);

// Consumer (scheduler)
bool rt_spsc_pop(rt_spsc_queue *q, void *entry_out);
bool rt_spsc_peek(rt_spsc_queue *q, void *entry_out);
```

## Actor Death Handling

When an actor dies (via `rt_exit()`, crash, or external kill):

**Exception:** When death reason is `RT_EXIT_CRASH_STACK` (stack overflow), steps 1-5 below are **skipped** (only step 6 is performed) to prevent cascading crashes from accessing corrupted memory. Links and monitors are **not notified** of stack overflow deaths.

**Normal death cleanup:**

1. **Mailbox cleared:** All pending messages are discarded. Actors blocked on `IPC_BORROW` send receive `RT_ERR_CLOSED`.

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
   - Example: If A sends M1, M2 to B, then A dies, B receives: M1 → M2 → EXIT(A)

2. **Messages sent by dying actor:**
   - Messages successfully enqueued before death remain in recipient mailboxes
   - These messages will be delivered **before** exit notifications (FIFO)
   - Example: A sends M1 to B, then A dies, B receives: M1 → EXIT(A)

3. **Messages in dying actor's mailbox:**
   - Dying actor's mailbox is **cleared** (step 1)
   - All pending messages are **discarded**
   - Senders are **not** notified of message loss
   - Exception: IPC_BORROW senders are unblocked (see IPC_BORROW safety section)

4. **Multiple recipients:**
   - Each recipient's exit notification is enqueued independently
   - No ordering guarantee across different recipients
   - Example: A linked to B and C, dies. B and C both receive EXIT(A), but no guarantee which processes it first.

**Consequences:**

```c
// Dying actor sends messages before death
void actor_A(void *arg) {
    rt_ipc_send(B, &msg1, sizeof(msg1), IPC_COPY);  // Enqueued in B's mailbox
    rt_ipc_send(C, &msg2, sizeof(msg2), IPC_COPY);  // Enqueued in C's mailbox
    rt_exit();  // Exit notifications sent to links/monitors
}
// Linked actor B will receive: msg1, then EXIT(A)
// Actor C will receive: msg2 (no exit notification, not linked)

// Messages sent TO dying actor are lost
void actor_B(void *arg) {
    rt_ipc_send(A, &msg, sizeof(msg), IPC_COPY);  // Enqueued in A's mailbox
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
    while not shutdown_requested and actors_alive > 0:
        # 1. Process I/O completions
        process_net_completions()
        process_file_completions()
        process_timer_completions()

        # 2. Pick highest-priority runnable actor
        actor = pick_next_runnable()

        if actor exists:
            # 3. Context switch to actor
            current_actor = actor
            context_switch(scheduler_ctx, actor.ctx)
            # Returns here when actor yields/blocks

        else:
            # 4. No runnable actors, wait for I/O
            wait_for_completions()
```

## Scheduler Wakeup Mechanism

When all actors are blocked on I/O, the scheduler must efficiently wait for I/O completions rather than busy-polling. The runtime uses a **single shared wakeup primitive** that all I/O threads signal when posting completions.

### Implementation

**Linux (eventfd):**
```c
// Single eventfd shared by all I/O threads
int wakeup_fd = eventfd(0, EFD_SEMAPHORE);

// I/O thread posts completion
rt_spsc_push(&completion_queue, &completion);
eventfd_write(wakeup_fd, 1);  // Wake scheduler

// Scheduler waits when idle
if (no_runnable_actors) {
    eventfd_read(wakeup_fd, &val);  // Block until I/O completion
}
```

**FreeRTOS (binary semaphore):**
```c
// Single semaphore shared by all I/O threads
SemaphoreHandle_t wakeup_sem = xSemaphoreCreateBinary();

// I/O task posts completion
rt_spsc_push(&completion_queue, &completion);
xSemaphoreGiveFromISR(wakeup_sem, &higher_prio_woken);  // Wake scheduler

// Scheduler waits when idle
if (no_runnable_actors) {
    xSemaphoreTake(wakeup_sem, portMAX_DELAY);  // Block until I/O completion
}
```

### Semantics

- **Blocks the entire runtime task**: When no actors are runnable, the scheduler blocks on the wakeup primitive
- **Multiple producers**: File, network, and timer threads all signal the same primitive
- **Simple SPSC queues**: Each I/O subsystem has its own completion queue (no queue sets needed)
- **Signal after push**: I/O threads always signal after pushing completion (at most one extra wakeup per I/O operation)
- **Check all queues**: When woken, scheduler checks all three completion queues (file, net, timer)

### Advantages

- **Simple**: Single primitive, ~10 lines of code, no epoll/queue-set complexity
- **Efficient**: No CPU waste, blocks efficiently until I/O
- **Portable**: Maps cleanly to eventfd (Linux) and semaphore (FreeRTOS)
- **Low overhead**: One syscall per I/O completion (eventfd write)

### Alternative Approaches Considered

- **Polling with sleep**: Wastes CPU, adds latency (original implementation)
- **epoll + multiple eventfds**: More complex, no benefit for this use case
- **Queue sets (FreeRTOS)**: Unnecessary complexity, single semaphore works fine

## Platform Abstraction

The runtime abstracts platform-specific functionality:

| Component | Linux (dev) | FreeRTOS (prod) |
|-----------|-------------|-----------------|
| Context switch | x86-64 asm | ARM Cortex-M asm |
| I/O threads | pthreads | FreeRTOS tasks |
| Completion queue | SPSC with atomics | SPSC with atomics |
| Scheduler wakeup | eventfd | Binary semaphore |
| Timer | timerfd | FreeRTOS timer or hardware timer |
| Network | BSD sockets | lwIP |
| File | POSIX | FATFS or littlefs |

## Stack Overflow

### Semantic Contract (Defined Behavior)

**Stack overflow is NOT undefined behavior.** The runtime provides defined semantics even when detection is imperfect:

**Guaranteed semantics:**
1. **Actor crashes** with exit reason `RT_EXIT_CRASH_STACK`
2. **Links/monitors are notified** (receive exit message with `RT_EXIT_CRASH_STACK`)
3. **Runtime remains stable** (other actors continue running)
4. **Actor resources cleaned up** (stack freed, mailbox cleared, timers cancelled)

**Rationale:** Even if detection fails (severe corruption before guard check), the semantic CONTRACT is that stack overflow results in actor crash with notification, not undefined behavior. This enables recovery logic and supervisor patterns.

### Detection Mechanism (Best-Effort)

**Implementation:** Guard pattern detection (Linux/FreeRTOS)

**Mechanism:**
- 8-byte guard patterns placed at both ends of each actor stack
- Guards checked on every context switch
- Pattern: `0xDEADBEEFCAFEBABE` (uint64_t)
- Overhead: 16 bytes per stack (8 bytes × 2)

**Detection quality:**
- **Best-effort:** Catches most overflows during normal operation
- **Post-facto:** Detection occurs on next context switch after overflow
- **Not guaranteed:** Severe overflows may corrupt guards before check
- **Timing:** Overflow → corruption → next context switch → detection

**When detection succeeds:**
1. Guard check fails during context switch
2. Actor immediately marked DEAD with `RT_EXIT_CRASH_STACK`
3. Links/monitors receive exit notification
4. Mailbox cleared, stack freed, timers cancelled
5. Error logged: "Actor N stack overflow detected"
6. Runtime continues normally

**When detection fails (severe corruption):**
- Guard patterns themselves corrupted before check
- Actor may crash with segfault or corrupt other memory
- **Semantic contract still applies:** Intended behavior is actor crash with notification
- **Mitigation:** Size stacks appropriately, use MPU on production hardware

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
