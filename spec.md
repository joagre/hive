# Actor Runtime Specification

## Overview

A minimalistic actor-based runtime for embedded systems, initially targeting autopilot applications on STM32 (ARM Cortex-M). The runtime implements cooperative multitasking with message passing, inspired by Erlang's actor model.

**Design principles:**

- Minimalistic and predictable
- Modern C (C11 or later)
- No heap allocation in hot paths
- Least surprise API design
- Fast context switching via manual assembly

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

## Memory Model

### Actor Stacks

Each actor has a fixed-size stack allocated at spawn time. Stack size is configurable per actor, with a system-wide default.

Stack growth/reallocation is not supported. If an actor overflows its stack, behavior is undefined (on debug builds, a guard pattern may detect overflow).

### Memory Allocation

The runtime uses simple allocation strategies suitable for MCU:

- Actor table: Static array, size configured at init
- Mailbox entries: Simple linked list with fixed-size entries from a pool
- Bus entries: Ring buffer per bus

First version uses malloc/free for simplicity. Future versions may use dedicated memory pools.

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
    RT_EXIT_NORMAL,    // actor called rt_exit()
    RT_EXIT_CRASH,     // actor crashed (stack overflow, etc.)
    RT_EXIT_KILLED,    // actor was killed externally
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
    const void *data;   // valid until rt_ipc_release() or next rt_ipc_recv()
} rt_message;
```

### Send Modes

```c
typedef enum {
    IPC_COPY,    // memcpy payload to receiver's mailbox
    IPC_BORROW,  // zero-copy, sender blocks until receiver consumes
} rt_ipc_mode;
```

**IPC_COPY:** Payload is copied to receiver's mailbox. Sender continues immediately. Suitable for small messages.

**IPC_BORROW:** Zero-copy transfer. Payload remains on sender's stack. Sender blocks until receiver calls `rt_ipc_release()`. Provides implicit backpressure.

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

### Retention Semantics

- **max_readers:** Entry is removed after N actors have read it. If 0, entry persists until aged out or buffer wraps.
- **max_age_ms:** Entry is removed after this many milliseconds. If 0, no time-based expiry.
- **Buffer full:** Oldest entry is evicted when publishing to a full buffer.

Typical use cases:

- Sensor data: `max_readers=0, max_age_ms=100` – Multiple readers, data expires quickly
- Configuration: `max_readers=0, max_age_ms=0` – Persistent until overwritten
- Events: `max_readers=N, max_age_ms=0` – Consumed after all subscribers read

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

## Runtime Initialization

```c
typedef struct {
    size_t default_stack_size;    // default actor stack, bytes
    size_t max_actors;            // maximum concurrent actors
    size_t completion_queue_size; // entries per I/O completion queue
} rt_config;

#define RT_CONFIG_DEFAULT { \
    .default_stack_size = 65536, \
    .max_actors = 64, \
    .completion_queue_size = 64 \
}

// Initialize runtime (call once from main)
rt_status rt_init(const rt_config *cfg);

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

1. **Mailbox cleared:** All pending messages are discarded. Actors blocked on `IPC_BORROW` send receive `RT_ERR_CLOSED`.

2. **Links notified:** All linked actors receive an exit message with `sender == RT_SENDER_SYSTEM`.

3. **Monitors notified:** All monitoring actors receive an exit message.

4. **Bus subscriptions removed:** Actor is unsubscribed from all buses.

5. **Timers cancelled:** All timers owned by the actor are cancelled.

6. **Resources freed:** Stack and actor table entry are released.

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

## Platform Abstraction

The runtime abstracts platform-specific functionality:

| Component | Linux (dev) | FreeRTOS (prod) |
|-----------|-------------|-----------------|
| Context switch | x86-64 asm | ARM Cortex-M asm |
| I/O threads | pthreads | FreeRTOS tasks |
| Completion queue | SPSC with atomics | SPSC with atomics |
| Timer | timerfd | FreeRTOS timer or hardware timer |
| Network | BSD sockets | lwIP |
| File | POSIX | FATFS or littlefs |

## Future Considerations

Not in scope for first version, but noted for future:

- **Throttling on IPC send:** D-language style backpressure when mailbox is full
- **Memory pools:** Dedicated allocators for mailbox entries, bus entries, actor stacks
- **Stack overflow detection:** Guard patterns or MPU-based protection on Cortex-M
