# Pool Exhaustion Handling

## Problem Statement

**Normal backpressure** is already handled by `hive_ipc_request()`:
- Sender sends request, waits for reply
- Naturally rate-limits the sender
- This is the idiomatic actor pattern for flow control

**Pool exhaustion** is a different problem:
- All pool entries are in use
- New notify/request/reply/publish calls fail with `HIVE_ERR_NOMEM`
- This is an error condition, not normal flow control

Currently, when the global message pool is exhausted, all pool-using functions
return `HIVE_ERR_NOMEM` immediately. The caller must handle this by retrying,
backing off, or discarding the message. This is explicit and predictable, but
requires error handling at every call site.

This document describes the design for optional **blocking behavior** on pool
exhaustion, where the caller yields until pool space becomes available.

## Affected APIs

The following functions use the global message pool and can return `HIVE_ERR_NOMEM`:

| Function | Pool Usage | Notes |
|----------|------------|-------|
| `hive_ipc_notify()` | Message data pool | Fire-and-forget |
| `hive_ipc_notify_ex()` | Message data pool | With explicit class/tag |
| `hive_ipc_request()` | Message data pool | Blocking request/reply |
| `hive_ipc_reply()` | Message data pool | Reply to request |
| `hive_bus_publish()` | Message data pool | Pub/sub (also has per-bus buffer) |

**Note:** `hive_ipc_request()` already blocks (waiting for reply), but can still
fail immediately with `HIVE_ERR_NOMEM` if the pool is exhausted when sending the
request.

## Design Overview

Combine spawn-time defaults with runtime override capability. No API changes
to pool-using functions - behavior is controlled per-actor.

```c
// Set default at spawn time
actor_config cfg = {
    .priority = HIVE_PRIORITY_NORMAL,  // Also used for pool wait ordering
    .stack_size = 8192,
    .pool_block = true                 // Block on pool exhaustion
};
hive_spawn(my_actor, NULL, NULL, &cfg, &id);

// All calls use spawn default - no boilerplate!
hive_ipc_notify(to, tag, data, len);
hive_bus_publish(bus, data, len);

// Override temporarily
hive_pool_set_block(HIVE_POOL_NO_BLOCK);
hive_ipc_notify(to, tag, data, len);
hive_pool_set_block(HIVE_POOL_DEFAULT);  // Back to spawn default

// Query current setting
bool blocking = hive_pool_get_block();
```

**Key points:**
- Pool wait queue priority uses the actor's scheduling priority
  (`actor_config.priority`). No separate pool priority - critical actors
  naturally get pool space first.
- This is about **pool exhaustion** handling, not normal backpressure.
  Normal backpressure uses `hive_ipc_request()`. Pool exhaustion should not
  happen in a well-designed system - if it does, the pool is undersized.

## API

### Types

```c
typedef enum {
    HIVE_POOL_NO_BLOCK,  // Force non-blocking (return NOMEM on exhaustion)
    HIVE_POOL_BLOCK,     // Force blocking (yield until pool available)
    HIVE_POOL_DEFAULT    // Restore spawn default
} hive_pool_block_t;
```

### actor_config Extension

```c
typedef struct {
    hive_priority priority;
    size_t stack_size;
    bool malloc_stack;
    const char *auto_register;
    bool pool_block;  // NEW: false = return NOMEM (default), true = block
} actor_config;
```

### Runtime Functions

```c
// Override pool exhaustion behavior for current actor
void hive_pool_set_block(hive_pool_block_t mode);

// Query current effective setting
bool hive_pool_get_block(void);
```

## Implementation

### Storage

In `hive_actor_t`:

```c
bool pool_block_default;  // From actor_config at spawn
bool pool_block;          // Effective value now
```

### Behavior

- `actor_config.pool_block` not set (false) = current behavior (return NOMEM)
- `actor_config.pool_block = true` = block until pool available
- `hive_pool_set_block(HIVE_POOL_NO_BLOCK)` = set `pool_block = false`
- `hive_pool_set_block(HIVE_POOL_BLOCK)` = set `pool_block = true`
- `hive_pool_set_block(HIVE_POOL_DEFAULT)` = set `pool_block = pool_block_default`
- `hive_pool_get_block()` returns `pool_block`

### Wait Queue

- Scheduler maintains wait queue (static pool, bounded by `HIVE_MAX_ACTORS`)
- Wait queue ordered by actor's scheduling priority (CRITICAL > HIGH > NORMAL > LOW)
- On pool slot freed, wake highest priority waiter first (FIFO within same priority)
- Scheduler logs warning when actor blocks (rate-limited to avoid spam)

### Interactions

- `hive_actor_kill()` on blocked actor removes it from wait queue
- Supervisor `child_spec` must explicitly include `pool_block` (no inheritance)

### Memory Impact

Blocked actors consume:
- Their stack (already allocated)
- Wait queue entry (small, ~16 bytes)

No additional memory allocation in hot path.

## Rationale

### Why Hidden State is Acceptable

- Default behavior = current (non-blocking) - no surprises for existing code
- Blocking requires explicit opt-in at spawn time
- If user forgets they enabled blocking, that's their responsibility
- Library code is unaffected unless caller explicitly enabled blocking

### Why No Timeout

- KISS - simplest possible design
- Pool exhaustion is a design error (pool undersized), not a transient condition
- If pool exhaustion happens regularly, fix the pool size
- Deadlock = design error, not something to recover from with timeout

### Why Actor Priority for Wait Ordering

- Reuses existing concept - no new priority enum to learn
- Natural semantics: critical actors get resources first
- Consistent with scheduler behavior

### Semantic Tension: Actor Model vs Embedded Conventions

There is a fundamental tension between two communities:

**Actor Model Semantics** - In classic actor systems, notify is always
asynchronous and non-blocking:
- Actors are decoupled; sender doesn't wait for receiver
- Blocking on notify couples actors temporally, enabling deadlocks
- Backpressure is achieved via **request/reply patterns**, not blocking
- Erlang's `!` operator never blocks. Akka's `tell` never blocks.

**Embedded C Conventions** - Embedded developers expect blocking with timeouts:
- FreeRTOS `xQueueSend()` blocks if queue full
- POSIX `sem_wait()`, `pthread_cond_wait()` block
- Blocking is the norm; non-blocking is the variant (`_trylock`, `_trywait`)

**Our Resolution:**
- Default = actor model semantics (non-blocking, return NOMEM)
- Opt-in blocking for embedded developers who expect it
- Rate-limited warnings make blocking visible (it's still unusual)
- Deadlock = design error, documented prominently

## Comparison with Other Systems

| System | Mailbox Behavior |
|--------|------------------|
| Erlang/OTP | Unlimited mailbox, send never blocks (can OOM) |
| Akka | Bounded mailbox with configurable overflow strategy |
| QP/C | Event pools with assertions on exhaustion |
| FreeRTOS queues | Block with timeout, or return immediately |

**Erlang** - "Let it crash" philosophy. Unbounded mailboxes, system crashes on OOM.
Not suitable for embedded.

**Akka** - Configurable strategies: drop oldest, drop newest, drop head, backpressure.
Backpressure blocks sender. Configuration is per-mailbox (receiver side).

**QP/C** - Pools sized at design time. Exhaustion is a design error (assertion).
No runtime recovery.

**FreeRTOS** - `xQueueSend()` has timeout parameter. `portMAX_DELAY` = block forever.
This is the closest embedded precedent.

## Trade-offs

### Accepted Trade-offs

- Hidden state affects API behavior
- Deadlock risk when blocking enabled
- Scheduler complexity (wait queue)

### Mitigations

- Default = current behavior (non-blocking) - backward compatible
- Scheduler logs warnings when actors block (rate-limited)
- Actor priority used for wake-up ordering - critical actors get pool first
- `hive_actor_kill()` removes blocked actors from wait queue

### For Users Who Prefer Explicit Control

- Don't set `pool_block` (or set to false) - current behavior preserved
- They can still handle NOMEM explicitly at each call site

## Reserved System Entries

### Problem

When the message pool is exhausted, critical system messages could fail to be
delivered:

- **Timer messages** - Periodic timer ticks that actors depend on for scheduling
- **Exit messages** - Death notifications for linked/monitored actors

If these system messages fail, actors may hang waiting for timers that never
fire, or fail to detect crashed dependencies. This violates the reliability
guarantees that links and monitors are designed to provide.

### Solution

A portion of the pool is reserved exclusively for system messages (TIMER and
EXIT). When the pool is under pressure from user messages, system messages can
still be allocated from the reserved portion.

```
┌─────────────────────────────────────────────────────────────────┐
│                    Message Pool (256 entries)                   │
├─────────────────────────────────────────┬───────────────────────┤
│     User messages (240 entries)         │  Reserved (16 entries)│
│                                         │  TIMER + EXIT only    │
└─────────────────────────────────────────┴───────────────────────┘
```

### Configuration

```c
// Reserved pool entries for system messages (TIMER, EXIT)
// Default: 16 entries
#define HIVE_RESERVED_SYSTEM_ENTRIES 16
```

The reserved count applies to both the mailbox entry pool and the message data
pool.

### Behavior

**User messages** (`hive_ipc_notify()`, `hive_ipc_request()`, etc.):
- Can only allocate from the unreserved portion of the pool
- When unreserved entries exhausted: return `HIVE_ERR_NOMEM` or block (depending
  on `pool_block` setting)

**System messages** (TIMER, EXIT):
- Can allocate from the entire pool, including reserved entries
- Only fail when the *entire* pool is exhausted (extremely unlikely)
- Never block - system messages are generated by the runtime, not actors

### Implementation

The `hive_pool_alloc_reserved()` function implements this logic:

```c
void *hive_pool_alloc_reserved(pool_mgr_t *mgr, size_t reserved, bool is_system);
```

- `reserved` - Number of entries to reserve for system use
- `is_system` - True for TIMER/EXIT messages, false for user messages

If `is_system` is false, allocation fails when `free_count <= reserved`.
If `is_system` is true, allocation proceeds until pool is completely empty.

### Sizing Guidelines

The default of 16 reserved entries is sufficient for most systems:

- Each active timer consumes one entry per tick until consumed
- Each actor death generates one EXIT message per link/monitor
- Supervisor restarts may cause bursts of EXIT messages

If your system has many timers firing simultaneously, or supervisors with many
children, consider increasing `HIVE_RESERVED_SYSTEM_ENTRIES`.

**Warning:** Reserved entries reduce the pool available for user messages. If
you reserve too many, user messages may fail more often under normal load.

## Summary

- This is about **pool exhaustion**, not normal backpressure
- Normal backpressure uses `hive_ipc_request()`
- Pool exhaustion = design error (pool is undersized)
- Deadlock = design error (pool is undersized)
- Blocking with warnings is better than ad-hoc retry loops
- Reserved system entries guarantee timer and exit message delivery
