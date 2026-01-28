# Pool Exhaustion Handling

## Backpressure vs Pool Exhaustion

**Normal backpressure** is already handled by `hive_ipc_request()`:
- Sender sends request, waits for reply
- Naturally rate-limits the sender
- This is the idiomatic actor pattern for flow control

**Pool exhaustion** is a different problem:
- All pool entries are in use
- New notify/request/reply/publish calls fail with `HIVE_ERR_NOMEM`
- This is an error condition, not normal flow control

This document explores how to handle pool exhaustion - specifically, whether to
add blocking behavior when the pool is full.

## Problem Statement

Currently, when the global message pool is exhausted, all pool-using functions
return `HIVE_ERR_NOMEM` immediately. The caller must handle this by retrying,
backing off, or discarding the message. This is explicit and predictable, but
requires every call site to handle the error.

This document explores adding optional **blocking behavior** on pool exhaustion,
where the sending actor yields until pool space becomes available.

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

## Current Behavior

```c
hive_status s = hive_ipc_notify(target, tag, data, len);
if (HIVE_FAILED(s)) {
    // Must handle HIVE_ERR_NOMEM here
    // Options: retry, backoff, log and discard, crash
}
```

**Characteristics:**
- Non-blocking (returns immediately on pool exhaustion)
- Explicit error handling required at every call site
- No surprise blocking or deadlocks
- Caller decides retry policy

## Design Options

### Option 1: Per-Actor Spawn Configuration

Add a flag to `actor_config` that makes all pool operations from that actor
block on pool exhaustion.

```c
actor_config cfg = {
    .priority = HIVE_PRIORITY_NORMAL,
    .stack_size = 8192,
    .block_on_pool_full = true,  // New field
};
hive_spawn(my_actor, NULL, NULL, &cfg, &id);

// Query current actor's config
actor_config current;
hive_get_actor_config(&current);
if (current.block_on_pool_full) { ... }
```

**Pros:**
- Simple, declared at spawn time
- Behavior is fixed and predictable for the actor's lifetime
- No per-call overhead
- `hive_get_actor_config()` useful for other config inspection too

**Cons:**
- Inflexible (can't change at runtime)
- All-or-nothing for entire actor
- May not match real usage patterns (some messages critical, some optional)

### Option 2: Runtime Configuration Function

Add a function to change the current actor's pool exhaustion behavior dynamically.

```c
// At start of actor or when entering critical section
hive_pool_config cfg = {
    .block_on_full = true,
    .block_timeout_ms = 1000,  // -1 = infinite, 0 = try once
};
hive_pool_set_config(&cfg);

hive_ipc_notify(target, tag, data, len);  // Now blocks if pool full
hive_bus_publish(bus, data, len);         // This too

// Later, switch back
cfg.block_on_full = false;
hive_pool_set_config(&cfg);

// Query current setting
hive_pool_config current;
hive_pool_get_config(&current);
```

**Pros:**
- Flexible, can change behavior based on context
- Affects all pool-using operations uniformly

**Cons:**
- Hidden state affects API behavior
- Easy to forget current mode
- Potential for confusion in code review
- Applies to all operations - can't have blocking IPC but non-blocking bus

### Option 3: Per-Call Flag

Add blocking variants or flags to pool-using functions.

```c
// Blocking variant
hive_ipc_notify_blocking(target, tag, data, len, timeout);

// Or flag-based
hive_ipc_notify_ex(target, class, tag, data, len, HIVE_BLOCK_ON_POOL_FULL);
```

**Pros:**
- Explicit at each call site
- No hidden state
- Reviewer can see blocking behavior immediately

**Cons:**
- More API surface
- Every call site must decide
- Verbose

### Option 4: Separate Blocking API

Keep existing API non-blocking, add parallel blocking API with `_wait` suffix.

```c
// Existing - always returns immediately
hive_ipc_notify(target, tag, data, len);

// New - blocks until pool space available or timeout
hive_ipc_notify_wait(target, tag, data, len, timeout);
```

**Pros:**
- Clear separation of concerns
- Existing code unchanged
- Consistent naming (`_wait` suffix matches existing conventions)

**Cons:**
- Two ways to do the same thing
- Must maintain parallel APIs

### Option 5: Combine Option 1 + Option 2 (Recommended)

Combine spawn-time defaults with runtime override capability. No API changes
to pool-using functions - behavior is controlled per-actor.

```c
// Set default at spawn time
actor_config cfg = {
    .priority = HIVE_PRIORITY_NORMAL,  // Also used for pool wait ordering
    .stack_size = 8192,
    .block_on_pool_full = true         // Block on pool exhaustion
};
hive_spawn(my_actor, NULL, NULL, &cfg, &id);

// All calls use spawn default - no boilerplate!
hive_ipc_notify(to, tag, data, len);
hive_bus_publish(bus, data, len);

// Override temporarily
hive_pool_set_block(true);
hive_ipc_notify(to, tag, data, len);
hive_pool_restore_block();  // Back to spawn default

// Query current setting
bool blocking = hive_pool_get_block();
```

**Note:** Pool wait queue priority uses the actor's scheduling priority
(`actor_config.priority`). No separate pool priority - critical actors
naturally get pool space first.

**Important:** This is about **pool exhaustion** handling, not normal backpressure.
Normal backpressure uses `hive_ipc_request()`. Pool exhaustion should not happen
in a well-designed system - if it does, the pool is undersized.

**Pros:**
- No boilerplate at call sites - API unchanged
- Sensible defaults set once at spawn
- Runtime override when needed for special cases
- `hive_pool_restore_block()` restores spawn default
- No new function variants to maintain
- Single priority system - actor priority used for pool wait ordering
- Just a bool in actor_config - no struct
- KISS - no timeout complexity

**Cons:**
- Hidden state affects behavior
- Deadlock risk when `block_on_pool_full = true`
- Requires scheduler wait queue implementation

**Implementation:**
- `actor_config.block_on_pool_full` not set (false) = current behavior (return NOMEM)
- `actor_config.block_on_pool_full = true` = block until pool available
- `hive_pool_set_block(bool)` overrides for current actor
- `hive_pool_restore_block()` restores spawn default
- `hive_pool_get_block()` returns current active setting
- Scheduler maintains wait queue (static pool, bounded by HIVE_MAX_ACTORS)
- Wait queue ordered by actor's scheduling priority (CRITICAL > HIGH > NORMAL > LOW)
- On pool slot freed, wake highest priority waiter first (FIFO within same priority)
- Scheduler logs warning when actor blocks (rate-limited to avoid spam)
- `hive_kill()` on blocked actor removes it from wait queue
- Supervisor child_spec must include `block_on_pool_full` for restarts

**Hidden state is acceptable because:**
- Default behavior = current (non-blocking) - no surprises for existing code
- Blocking requires explicit opt-in at spawn time
- If user forgets they enabled blocking, that's their responsibility
- Library code is unaffected unless caller explicitly enabled blocking

**Deadlock:**
- `block_on_pool_full = true` can cause deadlock if all actors block
- Deadlock = design error - pool is undersized
- Rate-limited warnings make pool exhaustion visible

### Option 6: Per-Call Bool Parameter

Add an optional bool parameter to all pool-using functions.

```c
// Use actor's default
hive_ipc_notify(target, tag, data, len, HIVE_POOL_DEFAULT);

// Override for this call only
hive_ipc_notify(target, tag, data, len, HIVE_POOL_BLOCK);
hive_ipc_notify(target, tag, data, len, HIVE_POOL_NO_BLOCK);
```

**Cons:**
- API change (additional parameter to all functions)
- Boilerplate (must pass something everywhere)
- Defeats the purpose of Option 5's "no boilerplate" goal

**Not recommended** - Option 5 already solves the problem without API changes.

## Comparison with Other Actor Systems

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

## Semantic Tension: Actor Model vs Embedded Conventions

There is a fundamental tension between two communities:

### Actor Model Semantics

In classic actor systems, **send is always asynchronous and non-blocking**:

- Actors are decoupled; sender doesn't wait for receiver
- Blocking on send couples actors temporally, enabling deadlocks
- Backpressure is achieved via **request/reply patterns**, not blocking sends
- If you need guaranteed delivery, use `request()` which naturally rate-limits

Erlang's `!` (send) operator never blocks. Akka's `tell` never blocks. This is
fundamental to the actor model's ability to reason about concurrency.

### Embedded C Conventions

Embedded developers expect blocking operations with timeouts:

- FreeRTOS `xQueueSend()` blocks if queue full
- POSIX `sem_wait()`, `pthread_cond_wait()` block
- Blocking is the norm; non-blocking is the variant (`_trylock`, `_trywait`)

### The Problem with Blocking on Pool Full

Adding `hive_ipc_notify_wait()` introduces risks:

1. **Deadlock** - Actor A blocks notifying B, B blocks notifying A
2. **Priority inversion** - High-priority actor blocked by low-priority pool usage
3. **Breaks mental model** - "notify" implies fire-and-forget, not "wait until delivered"
4. **Hidden coupling** - Actors become temporally coupled

### Alternative: Embrace the Actor Model

Instead of adding blocking variants, Hive could:

1. **Size pools appropriately** - Pool exhaustion is a design error, not runtime condition
2. **Use request/reply** - `hive_ipc_request()` provides natural backpressure
3. **Drop and log** - For non-critical messages, NOMEM means "system overloaded, drop it"
4. **Crash early** - For critical messages, NOMEM is fatal (QP/C approach)

This keeps the API true to actor semantics while being honest about resource limits.

### Recommendation Revisited

Given this tension, there are two defensible positions:

**Position A: Don't add blocking variants**
- Keep `hive_ipc_notify()` non-blocking only
- Document that NOMEM requires explicit handling
- Recommend request/reply for backpressure
- Stay true to actor model semantics

**Position B: Add blocking variants with strong warnings**
- Add `_wait` variants for embedded developers who expect them
- Document deadlock risks prominently
- Recommend non-blocking as default
- Accept that some users will misuse them

Hive currently follows Position A. This document explores Position B but does
not necessarily recommend it.

## If We Choose Position B

**If** we decide to add blocking variants, Option 4 (Separate Blocking API) with
`_wait` suffix and timeout would follow FreeRTOS precedent:

```c
// Non-blocking (existing) - returns HIVE_ERR_NOMEM on pool exhaustion
hive_status hive_ipc_notify(actor_id to, uint32_t tag,
                            const void *data, size_t len);

// Blocking (new) - waits for pool space, with timeout
hive_status hive_ipc_notify_wait(actor_id to, uint32_t tag,
                                 const void *data, size_t len,
                                 int32_t timeout_ms);
```

**Rationale for this approach (if implementing):**
1. **Consistent naming** - Uses existing verbs (notify, request, reply, publish) with `_wait` suffix
2. **Least surprise for embedded** - Timeout parameter matches FreeRTOS, POSIX conventions
3. **Explicit** - Blocking is visible at call site, no hidden state
4. **Backward compatible** - Existing code unchanged
5. **Flexible** - Can use `0` for try-once, `-1` for infinite, or specific timeout

**However**, this conflicts with actor model semantics where "notify" means
fire-and-forget. A `notify` that blocks is semantically confusing.

## Implementation Considerations

### Wake-up Mechanism

When pool space becomes available, how do blocked senders wake up?

**Option A: Dedicated wait queue**
- Maintain list of actors blocked on pool exhaustion
- When message is freed, wake first waiter
- FIFO fairness

**Option B: Polling with backoff**
- Blocked actor retries periodically
- Simpler implementation
- Less efficient, but pool exhaustion should be rare

**Recommendation:** Option A for fairness and efficiency.

### Deadlock Prevention

Blocking on pool full can cause deadlocks:

```
Actor A notifies B (blocks, pool full)
Actor B notifies A (blocks, pool full)
```

**Mitigations:**
1. **Timeouts** - Always use finite timeouts in production
2. **Documentation** - Warn about deadlock potential
3. **Debug mode** - Detect and log potential deadlock cycles
4. **Design guidance** - Recommend non-blocking for replies

### Priority Inversion

If a high-priority actor blocks on pool full, and pool is held by low-priority actors:

**Mitigations:**
1. **Priority-based wake-up** - Wake highest priority waiter first
2. **Documentation** - Note that blocking may delay high-priority actors

### Memory/Stack Impact

Blocked actors consume:
- Their stack (already allocated)
- Wait queue entry (small, ~16 bytes)

No additional memory allocation in hot path.

## API Design

### Blocking Variants for All Pool-Using Functions

```c
// Existing non-blocking (unchanged)
hive_status hive_ipc_notify(actor_id to, uint32_t tag,
                            const void *data, size_t len);
hive_status hive_ipc_notify_ex(actor_id to, hive_msg_class class, uint32_t tag,
                               const void *data, size_t len);
hive_status hive_ipc_reply(const hive_message *request,
                           const void *data, size_t len);
hive_status hive_bus_publish(hive_bus *bus, const void *data, size_t len);

// New blocking variants (with _wait suffix)
hive_status hive_ipc_notify_wait(actor_id to, uint32_t tag,
                                 const void *data, size_t len,
                                 int32_t timeout_ms);
hive_status hive_ipc_notify_ex_wait(actor_id to, hive_msg_class class, uint32_t tag,
                                    const void *data, size_t len,
                                    int32_t timeout_ms);
hive_status hive_ipc_reply_wait(const hive_message *request,
                                const void *data, size_t len,
                                int32_t timeout_ms);
hive_status hive_bus_publish_wait(hive_bus *bus, const void *data, size_t len,
                                  int32_t timeout_ms);
```

**Note:** `hive_ipc_request()` already has a timeout parameter. If the pool is
exhausted, should it:
- (A) Return `HIVE_ERR_NOMEM` immediately (current behavior), or
- (B) Wait for pool space using the same timeout?

Option (B) is more consistent but changes existing semantics. Recommend keeping
(A) for backward compatibility, or adding `hive_ipc_request_wait()` if needed.

### Example: hive_ipc_notify_wait()

```c
/**
 * Notify an actor, blocking if pool exhausted.
 *
 * @param to        Target actor ID
 * @param tag       Message tag for correlation
 * @param data      Message payload (copied)
 * @param len       Payload length in bytes
 * @param timeout_ms Timeout: -1 = infinite, 0 = try once, >0 = milliseconds
 *
 * @return HIVE_SUCCESS on success
 *         HIVE_ERR_TIMEOUT if timeout expired waiting for pool space
 *         HIVE_ERR_INVALID if target doesn't exist
 *         HIVE_ERR_NOMEM if timeout=0 and pool full
 *
 * @note May cause deadlock if used carelessly. Prefer hive_ipc_notify()
 *       for fire-and-forget messages, use this only when delivery is
 *       critical and backpressure is acceptable.
 *
 * @warning Do NOT use with timeout=-1 in production without careful
 *          deadlock analysis.
 */
hive_status hive_ipc_notify_wait(actor_id to, uint32_t tag,
                                 const void *data, size_t len,
                                 int32_t timeout_ms);
```

## Alternative: Receiver-Side Limits

Instead of blocking callers, could implement per-actor mailbox limits:

```c
actor_config cfg = {
    .mailbox_limit = 16,  // Max pending messages for this actor
    .overflow = HIVE_OVERFLOW_DROP_OLDEST,  // or DROP_NEWEST, BLOCK_SENDER
};
```

This shifts control to the receiver, which may be more natural for some patterns.

**Pros:**
- Receiver controls its own load
- Natural for producer-consumer patterns
- Can have different policies per actor

**Cons:**
- More complex implementation (per-actor tracking)
- Still need to handle caller behavior on overflow

Could be combined with caller-side blocking for complete solution.

## Questions for Further Discussion

1. Should `hive_ipc_request()` wait for pool space using its existing timeout,
   or keep returning `HIVE_ERR_NOMEM` immediately? (See API Design section)

2. Is `hive_bus_publish_wait()` useful? Bus already drops oldest on buffer full,
   so blocking may not match typical pub/sub patterns.

3. Is per-actor mailbox limit worth the complexity? Current global pool is
   simpler and sufficient for many use cases.

4. Should blocked senders be killable via `hive_kill()`? (Probably yes)

5. Should there be a single unified `hive_pool_wait(timeout)` that blocks until
   pool space is available, rather than per-function variants?

## Conclusion

**Recommendation: Option 5 (Combine Option 1 + Option 2).**

The key insight is that requiring NOMEM handling at every call site leads to:
- Boilerplate code that developers resent
- Ad-hoc busy-loop retry patterns that are often worse than blocking
- Inconsistent error handling across the codebase

Option 5 solves this by:
- Setting pool behavior once at spawn time (no per-call boilerplate)
- Allowing runtime override for special cases
- Keeping the existing API unchanged
- Using actor priority for pool wait ordering (no new priority concept)
- Making pool exhaustion visible via scheduler warnings (rate-limited)

**The design is simple (KISS):**
```c
// In actor_config
bool block_on_pool_full;  // false = try once (default), true = block

// Runtime API
hive_pool_set_block(bool block);
hive_pool_restore_block(void);
bool hive_pool_get_block(void);
```

Just a bool. Actor's scheduling priority determines pool wait order. No timeouts.
No struct. YAGNI.

**Trade-offs accepted:**
- Hidden state affects API behavior
- Deadlock risk when blocking enabled
- Scheduler complexity (wait queue)

**Mitigations:**
- Default = current behavior (non-blocking) - backward compatible
- Scheduler logs warnings when actors block (rate-limited)
- Actor priority used for wake-up ordering - critical actors get pool first
- `hive_kill()` removes blocked actors from wait queue

**For users who prefer explicit control:**
- Don't set `block_on_pool_full` (or set to false) - current behavior preserved
- They can still handle NOMEM explicitly at each call site

**Remember:**
- This is about **pool exhaustion**, not normal backpressure
- Normal backpressure uses `hive_ipc_request()`
- Pool exhaustion = design error (pool is undersized)
- Deadlock = design error (pool is undersized)
- Blocking with warnings is better than bad retry loops
