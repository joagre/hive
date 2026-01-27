# Mailbox Backpressure Design

## Problem Statement

Currently, when the global message pool is exhausted, `hive_ipc_notify()` returns
`HIVE_ERR_NOMEM` immediately. The caller must handle this by retrying, backing
off, or discarding the message. This is explicit and predictable, but requires
every send site to handle the error.

This document explores adding optional **blocking behavior** on pool exhaustion,
where the sending actor yields until pool space becomes available.

## Current Behavior

```c
hive_status s = hive_ipc_notify(target, tag, data, len);
if (HIVE_FAILED(s)) {
    // Must handle HIVE_ERR_NOMEM here
    // Options: retry, backoff, log and discard, crash
}
```

**Characteristics:**
- Non-blocking (returns immediately)
- Explicit error handling required
- No surprise blocking or deadlocks
- Caller decides retry policy

## Design Options

### Option 1: Per-Actor Spawn Configuration

Add a flag to `actor_config` that makes all sends from that actor block on
pool exhaustion.

```c
actor_config cfg = {
    .priority = HIVE_PRIORITY_NORMAL,
    .stack_size = 8192,
    .block_on_send = true,  // New field
};
hive_spawn(my_actor, NULL, NULL, &cfg, &id);
```

**Pros:**
- Simple, declared at spawn time
- Behavior is fixed and predictable for the actor's lifetime
- No per-send overhead

**Cons:**
- Inflexible (can't change at runtime)
- All-or-nothing for entire actor
- May not match real usage patterns (some sends critical, some optional)

### Option 2: Runtime Configuration Function

Add a function to change the current actor's send behavior dynamically.

```c
// At start of actor or when entering critical section
hive_send_config(HIVE_SEND_BLOCK_ON_FULL);

hive_ipc_notify(target, tag, data, len);  // Now blocks if pool full

// Later, switch back
hive_send_config(HIVE_SEND_RETURN_ERROR);
```

**Pros:**
- Flexible, can change behavior based on context
- Actor can use blocking for critical messages, non-blocking for optional

**Cons:**
- Hidden state affects API behavior
- Easy to forget current mode
- Potential for confusion in code review

### Option 3: Per-Send Flag

Add blocking variants or flags to send functions.

```c
// Blocking send
hive_ipc_notify_blocking(target, tag, data, len, timeout);

// Or flag-based
hive_ipc_notify_ex(target, class, tag, data, len, HIVE_SEND_BLOCK);
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

Keep existing API non-blocking, add parallel blocking API.

```c
// Existing - always returns immediately
hive_ipc_notify(target, tag, data, len);

// New - blocks until sent or timeout
hive_ipc_send(target, tag, data, len, timeout);
```

**Pros:**
- Clear separation of concerns
- Existing code unchanged
- Name suggests blocking (`send` vs `notify`)

**Cons:**
- Two ways to do the same thing
- Must maintain parallel APIs

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

## Recommendation

**Option 4 (Separate Blocking API)** with timeout, following FreeRTOS precedent.

```c
// Non-blocking (existing) - returns HIVE_ERR_NOMEM on pool exhaustion
hive_status hive_ipc_notify(actor_id to, uint32_t tag,
                            const void *data, size_t len);

// Blocking (new) - waits for pool space, with timeout
hive_status hive_ipc_send(actor_id to, uint32_t tag,
                          const void *data, size_t len,
                          int32_t timeout_ms);
```

**Rationale:**
1. **Least surprise** - Embedded developers expect timeout parameter for blocking ops
2. **Explicit** - Blocking is visible at call site, no hidden state
3. **Backward compatible** - Existing code unchanged
4. **Flexible** - Can use `0` for try-once, `-1` for infinite, or specific timeout
5. **Matches precedent** - FreeRTOS, POSIX (`sem_timedwait`), etc.

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

Blocking sends can cause deadlocks:

```
Actor A sends to B (blocks, pool full)
Actor B sends to A (blocks, pool full)
```

**Mitigations:**
1. **Timeouts** - Always use finite timeouts in production
2. **Documentation** - Warn about deadlock potential
3. **Debug mode** - Detect and log potential deadlock cycles
4. **Design guidance** - Recommend non-blocking for replies

### Priority Inversion

If a high-priority actor blocks on send, and pool is held by low-priority actors:

**Mitigations:**
1. **Priority-based wake-up** - Wake highest priority waiter first
2. **Documentation** - Note that blocking send may delay high-priority actors

### Memory/Stack Impact

Blocked actors consume:
- Their stack (already allocated)
- Wait queue entry (small, ~16 bytes)

No additional memory allocation in hot path.

## API Design

```c
/**
 * Send a message, blocking if pool exhausted.
 *
 * @param to        Target actor ID
 * @param tag       Message tag for correlation
 * @param data      Message payload (copied)
 * @param len       Payload length in bytes
 * @param timeout_ms Timeout: -1 = infinite, 0 = try once, >0 = milliseconds
 *
 * @return HIVE_SUCCESS on success
 *         HIVE_ERR_TIMEOUT if timeout expired
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
hive_status hive_ipc_send(actor_id to, uint32_t tag,
                          const void *data, size_t len,
                          int32_t timeout_ms);
```

## Alternative: Receiver-Side Backpressure

Instead of blocking senders, could implement per-actor mailbox limits:

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
- Still need to handle sender behavior on overflow

Could be combined with sender-side blocking for complete solution.

## Questions for Further Discussion

1. Should `hive_ipc_request()` also have a blocking variant, or is the existing
   timeout sufficient?

2. Should bus publish (`hive_bus_publish()`) support blocking? Bus already
   drops oldest on buffer full, which may be acceptable.

3. Is per-actor mailbox limit worth the complexity? Current global pool is
   simpler and sufficient for many use cases.

4. Should blocked senders be killable via `hive_kill()`? (Probably yes)

## Conclusion

Adding `hive_ipc_send()` with timeout provides embedded developers a familiar
pattern for blocking sends while keeping the existing non-blocking API unchanged.
Implementation requires a wait queue for blocked actors and integration with
the pool free path.

This is a **low priority** enhancement - the current explicit error handling
works well and is arguably safer for embedded systems. Only implement if
real-world usage shows significant demand for blocking sends.
