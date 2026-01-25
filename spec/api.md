# API Reference

This document contains the complete API reference for the Hive actor runtime.

---

## Core Types

```c
// Handles (opaque to user)
typedef uint32_t actor_id_t;
typedef uint32_t bus_id_t;
typedef uint32_t timer_id_t;

#define ACTOR_ID_INVALID  ((actor_id_t)0)
#define BUS_ID_INVALID    ((bus_id_t)0)
#define TIMER_ID_INVALID  ((timer_id_t)0)

// Wildcard for selective receive filtering
#define HIVE_SENDER_ANY     ((actor_id_t)0xFFFFFFFF)

// Actor entry point (see Actor API section for full signature)
typedef void (*actor_fn_t)(void *args, const hive_spawn_info_t *siblings, size_t sibling_count);

// Init function called in spawner context
typedef void *(*hive_actor_init_fn_t)(void *init_args);

// Spawn info passed to actors at startup
typedef struct {
    const char *name;  // Actor name (may be NULL)
    actor_id_t id;       // Actor ID
    bool registered;   // Whether registered in name registry
} hive_spawn_info_t;

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    hive_priority_level_t priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
    bool        auto_register;// auto-register name in registry
} actor_config_t;
```

## Actor API

### Actor Function Signature

```c
// Actor entry function receives arguments, sibling info, and sibling count
typedef void (*actor_fn_t)(void *args, const hive_spawn_info_t *siblings, size_t sibling_count);

// Init function (optional) called in spawner context
typedef void *(*hive_actor_init_fn_t)(void *init_args);

// Spawn info passed to actors at startup
typedef struct {
    const char *name;  // Actor name (may be NULL)
    actor_id_t id;       // Actor ID
    bool registered;   // Whether registered in name registry
} hive_spawn_info_t;
```

### Lifecycle

```c
// Spawn a new actor
// fn: Actor entry function
// init: Init function (NULL = skip, pass init_args directly to actor)
// init_args: Arguments to init (or directly to actor if init is NULL)
// cfg: Actor configuration (NULL = use defaults)
// out: Receives the new actor's ID
hive_status_t hive_spawn(actor_fn_t fn, hive_actor_init_fn_t init, void *init_args,
                       const actor_config_t *cfg, actor_id_t *out);

// Terminate current actor
_Noreturn void hive_exit(void);

// Find sibling by name in spawn info array
const hive_spawn_info_t *hive_find_sibling(const hive_spawn_info_t *siblings,
                                          size_t count, const char *name);
```

**Spawn behavior**
- `init` (if provided) is called in spawner context before actor starts
- `init` return value is passed to actor as `args`
- Actor receives sibling info at startup:
  - Standalone actors: siblings[0] = self info, sibling_count = 1
  - Supervised actors: siblings = all sibling children
- If `cfg->auto_register` is true and `cfg->name` is set, actor is auto-registered in name registry

**Sibling array lifetime**
The sibling array is a **startup-time snapshot**. If a sibling restarts (getting a new `actor_id_t`), any cached IDs become stale. For scenarios where siblings may restart:
- Use `hive_whereis()` for dynamic lookups
- Sibling array is best for stable peer references in ONE_FOR_ALL supervision (all restart together)
- Registry is best for service discovery patterns where restarts are expected

**Actor function return behavior**

Actors **must** call `hive_exit()` to terminate cleanly. If an actor function returns without calling `hive_exit()`, the runtime detects this as a crash:

1. Exit reason is set to `HIVE_EXIT_CRASH`
2. Linked/monitoring actors receive exit notification with `HIVE_EXIT_CRASH` reason
3. Normal cleanup proceeds (mailbox cleared, resources freed)
4. An ERROR log is emitted: "Actor N returned without calling hive_exit()"

This crash detection prevents infinite loops and ensures linked actors are notified of the improper termination.

```c
// Get current actor's ID
actor_id_t hive_self(void);

// Yield to scheduler
void hive_yield(void);

// Check if actor is alive
bool hive_actor_alive(actor_id_t id);

// Kill an actor externally
hive_status_t hive_kill(actor_id_t target);
```

**hive_kill(target)**: Terminates the target actor immediately. This is a **hard kill** - the target cannot resist or defer termination. There is no graceful shutdown protocol; the actor is removed from the scheduler at the next opportunity. The target's exit reason is set to `HIVE_EXIT_KILLED`. Linked/monitoring actors receive exit notifications. Cannot kill self (use `hive_exit()` instead). Used internally by supervisors to terminate children during shutdown or strategy application.

For graceful shutdown, implement an application-level protocol: send a shutdown request message, wait for acknowledgment, then kill if needed.

### Stack Watermarking

When `HIVE_STACK_WATERMARK=1`, the runtime fills actor stacks with a pattern at allocation time. This allows measuring actual stack usage by scanning for untouched bytes.

```c
// Get stack usage for an actor (bytes used)
// Returns actual usage if watermarking enabled, or stack_size if disabled
size_t hive_actor_stack_usage(actor_id_t id);

// Callback for stack usage iteration
typedef void (*stack_usage_callback_t)(actor_id_t id, const char *name,
                                     size_t stack_size, size_t used);

// Iterate all live actors and report stack usage via callback
void hive_actor_stack_usage_all(stack_usage_callback_t cb);
```

**Usage example**

```c
void print_stack(actor_id_t id, const char *name, size_t stack_size, size_t used) {
    printf("Actor %u (%s): %zu/%zu bytes (%.1f%%)\n",
           id, name ? name : "unnamed", used, stack_size,
           100.0 * used / stack_size);
}

// In some monitoring actor:
hive_actor_stack_usage_all(print_stack);
```

**Notes**
- Adds overhead at spawn time (stack fill) and query time (pattern scan)
- Disabled by default (`HIVE_STACK_WATERMARK=0`)
- Enable via compile flag: `make CFLAGS+='-DHIVE_STACK_WATERMARK=1'`
- Pattern is configurable via `HIVE_STACK_WATERMARK_PATTERN` (default: `0xDEADBEEF`)

### Linking and Monitoring

Actors can link to other actors to receive notification when they die:

```c
// Bidirectional link: if either dies, the other receives exit message
hive_status_t hive_link(actor_id_t target);
hive_status_t hive_link_remove(actor_id_t target);

// Unidirectional monitor: receive notification when target dies
hive_status_t hive_monitor(actor_id_t target, uint32_t *out);
hive_status_t hive_monitor_cancel(uint32_t id);
```

Exit message structure:

```c
typedef enum {
    HIVE_EXIT_NORMAL,       // Actor called hive_exit()
    HIVE_EXIT_CRASH,        // Actor function returned without calling hive_exit()
    HIVE_EXIT_CRASH_STACK,  // Stack overflow detected
    HIVE_EXIT_KILLED,       // Actor was killed externally
} hive_exit_reason_t;

typedef struct {
    actor_id_t         actor;      // who died
    hive_exit_reason_t reason;     // why they died
    uint32_t         monitor_id; // 0 = link, non-zero = from monitor
} hive_exit_msg_t;

// Check if message is exit notification
bool hive_is_exit_msg(const hive_message_t *msg);

// Decode exit message into struct
hive_status_t hive_decode_exit(const hive_message_t *msg, hive_exit_msg_t *out);

// Convert exit reason to string (for logging/debugging)
const char *hive_exit_reason_str(hive_exit_reason_t reason);
```

**The monitor_id field** distinguishes exit notifications from links vs monitors:
- `monitor_id == 0`: notification came from a bidirectional link
- `monitor_id != 0`: notification came from a monitor; value matches the ID returned by `hive_monitor()`

**Handling exit messages**

Exit messages should be decoded using `hive_decode_exit()`:

```c
hive_message_t msg;
hive_ipc_recv(&msg, -1);

if (hive_is_exit_msg(&msg)) {
    hive_exit_msg_t exit_info;
    hive_decode_exit(&msg, &exit_info);
    if (exit_info.monitor_id == 0) {
        printf("Linked actor %u died: %s\n", exit_info.actor,
               hive_exit_reason_str(exit_info.reason));
    } else {
        printf("Monitored actor %u died (id=%u): %s\n", exit_info.actor,
               exit_info.monitor_id, hive_exit_reason_str(exit_info.reason));
    }
}
```

### Name Registry

Actor naming. Actors can register themselves with a symbolic name, and other actors can look up actor IDs by name. Names are automatically unregistered when the actor exits.

```c
// Register calling actor with a name (must be unique)
hive_status_t hive_register(const char *name);

// Look up actor ID by name
hive_status_t hive_whereis(const char *name, actor_id_t *out);

// Unregister a name (also automatic on actor exit)
hive_status_t hive_unregister(const char *name);
```

**Behavior**

- `hive_register(name)`: Associates the calling actor with `name`. The name must be unique. The name string must remain valid for the lifetime of the registration (typically a string literal). Returns `HIVE_ERR_INVALID` if name is NULL or already registered. Returns `HIVE_ERR_NOMEM` if registry is full (`HIVE_MAX_REGISTERED_NAMES`).

- `hive_whereis(name, out)`: Looks up an actor ID by name. Returns `HIVE_ERR_INVALID` if name is NULL or not found. If the named actor has exited and re-registered (e.g., after supervisor restart), returns the new actor ID.

- `hive_unregister(name)`: Removes a name registration. Only the owning actor can unregister its own names. Returns `HIVE_ERR_INVALID` if name is NULL, not found, or not owned by caller. Rarely needed since names are automatically unregistered on actor exit.

**Implementation**

- Static table of `(name, actor_id_t)` pairs (`HIVE_MAX_REGISTERED_NAMES` = 32 default)
- Linear scan for lookups (O(n), suitable for small registries)
- Names are pointers to user-provided strings (not copied)
- Auto-cleanup in `hive_actor_free()` removes all entries for the exiting actor

**Use with supervisors**

When actors are restarted by a supervisor, they should call `hive_register()` with the same name. Actors that need to communicate with them should call `hive_whereis()` each time they need to send a message, rather than caching the actor ID at startup. This ensures they get the current actor ID even after restarts.

```c
// Service actor that registers itself
void database_service(void *arg) {
    hive_register("database");
    while (1) {
        hive_message_t msg;
        hive_ipc_recv(&msg, -1);
        // Handle database requests...
    }
}

// Client actor that uses whereis (called on each send)
void send_query(const char *query) {
    actor_id_t db;
    if (HIVE_SUCCEEDED(hive_whereis("database", &db))) {
        hive_ipc_notify(db, HIVE_TAG_NONE, query, strlen(query) + 1);
    }
}
```

### Siblings vs Registry: When to Use Each

| Use Case | Siblings | Registry |
|----------|----------|----------|
| Supervised pipeline (ONE_FOR_ALL) | Best | Works |
| Supervised workers (ONE_FOR_ONE) | Careful | Best |
| Service discovery pattern | No | Best |
| Tight peer coupling | Best | Overkill |
| Cross-supervisor communication | No | Required |

**Heuristics**

1. **Use siblings when**
   - All communicating actors restart together (ONE_FOR_ALL strategy)
   - Actors are tightly coupled and always start/stop as a group
   - You want zero-overhead lookup (sibling array is passed at startup)

2. **Use registry when**
   - Actors may restart independently (ONE_FOR_ONE or separate supervisors)
   - You need late binding (sender doesn't know receiver at compile time)
   - Cross-supervisor or cross-subsystem communication
   - Service discovery pattern (many clients, one named service)

3. **Avoid caching IDs from either**
   - Both sibling IDs and registry IDs become stale after target restarts
   - Always lookup fresh when the target may have restarted
   - Exception: ONE_FOR_ALL where you restart together anyway

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

**Header overhead** - 4 bytes per message.

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
} hive_msg_class_t;
```

### Tag System

```c
#define HIVE_TAG_NONE        0            // No tag (for simple NOTIFY messages)
#define HIVE_TAG_ANY         0x0FFFFFFF   // Wildcard for selective receive filtering

// Note: HIVE_TAG_GEN_BIT and HIVE_TAG_VALUE_MASK are internal implementation
// details and not part of the public API
```

**Tag semantics**
- **HIVE_TAG_NONE**: Used for simple NOTIFY messages where no correlation is needed
- **HIVE_TAG_ANY**: Used in `hive_ipc_recv_match()` to match any tag
- **Generated tags**: Created automatically by `hive_ipc_request()` for request/reply correlation

**Tag generation** - Internal to `hive_ipc_request()`. Global counter increments on each call. Generated tags have `HIVE_TAG_GEN_BIT` set. Wraps at 2^27 (134M values).

**Namespace separation** - Generated tags (gen=1) and user tags (gen=0) can never collide.

### Message Structure

```c
typedef struct {
    actor_id_t       sender;       // Sender actor ID
    hive_msg_class_t class;        // Message class
    uint32_t       tag;          // Message tag
    size_t         len;          // Payload length (excludes 4-byte header)
    const void    *data;         // Payload pointer (past header)
} hive_message_t;
```

The message struct provides **direct access to all fields**:

```c
hive_message_t msg;
hive_ipc_recv(&msg, -1);

// Direct access - no boilerplate
my_data *payload = (my_data *)msg.data;
if (msg.class == HIVE_MSG_REQUEST) {
    hive_ipc_reply(&msg, &response, sizeof(response));
}
```

**Lifetime rule** - Data is valid until the next successful `hive_ipc_recv()`, `hive_ipc_recv_match()`, or `hive_ipc_recv_matches()` call. Copy immediately if needed beyond current iteration.

### Functions

#### Basic Messaging

```c
// Fire-and-forget message (class=NOTIFY)
// Tag enables selective receive filtering on the receiver side
hive_status_t hive_ipc_notify(actor_id_t to, uint32_t tag, const void *data, size_t len);

// Send with explicit class and tag (sender is current actor)
hive_status_t hive_ipc_notify_ex(actor_id_t to, hive_msg_class_t class,
                               uint32_t tag, const void *data, size_t len);

// Receive any message (no filtering)
// timeout_ms == 0:  non-blocking, returns HIVE_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns HIVE_ERR_TIMEOUT if exceeded
hive_status_t hive_ipc_recv(hive_message_t *msg, int32_t timeout_ms);
```

#### Selective Receive

```c
// Receive with filtering on sender, class, and/or tag
// Blocks until message matches ALL non-wildcard criteria, or timeout
// Use HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY as wildcards
hive_status_t hive_ipc_recv_match(actor_id_t from, hive_msg_class_t class,
                            uint32_t tag, hive_message_t *msg, int32_t timeout_ms);
```

**Filter semantics**
- `from == HIVE_SENDER_ANY` -> match any sender
- `class == HIVE_MSG_ANY` -> match any class
- `tag == HIVE_TAG_ANY` -> match any tag
- Non-wildcard values must match exactly

**Usage examples**
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

#### Multi-Pattern Selective Receive

```c
// Filter structure for multi-pattern matching
typedef struct {
    actor_id_t sender;      // HIVE_SENDER_ANY for any sender
    hive_msg_class_t class; // HIVE_MSG_ANY for any class
    uint32_t tag;         // HIVE_TAG_ANY for any tag
} hive_recv_filter_t;

// Receive message matching ANY of the provided filters
// matched_index (optional): which filter matched (0-based)
hive_status_t hive_ipc_recv_matches(const hive_recv_filter_t *filters,
                            size_t num_filters, hive_message_t *msg,
                            int32_t timeout_ms, size_t *matched_index);
```

**Use cases**
- Waiting for REPLY or EXIT (used internally by `hive_ipc_request()`)
- Waiting for multiple timer types (sync timer OR flight timer)
- State machines waiting for multiple event types

**Usage example**
```c
// Wait for either a sync timer or a landed notification
enum { FILTER_SYNC_TIMER, FILTER_LANDED };
hive_recv_filter_t filters[] = {
    [FILTER_SYNC_TIMER] = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, sync_timer},
    [FILTER_LANDED] = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_LANDED},
};
hive_message_t msg;
size_t matched;
hive_ipc_recv_matches(filters, 2, &msg, -1, &matched);
if (matched == FILTER_SYNC_TIMER) {
    // Handle sync timer
} else {
    // Handle landed notification
}
```

#### Request/Reply

```c
// Send REQUEST, block until REPLY with matching tag, or timeout
hive_status_t hive_ipc_request(actor_id_t to, const void *request, size_t req_len,
                      hive_message_t *reply, int32_t timeout_ms);

// Reply to a received REQUEST (extracts sender and tag from request automatically)
hive_status_t hive_ipc_reply(const hive_message_t *request, const void *data, size_t len);
```

**Request/reply implementation**
```c
// hive_ipc_request internally does:
// 1. Set up temporary monitor on target (to detect death)
// 2. Generate unique tag
// 3. Send message with class=REQUEST
// 4. Wait for REPLY with matching tag OR EXIT from monitor
// 5. Clean up monitor and return reply, HIVE_ERR_CLOSED, or timeout error

// hive_ipc_reply internally does:
// 1. Decode sender and tag from request
// 2. Send message with class=REPLY and same tag
```

**Error conditions for `hive_ipc_request()`**
- `HIVE_ERR_CLOSED`: Target actor died before sending a reply (detected via internal monitor)
- `HIVE_ERR_TIMEOUT`: No reply received within timeout period
- `HIVE_ERR_NOMEM`: Pool exhausted when sending request
- `HIVE_ERR_INVALID`: Invalid target actor ID or NULL request with non-zero length

**Target death detection** - `hive_ipc_request()` internally monitors the target actor for the duration of the request. If the target dies before replying, the function returns `HIVE_ERR_CLOSED` immediately without waiting for timeout:

```c
hive_message_t reply;
hive_status_t status = hive_ipc_request(target, &req, sizeof(req), &reply, 5000);
if (status.code == HIVE_ERR_CLOSED) {
    // Target died during request - no ambiguity with timeout
    printf("Target actor died\n");
} else if (status.code == HIVE_ERR_TIMEOUT) {
    // Target is alive but didn't reply in time
    printf("Request timed out\n");
}
```

This eliminates the "timeout but actually dead" ambiguity from previous versions.

**Concurrency constraint** - An actor can only have **one outstanding request at a time**. Since `hive_ipc_request()` blocks the caller until a reply arrives (or timeout), the actor cannot issue concurrent requests. To implement scatter/gather patterns, spawn multiple worker actors that each make one request.

### API Contract: hive_ipc_notify()

**Parameter validation**
- If `data == NULL && len > 0`: Returns `HIVE_ERR_INVALID`
- If `len > HIVE_MAX_MESSAGE_SIZE - 4` (252 bytes payload): Returns `HIVE_ERR_INVALID`
- Oversized messages are rejected immediately, not truncated

**Behavior when pools are exhausted**

`hive_ipc_notify()` uses two global pools:
1. **Mailbox entry pool** (`HIVE_MAILBOX_ENTRY_POOL_SIZE` = 256)
2. **Message data pool** (`HIVE_MESSAGE_DATA_POOL_SIZE` = 256)

**Fail-fast semantics**

- **Returns `HIVE_ERR_NOMEM` immediately** if either pool is exhausted
- **Does NOT block** waiting for pool space
- **Does NOT drop** messages silently
- **Atomic operation** - Either succeeds completely or fails

**Caller responsibilities**
- **MUST** check return value and handle `HIVE_ERR_NOMEM`
- **MUST** implement backpressure/retry logic if needed
- **MUST NOT** assume message was delivered if `HIVE_FAILED(status)`

**Example: Handling pool exhaustion**

```c
// Bad: Ignoring return value
hive_ipc_notify(target, HIVE_TAG_NONE, &data, sizeof(data));  // WRONG - message may be lost

// Good: Check and handle with backoff-retry
hive_status_t status = hive_ipc_notify(target, HIVE_TAG_NONE, &data, sizeof(data));
if (status.code == HIVE_ERR_NOMEM) {
    // Pool exhausted - backoff and retry
    hive_message_t msg;
    hive_ipc_recv(&msg, 10);  // Wait 10ms, process any incoming messages
    // Retry the send
    status = hive_ipc_notify(target, HIVE_TAG_NONE, &data, sizeof(data));
    if (HIVE_FAILED(status)) {
        // Still failing - drop message or take other action
    }
}
```

### Message Data Lifetime

**CRITICAL LIFETIME RULE**
- **Data is ONLY valid until the next successful `hive_ipc_recv()`, `hive_ipc_recv_match()`, or `hive_ipc_recv_matches()` call**
- **Per actor: only ONE message payload pointer is valid at a time**
- **Storing `msg.data` across receive iterations causes use-after-free**
- **If you need the data later, COPY IT IMMEDIATELY**

**Failed recv does NOT invalidate**
- If recv returns `HIVE_ERR_TIMEOUT` or `HIVE_ERR_WOULDBLOCK`, previous buffer remains valid
- Only a successful recv invalidates the previous pointer

**Correct pattern**
```c
hive_message_t msg;
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

**Capacity model**

Per-actor mailbox limits: **No per-actor quota** - capacity is constrained by global pools

Global pool limits: **Yes** - all actors share:
- `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256 default) - mailbox entries
- `HIVE_MESSAGE_DATA_POOL_SIZE` (256 default) - message data

**Important** - One slow receiver can consume all mailbox entries, starving other actors.

**Fairness guarantees** - The runtime does not provide per-actor fairness guarantees; resource exhaustion caused by a misbehaving actor is considered an application-level fault. Applications requiring protection against resource starvation can implement application-level quotas or monitoring.

### Selective Receive Semantics

`hive_ipc_recv_match()` and `hive_ipc_recv_matches()` implement selective receive. This is the key mechanism for building complex protocols like request/reply.

**Blocking behavior**

1. Scan mailbox from head for message matching all filter criteria
2. If found -> remove from mailbox, return immediately
3. If not found -> block, yield to scheduler
4. When any message arrives -> wake, rescan from head
5. If match -> return
6. If no match -> go back to sleep
7. Repeat until match or timeout

**Key properties**

- **Non-matching messages are NOT dropped** - they stay in mailbox
- **Order preserved** - messages remain in FIFO order
- **Later retrieval** - non-matching messages retrieved by subsequent `hive_ipc_recv()` calls
- **Scan complexity** - O(n) where n = mailbox depth

**Example: Waiting for specific reply**

```c
// Using hive_ipc_request() for request/reply (recommended - handles tag generation internally)
hive_message_t reply;
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

**When selective receive is efficient**

- Typical request/reply: mailbox is empty or near-empty while waiting for reply
- Shallow mailbox: O(n) scan is fast when n is small

**When selective receive is less efficient**

- Deep mailbox with many non-matching messages
- Example: 100 pending NOTIFYs while waiting for specific REPLY -> scans 100 messages

**Mitigation** - Process messages promptly. Don't let mailbox grow deep. The request/reply pattern naturally keeps mailbox shallow because you block waiting for reply.

### Message Ordering

**Mailbox implementation** - FIFO linked list (enqueue at tail, dequeue from head)

**Ordering guarantees**

**Single sender to single receiver**
- Messages are received in the order they were sent
- **FIFO guaranteed**
- Example: If actor A sends M1, M2, M3 to actor B, B receives them in order M1 -> M2 -> M3

**Multiple senders to single receiver**
- Message order depends on scheduling (which sender runs first)
- **Arrival order is scheduling-dependent**
- Example: If actor A sends M1 and actor B sends M2, receiver may get M1->M2 or M2->M1

**Selective receive and ordering**
- Selective receive can retrieve messages out of FIFO order
- Non-matching messages are skipped but not reordered
- Example: Mailbox has [NOTIFY, TIMER, REPLY]. Filtering for REPLY returns REPLY first.

**Consequences**

```c
// Single sender - FIFO guaranteed
void sender_actor(void *arg) {
    hive_ipc_notify(receiver, HIVE_TAG_NONE, &msg1, sizeof(msg1));  // Sent first
    hive_ipc_notify(receiver, HIVE_TAG_NONE, &msg2, sizeof(msg2));  // Sent second
    // Receiver will see: msg1, then msg2 (guaranteed)
}

// Multiple senders - order depends on scheduling
void sender_A(void *arg) {
    hive_ipc_notify(receiver, HIVE_TAG_NONE, &msgA, sizeof(msgA));
}
void sender_B(void *arg) {
    hive_ipc_notify(receiver, HIVE_TAG_NONE, &msgB, sizeof(msgB));
}
// Receiver may see: msgA then msgB, OR msgB then msgA

// Selective receive - can retrieve out of order
void request_reply_actor(void *arg) {
    // Start timer
    timer_id_t t;
    hive_timer_after(1000000, &t);

    // Do request/reply
    hive_message_t reply;
    hive_ipc_request(server, &req, sizeof(req), &reply, 5000);
    // Timer tick arrived during request/reply wait - it's in mailbox

    // Now process timer using selective receive with timer_id_t
    hive_message_t timer_msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, t, &timer_msg, 0);
}
```

### Timer and System Messages

Timer and system messages use the same mailbox and message format as IPC.

**Timer messages**
- Class: `HIVE_MSG_TIMER`
- Sender: Actor that owns the timer
- Tag: `timer_id_t`
- Payload: Empty (len = 0 after decoding header)

**System messages (exit notifications)**
- Class: `HIVE_MSG_EXIT`
- Sender: Actor that died
- Tag: `HIVE_TAG_NONE`
- Payload: `hive_exit_msg_t` struct

**Checking message type**

```c
hive_message_t msg;
hive_ipc_recv(&msg, -1);

switch (msg.class) {
    case HIVE_MSG_NOTIFY:
        handle_cast(msg.sender, msg.data, msg.len);
        break;
    case HIVE_MSG_REQUEST:
        handle_call_and_reply(&msg, msg.data, msg.len);
        break;
    case HIVE_MSG_TIMER:
        handle_timer_tick(msg.tag);  // tag is timer_id_t
        break;
    case HIVE_MSG_EXIT:
        handle_exit_notification(msg.sender, msg.data);
        break;
    default:
        break;
}
```

**Convenience function for timer checking**

```c
// Returns true if message is a timer tick
bool hive_msg_is_timer(const hive_message_t *msg);
```

### Query Functions

```c
// Check if any message is pending in current actor's mailbox
bool hive_ipc_pending(void);

// Count messages in current actor's mailbox
size_t hive_ipc_count(void);
```

**Semantics**
- Query the **current actor's** mailbox only
- Cannot query another actor's mailbox
- Return `false` / `0` if called outside actor context

### Pool Exhaustion Behavior

IPC uses two global pools shared by all actors:
- **Mailbox entry pool**: `HIVE_MAILBOX_ENTRY_POOL_SIZE` (256 default)
- **Message data pool**: `HIVE_MESSAGE_DATA_POOL_SIZE` (256 default)

**When pools are exhausted**
- `hive_ipc_notify()` returns `HIVE_ERR_NOMEM` immediately
- Send operation **does NOT block** waiting for space
- Send operation **does NOT drop** messages automatically
- Caller **must check** return value and handle failure

**No per-actor mailbox limit**: All actors share the global pools. A single actor can consume all available entries if receivers don't process messages.

**Mitigation strategies**
- Size pools appropriately: `1.5x peak concurrent messages`
- Check return values and implement retry logic or backpressure
- Use `hive_ipc_request()` for natural backpressure (sender waits for reply)
- Ensure receivers process messages promptly

**Backoff-retry example**
```c
hive_status_t status = hive_ipc_notify(target, HIVE_TAG_NONE, data, len);
if (status.code == HIVE_ERR_NOMEM) {
    // Pool exhausted - backoff before retry
    hive_message_t msg;
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
} hive_bus_config_t;
```

**Configuration constraints (normative)**
- `max_subscribers`: **Architectural limit of 32 subscribers** (enforced by 32-bit `readers_mask`)
  - Valid range: 1..32
  - `HIVE_MAX_BUS_SUBSCRIBERS = 32` is a hard architectural invariant, not a tunable parameter
  - Attempts to configure `max_subscribers > 32` return `HIVE_ERR_INVALID`
- `consume_after_reads`: Valid range: 0..max_subscribers
- Subscriber index assignment is stable for the lifetime of a subscription (affects `readers_mask` bit position)

### Functions

```c
// Create bus
hive_status_t hive_bus_create(const hive_bus_config_t *cfg, bus_id_t *out);

// Destroy bus (fails if subscribers exist)
hive_status_t hive_bus_destroy(bus_id_t bus);

// Publish data
hive_status_t hive_bus_publish(bus_id_t bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
hive_status_t hive_bus_subscribe(bus_id_t bus);
hive_status_t hive_bus_unsubscribe(bus_id_t bus);

// Read entry (non-blocking)
// Returns HIVE_ERR_WOULDBLOCK if no data available
hive_status_t hive_bus_read(bus_id_t bus, void *buf, size_t max_len, size_t *bytes_read);

// Read with blocking
hive_status_t hive_bus_read_wait(bus_id_t bus, void *buf, size_t max_len,
                               size_t *bytes_read, int32_t timeout_ms);

// Query bus state
size_t hive_bus_entry_count(bus_id_t bus);
```

**Message size validation**

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
- `*bytes_read` returns the **actual bytes copied** (truncated length)
- Returns `HIVE_ERR_TRUNCATED` when truncation occurs (data was still read successfully)
- Caller can check for truncation: `if (status.code == HIVE_ERR_TRUNCATED) { /* handle */ }`

### Bus Consumption Model (Semantic Contract)

The bus implements **per-subscriber read cursors** with the following **three contractual rules**:

---

#### **RULE 1: Subscription Start Position**

**Contract** - `hive_bus_subscribe()` initializes the subscriber's read cursor to **"next publish"** (current `bus->head` write position).

**Guaranteed semantics**
- Subscriber **CANNOT** read retained entries published before subscription
- Subscriber **ONLY** sees entries published **after** `hive_bus_subscribe()` returns
- First `hive_bus_read()` call returns `HIVE_ERR_WOULDBLOCK` if no new entries published since subscription
- Implementation: `subscriber.next_read_idx = bus->head`

**Implications**
- New subscribers do NOT see history
- If you need to read retained entries, subscribe **before** publishing starts
- Late subscribers will miss all prior messages

**Example**
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

**Contract** - Each subscriber has an independent read cursor. Slow subscribers may miss entries due to buffer wraparound; no error or notification is generated. The bus implementation supports a maximum of 32 concurrent subscribers per bus, enforced by the 32-bit `readers_mask`.

**Guaranteed semantics**
1. **Storage per subscriber**
   - `bus_subscriber` struct with `next_read_idx` field (tracks next entry to read)
   - Each subscriber reads at their own pace independently
   - Storage cost: **O(max_subscribers)** fixed overhead

2. **Storage per entry**
   - 32-bit `readers_mask` bitmask (max 32 subscribers per bus)
   - Bit N set -> subscriber N has read this entry
   - Storage cost: **O(1)** per entry (4 bytes bitmask + 1 byte read_count)

3. **Eviction behavior (buffer full)**
   - When `hive_bus_publish()` finds buffer full (`count >= max_entries`):
     - Oldest entry at `bus->tail` is **evicted immediately** (freed from message pool)
     - Tail advances: `bus->tail = (bus->tail + 1) % max_entries`
     - **No check if subscribers have read the evicted entry**
   - If slow subscriber's cursor pointed to evicted entry:
     - On next `hive_bus_read()`, search starts from **current `bus->tail`** (oldest surviving entry)
     - Subscriber **silently skips** to next available unread entry
     - **No error** returned (appears as normal read)
     - **No signal** of data loss (by design)

**Implications**
- Slow subscribers lose data without notification
- Fast subscribers never lose data (assuming buffer sized for publish rate)
- No backpressure mechanism (unlike `hive_ipc_request()` request/reply pattern)
- Real-time principle: Prefer fresh data over old data

**Example (data loss)**
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

**Eviction does NOT notify slow subscribers**
- No `HIVE_ERR_OVERFLOW` or similar
- No special message indicating data loss
- Application must detect via message sequence numbers if needed

---

#### **RULE 3: consume_after_reads Counting Semantics**

**Contract** - `consume_after_reads` counts **unique subscribers** who have read an entry, **NOT** total reads.

**Guaranteed semantics**
- Each entry has a `readers_mask` bitmask (32 bits, max 32 subscribers per bus)
- When subscriber N reads an entry:
  1. Check if bit N is set in `readers_mask` -> if yes, skip (already read)
  2. If no, set bit N, increment `read_count`, return entry
- Entry is removed when `read_count >= consume_after_reads` (N unique subscribers have read)
- Same subscriber CANNOT read the same entry twice (deduplication)

**Implementation mechanism**
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

**Implications**
- `consume_after_reads=3` means "remove after 3 **different** subscribers read it"
- If subscriber A reads entry twice (e.g., re-subscribes), that's still 1 read
- If 3 subscribers each read entry once, that's 3 reads -> entry removed
- Set `consume_after_reads=0` to disable (entry persists until aged out or evicted)

**Example (unique counting)**
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

1. **consume_after_reads (unique subscriber counting)**
   - Entry removed when `read_count >= consume_after_reads` (N unique subscribers have read it)
   - Value `0` = disabled (entry persists until aged out or evicted)
   - See **RULE 3** above for exact counting semantics

2. **max_age_ms (time-based expiry)**
   - Entry removed when `(current_time_ms - entry.timestamp_ms) >= max_age_ms`
   - Value `0` = disabled (no time-based expiry)
   - Checked on every `hive_bus_read()` and `hive_bus_publish()` call

3. **Buffer full (forced eviction)**
   - Oldest entry at `bus->tail` evicted when `count >= max_entries` on publish
   - ALWAYS enabled (cannot be disabled)
   - See **RULE 2** above for eviction semantics

**Typical configurations**

| Use Case | consume_after_reads | max_age_ms | Behavior |
|----------|-------------|------------|----------|
| **Sensor data** | `0` | `100` | Multiple readers, data stale after 100ms |
| **Configuration** | `0` | `0` | Persistent until buffer wraps |
| **Events** | `N` | `0` | Consumed after N subscribers read, no timeout |
| **Recent history** | `0` | `5000` | Keep 5 seconds of history, multiple readers |

**Interaction of mechanisms**
- Entry is removed when **FIRST** condition is met (OR, not AND)
- Example: `consume_after_reads=3, max_age_ms=1000`
  - Entry removed after 3 subscribers read it, **OR**
  - Entry removed after 1 second, **OR**
  - Entry removed when buffer full (forced eviction)

### Pool Exhaustion and Buffer Full Behavior

**WARNING: Resource Contention Between IPC and Bus**

Bus publishing consumes the same message data pool as IPC (`HIVE_MESSAGE_DATA_POOL_SIZE`). A misconfigured or high-rate bus can exhaust the message pool and cause **all** IPC sends to fail with `HIVE_ERR_NOMEM`, potentially starving critical actor communication.

**Architectural consequences**
- Bus auto-evicts oldest entries when its ring buffer fills (graceful degradation)
- IPC never auto-drops (fails immediately with `HIVE_ERR_NOMEM`)
- A single high-rate bus publisher can starve IPC globally
- No per-subsystem quotas or fairness guarantees

**Design implications**
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

**Key Differences from IPC**
- IPC never drops messages automatically (returns error instead)
- Bus automatically drops oldest entry when ring buffer is full
- Both share the same message data pool (`HIVE_MESSAGE_DATA_POOL_SIZE`)

**Mitigation strategies**
- Size message pool appropriately for combined IPC + bus load
- Configure per-bus `max_entries` based on publish rate vs read rate
- Use retention policies (`consume_after_reads`, `max_age_ms`) to prevent accumulation
- Monitor `hive_bus_entry_count()` to detect slow readers

## Unified Event Waiting API

`hive_select()` provides a unified primitive for waiting on multiple event sources (IPC messages + bus data). The existing blocking APIs (`hive_ipc_recv*`, `hive_bus_read_wait`) are implemented as thin wrappers around this primitive.

### Types

```c
// Source types
typedef enum {
    HIVE_SEL_IPC,  // Wait for IPC message
    HIVE_SEL_BUS,  // Wait for bus data
} hive_select_type;

// Select source (tagged union)
typedef struct {
    hive_select_type type;
    union {
        hive_recv_filter_t ipc;  // For HIVE_SEL_IPC
        bus_id_t bus;            // For HIVE_SEL_BUS
    };
} hive_select_source_t;

// Select result
typedef struct {
    size_t index;           // Which source triggered (0-based)
    hive_select_type type;  // Type of triggered source
    union {
        hive_message_t ipc;   // For HIVE_SEL_IPC
        struct {
            void *data;     // For HIVE_SEL_BUS
            size_t len;
        } bus;
    };
} hive_select_result_t;
```

### Function

```c
// Wait on multiple sources
// Returns when any source has data or timeout expires
// timeout_ms == 0:  non-blocking, returns HIVE_ERR_WOULDBLOCK if no data
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns HIVE_ERR_TIMEOUT if exceeded
hive_status_t hive_select(const hive_select_source_t *sources, size_t num_sources,
                        hive_select_result_t *result, int32_t timeout_ms);
```

### Priority Semantics

When multiple sources have data ready simultaneously, sources are checked in **strict array order**. The first ready source wins. There is no type-based priority - bus and IPC sources are treated equally.

To prioritize certain sources, place them earlier in the array.

### Bus Readiness Semantics

A bus source is "ready" when the bus contains an entry the calling actor has not yet read. Each subscriber's read position is tracked independently via a bitmask. This means:

- A bus can have data for one subscriber but not another
- Reading an entry marks it as read for that subscriber only
- New publishes are unread for all subscribers until consumed

### Example Usage

```c
// Define event sources
enum { SEL_SENSOR, SEL_TIMER, SEL_COMMAND };
hive_select_source_t sources[] = {
    [SEL_SENSOR]  = {HIVE_SEL_BUS, .bus = sensor_bus},
    [SEL_TIMER]   = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, heartbeat}},
    [SEL_COMMAND] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_SHUTDOWN}},
};

while (running) {
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 3, &result, 1000);

    if (status.code == HIVE_ERR_TIMEOUT) {
        // No events for 1 second
        continue;
    }
    if (HIVE_FAILED(status)) {
        break;
    }

    switch (result.index) {
    case SEL_SENSOR:
        // Bus data available
        sensor_data *data = (sensor_data *)result.bus.data;
        process_sensor(data);
        break;

    case SEL_TIMER:
        // Heartbeat timer
        send_heartbeat();
        break;

    case SEL_COMMAND:
        // Command received
        running = false;
        break;
    }
}
```

### Wrapper Relationship

The existing blocking APIs are thin wrappers around `hive_select()`:

| Wrapper Function | Equivalent hive_select() |
|------------------|--------------------------|
| `hive_ipc_recv()` | Single IPC source with wildcard filter |
| `hive_ipc_recv_match()` | Single IPC source with specific filter |
| `hive_ipc_recv_matches()` | Multiple IPC sources |
| `hive_bus_read_wait()` | Single bus source |

This architectural design ensures consistent blocking behavior and wake-up logic across all APIs.

### Error Handling

| Error Code | Condition |
|------------|-----------|
| `HIVE_ERR_INVALID` | NULL sources/result, num_sources == 0, bus not subscribed |
| `HIVE_ERR_WOULDBLOCK` | timeout_ms == 0 and no data available |
| `HIVE_ERR_TIMEOUT` | timeout_ms > 0 and no data within timeout |

### Implementation Notes

- **Data lifetime** - All data in `result` (both `result.ipc` and `result.bus.data`) is valid until the next blocking call: `hive_select()`, `hive_ipc_recv*()`, or `hive_bus_read*()`. Copy immediately if needed longer.
- **Wake mechanism** - When blocked, the actor is woken by bus publishers (via `blocked` flag) or IPC senders (via `select_sources` check in mailbox wake logic).

## Timer API

Timers for periodic and one-shot wake-ups.

```c
// One-shot: wake current actor after delay
hive_status_t hive_timer_after(uint32_t delay_us, timer_id_t *out);

// Periodic: wake current actor every interval
hive_status_t hive_timer_every(uint32_t interval_us, timer_id_t *out);

// Cancel timer
hive_status_t hive_timer_cancel(timer_id_t id);

// Sleep for specified duration (microseconds)
// Uses selective receive - other messages remain in mailbox
hive_status_t hive_sleep(uint32_t delay_us);

// Get current time in microseconds (monotonic)
// Returns monotonic time suitable for measuring elapsed durations.
// In simulation mode, returns simulated time.
uint64_t hive_get_time(void);

// Check if message is a timer tick
bool hive_msg_is_timer(const hive_message_t *msg);
```

Timer wake-ups are delivered as messages with `class == HIVE_MSG_TIMER`. The tag contains the `timer_id_t`. The actor receives these in its normal `hive_ipc_recv()` loop and can use `hive_msg_is_timer()` to identify timer messages.

**Important** - When waiting for a specific timer, use selective receive with the timer_id_t as the tag filter:
```c
timer_id_t my_timer;
hive_timer_after(500000, &my_timer);
hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, my_timer, &msg, -1);
```
Do **not** use `HIVE_TAG_ANY` for timer messages - this could consume the wrong timer's message if multiple timers are active.

### Timer Tick Coalescing (Periodic Timers)

**Behavior** When the scheduler reads a timerfd, it obtains an expiration count (how many intervals elapsed since last read). The runtime sends **exactly one tick message** regardless of the expiration count.

**Rationale**
- Simplicity: Actor receives predictable single-message notification
- Real-time principle: Current state matters more than history
- Memory efficiency: No risk of mailbox flooding from fast timers

**Implications**
- If scheduler is delayed (file I/O stall, long actor computation), periodic timer ticks are **coalesced**
- Actor cannot determine how many intervals actually elapsed
- For precise tick counting, use `hive_get_time()` to measure elapsed time

**Example**
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

**Alternative not implemented** - Enqueuing N tick messages for N expirations was rejected because:
- Risk of mailbox overflow for fast timers
- Most embedded use cases don't need tick counting
- Actors needing precise counts can use `hive_get_time()` to measure elapsed time

### Timer Precision and Monotonicity

**Unit mismatch by design**
- Timer API uses **microseconds** (`uint32_t delay_us`, `uint32_t interval_us`)
- Rest of system (IPC, file, network) uses **milliseconds** (`int32_t timeout_ms`)
- Rationale: Timers often need sub-millisecond precision; I/O timeouts rarely do

**Resolution and precision**

Platform | Clock Source | API Precision | Actual Precision | Notes
---------|-------------|---------------|------------------|------
**Linux (x86-64)** | `CLOCK_MONOTONIC` via `timerfd` | Nanosecond (`itimerspec`) | ~1 ms typical | Kernel-limited, non-realtime scheduler
**STM32 (ARM)** | Hardware timer (SysTick/TIM) | Microsecond | ~1-10 us typical | Depends on timer configuration

- On Linux, timers use `CLOCK_MONOTONIC` clock source via `timerfd_create()`
- On Linux, requests < 1ms may still fire with ~1ms precision due to kernel scheduling
- On STM32, hardware timers provide microsecond-level precision

**Monotonic clock guarantee**
- Uses **monotonic clock** on both platforms (CLOCK_MONOTONIC on Linux, hardware timer on STM32)
- **NOT affected by**:
  - System time changes (NTP adjustments, user setting clock)
  - Timezone changes
  - Daylight saving time
- **Guaranteed to**:
  - Never go backwards
  - Count elapsed time accurately (subject to clock drift, typically <100 ppm)
- **Use case**: Timers measure **elapsed time**, not wall-clock time

**hive_get_time() precision**

`hive_get_time()` returns the current monotonic time in microseconds:

Platform | Implementation | Resolution | Notes
---------|----------------|------------|------
**Linux** | `clock_gettime(CLOCK_MONOTONIC)` | Nanosecond | vDSO, minimal overhead
**STM32** | `tick_count * HIVE_TIMER_TICK_US` | 1 ms default | Limited by tick rate

- Linux: Uses vDSO for low-overhead system call (no kernel trap in most cases)
- STM32: Resolution limited by `HIVE_TIMER_TICK_US` (default 1000us = 1ms)
- For sub-millisecond precision on STM32, reduce `HIVE_TIMER_TICK_US` or use DWT cycle counter
- In simulation mode, returns simulated time (advanced by `hive_timer_advance_time()`)

**Delivery guarantee**
- Timer callbacks are delivered **at or after** the requested time; early delivery never occurs
- Delays may occur due to scheduling (higher priority actors, blocked scheduler)
- In safety-critical systems, timers provide lower bounds on timing, not exact timing

**Wraparound behavior**

1. **Timer interval wraparound** (`uint32_t interval_us`):
   - Type: 32-bit unsigned integer
   - Max value: 4,294,967,295 microseconds = **~4295 seconds** = **~71.6 minutes**
   - Wraparound: Values > 71.6 minutes wrap around (e.g., 72 minutes becomes 24 seconds)
   - **Mitigation**: Use multiple timers or external tick counting for intervals > 1 hour

2. **Timer ID wraparound** (`timer_id_t` = `uint32_t`):
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

**Comparison with rest of system**

Feature | Timer API | IPC/File/Network API
--------|-----------|---------------------
**Units** | Microseconds (`uint32_t`) | Milliseconds (`int32_t`)
**Max value** | ~71 minutes | ~24.8 days (2^31 ms)
**Signed/Unsigned** | Unsigned (always positive) | Signed (negative = block forever)
**Wraparound** | At ~71 minutes | At ~24.8 days (unlikely in practice)

**Design rationale**
- Microseconds for timers: Sub-millisecond intervals common in embedded systems (sensor sampling, PWM, etc.)
- Milliseconds for I/O: Network/file timeouts rarely need microsecond precision
- 32-bit limit: Embedded systems prefer fixed-size types; 64-bit would waste memory
- Trade-off: Accept wraparound at ~71 minutes for memory efficiency

## Supervisor API

Supervision for automatic child actor restart. A supervisor is an actor that monitors children and restarts them according to configurable policies.

```c
#include "hive_supervisor.h"

// Start supervisor with configuration
hive_status_t hive_supervisor_start(const hive_supervisor_config_t *config,
                                  const actor_config_t *sup_actor_cfg,
                                  actor_id_t *out_supervisor);

// Request graceful shutdown (async)
hive_status_t hive_supervisor_stop(actor_id_t supervisor);

// Utility functions
const char *hive_restart_strategy_str(hive_restart_strategy_t strategy);
const char *hive_child_restart_str(hive_child_restart_t restart);
```

### Child Restart Types

```c
typedef enum {
    HIVE_CHILD_PERMANENT,  // Always restart, regardless of exit reason
    HIVE_CHILD_TRANSIENT,  // Restart only on abnormal exit (crash)
    HIVE_CHILD_TEMPORARY   // Never restart
} hive_child_restart_t;
```

| Type | Normal Exit | Crash |
|------|-------------|-------|
| `PERMANENT` | Restart | Restart |
| `TRANSIENT` | No restart | Restart |
| `TEMPORARY` | No restart | No restart |

### Restart Strategies

```c
typedef enum {
    HIVE_STRATEGY_ONE_FOR_ONE,   // Restart only the failed child
    HIVE_STRATEGY_ONE_FOR_ALL,   // Restart all children when one fails
    HIVE_STRATEGY_REST_FOR_ONE   // Restart failed child and all started after it
} hive_restart_strategy_t;
```

**one_for_one**: When a child crashes, only that child is restarted. Other children continue running undisturbed. Use for independent workers.

**one_for_all**: When any child crashes, all children are stopped and restarted. Use when children have strong interdependencies.

**rest_for_one**: When a child crashes, that child and all children started after it are restarted. Children started before continue running. Use for pipelines or sequential dependencies.

### Child Specification

```c
typedef struct {
    actor_fn_t start;              // Actor entry point
    hive_actor_init_fn_t init;     // Init function (NULL = skip)
    void *init_args;             // Arguments to init/actor
    size_t init_args_size;       // Size to copy (0 = pass pointer directly)
    const char *name;            // Child identifier for tracking
    bool auto_register;          // Auto-register in name registry
    hive_child_restart_t restart;  // Restart policy
    actor_config_t actor_cfg;      // Actor configuration (stack size, priority, etc.)
} hive_child_spec_t;
```

**Argument handling**
- If `init_args_size > 0`: Argument is copied to supervisor-managed storage (max `HIVE_MAX_MESSAGE_SIZE` bytes). Safe for stack-allocated arguments.
- If `init_args_size == 0`: Pointer is passed directly. Caller must ensure lifetime.

**Two-phase child start**
1. All children are allocated (actor structures created)
2. Sibling info array is built with all child names and IDs
3. All children are started, each receiving the complete sibling array

This allows children to discover sibling actor IDs at startup without using the name registry.

### Supervisor Configuration

```c
typedef struct {
    hive_restart_strategy_t strategy;  // How to handle child failures
    uint32_t max_restarts;           // Max restarts in period (0 = unlimited)
    uint32_t restart_period_ms;      // Time window for max_restarts
    const hive_child_spec_t *children; // Array of child specifications
    size_t num_children;             // Number of children
    void (*on_shutdown)(void *ctx);  // Called when supervisor shuts down
    void *shutdown_ctx;              // Context for shutdown callback
} hive_supervisor_config_t;

#define HIVE_SUPERVISOR_CONFIG_DEFAULT {           \
    .strategy = HIVE_STRATEGY_ONE_FOR_ONE,         \
    .max_restarts = 3,                             \
    .restart_period_ms = 5000,                     \
    .children = NULL, .num_children = 0,           \
    .on_shutdown = NULL, .shutdown_ctx = NULL      \
}
```

### Restart Intensity

The supervisor tracks restart attempts within a sliding time window. If `max_restarts` is exceeded within `restart_period_ms` milliseconds, the supervisor gives up and shuts down (with normal exit).

- `max_restarts = 0`: Unlimited restarts (never give up)
- `max_restarts = 5, restart_period_ms = 10000`: Allow 5 restarts in 10 seconds

**Algorithm** - The supervisor maintains a ring buffer of the last `HIVE_MAX_SUPERVISOR_CHILDREN` restart timestamps. On each restart attempt:
1. Record current timestamp in ring buffer
2. Count entries where `now - timestamp <= restart_period_ms`
3. If `count >= max_restarts`, intensity exceeded -> supervisor shuts down

**Rationale** - Prevents infinite restart loops when a child has a persistent bug (e.g., crashes immediately on startup).

### Functions

**hive_supervisor_start(config, sup_actor_cfg, out_supervisor)**

Creates a new supervisor actor that immediately spawns and monitors all specified children.

| Parameter | Description |
|-----------|-------------|
| `config` | Supervisor configuration (children, strategy, intensity) |
| `sup_actor_cfg` | Optional actor config for supervisor itself (NULL = defaults) |
| `out_supervisor` | Receives supervisor's actor ID |

Returns:
- `HIVE_OK`: Supervisor started, all children spawned
- `HIVE_ERR_INVALID`: NULL config, NULL out_supervisor, too many children, NULL children with non-zero count, NULL child function
- `HIVE_ERR_NOMEM`: No supervisor slots available or spawn failed

**hive_supervisor_stop(supervisor)**

Sends asynchronous stop request to supervisor. The supervisor will:
1. Stop all children (in reverse start order)
2. Call `on_shutdown` callback if configured
3. Exit normally

Use `hive_monitor()` to be notified when shutdown completes.

Returns:
- `HIVE_OK`: Stop request sent
- `HIVE_ERR_INVALID`: Invalid supervisor ID
- `HIVE_ERR_NOMEM`: Failed to send shutdown message

### Compile-Time Configuration

```c
// hive_static_config.h
#define HIVE_MAX_SUPERVISOR_CHILDREN 16  // Max children per supervisor
#define HIVE_MAX_SUPERVISORS 8           // Max concurrent supervisors
```

### Implementation Notes

**Architecture** - The supervisor is a regular actor using `hive_monitor()` to watch children. When a monitored child dies, the supervisor receives an EXIT message and applies the restart strategy.

**Memory** - Supervisor state allocated from static pool (no malloc). Child arguments copied to fixed-size storage within supervisor state.

**Monitor pool usage** - Each child consumes one entry from `HIVE_MONITOR_ENTRY_POOL_SIZE`. Plan pool sizes accordingly.

**Shutdown callback** - Called from supervisor actor context just before exit. All children already stopped when callback runs.

**hive_kill()** - Supervisors use `hive_kill(target)` to terminate children during shutdown or when applying `one_for_all`/`rest_for_one` strategies.

### Restart Semantics

**A restarted child starts with a clean slate.** It MUST NOT assume:

- Preserved mailbox state (mailbox is empty)
- Preserved bus cursor position (must re-subscribe)
- Preserved timer IDs (old timers cancelled, must create new ones)
- Preserved actor_id_t (new ID assigned on restart)
- Preserved monitor/link state (must re-establish)

The only state preserved across restarts is the argument passed to the child function (copied by the supervisor at configuration time).

**External Resources** - A restarted child MUST treat all external handles as invalid and reacquire them. This includes:

- File descriptors
- Sockets
- HAL handles
- Device state
- Shared-memory pointers not owned by the runtime

Failure to do so causes "works in simulation, dies on hardware" bugs because resource lifetime is not actor lifetime.

**Name Registration** - A supervised child intended to be discoverable MUST:

- Register its name during startup (call `hive_register()` early)
- Tolerate name lookup from other actors at any time

**Client Rule** - Actors communicating with supervised children MUST NOT cache `actor_id_t` across awaits, timeouts, or receive calls. They MUST re-resolve by name (`hive_whereis()`) on each interaction or after any failure signal (timeout, EXIT message).

This prevents the classic bug: client caches ID -> server restarts -> client sends to dead ID -> silent failure or mysterious behavior.

### Restart Contract Checklist

**On child restart, these are always reset**

- [ ] Mailbox empty
- [ ] Bus subscriptions gone
- [ ] Bus cursors reset (fresh subscribe required)
- [ ] Timers cancelled
- [ ] Links and monitors cleared
- [ ] actor_id_t changes
- [ ] Name registration removed (must re-register)
- [ ] External handles invalid (must reacquire)

**Supervisor guarantees**

- [ ] Restart order is deterministic: child spec order
- [ ] Restart strategy applied exactly as defined (no hidden backoff)
- [ ] Intensity limit is deterministic: when exceeded, supervisor shuts down, no further restarts
- [ ] Supervisor never uses heap in hot paths; child-arg copies are bounded and static

**Failure visibility**

- [ ] Every restart attempt is observable (log)
- [ ] Every give-up is observable (log + shutdown callback)

### Example

```c
#include "hive_supervisor.h"

void worker(void *args, const hive_spawn_info_t *siblings, size_t sibling_count) {
    int id = *(int *)args;
    printf("Worker %d started\n", id);

    // Can find siblings by name
    const hive_spawn_info_t *peer = hive_find_sibling(siblings, sibling_count, "worker-1");
    if (peer) {
        printf("Found sibling worker-1 at actor %u\n", peer->id);
    }

    // Do work, may crash...

    hive_exit();
}

void orchestrator(void *args, const hive_spawn_info_t *siblings, size_t sibling_count) {

    // Define children
    static int ids[3] = {0, 1, 2};
    hive_child_spec_t children[3] = {
        {.start = worker, .init = NULL, .init_args = &ids[0],
         .init_args_size = sizeof(int), .name = "worker-0",
         .auto_register = false, .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = worker, .init = NULL, .init_args = &ids[1],
         .init_args_size = sizeof(int), .name = "worker-1",
         .auto_register = true, .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = worker, .init = NULL, .init_args = &ids[2],
         .init_args_size = sizeof(int), .name = "worker-2",
         .auto_register = false, .restart = HIVE_CHILD_TRANSIENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };

    // Configure supervisor
    hive_supervisor_config_t cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ONE,
        .max_restarts = 5,
        .restart_period_ms = 10000,
        .children = children,
        .num_children = 3,
    };

    // Start supervisor
    actor_id_t supervisor;
    hive_supervisor_start(&cfg, NULL, &supervisor);

    // Monitor supervisor to know when it exits
    uint32_t monitor_id;
    hive_monitor(supervisor, &monitor_id);

    // Wait for supervisor exit (intensity exceeded or explicit stop)
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg, -1);

    hive_exit();
}
```

## Network API

Non-blocking network I/O with blocking wrappers.

```c
// Socket management
hive_status_t hive_net_listen(uint16_t port, int *fd_out);
hive_status_t hive_net_accept(int listen_fd, int *conn_fd_out, int32_t timeout_ms);
hive_status_t hive_net_connect(const char *ip, uint16_t port, int *fd_out, int32_t timeout_ms);
hive_status_t hive_net_close(int fd);

// Data transfer
hive_status_t hive_net_recv(int fd, void *buf, size_t len, size_t *received, int32_t timeout_ms);
hive_status_t hive_net_send(int fd, const void *buf, size_t len, size_t *sent, int32_t timeout_ms);
```

**DNS resolution is out of scope.** The `ip` parameter must be a numeric IPv4 address (e.g., "192.168.1.1"). Hostnames are not supported. Rationale:
- DNS resolution (`getaddrinfo`, `gethostbyname`) is blocking and would stall the scheduler
- On STM32 bare metal, DNS typically unavailable or requires complex async plumbing
- Callers needing DNS should resolve externally before calling `hive_net_connect()`

All functions with `timeout_ms` parameter support **timeout enforcement**:

- `timeout_ms == 0`: Non-blocking, returns `HIVE_ERR_WOULDBLOCK` if would block
- `timeout_ms < 0`: Block forever until I/O completes
- `timeout_ms > 0`: Block up to timeout, returns `HIVE_ERR_TIMEOUT` if exceeded

**Timeout implementation** - Uses timer-based enforcement (consistent with `hive_ipc_recv`). When timeout expires, a timer message wakes the actor and `HIVE_ERR_TIMEOUT` is returned. This is essential for handling unreachable hosts, slow connections, and implementing application-level keepalives.

On blocking calls, the actor yields to the scheduler. The scheduler's event loop registers the I/O operation with the platform's event notification mechanism (epoll on Linux, interrupt flags on STM32) and dispatches the operation when the socket becomes ready.

### Completion Semantics

**`hive_net_recv()` - Partial completion**
- Returns successfully when **at least 1 byte** is read (or 0 for EOF/peer closed)
- Does NOT loop until `len` bytes are received
- `*received` contains actual bytes read (may be less than `len`)
- Caller must loop if full message is required:
```c
size_t total = 0;
while (total < expected_len) {
    size_t n;
    hive_status_t s = hive_net_recv(fd, buf + total, expected_len - total, &n, timeout);
    if (HIVE_FAILED(s)) return s;
    if (n == 0) return HIVE_ERROR(HIVE_ERR_IO, "Connection closed");
    total += n;
}
```

**`hive_net_send()` - Partial completion**
- Returns successfully when **at least 1 byte** is written
- Does NOT loop until `len` bytes are sent
- `*sent` contains actual bytes written (may be less than `len`)
- Caller must loop if full buffer must be sent:
```c
size_t total = 0;
while (total < len) {
    size_t n;
    hive_status_t s = hive_net_send(fd, buf + total, len - total, &n, timeout);
    if (HIVE_FAILED(s)) return s;
    total += n;
}
```

**`hive_net_connect()` - Async connect completion**
- If `connect()` returns `EINPROGRESS`: registers for `EPOLLOUT`, actor yields
- When socket becomes writable: checks `getsockopt(fd, SOL_SOCKET, SO_ERROR, ...)`
- If `SO_ERROR == 0`: success, returns `HIVE_SUCCESS` with connected socket in `*fd_out`
- If `SO_ERROR != 0`: returns `HIVE_ERR_IO` with error message, socket is closed
- If timeout expires before writable: returns `HIVE_ERR_TIMEOUT`, socket is closed

**`hive_net_accept()` - Connection ready**
- Waits for `EPOLLIN` on listen socket (incoming connection ready)
- Calls `accept()` to obtain connected socket
- Returns `HIVE_SUCCESS` with new socket in `*conn_fd_out`

**Rationale for partial completion**
- Matches POSIX socket semantics (recv/send may return partial)
- Avoids hidden loops that could block indefinitely
- Caller controls retry policy and timeout behavior
- Simpler implementation, more predictable behavior

## File API

File I/O operations.

> **Note** - File I/O is synchronous and briefly pauses the scheduler. This is fine for short operations - use `LOW` priority actors for file work. See [Scheduler-Stalling Calls](design.md#scheduler-stalling-calls) for details.

```c
hive_status_t hive_file_open(const char *path, int flags, int mode, int *fd_out);
hive_status_t hive_file_close(int fd);

hive_status_t hive_file_read(int fd, void *buf, size_t len, size_t *bytes_read);
hive_status_t hive_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read);

hive_status_t hive_file_write(int fd, const void *buf, size_t len, size_t *bytes_written);
hive_status_t hive_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written);

hive_status_t hive_file_sync(int fd);

hive_status_t hive_file_mount_available(const char *path);
```

### Mount Availability

`hive_file_mount_available()` checks if a mount point is ready for I/O. This is useful for checking SD card presence before attempting file operations.

**Returns:**
- `HIVE_OK`: Mount exists and backend is ready
- `HIVE_ERR_INVALID`: No mount for path
- `HIVE_ERR_IO`: Mount exists but backend unavailable (e.g., SD card not inserted)

**Usage example:**
```c
if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
    hive_file_open("/sd/flight.bin", HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);
} else {
    hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);
}
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

**Linux**
- Standard filesystem paths (e.g., `/tmp/log.bin`)
- Uses POSIX `open()`, `read()`, `write()`, `fsync()`
- Synchronous blocking I/O
- All flags and functions fully supported

**STM32**
- Virtual file paths mapped to flash sectors (e.g., `/log`)
- Board configuration via -D flags (each virtual file is optional):
  ```makefile
  # /log virtual file (required for flight logging)
  CFLAGS += -DHIVE_VFILE_LOG_BASE=0x08020000
  CFLAGS += -DHIVE_VFILE_LOG_SIZE=131072
  CFLAGS += -DHIVE_VFILE_LOG_SECTOR=5
  # /config virtual file (optional, for calibration data)
  # CFLAGS += -DHIVE_VFILE_CONFIG_BASE=0x08040000
  # CFLAGS += -DHIVE_VFILE_CONFIG_SIZE=16384
  # CFLAGS += -DHIVE_VFILE_CONFIG_SECTOR=6
  ```
- Ring buffer for efficient writes (most are O(1), blocks to flush when full)
- `HIVE_O_TRUNC` triggers flash sector erase (blocks 1-4 seconds)
- `hive_file_sync()` drains ring buffer to flash (blocking)
- No data loss - writes block when buffer is full to ensure delivery

**STM32 Write Behavior**

The STM32 implementation uses a ring buffer for efficiency. Most writes complete
immediately. When the buffer fills up, `write()` blocks to flush data to flash
before continuing. This ensures the same no-data-loss semantics as Linux, while
still providing fast writes in the common case.

| Platform | Behavior | Data Loss |
|----------|----------|-----------|
| Linux | Blocking | Never (or error) |
| STM32 | Fast (ring buffer), blocks when full | Never (or error) |

**STM32 API Restrictions**

| Feature | Linux | STM32 |
|---------|-------|-------|
| Arbitrary paths | Yes | No - only virtual paths (board-defined, e.g., `/log`) |
| `hive_file_read()` | Works | **Returns error** - use `pread()` |
| `hive_file_pread()` | Works | Works (direct flash read) |
| `hive_file_write()` | Blocking | Ring buffer (fast, blocks when full) |
| `hive_file_pwrite()` | Works | **Returns error** |
| Multiple writers | Yes | No - single writer at a time |

**STM32 Flag Restrictions**

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

**STM32 Usage Example**
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

### STM32 SD Card Support (Optional)

When built with `HIVE_ENABLE_SD=1`, the STM32 implementation supports SD cards via SPI using the FatFS library. SD card files are accessed through the `/sd` mount point.

**SD Card Characteristics:**
- Full FatFS filesystem support (FAT12/16/32, exFAT)
- Standard file operations work as expected (read, write, seek)
- `HIVE_O_RDWR` is supported (unlike flash virtual files)
- No `HIVE_O_TRUNC` requirement for writes
- Multiple files can be open simultaneously (up to `HIVE_MAX_SD_FILES`)

**SD Card Limitations:**
- **No hot-plug detection** - Card presence is checked at initialization only. If the card is removed during operation, subsequent I/O fails with `HIVE_ERR_IO`.
- **No automatic directory creation** - `hive_file_open()` with `HIVE_O_CREAT` does not create parent directories. Use flat file paths like `/sd/flight_001.bin`.
- **No file listing API** - There is no `readdir()` equivalent. Use sequential filenames for logging.

**SD Card Configuration** (`hive_static_config.h` or Makefile):
```c
#define HIVE_ENABLE_SD 1       // Enable SD card support
#define HIVE_MAX_SD_FILES 4    // Max concurrent open files on SD
```

**Porting:** Each board must implement `spi_ll_sd.c` with board-specific SPI pin configuration. See `src/hal/stm32/spi_ll.h` for the required interface.

**SD Card Usage Example:**
```c
int fd;
hive_status_t status;

// Check if SD card is available
if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
    // SD card present - use it
    status = hive_file_open("/sd/flight_001.bin",
                            HIVE_O_WRONLY | HIVE_O_CREAT, 0644, &fd);
} else {
    // Fall back to flash
    status = hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);
}

// Write works the same regardless of backend
hive_file_write(fd, data, len, &written);
hive_file_sync(fd);
hive_file_close(fd);
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
hive_status_t hive_log_init(void);              // Initialize (call once at startup)
hive_status_t hive_log_file_open(const char *path);  // Open log file
hive_status_t hive_log_file_sync(void);         // Flush to storage
hive_status_t hive_log_file_close(void);        // Close log file
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

**Decoding:** Use `tools/decode_log.py` to convert binary logs to text:
```bash
python3 tools/decode_log.py flight.bin              # Decode to stdout
python3 tools/decode_log.py flight.bin -o out.txt   # Decode to file
python3 tools/decode_log.py flight.bin --stats      # Show statistics
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
// Main actor manages log lifecycle
void main_actor(void *arg) {
    // ARM phase: open log file (on STM32, erases flash sector)
    hive_log_file_open(HIVE_LOG_FILE_PATH);

    // Start periodic sync timer (every 4 seconds)
    timer_id_t sync_timer;
    hive_timer_every(4000000, &sync_timer);

    while (flying) {
        hive_message_t msg;
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
