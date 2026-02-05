# Explicit Message ID Field

## Problem

The current IPC API makes it awkward to dispatch on request types. `hive_ipc_request()` auto-generates a tag for request/reply correlation, leaving no way to filter by request type. Receivers must use `HIVE_TAG_ANY` and manually inspect payloads.

### Current Pattern (Verbose)

```c
// Can only filter sender + class, must use HIVE_TAG_ANY for requests
hive_select_source_t sources[] = {
    {HIVE_SEL_IPC, .ipc = {flight_manager, HIVE_MSG_REQUEST, HIVE_TAG_ANY}},
};

// Manual payload inspection for dispatch
if (result.index == SEL_REQUEST) {
    uint8_t reply = REPLY_OK;
    if (result.ipc.len != 1 ||
        ((uint8_t *)result.ipc.data)[0] != REQUEST_RESET) {
        HIVE_LOG_WARN("[XXX] Unknown request ignored");
        reply = REPLY_FAIL;
    } else {
        do_reset();
    }
    hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
}
```

This pattern repeats in every actor that handles requests.

## Solution

Add an explicit `id` field to the message header for dispatch. Separate concerns:

- `id` - Message type (user-provided, for dispatch)
- `tag` - Correlation (internal - auto-generated for request/reply, runtime-set for timers)

### Mental Model

User only thinks about `id`. Tag is plumbing:

| Message Type | id | tag |
|--------------|-----|-----|
| Notify | User-provided | Unused (0) |
| Request | User-provided | Auto-generated |
| Reply | Preserved | Preserved |
| Timer | Unused | timer_id (runtime-set) |

### Message Structure

```c
typedef struct {
    hive_actor_id_t sender;
    hive_msg_class_t class;
    uint16_t id;       // Message type - user provided
    uint32_t tag;      // Correlation - internal use
    void *data;
    size_t len;
} hive_message_t;
```

Header grows by 2 bytes. For 128-byte max messages, that's 1.5% overhead.

### API Changes

```c
// Notify - user provides id only
hive_ipc_notify(to, id, data, len);

// Request - user provides id, tag auto-generated for correlation
hive_ipc_request(to, id, data, len, reply, timeout);

// Receive - filter includes id
hive_ipc_recv_match(from, class, id, tag, msg, timeout);
```

### Filter Structure

```c
typedef struct {
    hive_actor_id_t sender;   // HIVE_SENDER_ANY for wildcard
    hive_msg_class_t class;   // HIVE_MSG_ANY for wildcard
    uint16_t id;              // HIVE_ID_ANY for wildcard
    uint32_t tag;             // HIVE_TAG_ANY for wildcard
} hive_ipc_filter_t;
```

### Wildcard Defaults

All `*_ANY` constants are 0. C's designated initializer behavior (unspecified fields = 0) gives wildcards for free:

```c
#define HIVE_SENDER_ANY  0
#define HIVE_MSG_ANY     0
#define HIVE_ID_ANY      0
#define HIVE_TAG_ANY     0

// Valid values start from 1
#define HIVE_MSG_NOTIFY  1
#define HIVE_MSG_REQUEST 2
// etc.
```

You only specify what you care about. Everything else matches anything.

### Simple Example: Two Notifications

```c
hive_select_source_t sources[] = {
    {HIVE_SEL_IPC, .ipc = {.sender = actor_a,
                           .class = HIVE_MSG_NOTIFY,
                           .id = NOTIFY_START}},
    {HIVE_SEL_IPC, .ipc = {.sender = actor_b,
                           .class = HIVE_MSG_NOTIFY,
                           .id = NOTIFY_LANDED}},
};

hive_select_result_t result;
hive_select(sources, 2, &result, -1);

if (result.index == 0) {
    handle_start();
} else {
    handle_landed();
}
```

Same class, different ids. Tag defaults to ANY (0).

### Complete Example

```c
enum { SEL_RESET, SEL_ARM, SEL_LANDED, SEL_TIMER };

hive_select_source_t sources[] = {
    // Filter requests by id (tag defaults to ANY)
    [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {.sender = fm,
                                          .class = HIVE_MSG_REQUEST,
                                          .id = REQUEST_RESET}},
    [SEL_ARM]   = {HIVE_SEL_IPC, .ipc = {.sender = fm,
                                          .class = HIVE_MSG_REQUEST,
                                          .id = REQUEST_ARM}},
    // Filter notifications by id (tag defaults to ANY)
    [SEL_LANDED] = {HIVE_SEL_IPC, .ipc = {.sender = altitude,
                                           .class = HIVE_MSG_NOTIFY,
                                           .id = NOTIFY_LANDED}},
    // Filter timers by tag (id defaults to ANY)
    [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_TIMER,
                                          .tag = my_timer}},
};

hive_select_result_t result;
hive_select(sources, 4, &result, -1);

switch (result.index) {
case SEL_RESET:
    // Request - do work and reply
    do_reset();
    uint8_t status = REPLY_OK;
    hive_ipc_reply(&result.ipc, &status, sizeof(status));
    break;

case SEL_ARM:
    // Request - do work and reply
    do_arm();
    uint8_t status = REPLY_OK;
    hive_ipc_reply(&result.ipc, &status, sizeof(status));
    break;

case SEL_LANDED:
    // Notification - no reply needed
    handle_landed();
    break;

case SEL_TIMER:
    // Timer tick - no reply needed
    handle_tick();
    break;
}
```

Reply preserves tag for correlation. The id in replies is irrelevant - the requester matches by tag.

## Migration

This is a breaking API change affecting:

**Send side:**
- `hive_ipc_notify()` - `tag` parameter becomes `id`
- `hive_ipc_notify_ex()` - `tag` parameter becomes `id`
- `hive_ipc_request()` - adds `id` parameter
- `hive_ipc_named_notify()` - `tag` parameter becomes `id`
- `hive_ipc_named_request()` - adds `id` parameter

**Receive side:**
- `hive_ipc_recv_match()` - adds `id` parameter
- `hive_ipc_recv_matches()` - filter struct gains `id` field
- `hive_select()` - source filter struct gains `id` field

**Internal:**
- Message header format - gains `id` field (2 bytes)
- `hive_ipc_filter_t` struct - gains `id` field

All existing code using IPC will need updates.

### Migration Impact

**Core files to modify:**

| File | Lines | Change |
|------|-------|--------|
| `src/hive_ipc.c` | 655 | Add id to message header, update functions |
| `src/hive_select.c` | 193 | Add id matching logic |
| `include/hive_ipc.h` | 97 | Update function signatures, filter struct |
| `include/hive_select.h` | 83 | Update source struct |

**Call sites (excluding pilot example):**

| Category | Count | Change |
|----------|-------|--------|
| `hive_ipc_notify()` | ~70 | Semantic: tag to id (same position) |
| `hive_ipc_request()` | ~6 | Add id parameter |
| `hive_ipc_recv_match()` | ~76 | Add id parameter |
| `HIVE_SEL_IPC` sources | ~20 | Add .id field |

**Pilot example:**

| Category | Count | Change |
|----------|-------|--------|
| `hive_ipc_notify()` | 8 | Semantic: tag to id |
| `hive_ipc_request()` | 5 | Add id parameter |
| `HIVE_SEL_IPC` sources | 32 | Add .id field |
| `hive_ipc_reply()` | 17 | Unchanged |

**Total: ~215 call sites**

Most changes are mechanical:
- `HIVE_TAG_NONE` becomes `HIVE_ID_NONE`
- Add `.id = REQUEST_RESET` or `.id = HIVE_ID_ANY` to select sources
- Add id parameter to request calls

**Net effect:** Delete boilerplate payload validation, replace with clean filters. You delete more code than you add.

## API Summary

### Send Functions

| Function | Old | New | Change |
|----------|-----|-----|--------|
| `hive_ipc_notify` | `(to, tag, data, len)` | `(to, id, data, len)` | Semantic (tag to id) |
| `hive_ipc_notify_ex` | `(to, class, tag, data, len)` | `(to, class, id, data, len)` | Semantic (tag to id) |
| `hive_ipc_request` | `(to, data, len, reply, timeout)` | `(to, id, data, len, reply, timeout)` | Adds id |
| `hive_ipc_reply` | `(msg, data, len)` | `(msg, data, len)` | Unchanged |
| `hive_ipc_named_notify` | `(name, tag, data, len)` | `(name, id, data, len)` | Semantic (tag to id) |
| `hive_ipc_named_request` | `(name, data, len, reply, timeout)` | `(name, id, data, len, reply, timeout)` | Adds id |

### Receive Functions

| Function | Old | New | Change |
|----------|-----|-----|--------|
| `hive_ipc_recv` | `(msg, timeout)` | `(msg, timeout)` | Unchanged |
| `hive_ipc_recv_match` | `(from, class, tag, msg, timeout)` | `(from, class, id, tag, msg, timeout)` | Adds id |
| `hive_ipc_recv_matches` | `(filters, n, msg, timeout, idx)` | `(filters, n, msg, timeout, idx)` | Filter struct gains id |

### Select Functions

| Function | Old | New | Change |
|----------|-----|-----|--------|
| `hive_select` | `(sources, n, result, timeout)` | `(sources, n, result, timeout)` | Source filter gains id |
| `hive_bus_read` | - | - | Unchanged (bus, not IPC) |
| `hive_event_wait` | - | - | Unchanged (HAL events) |

### Filter Struct

```c
// Old
typedef struct {
    hive_actor_id_t sender;
    hive_msg_class_t class;
    uint32_t tag;
} hive_ipc_filter_t;

// New
typedef struct {
    hive_actor_id_t sender;   // HIVE_SENDER_ANY = 0
    hive_msg_class_t class;   // HIVE_MSG_ANY = 0
    uint16_t id;              // HIVE_ID_ANY = 0 (NEW)
    uint32_t tag;             // HIVE_TAG_ANY = 0
} hive_ipc_filter_t;
```

Used by: `hive_ipc_recv_match`, `hive_ipc_recv_matches`, `hive_select` (inside `hive_select_source_t`)

## Use Case Verification

| Use Case | Status | Example |
|----------|--------|---------|
| Send notification by type | OK | `notify(to, NOTIFY_START, data, len)` |
| Send request by type | OK | `request(to, REQUEST_RESET, data, len, &reply, timeout)` |
| Reply to request | OK | `reply(&msg, data, len)` - unchanged |
| Filter notification by type | OK | `.class = NOTIFY, .id = NOTIFY_START` |
| Filter request by type | OK | `.class = REQUEST, .id = REQUEST_RESET` |
| Filter timer by timer_id | OK | `.class = TIMER, .tag = my_timer` |
| Filter any from sender | OK | `.sender = actor` (id, tag default to ANY) |
| Exit messages | OK | `.class = EXIT` (id unused) |
| Multiple filters in select | OK | Each source with own id |

## Implementation Notes

**Enum reordering required** - wildcards must be 0:

```c
typedef enum {
    HIVE_MSG_ANY = 0,      // Wildcard (verify current value)
    HIVE_MSG_NOTIFY = 1,
    HIVE_MSG_REQUEST = 2,
    HIVE_MSG_REPLY = 3,
    HIVE_MSG_TIMER = 4,
    HIVE_MSG_EXIT = 5,
} hive_msg_class_t;
```

## Review Verdict

| Criterion | Status | Notes |
|-----------|--------|-------|
| Clean | YES | One concept per field: id = dispatch, tag = correlation |
| Solid | YES | All use cases work, problem case solved elegantly |
| KISS | YES | "id is what I care about, tag is plumbing" |
| Beautiful | YES | Self-documenting, no payload inspection, no boilerplate |

**Cost:**
- 2 bytes per message header
- Breaking API change

**Conclusion:** The 2-byte overhead buys a beautiful, honest API. Ship it.
