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

### Clean Usage

```c
hive_select_source_t sources[] = {
    // Filter requests by id
    {HIVE_SEL_IPC, .ipc = {.sender = fm,
                           .class = HIVE_MSG_REQUEST,
                           .id = REQUEST_RESET}},
    {HIVE_SEL_IPC, .ipc = {.sender = fm,
                           .class = HIVE_MSG_REQUEST,
                           .id = REQUEST_ARM}},
    // Filter notifications by id
    {HIVE_SEL_IPC, .ipc = {.sender = altitude,
                           .class = HIVE_MSG_NOTIFY,
                           .id = NOTIFY_LANDED}},
    // Filter timers by tag (timer_id)
    {HIVE_SEL_IPC, .ipc = {.sender = HIVE_SENDER_ANY,
                           .class = HIVE_MSG_TIMER,
                           .tag = my_timer}},
};

// Handler - no payload inspection needed
if (result.index == SEL_RESET) {
    do_reset();
    uint8_t reply = REPLY_OK;
    hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
}
```

## Migration

This is a breaking API change affecting:

- `hive_ipc_notify()` - `tag` parameter becomes `id` (semantic change)
- `hive_ipc_request()` - adds `id` parameter
- `hive_ipc_recv_match()` - adds `id` parameter
- `hive_select_source_t` - filter struct gains `id` field
- Message header format - gains `id` field

All existing code using IPC will need updates.

## Benefits

- No hidden bit packing or magic
- No manual payload inspection
- Self-documenting: `.id = REQUEST_RESET` is explicit
- Simple mental model: "id is what I care about, tag is plumbing"
- Clean separation of concerns

The 2-byte overhead buys a beautiful, honest API.
