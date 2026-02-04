# Payload-Based Selective Receive

## Problem

The current IPC API makes it awkward to dispatch on request types when using `hive_ipc_request()`. The receiver cannot filter by specific request type in `hive_select()` or `hive_ipc_recv_match()` - they must accept all requests and manually dispatch based on payload content.

### Why This Happens

`hive_ipc_request()` auto-generates a unique tag for request/reply correlation. This means the tag field is consumed for internal use and unavailable for application-level dispatch. Receivers must use `HIVE_TAG_ANY` to accept requests.

### Current Pattern (Verbose)

```c
// Receiver must accept ANY request, then manually dispatch
hive_select_source_t sources[] = {
    [SEL_REQUEST] = {HIVE_SEL_IPC,
                     .ipc = {flight_manager, HIVE_MSG_REQUEST, HIVE_TAG_ANY}},
};

// ... after hive_select() ...

if (result.index == SEL_REQUEST) {
    // Manual payload inspection for dispatch
    uint8_t reply = REPLY_OK;
    if (result.ipc.len != 1 ||
        ((uint8_t *)result.ipc.data)[0] != REQUEST_RESET) {
        HIVE_LOG_WARN("[XXX] Unknown request ignored");
        reply = REPLY_FAIL;
    } else {
        // Handle RESET
        do_reset();
    }
    hive_ipc_reply(&result.ipc, &reply, sizeof(reply));
}
```

This pattern repeats in every actor that handles requests, leading to boilerplate and potential bugs.

### Impact

- Every actor handling requests needs the same boilerplate
- Easy to forget payload validation
- Cannot wait for specific request types - must accept all and filter
- Multiple request types require nested if/else or switch on payload

## Proposed Solutions

### Option 1: Payload Prefix Filter

Add optional payload prefix matching to selective receive.

```c
// Filter by first N bytes of payload
typedef struct {
    hive_actor_id_t sender;
    hive_message_class_t class;
    hive_tag_t tag;
    const void *payload_prefix;  // NEW: Match if payload starts with this
    size_t payload_prefix_len;   // NEW: Length of prefix to match
} hive_ipc_filter_t;

// Usage - only receive RESET requests
hive_select_source_t sources[] = {
    [SEL_RESET] = {HIVE_SEL_IPC,
                   .ipc = {flight_manager, HIVE_MSG_REQUEST, HIVE_TAG_ANY},
                   .payload_prefix = &REQUEST_RESET,
                   .payload_prefix_len = 1},
    [SEL_ARM] = {HIVE_SEL_IPC,
                 .ipc = {flight_manager, HIVE_MSG_REQUEST, HIVE_TAG_ANY},
                 .payload_prefix = &REQUEST_ARM,
                 .payload_prefix_len = 1},
};
```

**Pros:**
- Clean API extension
- Backward compatible (prefix = NULL means no filtering)
- Works with existing message format

**Cons:**
- Adds complexity to mailbox scanning
- Multiple sources may need to scan same messages

### Option 2: Request Subtype Field

Add a dedicated subtype field to the message header for application use.

```c
// In message header (alongside class, tag)
typedef struct {
    uint8_t class;
    uint8_t subtype;  // NEW: Application-defined request type
    uint16_t tag;
    // ... payload follows
} hive_message_header_t;

// New API
hive_ipc_request_typed(to, subtype, data, len, reply, timeout);

// Usage
hive_select_source_t sources[] = {
    [SEL_RESET] = {HIVE_SEL_IPC,
                   .ipc = {flight_manager, HIVE_MSG_REQUEST, HIVE_TAG_ANY,
                           .subtype = REQUEST_RESET}},
};
```

**Pros:**
- Clean separation of correlation (tag) and dispatch (subtype)
- No payload inspection needed
- Efficient filtering in mailbox

**Cons:**
- Requires header format change (breaking change)
- Uses more header space (may matter on constrained systems)

### Option 3: User-Provided Tag for Request

Allow callers to provide their own tag when correlation isn't needed.

```c
// Existing API - auto-generated tag
hive_ipc_request(to, data, len, reply, timeout);

// New API - user-provided tag
hive_ipc_request_with_tag(to, tag, data, len, reply, timeout);

// Sender uses known tag
hive_ipc_request_with_tag(motor, REQUEST_RESET, &payload, len, &reply, timeout);

// Receiver can filter by tag
hive_select_source_t sources[] = {
    [SEL_RESET] = {HIVE_SEL_IPC,
                   .ipc = {HIVE_SENDER_ANY, HIVE_MSG_REQUEST, REQUEST_RESET}},
};
```

**Pros:**
- Minimal API change
- Reuses existing tag field
- No header format change

**Cons:**
- Tag serves dual purpose (correlation + dispatch) - confusing
- Caller must ensure tags don't collide with auto-generated ones
- Breaks request/reply correlation if same tag used for multiple requests

## Recommendation

**Option 1 (Payload Prefix Filter)** is the cleanest solution:
- No breaking changes to message format
- Backward compatible
- Matches how applications already use payloads
- Can be implemented incrementally

Implementation would add two fields to `hive_ipc_filter_t` and modify `hive_mailbox_scan()` to compare payload prefix when specified.

## Workaround (Current)

Until this is implemented, use a helper macro to reduce boilerplate:

```c
// Helper macro for request dispatch
#define HANDLE_REQUEST(result, expected_type, handler, fail_handler) \
    do { \
        uint8_t _reply = REPLY_OK; \
        if ((result).ipc.len != 1 || \
            ((uint8_t *)(result).ipc.data)[0] != (expected_type)) { \
            _reply = REPLY_FAIL; \
            fail_handler; \
        } else { \
            handler; \
        } \
        hive_ipc_reply(&(result).ipc, &_reply, sizeof(_reply)); \
    } while (0)

// Usage
if (result.index == SEL_REQUEST) {
    HANDLE_REQUEST(result, REQUEST_RESET,
        { do_reset(); },
        { HIVE_LOG_WARN("[XXX] Unknown request"); });
}
```

## References

- `examples/pilot/` - Multiple actors demonstrate the verbose pattern
- `docs/spec/api.md` - IPC API specification
- `man/man3/hive_ipc.3` - IPC man page
