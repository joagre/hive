# Unified hive_select() API Design Sketch

**Status** - Implemented (2026-01-17)

## Motivation

Currently, actors must choose one blocking primitive at a time:
- `hive_ipc_recv()` / `hive_ipc_recv_matches()` - wait for messages
- `hive_bus_read()` with timeout - wait for bus data

This forces awkward patterns when an actor needs to respond to multiple event sources (e.g., timer ticks AND sensor bus data AND commands). A unified select would enable clean event loops.

## Real-World Example: Pilot Altitude Actor

The current `altitude_actor.c` shows the problem. It needs to:
1. Process state updates from the state bus (100 Hz)
2. Respond to LANDING commands immediately

**Current code (problematic)**
```c
while (true) {
    // Block until state available - LANDING command delayed while blocked!
    hive_bus_read(s_state_bus, &state, sizeof(state), &len, HIVE_TIMEOUT_INFINITE);

    // ... process state ...

    // Check for landing command (non-blocking poll - may miss if blocked above)
    if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                           NOTIFY_LANDING, &msg, 0))) {
        landing_mode = true;
    }
}
```

**Problem** - If LANDING command arrives while blocked on bus, response is delayed until next state update (up to 10ms at 100 Hz).

**With `hive_select()`**
```c
enum { SEL_STATE, SEL_LANDING };
hive_select_source_t sources[] = {
    [SEL_STATE] = {.type = HIVE_SEL_BUS, .bus = s_state_bus},
    [SEL_LANDING] = {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = NOTIFY_LANDING}},
};

while (true) {
    hive_select_result_t result;
    hive_select(sources, 2, &result, -1);

    if (result.index == SEL_LANDING) {
        landing_mode = true;
        continue;  // Respond immediately, don't wait for state
    }

    // Process state from result.bus.data
    state_estimate_t *state = (state_estimate_t *)result.bus.data;
    // ...
}
```

**Benefits**
- Immediate response to LANDING command (no 10ms delay)
- Clean event loop structure
- No non-blocking polling

Similar patterns exist in `waypoint_actor.c`, `motor_actor.c`, and others.

## Proposed API

```c
// Select source types
typedef enum {
    HIVE_SEL_IPC,       // Message matching filter
    HIVE_SEL_BUS,       // Bus data available
} hive_select_type;

// Select source (tagged union)
typedef struct {
    hive_select_type type;
    union {
        hive_recv_filter_t ipc;    // For HIVE_SEL_IPC
        hive_bus_id_t bus;              // For HIVE_SEL_BUS
    };
} hive_select_source_t;

// Select result
typedef struct {
    size_t index;               // Which source triggered (0-based)
    hive_select_type type;      // Convenience copy of triggered type
    union {
        hive_message_t ipc;       // For HIVE_SEL_IPC
        struct {                // For HIVE_SEL_BUS
            void *data;
            size_t len;
        } bus;
    };
} hive_select_result_t;

// Block until any source has data
hive_status_t hive_select(const hive_select_source_t *sources, size_t num_sources,
                        hive_select_result_t *result, int32_t timeout_ms);
```

## Usage Example

```c
enum { SEL_TIMER, SEL_SENSOR_BUS, SEL_COMMAND };
hive_select_source_t sources[] = {
    [SEL_TIMER] = {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_TIMER, .tag = my_timer}},
    [SEL_SENSOR_BUS] = {.type = HIVE_SEL_BUS, .bus = sensor_bus},
    [SEL_COMMAND] = {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = CMD_TAG}},
};

hive_select_result_t result;
hive_select(sources, 3, &result, -1);

switch (result.index) {
    case SEL_TIMER:
        handle_tick();
        break;
    case SEL_SENSOR_BUS:
        process_sensor(result.bus.data, result.bus.len);
        break;
    case SEL_COMMAND:
        handle_command(&result.ipc);
        break;
}
```

## Implementation Notes

- Scans IPC mailbox and bus buffers first (non-blocking check)
- If nothing ready, registers sources with scheduler and blocks
- Wakes on first match from any source
- Timers delivered as IPC messages (no change needed)
- Zero heap allocation - all structures caller-provided
- Works on all platforms (Linux and STM32)

## Wake Mechanism Integration

### Current IPC Wake (`hive_ipc.c`)

IPC stores filter info in the actor struct:
```c
// In actor struct (hive_actor.h):
const hive_recv_filter_t *recv_filters;  // NULL = no filter active
size_t recv_filter_count;
```

Wake logic in `hive_mailbox_add_entry()`:
```c
if (recipient->state == ACTOR_STATE_WAITING) {
    if (recipient->recv_filters == NULL) {
        // No filter - wake on any message
        recipient->state = ACTOR_STATE_READY;
    } else {
        // Check if message matches any filter
        for (size_t i = 0; i < recipient->recv_filter_count; i++) {
            if (entry_matches_filter(entry, &recipient->recv_filters[i])) {
                recipient->state = ACTOR_STATE_READY;
                break;
            }
        }
        // Also wake on TIMER (could be timeout timer)
    }
}
```

### Current Bus Wake (`hive_bus.c`)

Bus uses a `blocked` flag in the subscriber struct:
```c
// In bus_subscriber struct:
bool blocked;  // Is actor blocked waiting for data?
```

Wake logic in `hive_bus_publish()`:
```c
for (size_t i = 0; i < bus->config.max_subscribers; i++) {
    bus_subscriber *sub = &bus->subscribers[i];
    if (sub->active && sub->blocked) {
        actor *a = hive_actor_get(sub->id);
        if (a && a->state == ACTOR_STATE_WAITING) {
            a->state = ACTOR_STATE_READY;
        }
    }
}
```

### Comparison

| Aspect | IPC | Bus |
|--------|-----|-----|
| Filter storage | Actor struct | Bus subscriber struct |
| Wake condition | Filter match OR timer | Any publish (if blocked) |
| Granularity | Per-filter matching | Binary blocked flag |

### Integration for `hive_select()`

Add new fields to actor struct:
```c
const hive_select_source_t *select_sources;  // NULL = not in select
size_t select_source_count;
```

**Modified wake paths**

1. **`hive_mailbox_add_entry()`** - also check `select_sources`:
   ```c
   if (recipient->select_sources) {
       for (size_t i = 0; i < recipient->select_source_count; i++) {
           if (recipient->select_sources[i].type == HIVE_SEL_IPC &&
               entry_matches_filter(entry, &recipient->select_sources[i].ipc)) {
               recipient->state = ACTOR_STATE_READY;
               break;
           }
       }
   }
   ```

2. **`hive_bus_publish()`** - check `select_sources` for matching bus:
   ```c
   if (sub->blocked && a->state == ACTOR_STATE_WAITING) {
       if (a->select_sources) {
           // Check if this bus is in select sources
           for (size_t i = 0; i < a->select_source_count; i++) {
               if (a->select_sources[i].type == HIVE_SEL_BUS &&
                   a->select_sources[i].bus == bus->id) {
                   a->state = ACTOR_STATE_READY;
                   break;
               }
           }
       } else {
           // Legacy single-bus wait
           a->state = ACTOR_STATE_READY;
       }
   }
   ```

**Key design decisions**
- Existing `recv_filters` stays for `hive_ipc_recv_matches()` - no breaking changes
- `select_sources` is separate, used only by `hive_select()`
- Bus still uses `sub->blocked` flag for quick filtering before checking `select_sources`

### Error Handling

- `num_sources == 0` -> `HIVE_ERR_INVALID` ("no sources")
- `sources == NULL` -> `HIVE_ERR_INVALID` ("null sources")
- `result == NULL` -> `HIVE_ERR_INVALID` ("null result")
- Bus source with invalid/unsubscribed bus -> `HIVE_ERR_INVALID` ("not subscribed")
- No auto-subscribe - explicit subscription required before using bus in select

## Efficiency

**Complexity by operation**

| API | Complexity |
|-----|------------|
| `hive_ipc_recv()` | O(1) - first entry always matches |
| `hive_ipc_recv_match()` | O(mailbox_depth) |
| `hive_ipc_recv_matches()` | O(filters x mailbox_depth) |
| `hive_select()` with 1 source | O(1) if wildcards, O(depth) if specific |
| `hive_select()` with N sources | O(N x sources_depth) |

**Why this is acceptable**
- Source counts are tiny (2-5 typical)
- Depths bounded by pool sizes (256 default)
- Same O(n) pattern as existing `hive_ipc_recv_matches()`

**The real win** - avoiding busy-polling
```c
// WITHOUT hive_select - must busy-poll:
while (true) {
    if (hive_ipc_recv(&msg, 0) == HIVE_OK) { handle_msg(); continue; }
    if (hive_bus_read(&bus, &data, 0) == HIVE_OK) { handle_bus(); continue; }
    hive_yield();  // Wasteful
}

// WITH hive_select - efficient block:
hive_select(sources, 2, &result, -1);  // Zero CPU while waiting
```

## Relationship to Existing APIs

**Decision** - All blocking receive/read APIs become thin wrappers around `hive_select()`.

This is the official approach - no separate implementations.

### APIs That Become Wrappers

```c
// These all delegate to hive_select() internally:
hive_ipc_recv(&msg, -1);                    // Any message
hive_ipc_recv_match(from, class, id, ...);  // Single filter
hive_ipc_recv_matches(filters, n, ...);     // Multiple IPC filters
hive_bus_read(bus, &data, len, &actual, -1);  // Single bus (with timeout)

// The unified primitive:
hive_select(sources, n, &result, -1);       // IPC + bus combined
```

### Cost Analysis

| Overhead | Impact |
|----------|--------|
| Stack: ~70 bytes extra | Negligible (64KB default stack) |
| One extra function call | Negligible |
| One struct copy (~32 bytes) | Negligible |
| Algorithmic complexity | Identical - O(1) for wildcards |

### Rationale

- **Single implementation** - one place for blocking/wake logic
- **Fewer code paths** - fewer bugs, easier testing
- **Consistent behavior** - guaranteed identical semantics
- **Maintainability** - clean code over micro-optimization

### Wrapper Implementations

```c
hive_status_t hive_ipc_recv(hive_message_t *msg, int32_t timeout_ms) {
    hive_select_source_t source = {.type = HIVE_SEL_IPC};
    hive_select_result_t result;
    hive_status_t s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) *msg = result.ipc;
    return s;
}

hive_status_t hive_ipc_recv_match(hive_actor_id_t from, hive_msg_class_t class,
                                uint16_t id, hive_message_t *msg,
                                int32_t timeout_ms) {
    hive_select_source_t source = {.type = HIVE_SEL_IPC, .ipc = {.sender = from, .class = class, .id = id}};
    hive_select_result_t result;
    hive_status_t s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) *msg = result.ipc;
    return s;
}

hive_status_t hive_ipc_recv_matches(const hive_recv_filter_t *filters,
                                  size_t num_filters, hive_message_t *msg,
                                  int32_t timeout_ms, size_t *matched_index) {
    // Build select sources from filters
    hive_select_source_t sources[num_filters];  // VLA or fixed max
    for (size_t i = 0; i < num_filters; i++) {
        sources[i] = (hive_select_source_t){.type = HIVE_SEL_IPC, .ipc = filters[i]};
    }
    hive_select_result_t result;
    hive_status_t s = hive_select(sources, num_filters, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) {
        *msg = result.ipc;
        if (matched_index != NULL) *matched_index = result.index;
    }
    return s;
}

// hive_bus_read() uses hive_select() internally when timeout is non-zero:
hive_status_t hive_bus_read(hive_bus_id_t bus, void *buf, size_t max_len,
                            size_t *actual_len, int32_t timeout_ms) {
    // ... validation ...

    // Blocking path: use hive_select
    if (timeout_ms != HIVE_TIMEOUT_NONBLOCKING) {
        hive_select_source_t source = {.type = HIVE_SEL_BUS, .bus = bus};
        hive_select_result_t result;
        hive_status_t s = hive_select(&source, 1, &result, timeout_ms);
        if (HIVE_SUCCEEDED(s)) {
            size_t copy_len = result.bus.len < max_len ? result.bus.len : max_len;
            memcpy(buf, result.bus.data, copy_len);
            *actual_len = copy_len;
        }
        return s;
    }

    // Non-blocking path: direct read from ring buffer
    // ...
}
```

## Data Lifetime

Result data follows the same "valid until next call" semantics as existing APIs:
- `result.ipc` - valid until next `hive_select()` (same as `hive_ipc_recv()`)
- `result.bus.data` - valid until next `hive_select()` (same as `hive_bus_read()`)

This maintains consistency with existing APIs, enables zero-copy, and matches the mental model users already have. Copy data immediately if needed longer:

```c
hive_select_result_t result;
hive_select(sources, 2, &result, -1);

if (result.type == HIVE_SEL_BUS) {
    my_data copy = *(my_data *)result.bus.data;  // Copy if needed
}
```

## Priority When Multiple Sources Ready

When multiple sources have data simultaneously, sources are checked in **strict array order**. The first ready source wins. There is no type-based priority - bus and IPC sources are treated equally.

**Rationale**
- **Explicit control** - User decides priority via array ordering
- **Predictable** - No hidden rules about which source type wins
- **Simple** - First ready source in array wins

**Example**
```c
enum { SEL_STATE, SEL_SENSOR, SEL_COMMAND, SEL_TIMER };
hive_select_source_t sources[] = {
    [SEL_STATE] = {.type = HIVE_SEL_BUS, .bus = state_bus},      // Checked 1st
    [SEL_SENSOR] = {.type = HIVE_SEL_BUS, .bus = sensor_bus},    // Checked 2nd
    [SEL_COMMAND] = {.type = HIVE_SEL_IPC, .ipc = {...}},        // Checked 3rd
    [SEL_TIMER] = {.type = HIVE_SEL_IPC, .ipc = {...}},          // Checked 4th
};
```

If both `SEL_STATE` and `SEL_COMMAND` are ready simultaneously, `SEL_STATE` wins (first in array).

To prioritize time-sensitive data, place those sources earlier in the array.

## Design Decisions

### TCP

Intentionally excluded for now. Reasons:
- TCP uses fd-based semantics (different from IPC/bus)
- Linux-only (STM32 has no TCP support)
- Different buffer management requirements

**May be added in future** - the API is extensible
- Tagged union allows adding `HIVE_SEL_NET` enum value and union member
- No breaking changes to existing code
- Priority follows array order (user controls via source placement)

**Current workaround** - idiomatic actor model pattern
```c
// Dedicated TCP reader actor - separation of concerns
void net_reader(void *arg) {
    int fd = *(int *)arg;
    char buf[256];
    while (true) {
        size_t n;
        hive_tcp_recv(fd, buf, sizeof(buf), &n, -1);
        hive_ipc_notify(handler_actor, NET_DATA_TAG, buf, n);  // Forward as IPC
    }
}
```

This keeps TCP complexity isolated and lets `hive_select()` remain simple and portable.

### `hive_recv_filter_t` Location

**Decision** - Move `hive_recv_filter_t` from `hive_ipc.h` to `hive_types.h`.

**Rationale**
- **Follows existing pattern** - `hive_message_t` is in `hive_types.h`, and `hive_recv_filter_t` is a filter pattern for messages. They belong together.
- **Cross-subsystem type** - Used by both IPC (`hive_ipc_recv_matches`) and select (`hive_select_source_t`). Shared types belong in `hive_types.h`.
- **Conceptual grouping** - Filter constants (`HIVE_SENDER_ANY`, `HIVE_MSG_ANY`, `HIVE_TAG_ANY`) are already in `hive_types.h`. The struct that uses them should be there too.

**Include hierarchy**
```
hive_types.h      <- hive_recv_filter_t, hive_message_t, constants
    ^
hive_ipc.h        <- uses hive_recv_filter_t
hive_select.h     <- uses hive_recv_filter_t
```

## Implementation Checklist

Implementation completed 2026-01-17:

### Core Implementation
- [x] `include/hive_types.h` - Added select types (`hive_select_source_t`, `hive_select_result_t`, `hive_bus_id_t`)
- [x] `include/hive_select.h` - New header with types and API declaration
- [x] `src/hive_select.c` - Implementation
- [x] `include/hive_actor.h` - Added `select_sources` and `select_source_count` fields
- [x] `src/hive_ipc.c` - Rewrote `hive_ipc_recv`, `hive_ipc_recv_match`, `hive_ipc_recv_matches` as wrappers
- [x] `src/hive_bus.c` - Updated `hive_bus_read()` to use `hive_select()` for blocking path

### Documentation
- [x] `man/man3/hive_select.3` - New man page
- [x] `man/man3/hive_ipc.3` - Added note about wrapper implementation
- [x] `man/man3/hive_bus.3` - Added note about wrapper implementation
- [x] `man/man3/hive_types.3` - Added select types documentation
- [ ] `docs/spec/api.md` - Add hive_select() section (optional - can be done later)
- [ ] `README.md` - Add hive_select() to API overview (optional - can be done later)
- [ ] `CLAUDE.md` - Add hive_select() to IPC/Bus documentation (optional - can be done later)

### Pilot Example Updates
- [ ] `examples/pilot/altitude_actor.c` - Use hive_select() for state + landing
- [ ] `examples/pilot/waypoint_actor.c` - Use hive_select() where applicable
- [ ] `examples/pilot/motor_actor.c` - Use hive_select() where applicable
- [ ] Review other pilot actors for hive_select() opportunities

### Tests
- [x] `tests/select_test.c` - New test file with:
  - [x] Basic single IPC source (equivalent to recv_match)
  - [x] Basic single bus source (equivalent to bus_read_wait)
  - [x] Multi-source: IPC + IPC
  - [x] Multi-source: bus + bus
  - [x] Multi-source: IPC + bus (mixed)
  - [x] Priority ordering (array order)
  - [x] Timeout behavior
  - [x] Immediate return when data ready
  - [x] Error cases (NULL args, unsubscribed bus)

### Example
- [ ] `examples/select_example.c` - Standalone example demonstrating API (optional)
