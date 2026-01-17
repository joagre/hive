# Unified hive_select() API Design Sketch

**Status:** Idea / Future consideration

## Motivation

Currently, actors must choose one blocking primitive at a time:
- `hive_ipc_recv()` / `hive_ipc_recv_matches()` - wait for messages
- `hive_bus_read_wait()` - wait for bus data

This forces awkward patterns when an actor needs to respond to multiple event sources (e.g., timer ticks AND sensor bus data AND commands). A unified select would enable clean event loops.

## Real-World Example: Pilot Altitude Actor

The current `altitude_actor.c` shows the problem. It needs to:
1. Process state updates from the state bus (100 Hz)
2. Respond to LANDING commands immediately

**Current code (problematic):**
```c
while (1) {
    // Block until state available - LANDING command delayed while blocked!
    hive_bus_read_wait(s_state_bus, &state, sizeof(state), &len, -1);

    // ... process state ...

    // Check for landing command (non-blocking poll - may miss if blocked above)
    if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_NOTIFY,
                                           NOTIFY_LANDING, &msg, 0))) {
        landing_mode = true;
    }
}
```

**Problem:** If LANDING command arrives while blocked on bus, response is delayed until next state update (up to 10ms at 100 Hz).

**With `hive_select()`:**
```c
enum { SEL_STATE, SEL_LANDING };
hive_select_source sources[] = {
    [SEL_STATE] = {HIVE_SEL_BUS, .bus = s_state_bus},
    [SEL_LANDING] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_LANDING}},
};

while (1) {
    hive_select_result result;
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

**Benefits:**
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
        hive_recv_filter ipc;    // For HIVE_SEL_IPC
        bus_id bus;              // For HIVE_SEL_BUS
    };
} hive_select_source;

// Select result
typedef struct {
    size_t index;               // Which source triggered (0-based)
    hive_select_type type;      // Convenience copy of triggered type
    union {
        hive_message ipc;       // For HIVE_SEL_IPC
        struct {                // For HIVE_SEL_BUS
            void *data;
            size_t len;
        } bus;
    };
} hive_select_result;

// Block until any source has data
hive_status hive_select(const hive_select_source *sources, size_t num_sources,
                        hive_select_result *result, int32_t timeout_ms);
```

## Usage Example

```c
enum { SEL_TIMER, SEL_SENSOR_BUS, SEL_COMMAND };
hive_select_source sources[] = {
    [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, my_timer}},
    [SEL_SENSOR_BUS] = {HIVE_SEL_BUS, .bus = sensor_bus},
    [SEL_COMMAND] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_TAG}},
};

hive_select_result result;
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

## Efficiency

**Complexity by operation:**

| API | Complexity |
|-----|------------|
| `hive_ipc_recv()` | O(1) - first entry always matches |
| `hive_ipc_recv_match()` | O(mailbox_depth) |
| `hive_ipc_recv_matches()` | O(filters × mailbox_depth) |
| `hive_select()` with 1 source | O(1) if wildcards, O(depth) if specific |
| `hive_select()` with N sources | O(N × sources_depth) |

**Why this is acceptable:**
- Source counts are tiny (2-5 typical)
- Depths bounded by pool sizes (256 default)
- Same O(n) pattern as existing `hive_ipc_recv_matches()`

**The real win** - avoiding busy-polling:
```c
// WITHOUT hive_select - must busy-poll:
while (1) {
    if (hive_ipc_recv(&msg, 0) == HIVE_OK) { handle_msg(); continue; }
    if (hive_bus_read(&bus, &data, 0) == HIVE_OK) { handle_bus(); continue; }
    hive_yield();  // Wasteful
}

// WITH hive_select - efficient block:
hive_select(sources, 2, &result, -1);  // Zero CPU while waiting
```

## Relationship to Existing APIs

Existing APIs remain as convenience wrappers - no breaking changes:

```c
// Simple cases (unchanged)
hive_ipc_recv(&msg, -1);                    // Any message
hive_ipc_recv_matches(filters, n, ...);     // Multiple IPC filters
hive_bus_read_wait(bus, &data, -1);         // Single bus

// Complex cases (new)
hive_select(sources, n, &result, -1);         // IPC + bus combined
```

**Wrapper implementation:**

```c
hive_status hive_ipc_recv(hive_message *msg, int32_t timeout_ms) {
    hive_select_source source = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY}};
    hive_select_result result;
    hive_status s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) *msg = result.ipc;
    return s;
}

hive_status hive_bus_read_wait(bus_id bus, void **data, size_t *len, int32_t timeout_ms) {
    hive_select_source source = {HIVE_SEL_BUS, .bus = bus};
    hive_select_result result;
    hive_status s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) { *data = result.bus.data; *len = result.bus.len; }
    return s;
}
```

## Data Lifetime

Result data follows the same "valid until next call" semantics as existing APIs:
- `result.ipc` - valid until next `hive_select()` (same as `hive_ipc_recv()`)
- `result.bus.data` - valid until next `hive_select()` (same as `hive_bus_read()`)

This maintains consistency with existing APIs, enables zero-copy, and matches the mental model users already have. Copy data immediately if needed longer:

```c
hive_select_result result;
hive_select(sources, 2, &result, -1);

if (result.type == HIVE_SEL_BUS) {
    my_data copy = *(my_data *)result.bus.data;  // Copy if needed
}
```

## Priority When Multiple Sources Ready

When multiple sources have data simultaneously, the check order is:
1. All bus sources (in array order)
2. All IPC sources (in array order)
3. First match wins

**Rationale:**
- **Bus data is time-sensitive** - Sensor streams, state updates. Stale data = bad control decisions.
- **Bus can expire** - Ring buffer wraps, `max_age_ms` expires entries. Missing bus data = data loss.
- **IPC is queued** - Messages stay in mailbox until consumed. They won't be lost.

**Example:**
```c
enum { SEL_STATE, SEL_SENSOR, SEL_COMMAND, SEL_TIMER };
hive_select_source sources[] = {
    [SEL_STATE] = {HIVE_SEL_BUS, .bus = state_bus},      // Checked 1st
    [SEL_SENSOR] = {HIVE_SEL_BUS, .bus = sensor_bus},    // Checked 2nd
    [SEL_COMMAND] = {HIVE_SEL_IPC, .ipc = {...}},        // Checked 3rd
    [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {...}},          // Checked 4th
};
```

If both `state_bus` and `SEL_COMMAND` are ready simultaneously, `state_bus` wins.

This is **guaranteed behavior**, not an implementation detail. Users can rely on it for correctness.

## Design Decisions

### Network I/O

Intentionally excluded for now. Reasons:
- Network uses fd-based semantics (different from IPC/bus)
- Linux-only (STM32 has no network support)
- Different buffer management requirements

**May be added in future** - the API is extensible:
- Tagged union allows adding `HIVE_SEL_NET` enum value and union member
- No breaking changes to existing code
- Priority order can be decided when needed (likely: bus → IPC → network)

**Current workaround** - idiomatic actor model pattern:
```c
// Dedicated network reader actor - separation of concerns
void net_reader(void *arg) {
    int fd = *(int *)arg;
    char buf[256];
    while (1) {
        size_t n;
        hive_net_recv(fd, buf, sizeof(buf), &n, -1);
        hive_ipc_notify(handler_actor, NET_DATA_TAG, buf, n);  // Forward as IPC
    }
}
```

This keeps network complexity isolated and lets `hive_select()` remain simple and portable.

## Implementation Checklist

When implementing `hive_select()`, update the following:

### Core Implementation
- [ ] `include/hive_select.h` - New header with types and API declaration
- [ ] `src/hive_select.c` - Implementation
- [ ] `include/hive_actor.h` - Add select-related fields to actor struct
- [ ] `src/hive_ipc.c` - Update wake logic for select sources
- [ ] `src/hive_bus.c` - Update wake logic for select sources

### Documentation
- [ ] `SPEC.md` - Add hive_select() section
- [ ] `README.md` - Add hive_select() to API overview
- [ ] `CLAUDE.md` - Add hive_select() to IPC/Bus documentation
- [ ] `man/man3/hive_select.3` - New man page

### Pilot Example Updates
- [ ] `examples/pilot/altitude_actor.c` - Use hive_select() for state + landing
- [ ] `examples/pilot/waypoint_actor.c` - Use hive_select() where applicable
- [ ] `examples/pilot/motor_actor.c` - Use hive_select() where applicable
- [ ] Review other pilot actors for hive_select() opportunities

### Tests
- [ ] `tests/select_test.c` - New test file with:
  - Basic single IPC source (equivalent to recv_match)
  - Basic single bus source (equivalent to bus_read_wait)
  - Multi-source: IPC + IPC
  - Multi-source: bus + bus
  - Multi-source: IPC + bus (mixed)
  - Priority ordering (bus before IPC)
  - Timeout behavior
  - Immediate return when data ready

### Example
- [ ] `examples/select_example.c` - Standalone example demonstrating API
