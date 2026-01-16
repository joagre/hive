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
hive_select_cond conds[] = {
    [SEL_STATE] = {HIVE_SEL_BUS, .bus = s_state_bus},
    [SEL_LANDING] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, NOTIFY_LANDING}},
};

while (1) {
    hive_select_result result;
    hive_select(conds, 2, &result, -1);

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

## Naming

| Name | Rationale |
|------|-----------|
| `hive_select()` | Mirrors POSIX `select()`. Implies choosing from multiple sources. |
| `hive_poll()` | Mirrors POSIX `poll()`. More modern than select. |
| `hive_wait()` | Generic. Doesn't hint at "multiple sources". |

Recommendation: `hive_select()` - familiar, implies selection from alternatives.

## Proposed API

```c
// Select condition types
typedef enum {
    HIVE_SEL_IPC,       // Message matching filter
    HIVE_SEL_BUS,       // Bus data available
} hive_select_type;

// Select condition (tagged union)
typedef struct {
    hive_select_type type;
    union {
        hive_recv_filter ipc;    // For HIVE_SEL_IPC
        bus_id bus;              // For HIVE_SEL_BUS
    };
} hive_select_cond;

// Select result
typedef struct {
    size_t index;               // Which condition triggered (0-based)
    hive_select_type type;      // Convenience copy of triggered type
    union {
        hive_message ipc;       // For HIVE_SEL_IPC
        struct {                // For HIVE_SEL_BUS
            void *data;
            size_t len;
        } bus;
    };
} hive_select_result;

// Block until any condition is satisfied
hive_status hive_select(const hive_select_cond *conds, size_t num_conds,
                        hive_select_result *result, int32_t timeout_ms);
```

## Usage Example

```c
enum { SEL_TIMER, SEL_SENSOR_BUS, SEL_COMMAND };
hive_select_cond conds[] = {
    [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, my_timer}},
    [SEL_SENSOR_BUS] = {HIVE_SEL_BUS, .bus = sensor_bus},
    [SEL_COMMAND] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_TAG}},
};

hive_select_result result;
hive_select(conds, 3, &result, -1);

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
- If nothing ready, registers conditions with scheduler and blocks
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
| `hive_select()` with 1 cond | O(1) if wildcards, O(depth) if specific |
| `hive_select()` with N conds | O(N × sources_depth) |

**Why this is acceptable:**
- Condition counts are tiny (2-5 typical)
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
hive_select(conds, 2, &result, -1);  // Zero CPU while waiting
```

## Relationship to Existing APIs

Existing APIs remain as convenience wrappers - no breaking changes:

```c
// Simple cases (unchanged)
hive_ipc_recv(&msg, -1);                    // Any message
hive_ipc_recv_matches(filters, n, ...);     // Multiple IPC filters
hive_bus_read_wait(bus, &data, -1);         // Single bus

// Complex cases (new)
hive_select(conds, n, &result, -1);         // IPC + bus combined
```

**Wrapper implementation:**

```c
hive_status hive_ipc_recv(hive_message *msg, int32_t timeout_ms) {
    hive_select_cond cond = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY}};
    hive_select_result result;
    hive_status s = hive_select(&cond, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) *msg = result.ipc;
    return s;
}

hive_status hive_bus_read_wait(bus_id bus, void **data, size_t *len, int32_t timeout_ms) {
    hive_select_cond cond = {HIVE_SEL_BUS, .bus = bus};
    hive_select_result result;
    hive_status s = hive_select(&cond, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) { *data = result.bus.data; *len = result.bus.len; }
    return s;
}
```

## Open Questions

1. **Bus data lifetime:** Should bus data be copied into result, or use same "valid until next read" semantics as current API?

2. **Priority:** If multiple conditions satisfied simultaneously, which wins? First in array? IPC before bus?

3. **Network I/O:** Intentionally excluded. Network uses fd-based semantics and is Linux-only. If needed, use a dedicated network reader actor that forwards data as IPC messages.
