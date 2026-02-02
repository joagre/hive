# Enhanced Actor Spawn API

**Status** - Implemented (January 2026)

## Motivation

Looking at pilot actors, a common pattern emerges:

1. Actor starts, immediately calls `hive_register("name")`
2. Looks up sibling actors via `hive_whereis()`
3. Initializes state based on spawn arguments

This boilerplate could be eliminated by enhancing `hive_spawn()` and `hive_spawn_ex()` with:
- An init function for argument transformation
- Auto-registration support
- Sibling actor info passed to each actor

## Proposed API Changes

### New Types

```c
// Init function: transforms init_args before actor runs
// Called in spawner context. Return value becomes args to actor function.
// Returning NULL is valid (actor receives NULL args).
typedef void *(*hive_actor_init_fn_t)(void *init_args);

// Info about a spawned actor (passed to actor function)
typedef struct {
    const char *name;       // Actor name (NULL if unnamed)
    hive_actor_id_t id;            // Actor ID
    bool registered;        // Whether registered in name registry
} hive_spawn_info_t;
```

### Changed Signatures

#### actor_config_t (extended)

```c
// Current
typedef struct {
    const char *name;       // Actor name (for debugging)
    uint8_t priority;       // 0=CRITICAL to 3=LOW
    size_t stack_size;      // Stack size (0 = default)
    bool malloc_stack;      // Use malloc instead of arena
} actor_config_t;

// New
typedef struct {
    const char *name;       // Actor name (for debugging AND registry)
    uint8_t priority;       // 0=CRITICAL to 3=LOW
    size_t stack_size;      // Stack size (0 = default)
    bool malloc_stack;      // Use malloc instead of arena
    bool auto_register;     // Register name in registry (requires name != NULL)
} actor_config_t;
```

#### hive_spawn()

```c
// Current (two functions)
hive_status_t hive_spawn(
    hive_actor_fn fn,
    void *arg,
    hive_actor_id_t *out
);

hive_status_t hive_spawn_ex(
    hive_actor_fn fn,
    void *arg,
    const actor_config_t *cfg,
    hive_actor_id_t *out
);

// New (single function, hive_spawn_ex removed)
hive_status_t hive_spawn(
    hive_actor_fn fn,
    hive_actor_init_fn_t init,    // Init function (NULL = skip)
    void *init_args,            // Arguments to init (or direct args if init is NULL)
    const actor_config_t *cfg,    // Config (NULL = defaults: no name, no registration)
    hive_actor_id_t *out
);
```

When `cfg` is NULL, defaults are used (no name, no auto-registration, default stack size, default priority).

#### Actor Function Signature

```c
// Current
typedef void (*hive_actor_fn)(void *arg);

// New
typedef void (*hive_actor_fn)(void *args, const hive_spawn_info_t *siblings, size_t sibling_count);
```

### Supervisor Child Spec Changes

```c
// Current
typedef struct {
    const char *id;
    hive_actor_fn start;
    void *arg;
    hive_restart_type restart;
} hive_child_spec_t;

// New
typedef struct {
    hive_actor_fn start;        // Actor function
    hive_actor_init_fn_t init;    // Init function (NULL = skip)
    void *init_args;            // Arguments to init function
    const char *name;           // Actor name (for registry AND supervisor tracking)
    bool auto_register;         // Register in name registry
    hive_restart_type restart;  // permanent, transient, temporary
} hive_child_spec_t;
```

Note: Removed separate `id` field. The `name` field serves both purposes:
- Supervisor uses it to identify children (for restart strategies)
- Used for registry if `auto_register` is true

## Behavior

### Spawn Sequence (standalone hive_spawn)

1. Allocate actor (stack, mailbox, etc.)
2. **In spawner context**
   - If `init` is non-NULL: call `init(init_args)`, result becomes `args`
   - If `init` is NULL: `init_args` becomes `args` directly
   - If `auto_register` and `name` is non-NULL: call `hive_register(name)`
     - If registration fails: deallocate actor, return `HIVE_ERR_EXISTS`
3. Build `hive_spawn_info_t` array with single entry (the actor itself)
4. Schedule actor to run `fn(args, siblings, 1)`
5. Return `HIVE_OK` with hive_actor_id_t

### Spawn Sequence (supervisor starting children)

1. **Phase 1 - Allocate all children**
   For each child spec (in order):
   - Allocate actor
   - Run `init(init_args)` if provided
   - Register name if `auto_register` is true
   - If any fails: deallocate all previously allocated children, return error

2. **Phase 2 - Build sibling array**
   - Create `hive_spawn_info_t` array with all children (in spec order)
   - Each entry contains: name, hive_actor_id_t, registered status

3. **Phase 3 - Start all children**
   - Schedule each actor with `fn(args, siblings, sibling_count)`
   - All children receive the same complete sibling array

This two-phase approach ensures every child sees all siblings, including those defined later in the spec.

### Sibling Info Array

- For standalone `hive_spawn()`: array contains only the spawned actor (`sibling_count = 1`)
- For supervisor children: array contains ALL children in spec order
- Array is stack-allocated, valid only at actor function entry
- Actor must copy needed `hive_actor_id_t`s to local variables

### Example

```c
// Child specs for supervisor
hive_child_spec_t children[] = {
    {sensor_actor, sensor_init, &sensor_cfg, "sensor", true, HIVE_RESTART_PERMANENT},
    {motor_actor, NULL, &motor_cfg, "motor", true, HIVE_RESTART_PERMANENT},
    {altitude_actor, altitude_init, &alt_cfg, "altitude", true, HIVE_RESTART_PERMANENT},
};

// All three actors receive the same siblings array:
// siblings[0] = {"sensor", <sensor_id>, true}
// siblings[1] = {"motor", <motor_id>, true}
// siblings[2] = {"altitude", <altitude_id>, true}

void altitude_actor(void *args, const hive_spawn_info_t *siblings, size_t count) {
    // args = return value from altitude_init(&alt_cfg)
    // count = 3

    // Find motor - no whereis() needed
    hive_actor_id_t motor = find_sibling(siblings, count, "motor");

    // Already registered as "altitude" - no hive_register() needed
    // ...
}

// Helper function (could be provided by runtime)
hive_actor_id_t find_sibling(const hive_spawn_info_t *siblings, size_t count, const char *name) {
    for (size_t i = 0; i < count; i++) {
        if (siblings[i].name && strcmp(siblings[i].name, name) == 0) {
            return siblings[i].id;
        }
    }
    return ACTOR_ID_INVALID;
}
```

## Migration

All existing actor functions need signature update:

```c
// Before
void my_actor(void *arg) { ... }

// After
void my_actor(void *args, const hive_spawn_info_t *siblings, size_t count) {
    ...
}
```

All existing spawn calls need update:

```c
// Before
hive_spawn(my_actor, &args, &id);
hive_spawn_ex(my_actor, &args, &cfg, &id);

// After (both become)
hive_spawn(my_actor, NULL, &args, NULL, &id);  // Simple case: no init, no config
hive_spawn(my_actor, NULL, &args, &cfg, &id);  // With config
//                   ^     ^      ^
//                   |     |      +-- cfg (NULL = defaults)
//                   |     +--------- init_args (passed directly since init is NULL)
//                   +--------------- init (NULL = skip)
```

## Design Decisions

1. **Init function context**: Runs in spawner context (not actor context). This allows spawn to return meaningful errors if init fails or registration fails.

2. **Registration failure**: If `auto_register` is true but name is taken, spawn fails with `HIVE_ERR_EXISTS`. Actor is not created.

3. **Restart behavior**: On supervisor restart, init function is called again (fresh initialization). The restarted actor receives updated sibling info (with new actor_ids for any other restarted siblings).

4. **Sibling array lifetime**: Stack-allocated by runtime, valid only at actor function entry. Actor copies needed `hive_actor_id_t`s to local variables.

5. **Init returning NULL**: Valid. Actor receives `NULL` as `args` parameter.

6. **Supervisor two-phase start**: All children are allocated and registered before any start running. This ensures complete sibling info. If any child fails to allocate/register, all are rolled back.

7. **Unified name field**: Single `name` field serves both debugging, registry, and supervisor child identification. Removes redundancy between old `id` and `name` fields.

8. **Single spawn function**: `hive_spawn_ex()` removed. Single `hive_spawn()` with `cfg` parameter that can be NULL for defaults. Simpler API.

## Handling Sibling Restarts

If a sibling is restarted by a supervisor, its `hive_actor_id_t` changes. Actors that cache sibling IDs should:

1. Monitor siblings via `hive_monitor()`
2. Handle exit messages in their event loop
3. Use `hive_whereis()` to get the new ID after restart

```c
void my_actor(void *args, const hive_spawn_info_t *siblings, size_t count) {
    hive_actor_id_t motor = find_sibling(siblings, count, "motor");
    hive_monitor(motor);  // Get notified if motor restarts

    while (1) {
        hive_message_t msg;
        hive_ipc_recv(&msg, -1);

        if (hive_msg_is_exit(&msg)) {
            hive_exit_msg_t exit;
            hive_decode_exit(&msg, &exit);
            if (exit.actor == motor) {
                // Motor restarted, get new ID
                hive_whereis("motor", &motor);
                hive_monitor(motor);
            }
        }
        // ... handle other messages
    }
}
```

## Open Questions

1. **Init function error handling**: What if init needs to signal an error? Currently only return value is the transformed args. Options:
   - Return NULL to signal error (but NULL is valid for "no args")
   - Add error output parameter to init signature
   - Init cannot fail (document this constraint)

2. **Child with NULL name in supervisor**: If `name = NULL`, how does supervisor identify the child for restart logging/strategies? Require name to be non-NULL for supervised children?

3. **Partial restart sibling info**: With `one_for_one` restart, what siblings array does the restarted child receive?
   - Just itself (sibling_count=1)?
   - All original siblings (with current actor_ids)?
   - Recommendation: All original siblings with current IDs

4. **Memory ownership for init return value**: If init mallocs memory, who frees it? Document that actor owns the memory and must free it on exit.

5. **Sibling array storage for supervisor**: Spec says "stack-allocated" but whose stack? Needs clarification - likely allocated on supervisor's stack during start phase, copied to each child's stack when scheduled.

## Implementation Checklist

### Core Runtime Changes
- [x] `include/hive_types.h` - Add `hive_spawn_info_t`, `hive_actor_init_fn_t`
- [x] `include/hive_actor.h` - Update `actor_config_t`, `hive_actor_fn` signature
- [x] `include/hive_supervisor.h` - Update `hive_child_spec_t`
- [x] `src/hive_actor.c` - Implement new `hive_spawn()` logic
- [x] `src/hive_runtime.c` - Update actor entry point to pass siblings
- [x] `src/hive_supervisor.c` - Implement two-phase child start
- [x] Update `HIVE_ACTOR_CONFIG_DEFAULT` macro

### Documentation Updates
- [x] `README.md` - Update spawn examples, API overview
- [x] `docs/spec/` - Update actor lifecycle, spawn, supervisor sections
- [x] `CLAUDE.md` - Update spawn documentation
- [x] `man/man3/hive_spawn.3` - Rewrite for new signature
- [x] `man/man3/hive_supervisor.3` - Update child spec documentation
- [x] `man/man3/hive_types.3` - Add new types

### Test Updates (all actor signatures change)
- [x] `tests/actor_test.c`
- [x] `tests/ipc_test.c`
- [x] `tests/timer_test.c`
- [x] `tests/link_test.c`
- [x] `tests/monitor_test.c`
- [x] `tests/bus_test.c`
- [x] `tests/priority_test.c`
- [x] `tests/runtime_test.c`
- [x] `tests/timeout_test.c`
- [x] `tests/arena_test.c`
- [x] `tests/pool_exhaustion_test.c`
- [x] `tests/backoff_retry_test.c`
- [x] `tests/simple_backoff_test.c`
- [x] `tests/congestion_demo.c`
- [x] `tests/select_test.c`
- [x] `tests/tcp_test.c`
- [x] `tests/file_test.c`
- [x] `tests/registry_test.c`
- [x] `tests/supervisor_test.c`
- [x] Add new test: `tests/spawn_init_test.c` (test init function, auto-register)
- [x] Add new test: `tests/sibling_test.c` (test sibling array)

### Example Updates (all actor signatures change)
- [x] `examples/pingpong.c`
- [x] `examples/timer.c`
- [x] `examples/priority.c`
- [x] `examples/link_demo.c`
- [x] `examples/supervisor.c`
- [x] `examples/supervisor_manual.c`
- [x] `examples/bus.c`
- [x] `examples/request_reply.c`
- [x] `examples/select.c`
- [x] `examples/echo.c`
- [x] `examples/fileio.c`

### Pilot Example Updates
- [x] `examples/pilot/sensor_actor.c`
- [x] `examples/pilot/estimator_actor.c`
- [x] `examples/pilot/altitude_actor.c`
- [x] `examples/pilot/waypoint_actor.c`
- [x] `examples/pilot/position_actor.c`
- [x] `examples/pilot/attitude_actor.c`
- [x] `examples/pilot/rate_actor.c`
- [x] `examples/pilot/motor_actor.c`
- [x] `examples/pilot/flight_manager_actor.c`
- [x] `examples/pilot/pilot.c` - Update to use supervisor with new child specs

### Benchmark Updates
- [x] `benchmarks/bench.c` - Update all actor signatures and spawn calls

### QEMU Updates
- [x] `tests/qemu/test_main.c`
- [x] `tests/qemu/test_runner.c`
- [x] `tests/qemu/hive_select.c` - New select test for QEMU
- [x] Verify all QEMU tests pass
- [x] Verify all QEMU examples pass
