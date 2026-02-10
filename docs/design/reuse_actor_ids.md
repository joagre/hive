# Stable Actor IDs for Registered Actors

**Status** - Implemented

## Problem

When a supervisor restarts an actor with ONE_FOR_ONE strategy, the respawned
actor gets a new `hive_actor_id_t`. This breaks:

- **Cached IDs** - Other actors that stored the old ID from `hive_find_sibling()`
  now notify a dead actor
- **Sender filters** - `hive_select()` sources that filter on
  `{.sender = old_id}` silently ignore messages from the restarted actor
- **Direct notify** - `hive_ipc_notify(old_id, ...)` goes nowhere

With ONE_FOR_ALL this is not a problem because all actors restart together and
receive fresh sibling arrays. But ONE_FOR_ONE (independent workers) and
REST_FOR_ONE (pipelines) are broken by stale IDs.

Previous workarounds:

- Use `hive_ipc_named_notify()` instead of `hive_ipc_notify()` - fixes notify
  targets but not sender filters
- Use `HIVE_SENDER_ANY` in select sources - gives up sender authentication
- Stick with ONE_FOR_ALL - correct but heavy-handed for loosely coupled systems

## Solution

When a supervised child with `auto_register = true` is restarted, the supervisor
reuses the old actor slot so the respawned actor keeps the same
`hive_actor_id_t`. All cached references remain valid.

### Why tie it to auto_register

If you register a name, you want stable identity - that is the whole point of
naming. Making ID reuse implicit with `auto_register = true` avoids a separate
flag and prevents the confusing combination of "registered name but different
ID on restart."

If the need arises for stable IDs without name registration, a separate
`reuse_actor_id` flag can be added to `hive_child_spec_t` later.

### Implementation

**`hive_actor_config_t`** gained a `reuse_actor_id` field (default 0). When
non-zero, `hive_actor_alloc()` searches for a dead slot with the matching stale
ID before falling back to the normal free-slot search. If the reuse slot is
found, the actor keeps the old ID instead of getting `next_id++`.

**`child_state_t`** in the supervisor gained a `saved_actor_id` field. Each
restart strategy saves the old actor ID before clearing the child state:

- **one_for_one** - saves the failed child's ID
- **one_for_all** - saves all children's IDs before stopping them
- **rest_for_one** - saves the failed child's ID and all children after it

The supervisor's `spawn_child()` sets `cfg.reuse_actor_id` from
`saved_actor_id` when `auto_register = true`, then resets the saved ID.

Users never interact with `reuse_actor_id` directly - the supervisor manages
it internally.

## Design considerations

### Death-restart window

Between death and respawn, the slot exists but the actor is dead. Messages
sent to that ID during the gap are dropped silently - same as current
dead-actor behavior. The new incarnation starts with a clean mailbox,
consistent with the restart contract.

### Generation counters

`hive_actor_id_t` is a plain `uint32_t` with no generation counter. If one
were added in the future, reused slots must skip the generation bump.
Acceptable because the supervisor controls who occupies the slot - there is no
ABA risk when the same child spec always maps to the same slot.

### Links and monitors

Links and monitors are still cleared on restart. A client actor that monitors a
stable-ID actor still needs to re-establish the monitor after restart. This is
a smaller problem than stale IDs and consistent with the existing restart
contract.

### Memory cost

None. The dead actor slot already exists in the actor table. Reusing it just
means finding it by ID instead of by first-fit.

## Example

No API changes for client actors:

```c
hive_child_spec_t children[] = {
    {.start = sensor_actor,
     .name = "sensor",
     .auto_register = true,           /* implies stable ID on restart */
     .restart = HIVE_CHILD_PERMANENT,
     .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    {.start = motor_actor,
     .name = "motor",
     .auto_register = true,
     .restart = HIVE_CHILD_PERMANENT,
     .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
};

hive_supervisor_config_t cfg = {
    .strategy = HIVE_STRATEGY_ONE_FOR_ONE,  /* safe now */
    .max_restarts = 5,
    .restart_period_ms = 10000,
    .children = children,
    .num_children = 2,
};
```

If sensor crashes and restarts, motor's cached `sensor_id` from
`hive_find_sibling()` still works. Sender filters in `hive_select()` still
match. No code changes in any actor.

## Test coverage

- `tests/supervisor_test.c` test 10 - ONE_FOR_ONE preserves actor ID for
  auto_register children
- `tests/supervisor_test.c` test 11 - ONE_FOR_ALL preserves actor IDs for
  auto_register children
- `tests/supervisor_test.c` test 12 - Non-registered children get new IDs
  (existing behavior preserved)
