# Stable Actor IDs for Registered Actors

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

Current workarounds:

- Use `hive_ipc_named_notify()` instead of `hive_ipc_notify()` - fixes notify
  targets but not sender filters
- Use `HIVE_SENDER_ANY` in select sources - gives up sender authentication
- Stick with ONE_FOR_ALL - correct but heavy-handed for loosely coupled systems

## Proposal

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

### Spawn mechanism

Add a field to `hive_actor_config_t`:

```c
typedef struct {
    // ... existing fields ...
    hive_actor_id_t reuse_id;  /* 0 = allocate new, non-zero = reuse slot */
} hive_actor_config_t;
```

The supervisor sets `reuse_id` internally before respawning - users never touch
it directly. `hive_spawn()` checks this field: if non-zero, it claims that
specific slot instead of searching the actor table.

An alternative is a separate `hive_spawn_ex()` variant to keep the normal
spawn path clean.

### Supervisor changes

When a child with `auto_register = true` dies:

1. Supervisor records the old `hive_actor_id_t`
2. Actor slot is marked "reserved" instead of freed
3. On respawn, supervisor sets `reuse_id = old_id` in the actor config
4. `hive_spawn()` reuses the slot, preserving the ID
5. Name registration transfers to the new incarnation

## Design considerations

### Death-restart window

Between death and respawn, the slot exists but the actor is dead. Messages
sent to that ID during the gap need defined behavior:

- **Drop silently** - Same as current dead-actor behavior. Simple, predictable.
  The new incarnation starts clean.
- **Queue for new incarnation** - Tempting but violates the restart contract
  ("mailbox empty on restart"). Stale messages from the previous life could
  confuse the new actor.

Recommendation: drop silently. The clean-slate contract is more important than
message preservation.

### Generation counters

If `hive_actor_id_t` encodes a generation counter (to detect use-after-free on
reused slots), permanent IDs must skip the generation bump for reserved slots.
This is a deliberate tradeoff: giving up stale-reference detection in exchange
for stable identity.

Acceptable because the supervisor controls who occupies the slot - there is no
ABA risk when the same child spec always maps to the same slot.

### Links and monitors

Links and monitors are still cleared on restart. A client actor that monitors a
stable-ID actor still needs to re-establish the monitor after restart. This is
a smaller problem than stale IDs and consistent with the existing restart
contract.

### Memory cost

None. The actor slot (stack, mailbox) is already allocated. Reserving it just
prevents the slot from being reclaimed during the restart window.

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
