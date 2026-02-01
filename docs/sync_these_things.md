# Documentation Sync Checklist

This file tracks what documentation must stay in sync. Review periodically after
code changes, especially after modifying APIs, actor counts, memory config, or
adding new features.

---

## Documentation Structure

```
docs/spec/                           # Runtime specification (source of truth)
  README.md                     # Overview and navigation
  design.md                     # Architecture, scheduling, memory model
  api.md                        # Complete API reference
  internals.md                  # Implementation details, HAL

docs/                           # Internal working documents
  sync_these_things.md          # This file
  design/                       # Design documents
  reviews/                      # Review feedback

examples/pilot/
  README.md                     # Usage and build instructions
  docs/spec/                         # Pilot specification
    README.md                   # Overview
    design.md                   # Architecture, supervision, error handling
    implementation.md           # Control algorithms, HAL, file structure
    evolution.md                # Architecture roadmap, future work
  docs/
    first_flight_checklist.md   # Hardware bring-up guide

man/man3/                       # API man pages
```

---

## Top-Level Runtime Documentation

**Source of truth** - `docs/spec/` directory

**Must stay in sync with docs/spec/**

| Document | What to sync |
|----------|--------------|
| `README.md` | API overview, memory config, error handling, platform differences |
| `CLAUDE.md` | Key concepts, memory limits, HAL functions, error handling |
| `man/man3/*.3` | API signatures, return values, error codes, code examples |

**Common drift points**
- [ ] Memory pool sizes (`HIVE_*_POOL_SIZE`, `HIVE_MAX_*`)
- [ ] Default stack sizes and arena size
- [ ] Error codes and status values
- [ ] HAL function lists
- [ ] Platform differences (Linux vs STM32)
- [ ] Bus retention policies
- [ ] Subscriber limits
- [ ] Type names in code examples (all typedefs must use `_t` suffix)

---

## Type Naming Convention

**All typedefs must end with `_t` suffix.** This includes:
- `actor_id_t`, `bus_id_t`, `timer_id_t`
- `hive_message_t`, `hive_status_t`, `hive_spawn_info_t`
- `actor_config_t`, `child_spec_t`, `supervisor_config_t`
- `hive_bus_config_t`, `hive_recv_filter_t`, `hive_exit_msg_t`
- `hive_select_source_t`, `hive_select_result_t`
- All internal types: `arena_block_t`, `bus_entry_t`, etc.

**Files to check for type name consistency:**
- `docs/spec/api.md` - all code examples
- `README.md` - API overview examples
- `CLAUDE.md` - key concepts examples
- `man/man3/*.3` - all man page examples
- `include/hive_*.h` - comment examples
- `docs/design/*.md` - design document examples

---

## Pilot Example Documentation

**Source of truth** - `examples/pilot/docs/spec/` + code

**Must stay in sync**

| Document | What to sync |
|----------|--------------|
| `examples/pilot/README.md` | Actor list, bus diagram, HAL functions, stack measurements |
| `examples/pilot/docs/spec/` | Architecture diagrams, actor tables, stack measurements |

**Common drift points**
- [ ] Actor count (currently 10-11 depending on platform)
- [ ] Bus subscriptions in diagrams (must match code)
- [ ] Stack measurements (run with `STACK_PROFILE=1` after changes)
- [ ] Sensor fusion details (Kalman filter + complementary filter)
- [ ] HAL function list
- [ ] Memory footprint (flash/RAM sizes)
- [ ] Spawn order table

**Diagrams to verify against code**
- [ ] `README.md` mermaid diagram - all bus connections
- [ ] `docs/spec/design.md` Architecture Overview diagram - all bus connections
- [ ] `docs/spec/evolution.md` Future Architecture diagram - intentionally simplified

**Code files to check for bus subscriptions**
```
sensor_actor.c      - publishes sensor_bus
estimator_actor.c   - subscribes sensor_bus, publishes state_bus
waypoint_actor.c    - subscribes state_bus, publishes position_target_bus
altitude_actor.c    - subscribes state_bus + position_target_bus, publishes thrust_bus
position_actor.c    - subscribes state_bus + position_target_bus, publishes attitude_sp_bus
attitude_actor.c    - subscribes state_bus + attitude_sp_bus, publishes rate_sp_bus
rate_actor.c        - subscribes state_bus + thrust_bus + rate_sp_bus, publishes torque_bus
motor_actor.c       - subscribes torque_bus
comms_actor.c       - subscribes sensor_bus + state_bus + thrust_bus
telemetry_logger_actor.c - subscribes sensor_bus + state_bus + thrust_bus + position_target_bus
flight_manager_actor.c   - no bus usage (uses IPC for control commands)
```

---

## Sync Procedure

### After API changes:
1. Update `docs/spec/` first (source of truth)
2. Update `README.md` to match
3. Update `CLAUDE.md` key concepts
4. Update relevant man pages
5. Run `make` to verify examples still compile
6. Run `make test` to verify tests pass

### After typedef changes:
1. Update all typedefs in headers
2. Update all source files
3. Update all examples, tests, benchmarks
4. Update all documentation code examples
5. Update man page examples
6. Run verification commands below

### After pilot changes:
1. Update code
2. Run with `STACK_PROFILE=1`, capture measurements
3. Update `examples/pilot/README.md` stack table
4. Update `examples/pilot/docs/spec/implementation.md` stack table
5. Verify diagrams match bus subscriptions in code
6. Update actor counts if changed

### Periodic review:
1. Grep for specific numbers (actor counts, pool sizes, percentages)
2. Compare diagrams against code
3. Run stack profiling and compare to documented values
4. Check HAL function lists are complete
5. Check type names in all documentation examples

---

## Quick Verification Commands

```bash
# Check actor counts across docs
grep -rn "10-11 actors\|11 actors\|10 actors" README.md docs/spec/ examples/pilot/

# Check memory footprint consistency
grep -rn "60KB\|58KB\|52KB" README.md examples/pilot/

# Check stack measurements match
grep -A15 "Stack Usage" examples/pilot/README.md examples/pilot/docs/spec/implementation.md

# Find all bus subscriptions in pilot code
grep -rn "hive_bus_subscribe\|hive_bus_publish" examples/pilot/*.c

# Check HAL function mentions
grep -rn "hal_" examples/pilot/README.md | grep -v "\.c:"

# Check for old type names without _t suffix (should return nothing)
grep -rn '\b\(actor_id\|hive_message\|hive_status\|bus_id\|timer_id\)\b' \
  --include='*.md' --include='*.3' . | grep -v '_t'

# Verify typedef endings in headers
grep -rn 'typedef' include/ src/hal/ | grep '} [a-z_]*;$' | grep -v '_t;$'
```

---

## Last Sync

**Date** - 2026-01-24

**What was synced**
- Added `_t` suffix to all typedefs for C convention compliance:
  - `actor_id` -> `actor_id_t`, `bus_id` -> `bus_id_t`, `timer_id` -> `timer_id_t`
  - `hive_message` -> `hive_message_t`, `hive_status` -> `hive_status_t`
  - All struct/enum typedefs updated (133 files changed)
- Updated all code examples in documentation to use correct type names
- Removed unnecessary `(void)arg;` lines from documentation examples
- Updated man pages with correct type names and signatures
- All tests pass (Linux 19/19, QEMU 12/12)

**Previous sync (2026-01-24)**
- Documentation reorganization:
  - Split former top-level SPEC.md into `docs/spec/` directory
  - Split former pilot SPEC.md into `examples/pilot/docs/spec/`
  - Organized `docs/` -> `design/` and `reviews/` subdirectories
- Converted ASCII diagram to mermaid in `docs/spec/design.md`
- Updated README.md with "Why Hive?" section

**Previous sync (2026-01-23)**
- Stack measurements updated after fixing STACK_PROFILE build
- Kalman filter mentions added
- Diagrams verified (StateBus-->Rate, TLog actor)
- Actor counts verified (10-11)
- Memory footprint verified (~60KB flash, ~58KB RAM, 52KB stack arena)
