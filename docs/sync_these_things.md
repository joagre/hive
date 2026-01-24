# Documentation Sync Checklist

This file tracks what documentation must stay in sync. Review periodically after
code changes, especially after modifying APIs, actor counts, memory config, or
adding new features.

---

## Top-Level Runtime Documentation

**Source of truth:** `spec/` directory (complete design specification)

**Must stay in sync with spec/:**

| Document | What to sync |
|----------|--------------|
| `README.md` | API overview, memory config, error handling, platform differences |
| `CLAUDE.md` | Key concepts, memory limits, HAL functions, error handling |
| `man/man3/*.3` | API signatures, return values, error codes |

**Common drift points:**
- [ ] Memory pool sizes (`HIVE_*_POOL_SIZE`, `HIVE_MAX_*`)
- [ ] Default stack sizes and arena size
- [ ] Error codes and status values
- [ ] HAL function lists
- [ ] Platform differences (Linux vs STM32)
- [ ] Bus retention policies
- [ ] Subscriber limits

---

## Pilot Example Documentation

**Source of truth:** `examples/pilot/spec/` + code

**Must stay in sync:**

| Document | What to sync |
|----------|--------------|
| `examples/pilot/README.md` | Actor list, bus diagram, HAL functions, stack measurements |
| `examples/pilot/spec/` | Architecture diagrams, actor tables, stack measurements |

**Common drift points:**
- [ ] Actor count (currently 10-11 depending on platform)
- [ ] Bus subscriptions in diagrams (must match code)
- [ ] Stack measurements (run with `STACK_PROFILE=1` after changes)
- [ ] Sensor fusion details (Kalman filter + complementary filter)
- [ ] HAL function list
- [ ] Memory footprint (flash/RAM sizes)
- [ ] Spawn order table

**Diagrams to verify against code:**
- [ ] `README.md` mermaid diagram - all bus connections
- [ ] `spec/design.md` Architecture Overview diagram - all bus connections
- [ ] `spec/evolution.md` Future Architecture diagram - intentionally simplified

**Code files to check for bus subscriptions:**
```
sensor_actor.c      - publishes to sensor_bus
estimator_actor.c   - subscribes sensor_bus, publishes state_bus
waypoint_actor.c    - subscribes state_bus, publishes position_target_bus
altitude_actor.c    - subscribes state_bus + position_target_bus, publishes thrust_bus
position_actor.c    - subscribes state_bus + position_target_bus, publishes attitude_sp_bus
attitude_actor.c    - subscribes state_bus + attitude_sp_bus, publishes rate_sp_bus
rate_actor.c        - subscribes state_bus + thrust_bus + rate_sp_bus, publishes torque_bus
motor_actor.c       - subscribes torque_bus
comms_actor.c       - subscribes sensor_bus + state_bus + thrust_bus
telemetry_logger_actor.c - subscribes sensor_bus + state_bus + thrust_bus + position_target_bus
```

---

## Sync Procedure

### After API changes:
1. Update `spec/` first (source of truth)
2. Update `README.md` to match
3. Update `CLAUDE.md` key concepts
4. Update relevant man pages
5. Run `make` to verify examples still compile

### After pilot changes:
1. Update code
2. Run with `STACK_PROFILE=1`, capture measurements
3. Update `examples/pilot/README.md` stack table
4. Update `examples/pilot/spec/implementation.md` stack table
5. Verify diagrams match bus subscriptions in code
6. Update actor counts if changed

### Periodic review:
1. Grep for specific numbers (actor counts, pool sizes, percentages)
2. Compare diagrams against code
3. Run stack profiling and compare to documented values
4. Check HAL function lists are complete

---

## Quick Verification Commands

```bash
# Check actor counts across docs
grep -rn "10-11 actors\|11 actors" README.md spec/ examples/pilot/

# Check memory footprint consistency
grep -rn "60KB\|58KB\|52KB" README.md examples/pilot/

# Check stack measurements match
grep -A15 "Stack Usage" examples/pilot/README.md examples/pilot/spec/implementation.md

# Find all bus subscriptions in pilot code
grep -rn "hive_bus_subscribe\|hive_bus_publish" examples/pilot/*.c

# Check HAL function mentions
grep -rn "hal_" examples/pilot/README.md | grep -v "\.c:"
```

---

## Last Sync

**Date:** 2026-01-23

**What was synced:**
- Stack measurements updated after fixing STACK_PROFILE build
- Kalman filter mentions added to SPEC.md
- Diagrams verified (StateBus-->Rate, TLog actor)
- Actor counts verified (10-11)
- Memory footprint verified (~60KB flash, ~58KB RAM, 52KB stack arena)
- Added examples and pilot reference to CLAUDE.md
- Full sync verification passed (all checks green)
