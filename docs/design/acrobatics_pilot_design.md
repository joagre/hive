# Acrobatics Pilot - Maneuver ROM Design

Contract between the Forth VM and the native C maneuver primitives.

For prior art, see
[ArduPilot scripted aerobatics](https://ardupilot.org/plane/docs/common-scripting-aerobatics.html)
(Lua, background thread, no timing guarantees). We do better: ROM words
run in C at control-loop rate, the Forth script only sequences them.

## How It Fits

The Forth VM runs inside a new `maneuver_actor` (TEMPORARY, NORMAL
priority). ROM words are C functions called from the inner interpreter.
They publish setpoints to the same buses the cascade uses. An override
bitmask tells cascade actors when to stop publishing.

```
                        ┌──────────────────────┐
                        │   maneuver_actor      │
                        │   (Forth VM + ROM)    │
                        └──────────┬───────────┘
                                   │
                      publishes override + setpoints
                                   │
         ┌─────────────────────────┼─────────────────────────┐
         v                         v                         v
  position_target_bus     attitude_setpoint_bus      rate_setpoint_bus
  (Level 1)               (Level 2)                  (Level 3)
         │                         │                         │
         v                         v                         v
  position_actor            attitude_actor             rate_actor
  (skips if L2 set)         (skips if L3 set)          (skips if L4 set)
         │                         │                         │
         v                         v                         v
  attitude_setpoint_bus     rate_setpoint_bus           torque_bus
                                                       (Level 4)
                                                             │
                                                             v
                                                        motor_actor
                                                   (always validates)
```

Existing actors gain ~10 lines each: subscribe to override bus, non-blocking
read at top of loop, `continue` if their level is overridden. No other
changes to the cascade.

## Injection Levels

| Level | Bus | Bypasses | Still active | Use case |
|-------|-----|----------|--------------|----------|
| 1 | position_target_bus | Nothing | Full cascade | Waypoints, orbits |
| 2 | attitude_setpoint_bus + thrust_bus | Position PD | Attitude + rate PID | Banked turns, tilts |
| 3 | rate_setpoint_bus + thrust_bus | Position + attitude | Rate PID | Flips, rolls, spins |
| 4 | torque_bus | All PIDs | Motor validation only | Raw torque |

Level 2+ must also publish to `thrust_bus` because the altitude actor is
bypassed. Levels are a bitmask - a script can override multiple levels at
once (e.g., Level 1 for position hold + Level 3 for yaw spin). Level 5
(HAL direct, no motor validation) is deliberately not implemented.

## Override Mechanism

```c
#define MANEUVER_LEVEL_NONE  0x00
#define MANEUVER_LEVEL_1     0x01
#define MANEUVER_LEVEL_2     0x02
#define MANEUVER_LEVEL_3     0x04
#define MANEUVER_LEVEL_4     0x08

typedef struct {
    uint8_t levels;         // Bitmask of active override levels
    uint32_t sequence;      // Monotonic, incremented on change
    uint64_t timestamp_us;  // For watchdog (100ms timeout)
} maneuver_override_t;
```

Published to a new `maneuver_override_bus` (added to `pilot_buses_t`).
Cascade actors check it non-blocking each cycle. If stale (>100ms), they
ignore it and resume normal operation.

On release, the ROM publishes a transition setpoint matching current state
before clearing the override bit. This avoids PID integral windup transients.

## ROM Word Reference

Stack effects use standard Forth notation. Floats are 32-bit IEEE 754
stored in cells. Flag: -1 = true, 0 = false.

### Navigation (Level 1)

| Word | Stack | Notes |
|------|-------|-------|
| `GOTO` | ( x y z yaw -- ) | Non-blocking. World frame, meters/radians. |
| `GOTO-REL` | ( dx dy dz -- ) | Offset from current position. Yaw unchanged. |
| `HOVER` | ( -- ) | Capture current position as target. |
| `ORBIT` | ( cx cy r rate -- ) | Blocks for one full circle. +rate = CCW from above. |
| `LAND` | ( -- flag ) | Notifies altitude actor. Blocks until landed or 10s timeout. |

### Attitude (Level 2)

| Word | Stack | Notes |
|------|-------|-------|
| `TILT` | ( roll pitch yaw thrust -- ) | Non-blocking. Radians, thrust 0.0-1.0. |
| `LEVEL` | ( -- ) | Roll=0, pitch=0, yaw=current, thrust=hover. Use before RELEASE. |
| `BANK` | ( angle duration_ms -- ) | Blocks. Roll hold at angle for duration. |

### Rate (Level 3)

| Word | Stack | Notes |
|------|-------|-------|
| `SPIN` | ( axis rate duration_ms -- ) | Blocks. axis: 0=roll 1=pitch 2=yaw. Other axes zeroed. |
| `FLIP-ROLL` | ( direction -- ) | Blocks ~600ms. 1=right, -1=left. Full 360. |
| `FLIP-PITCH` | ( direction -- ) | Blocks ~600ms. 1=forward, -1=backward. Full 360. |

### Torque (Level 4)

| Word | Stack | Notes |
|------|-------|-------|
| `THRUST-SET` | ( thrust -- ) | Zero roll/pitch/yaw torque. |
| `TORQUE-SET` | ( thrust roll pitch yaw -- ) | Motor actor still validates. |

### State Reading (no override needed)

| Word | Stack | Notes |
|------|-------|-------|
| `STATE@` | ( -- ptr len flag ) | Full `state_estimate_t` into data buffer. |
| `ALT@` | ( -- f ) | Altitude, meters. |
| `ROLL@` `PITCH@` `YAW@` | ( -- f ) | Attitude, radians. |
| `ROLL-RATE@` `PITCH-RATE@` `YAW-RATE@` | ( -- f ) | Angular rates, rad/s. |
| `VEL-X@` `VEL-Y@` `VVEL@` | ( -- f ) | Velocity, m/s. |
| `POS-X@` `POS-Y@` | ( -- f ) | Position, meters. |

All non-blocking. Push 0.0 if no state available.

### Safety

| Word | Stack | Notes |
|------|-------|-------|
| `FENCE` | ( x_min y_min z_min x_max y_max z_max -- ) | 3D bounding box. Checked every cycle in blocking words. Violation = ABORT. |
| `ASSERT-ALT` | ( min max -- ) | One-shot. Aborts if altitude outside range. |
| `ASSERT-TILT` | ( max_angle -- ) | One-shot. Aborts if roll or pitch exceeds limit. |
| `ABORT` | ( -- ) | Release all overrides, HOVER, exit VM with crash. |

### Timing

| Word | Stack | Notes |
|------|-------|-------|
| `WAIT-MS` | ( ms -- ) | Sleeps in 90ms chunks, updating watchdog timestamp between. |
| `WAIT-UNTIL` | ( condition -- flag ) | 0=ARRIVED 1=LEVEL 2=ALTITUDE 3=HEADING. 30s timeout. |
| `ELAPSED` | ( -- ms ) | Since script start. |

### Cascade Control

| Word | Stack | Notes |
|------|-------|-------|
| `OVERRIDE` | ( level_mask -- ) | OR'd into current mask. Level 4 requires FENCE first. |
| `RELEASE` | ( level_mask -- ) | Publishes transition setpoint before clearing bits. |
| `MODE@` | ( -- level_mask ) | Query current override state. |

## Blocking Words and the VM

ROM words that block (ORBIT, BANK, SPIN, FLIP-ROLL, FLIP-PITCH, LAND,
WAIT-MS, WAIT-UNTIL) call `hive_yield()` internally in a C loop, publishing
to the appropriate bus at 250Hz. From the VM's perspective, one opcode
takes many milliseconds. The yield interval resumes counting after the
word returns.

This is why flips are ROM words and not Forth loops - a flip needs
sub-10ms timing. The VM yields every 100 instructions (~4ms jitter).
The C state machine publishes deterministically every 4ms.

ROM word C signature:

```c
// Returns 0 on success, nonzero on abort
typedef int (*rom_word_fn)(maneuver_ctx_t *ctx);
```

The context holds VM state, buses, tunable params, override state, and
fence bounds. ROM opcodes occupy 0x60-0x7F (32 slots).

## Safety Constraints

Three independent layers. Each knows nothing about the others.

**ROM layer** (maneuver actor) - Geofence checked every cycle in blocking
words. ASSERT words for pre-maneuver checks. Watchdog timestamp must update
within 100ms or cascade actors ignore the override. Level 4 override
requires an active FENCE.

**Altitude actor** - Crash latch (altitude >2m) fires regardless of
override state. Tilt-based crash detection (>45 deg) is automatically
suspended when Level 3+ is active on the override bus - the altitude actor
already reads it, and tilt exceedance during a rate override is intentional,
not a control failure. When the override is released, tilt detection
resumes immediately. No ground station involvement needed.

**Motor actor** - Validates every torque command (NaN, magnitude limits).
50ms deadman timeout. START/STOP gating. Does not read the override bus.
Does not know maneuvers exist. Cannot be bypassed.

## Examples

### Orbit

```forth
-5 S>F -5 S>F 0 S>F 5 S>F 5 S>F 3 S>F FENCE
1 OVERRIDE
POS-X@ POS-Y@ 1 S>F 500 1000 */ S>F ORBIT
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
```

### Barrel Roll

```forth
-3 S>F -3 S>F 0 S>F 3 S>F 3 S>F 3 S>F FENCE
200 S>F 1800 S>F ASSERT-ALT
4 OVERRIDE
1 FLIP-ROLL
4 RELEASE
```

### Flip with Recovery

```forth
-3 S>F -3 S>F 0 S>F 3 S>F 3 S>F 3 S>F FENCE
300 S>F 1500 S>F ASSERT-ALT
4 OVERRIDE
1 FLIP-PITCH
LEVEL 1000 WAIT-MS
4 RELEASE
```

### Figure-8

```forth
-6 S>F -6 S>F 0 S>F 6 S>F 6 S>F 3 S>F FENCE
1 OVERRIDE
POS-X@ 2 S>F F+ POS-Y@ 2 S>F 500 1000 */ S>F ORBIT
POS-X@ 2 S>F F- POS-Y@ 2 S>F -500 1000 */ S>F ORBIT
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
```
