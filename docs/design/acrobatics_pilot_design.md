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

## Script Lifecycle

Scripts are uploaded to flash before flight (4 slots, 1KB each, see
`forth_actor_spec.md` upload protocol). The maneuver actor loads a
script at boot and waits for `NOTIFY_FLIGHT_START` from the flight
manager - the same signal the waypoint actor uses.

The script IS the mission. A scripted flight does not use the waypoint
actor. The script handles everything: takeoff via GOTO, navigation via
GOTO/ORBIT, acrobatics via FLIP-ROLL/SPIN, and landing via LAND. The
waypoint actor and maneuver actor are alternatives, not collaborators.

```
Normal flight:   flight_manager -> NOTIFY_FLIGHT_START -> waypoint_actor
Scripted flight: flight_manager -> NOTIFY_FLIGHT_START -> maneuver_actor
```

Which actor receives the start signal depends on which is spawned. The
supervisor child list selects the mission type at build time (or at
script upload time if the comms actor can swap child specs - future work).

When the script completes (returns from its main word or calls LAND),
the maneuver actor exits normally. The flight manager sees the exit and
proceeds to shutdown, same as when the waypoint actor finishes its
sequence.

If the script crashes (stack overflow, fence violation, ABORT), the
maneuver actor exits with `HIVE_EXIT_REASON_CRASH`. It is TEMPORARY,
so the supervisor does not restart it. All overrides go stale within
100ms (watchdog), the cascade resumes, and the vehicle holds position
on its last Level 1 target. The flight manager's existing landing
timeout handles the rest.

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

**ORBIT geometry** - Parametric circle in the horizontal plane:

    x(t) = cx + r * cos(theta_0 + omega * t)
    y(t) = cy + r * sin(theta_0 + omega * t)

where `omega = rate` (rad/s) and `theta_0` is the initial angle from center
to the vehicle's position at entry: `theta_0 = atan2(y_0 - cy, x_0 - cx)`.
The yaw target tracks the tangent: `yaw(t) = theta_0 + omega * t + PI/2`
(velocity vector direction). Period `T = 2*PI / |omega|`. The vehicle first
flies to the nearest point on the circle, then begins the orbit.

### Attitude (Level 2)

| Word | Stack | Notes |
|------|-------|-------|
| `TILT` | ( roll pitch yaw thrust -- ) | Non-blocking. Radians, thrust 0.0-1.0. |
| `LEVEL` | ( -- ) | Roll=0, pitch=0, yaw=current, thrust=hover. Use before RELEASE. |
| `BANK` | ( angle duration_ms -- ) | Blocks. Roll hold at angle for duration. |

**BANK dynamics** - A banked turn at roll angle `phi` with thrust `T = mg`:

    turn rate:    omega = g * tan(phi) / v
    turn radius:  R = v^2 / (g * tan(phi))
    load factor:  n = 1 / cos(phi)

The thrust must increase by the load factor to maintain altitude:
`T_banked = T_hover / cos(phi)`. BANK applies this correction automatically.
At 30 degrees, load factor is 1.15 (15% more thrust). At 60 degrees, 2.0
(double thrust - close to motor limits on a micro quad).

### Rate (Level 3)

| Word | Stack | Notes |
|------|-------|-------|
| `SPIN` | ( axis rate duration_ms -- ) | Blocks. axis: 0=roll 1=pitch 2=yaw. Other axes zeroed. |
| `FLIP-ROLL` | ( direction -- ) | Blocks ~600ms. 1=right, -1=left. Full 360. |
| `FLIP-PITCH` | ( direction -- ) | Blocks ~600ms. 1=forward, -1=backward. Full 360. |

**FLIP kinematics** - A flip is a full-rotation constant-rate maneuver
with thrust modulation. At rate `omega` (rad/s), the rotation angle:

    theta(t) = omega * t

completes at `t = 2*PI / omega`. At 12 rad/s (~690 deg/s), this takes
~524ms for the rotation itself.

The thrust vector rotates with the body. Its vertical component is:

    T_vertical(t) = T * cos(theta(t))

which goes negative during the inverted phase (`PI/2 < theta < 3*PI/2`).
To limit altitude loss, thrust is reduced during the inverted portion
(where it would push the vehicle downward) and restored in the upright
portion. The net altitude loss for a flip at hover thrust `T_h`:

    delta_z = integral_0^{2*PI/omega} (T_h * cos(omega*t) - g) dt / m
            = -g * (2*PI / omega)

At 12 rad/s: `delta_z ~ -9.81 * 0.524 ~ -5.1m` if thrust were constant.
The thrust modulation (0.3x when inverted) reduces this to roughly 0.5-1.0m
in practice. The ASSERT-ALT check before the flip ensures sufficient
altitude margin.

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
