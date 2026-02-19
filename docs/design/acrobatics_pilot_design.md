# Acrobatics Pilot - Maneuver ROM Design

Contract between the Forth VM and the native C maneuver primitives.
See [forth_actor_spec.md](forth_actor_spec.md) for the VM internals
(bytecode format, primitives, upload protocol, safety constraints).

For prior art, see
[ArduPilot scripted aerobatics](https://ardupilot.org/plane/docs/common-scripting-aerobatics.html)
(Lua, background thread, no timing guarantees). We do better: ROM words
run in C at control-loop rate, the Forth script only sequences them.

## How It Fits

For the full pipeline diagram (all 12-13 actors and 7 buses), see
[examples/pilot/README.md - Architecture](../../examples/pilot/README.md#architecture).

The Forth VM runs inside a new `maneuver_actor` (TEMPORARY, NORMAL
priority). ROM words are C functions called from the inner interpreter.
They publish setpoints to the same buses the cascade uses. An override
bitmask tells cascade actors when to stop publishing.

```
                        ┌────────────────────┐
                        │   maneuver_actor   │
                        │  (Forth VM + ROM)  │
                        └──────────┬─────────┘
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
[forth_actor_spec.md](forth_actor_spec.md) upload protocol). The maneuver actor loads a
script at boot and waits for `NOTIFY_FLIGHT_START` from the flight
manager - the same signal the waypoint actor uses.

The script IS the mission. The waypoint actor and maneuver actor are
alternatives, not collaborators. Which one is spawned selects the
mission type:

```
Normal flight:   flight_manager -> NOTIFY_FLIGHT_START -> waypoint_actor
Scripted flight: flight_manager -> NOTIFY_FLIGHT_START -> maneuver_actor
```

When the script completes or calls LAND, the maneuver actor exits
normally. The flight manager proceeds to shutdown.

If the script crashes (stack overflow, fence violation, ABORT), all
overrides go stale (100ms watchdog) and the cascade resumes on the
last Level 1 position target. If no target was published before the
crash, the vehicle drifts until the altitude ceiling or the flight
manager's landing timeout.

Every script should therefore establish a safety baseline early:

    FENCE ... HOVER 1 OVERRIDE

before escalating to higher levels. Convention, not enforced by the ROM.

## Injection Levels

| Level | Bus | Bypasses | Still active | Use case |
|-------|-----|----------|--------------|----------|
| 1 | position_target_bus | Nothing | Full cascade | Waypoints, orbits |
| 2 | attitude_setpoint_bus + thrust_bus | Position PD | Attitude + rate PID | Banked turns, tilts |
| 3 | rate_setpoint_bus + thrust_bus | Position + attitude | Rate PID | Flips, rolls, spins |
| 4 | torque_bus | All PIDs | Motor validation only | Raw torque |

Levels are a bitmask - a script can override multiple simultaneously.
Level 5 (HAL direct, no motor validation) is deliberately not implemented.

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

There is a 1-cycle gap (~4ms) on override and release. On override: the
cascade actor sees the flag and stops publishing, but the maneuver actor
hasn't published its first setpoint yet. The downstream actor runs on the
stale bus value from the previous cycle. On release: the ROM publishes a
transition setpoint matching current state before clearing the bit, so
the cascade resumes with near-zero error. The gap is bounded by one
control period and invisible in practice - the motor actor's 50ms deadman
is an order of magnitude larger.

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
Thrust is reduced during this portion and restored upright. The net
altitude loss for a flip at hover thrust `T_h`:

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

Blocking ROM words (ORBIT, BANK, SPIN, FLIP-ROLL, FLIP-PITCH, LAND,
WAIT-MS, WAIT-UNTIL) call `hive_yield()` internally, publishing to the
appropriate bus every 4ms. From the VM's perspective, one opcode takes
many milliseconds. The yield interval resumes counting after the word
returns.

This is why flips are ROM words and not Forth loops - the VM's 100-
instruction yield interval introduces ~4ms jitter. The C state machine
is deterministic.

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
words. ASSERT words for pre-maneuver checks. Level 4 requires FENCE.

**Altitude actor** - Crash latch (altitude >2m) fires regardless of
override state. Tilt-based crash detection (>45 deg) is automatically
suspended when Level 3+ is active - tilt exceedance during a rate
override is intentional. Resumes on release.

**Motor actor** - Validates every torque command (NaN, magnitude limits).
50ms deadman timeout. START/STOP gating. Does not read the override bus.
Cannot be bypassed.

## Examples

Each example is a complete mission: fence, takeoff, maneuver, land.

### Orbit Mission

```forth
( Safety baseline )
-5 S>F -5 S>F 0 S>F 5 S>F 5 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1m )
0 S>F 0 S>F 1 S>F 0 S>F GOTO
2 WAIT-UNTIL DROP              ( wait for altitude )

( Orbit: 2m diameter, 0.5 rad/s )
POS-X@ POS-Y@ 1 S>F 500 1000 */ S>F ORBIT

( Land )
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
LAND DROP
```

### Barrel Roll Mission

```forth
-3 S>F -3 S>F 0 S>F 3 S>F 3 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1.5m - need margin for altitude loss during flip )
0 S>F 0 S>F 1500 1000 */ S>F 0 S>F GOTO
2 WAIT-UNTIL DROP

( Flip )
200 S>F 1800 S>F ASSERT-ALT
4 OVERRIDE
1 FLIP-ROLL
4 RELEASE

( Stabilize and land )
HOVER 1000 WAIT-MS
1 RELEASE
LAND DROP
```

### Flip with Recovery Mission

```forth
-3 S>F -3 S>F 0 S>F 3 S>F 3 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1.5m )
0 S>F 0 S>F 1500 1000 */ S>F 0 S>F GOTO
2 WAIT-UNTIL DROP

( Forward flip )
300 S>F 1500 S>F ASSERT-ALT
4 OVERRIDE
1 FLIP-PITCH
LEVEL 1000 WAIT-MS
4 RELEASE

( Return to start and land )
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
LAND DROP
```

### Figure-8 Mission

```forth
-6 S>F -6 S>F 0 S>F 6 S>F 6 S>F 3 S>F FENCE
1 OVERRIDE
HOVER

( Takeoff to 1m )
0 S>F 0 S>F 1 S>F 0 S>F GOTO
2 WAIT-UNTIL DROP

( Two orbits forming figure-8 )
POS-X@ 2 S>F F+ POS-Y@ 2 S>F 500 1000 */ S>F ORBIT
POS-X@ 2 S>F F- POS-Y@ 2 S>F -500 1000 */ S>F ORBIT

( Land )
HOVER 0 WAIT-UNTIL DROP
1 RELEASE
LAND DROP
```
