# Tuning Log

Parameter changes and flight test results for the Crazyflie 2.1+
running the Hive pilot autopilot. Newest session first.

Hardware: Crazyflie 2.1+ with Flow Deck v2, SD card deck, 3g crash cage.

---

## Tuning Plan

### Goal

Precise XY position hold (< 0.10m drift), clean altitude (< 10% overshoot,
no oscillation), and stable attitude. In that priority order.

### Current state (compiled defaults in hal_config.h)

| Parameter | Value | Status |
|-----------|-------|--------|
| `att_kp` | 1.8 | Stable (session 2) |
| `att_kd` | 0.10 | Stable (session 2) |
| `rate_kp` | 0.020 | Stable (session 2) |
| `rate_ki` | 0.001 | Stable (session 2) |
| `rate_kd` | 0.0015 | Stable (session 2) |
| `rate_yaw_kp` | 0.12 | Stable (session 7) |
| `rate_yaw_ki` | 0.05 | Stable (session 7) |
| `alt_kp` | 0.06 | Reduced (session 14), overshoot +25% at this value |
| `alt_ki` | 0.005 | Needs work (weak integral, slow steady-state) |
| `vvel_damping` | 0.55 | Keep (0.75 worsened overshoot, see session 13) |
| `pos_kp` | 0.02 | Gentle position hold (session 14) |
| `pos_kd` | 0.05 | Reduced from 0.20 (session 13, real flow was too strong) |
| `max_tilt_angle` | 0.25 (14 deg) | OK |
| `liftoff_climb_rate` | 0.15 | Reduced from 0.2 (session 14) |
| `FLOW_SCALE` | 0.0005 | Moved from HAL to config.h (session 15). May need 4x increase (see session 11) |

### Architectural analysis

The fundamental problem was asymmetry between vertical and horizontal
estimation. Altitude had a 3-state KF; horizontal had raw flow (no
filtering, no bias estimation). Session 9 closed this gap with a
horizontal KF mirroring the altitude KF.

See [Estimator Evolution](docs/spec/future.md#estimator-evolution)
for architecture details and the EKF roadmap.

### Phase 1 - Horizontal Kalman filter - DONE (session 9)

Implemented `fusion/horizontal_kf.c` / `.h` mirroring `altitude_kf.c`.
Two independent 1D filters (X and Y), each with states [position,
velocity, accel_bias]. Velocity measurement from optical flow. Innovation
gating at 1.0 m/s rejects flow outliers. Full rotation matrix in
estimator_actor.c provides world-frame acceleration for all three axes.

Sim-validated in Webots: 7-8 cm hover RMS, velocity jitter 0.005 m/s.
See session 9 for full results and default parameters.

### Phase 2 - Validate HKF on hardware, then enable position hold

**Goal** - XY drift < 0.10m during 6s FIRST_TEST flight.

**Phase 2a - HKF validation (pos_kp=0, damping only) - DONE (session 13)**

Validated in tests 50-52. The HKF produces clean velocity on real
hardware with default parameters (hkf_r_velocity=0.01). No tuning
needed. Key results from test 52:
- XY error 0.061m (target was <0.10m) - BEST EVER
- Attitude 1-4 deg during hover (vs 25-51 deg in session 8)
- Successful landing

Discovery: pos_kd=0.20 caused growing oscillations with real flow data
(test 50, max tilt 44 deg). With 250x-corrected flow (session 11),
the damping response is much stronger. Reduced pos_kd to 0.05.

**Phase 2b - Position hold (re-enable pos_kp) - IN PROGRESS (session 14)**

Test 60 enabled pos_kp=0.02 via radio. Cannot evaluate position hold
until accelerometer bias calibration is validated - the uncorrected
Y-axis bias (0.134 m/s^2) causes 3m drift that overwhelms pos_kp.

With validated HKF velocity, re-enable pos_kp. Tune via radio.
The control law is: `accel = pos_kp * error - pos_kd * velocity`.

**Important** - pos_kd must stay at 0.05 (not 0.20). The old value
was tuned when flow was 250x too small. With real flow, 0.20 causes
a feedback loop between altitude oscillation and attitude.

| Step | pos_kp | pos_kd | Rationale |
|------|--------|--------|-----------|
| 1 | 0.02 | 0.05 | Very gentle start with new damping value |
| 2 | 0.04 | 0.05 | If step 1 shows drift reduction |
| 3 | 0.06 | 0.05 | Moderate hold |
| 4 | 0.08 | 0.05 | Stronger hold if step 3 is good |

**Stop condition** - If max tilt exceeds 25 deg, back off pos_kp.
If drift < 0.10m, lock in the value and move to phase 4.

### Phase 3 - Tame altitude overshoot - IN PROGRESS (session 14)

**Goal** - Altitude overshoot < 10%, thrust oscillation < 1/s.

Current: alt_kp=0.06, alt_ki=0.005, vvel_damping=0.55, alt_omax=0.20,
liftoff_climb_rate=0.15.

Session 14 progress:
- **alt_kp 0.08 -> 0.06** (radio): overshoot +57% -> +25% improvement
- **liftoff_climb_rate 0.2 -> 0.15** (radio): overshoot +59% -> +45%
- **Combined (test 60)**: +25% overshoot, -1.2% steady-state error
- Still above 10% target. Next steps: increase alt_ki or reduce
  liftoff_thrust_corr.

Session 13 findings:
- **Increasing vvel_damping made overshoot WORSE** (test 53: +156%).
  Root cause: feedforward = climb_rate * vvel_damping. Higher damping
  increases feedforward, launching the drone faster and building more
  momentum. The damping and feedforward are coupled through the same
  parameter.

**Remaining tuning steps** (via radio):

| Step | Parameter | Value | Rationale |
|------|-----------|-------|-----------|
| 1 | `alt_ki` | 0.01 | Increase integral to close steady-state gap |
| 2 | `liftoff_thrust_corr` | 0.88 | More aggressive thrust reduction at liftoff |

### Phase 4 - Position integral for steady-state hold

**Goal** - Zero steady-state position error under persistent disturbance.

The PD position controller (pos_kp + pos_kd) has zero steady-state
error only if the position estimate is perfect. In practice, small
persistent forces (CG offset, asymmetric prop wash, flow bias) cause
constant drift that the P term can track but never fully cancel.

Add integral term to position controller with anti-windup:
```
x_integral += x_error * dt
x_integral = clamp(x_integral, -pos_imax, pos_imax)
accel_x = pos_kp * x_error + pos_ki * x_integral - pos_kd * velocity
```

Start with pos_ki=0.01, pos_imax=0.05 (limits integral authority to
~3 deg tilt). Tune via radio.

### Phase 5 - Precision and longer missions

With stable hover in all axes, push toward perfection:

1. **Extend to ALTITUDE profile** (40s, altitude waypoints 0.3-1.0m).
   Validate waypoint transitions, altitude tracking accuracy.

2. **Validate consecutive flights** - Multi-GO on same power cycle.
   The RESET race condition is fixed; verify in flight.

3. **Landing optimization** - Landing takes 8.5s from 0.6m. Consider
   increasing landing_descent_rate or switching to a proportional
   descent (faster from higher, slower near ground).

4. **Yaw hold** - Yaw drifted 51 deg/10s in session 3. The improved
   yaw PID (session 7) should be better. Measure and tune if needed.
   Yaw drift corrupts the horizontal KF's world-frame rotation.

5. **Full 3D waypoints** - FULL_3D profile with XY waypoints.
   Requires position control to be solid before attempting.

---

## Session 15 - 2026-02-24 - HAL separation of concerns (architecture)

Moved all signal processing from platform-specific HAL code to
platform-independent sensor_actor.c. No tuning parameter changes, no
behavioral change on hardware. Architecture-only refactor.

### Motivation

The HAL should do ONLY: raw hardware access, scaling to physical units,
axis alignment/negation, and bias subtraction. Both HALs contained
signal processing that violated this:

- **Crazyflie HAL** (platform.c): pixel-to-velocity conversion,
  rotational flow compensation, yaw dead-reckoning, body-to-world
  rotation, position integration
- **Webots HAL** (hal_sensors.c): velocity differentiation from GPS,
  height range validation

### Changes

Added raw HAL fields to `sensor_data_t` (types.h):
- `range_height`, `range_valid` - raw rangefinder (VL53L1x or GPS Z)
- `flow_dpixel_x/y`, `flow_valid` - raw optical flow pixel deltas
- `raw_gps_x/y/z`, `raw_gps_valid` - raw GPS position (Webots only)

Existing `gps_x/y/z` and `velocity_x/y` fields now populated by
sensor_actor instead of HAL.

Added `flow_process()` to sensor_actor.c with three paths:
1. **Optical flow (Crazyflie)** - pixel-to-velocity, rotation
   compensation, yaw dead-reckoning, body-to-world, integration
2. **GPS (Webots)** - pass through position, differentiate for velocity
3. **Range only (fallback)** - altitude from range, hold last position

Moved flow constants to config.h: `FLOW_SCALE` (was `HAL_FLOW_SCALE`),
`FLOW_MIN_HEIGHT`, `FLOW_MAX_HEIGHT`.

### Noise model fix

Initial implementation put XY noise on `raw_gps_x/y` in the Webots HAL.
This broke velocity differentiation - noise amplifies through the
derivative: sigma_v = sigma_pos * sqrt(2) / dt = 0.002 * 1.414 / 0.004
= 0.707 m/s (vs HKF expected 0.1 m/s). Also caused false liftoff
detection when altitude noise pushed ground-level readings above the
0.02m threshold.

Fixed by providing clean GPS in `raw_gps_x/y/z` (for differentiation)
and noisy altitude only in `range_height` (simulating VL53L1x noise).
This matches the old code's behavior exactly.

### Code changes

| File | Change |
|------|--------|
| `include/types.h` | Added raw range/flow/GPS fields to sensor_data_t |
| `config.h` | Added FLOW_SCALE, FLOW_MIN_HEIGHT, FLOW_MAX_HEIGHT |
| `sensor_actor.c` | Added flow_process() with 3 paths, reset on RESET |
| `hal/crazyflie-2.1plus/platform.c` | Stripped flow integration, output raw fields |
| `hal/crazyflie-2.1plus/hal_config.h` | Removed HAL_FLOW_SCALE |
| `hal/webots-crazyflie/hal_sensors.c` | Stripped velocity diff, output clean raw GPS |
| `hal/webots-crazyflie/hal_internal.h` | Removed prev_gps state, FLOW height defines |
| `hal/webots-crazyflie/hal_init.c` | Removed g_prev_gps definitions |

### Verification (Webots sim)

| Profile | Noise | Waypoints | Verdict |
|---------|-------|-----------|---------|
| FIRST_TEST | 0 | 1/1 | OK |
| FULL_3D | 3 | 5/5 | OK |

Crazyflie builds clean (FIRST_TEST and FULL_3D profiles). Hardware
flight test pending.

### Impact on tuning

None. Same numerical processing, just moved to a different file. All
existing tuning parameters and compiled defaults unchanged. The
estimator reads the same `gps_z`, `velocity_x/y` fields regardless
of who populated them.

---

## Session 14 - 2026-02-24 - Altitude progress, accel bias, GO drain (tests 56-62)

Continued from session 13. Three major outcomes: altitude overshoot
reduced from +63% to +25%, discovered accelerometer bias as root cause
of 3m XY drift, and fixed a safety bug where stale GO messages caused
auto-restarts.

### Key discovery - accelerometer Y-axis bias causes all XY drift

Analysis of test 60 hover data revealed:
- Mean accel_y = +0.134 m/s^2 during "level" hover
- Mean flow VY = -0.008 m/s (near zero - no real motion detected by flow)
- HKF position stayed near (0,0) while drone drifted 3m right
- Physics: 0.5 * 0.134 * 7^2 = 3.3m - matches observed drift exactly

The complementary filter sees the biased accelerometer and computes a
"level" attitude that is actually tilted 0.78 degrees. This produces
constant lateral acceleration. The HKF cannot detect or correct this
because the flow sensor confirms near-zero velocity (the drone IS moving
at constant acceleration, but slowly enough that flow noise dominates).

Root cause: BMI088 accelerometer has per-axis offset that was never
calibrated. The existing calibration only computed a scale factor (gravity
magnitude), not per-axis bias.

### Key fix - per-axis accelerometer bias calibration

Added to `platform_calibrate()` in platform.c:
1. Sample 200 accelerometer readings over 1 second on a level surface
2. Apply existing scale factor for correct units
3. Compute bias as deviation from expected values: X=0, Y=0, Z=+9.81
4. Subtract bias from every subsequent sensor reading

Initial implementation had a Z-axis sign error (tests 61-62 failed -
Z-up convention means accel_z = +9.81 at rest, not -9.81). Fixed to
`bias_z = mean_z - GRAVITY`.

Awaiting flight verification with corrected code (flashed, self-test OK).

### Key fix - stale GO messages causing auto-restarts (safety bug)

The drone auto-started flights twice without GO being sent, including
once under a sofa. Root cause: ESB auto-retransmit causes duplicate
NOTIFY_GO messages in flight_manager's mailbox. After landing, stale
GO messages were immediately consumed when returning to IDLE, triggering
an immediate re-flight.

Fixed by draining stale GO messages in FM_STATE_LANDED before
transitioning to IDLE.

### Compiled default changes

| Parameter | Old | New | Reasoning |
|-----------|-----|-----|-----------|
| `alt_kp` | 0.08 | 0.06 | Radio-tuned: overshoot +57% at 0.08, +25% at 0.06 |
| `pos_kp` | 0.0 | 0.02 | Gentle position hold enabled (session 14) |
| `LIFTOFF_CLIMB_RATE` | 0.2 | 0.15 | Slower ramp: overshoot +59% at 0.2, +45% at 0.15 |

### Code changes

| File | Change |
|------|--------|
| `hal/crazyflie-2.1plus/platform.c` | Per-axis accel bias calibration (200 samples at startup) |
| `hal/crazyflie-2.1plus/platform.c` | Bias subtraction in sensor reading |
| `flight_manager_actor.c` | Drain stale GO messages in FM_STATE_LANDED |
| `tools/ground_station.py` | Added param IDs 55-59 to PARAM_NAMES dict |
| `hal/crazyflie-2.1plus/hal_config.h` | alt_kp 0.08 -> 0.06, pos_kp 0.0 -> 0.02 |
| `config.h` | LIFTOFF_CLIMB_RATE 0.2 -> 0.15 |

### Flight test results

| Test | alt_kp | climb_rate | pos_kp | Overshoot | Max tilt | XY drift | Notes |
|------|--------|------------|--------|-----------|----------|----------|-------|
| 56 | 0.08 | 0.2 | 0.0 | +57% | 11.2 deg | 2m | KF diverged post-landing |
| 57 | 0.06 | 0.2 | 0.0 | +59% | 8.9 deg | 2m | Clean landing |
| 58 | 0.06 | 0.15 | 0.0 | +45% | 15.6 deg | 2m | Climb rate helped |
| 59 | 0.06 | 0.15 | 0.02 | +48% | low | 2m | Stale KF packet confused summary |
| 60 | 0.06 | 0.15 | 0.02 | +25% | 30.5 deg | 3m | Best altitude! Accel bias found |
| 61 | 0.06 | 0.15 | 0.02 | N/A | - | - | Z bias bug, 0.3s flight |
| 62 | 0.06 | 0.15 | 0.02 | N/A | - | - | Z bias bug, 0.6s flight |

All params set via radio except test 56 (compiled defaults from session 13).

### Analysis

**Test 56** - Baseline with session 13 defaults. Overshoot +57%. The KF
diverged to -2.3m post-landing (accel bias drift with no rangefinder
correction). 2m XY drift to the right.

**Test 57** - alt_kp reduced to 0.06 via radio. Overshoot still +59%
(climb rate dominates the initial overshoot, not kp). Clean landing.
Same 2m drift pattern.

**Test 58** - liftoff_climb_rate reduced to 0.15. Overshoot dropped to
+45% - first clear improvement. Slower climb = less momentum at target.
Max tilt 15.6 deg during hover.

**Test 60** - Best flight of the session. +25% overshoot with -1.2%
steady-state error (integral is working). pos_kp=0.02 active but
overwhelmed by accel bias drift. Max roll 30.5 deg (single excursion).
Drone drifted 3m right into a sofa, steady hover throughout.

**Tests 61-62** - Accel bias calibration active but Z-axis sign was
wrong. Computed bias_z = +19.62 (should have been ~0). This flipped
gravity's sign in the KF, causing immediate altitude divergence to
negative values. Drone props started briefly then stopped (NO LIFTOFF).
Fixed Z formula and reflashed.

### Phase status

- **Phase 2b**: In progress. pos_kp=0.02 active but blocked on accel bias validation.
- **Phase 3**: Progress. Overshoot 63% -> 25%. Next: alt_ki increase.

---

## Session 13 - 2026-02-24 - First flights with corrected flow (tests 50-53)

First hardware flights since session 11's flow sensor fixes and session
12's sign convention unification. Phase 2a validated, altitude overshoot
is now the blocking problem.

### Key discovery - flow fixes work

For the first time in 52 tests, the drone holds XY position. The flow
sensor bug fixes (session 11: /dt + axis mapping) and sign convention
fix (session 12: flow gyro compensation) produce clean HKF velocity on
real hardware. No HKF parameter tuning needed.

### Key discovery - feedforward/damping coupling

Increasing vvel_damping from 0.55 to 0.75 (test 53) made overshoot
WORSE (+156% vs +63%). Root cause: the altitude controller's feedforward
term is `feedforward = liftoff_climb_rate * vvel_damping`. Higher damping
increases feedforward during the climb ramp, launching the drone faster
and building more momentum at target. This coupling was not obvious from
the code and invalidates the phase 3 tuning plan's step 1.

### Compiled default changes

| Parameter | Old | New | Reasoning |
|-----------|-----|-----|-----------|
| `alt_kp` | 0.12 | 0.08 | 0.12 overshot 103% (test 50), 0.08 overshot 63% (test 52) |
| `pos_kd` | 0.20 | 0.05 | 0.20 caused growing oscillation with real flow data (test 50) |
| `LIFTOFF_CLIMB_RATE` | 0.3 | 0.2 | Slower climb = less momentum at target |

### Code changes

| File | Change |
|------|--------|
| `config.h` | LIFTOFF_CLIMB_RATE 0.3 -> 0.2 |
| `hal/crazyflie-2.1plus/hal_config.h` | alt_kp 0.12 -> 0.08, pos_kd 0.20 -> 0.05 |
| `tunable_params.h` | 5 new params (55-59): liftoff_climb_rate, liftoff_ramp_rate, liftoff_thrust_corr, kf_max_innov, hkf_max_innov |
| `tunable_params.c` | Metadata and init for new params |
| `altitude_actor.c` | Use tunable params for climb rate, ramp rate, thrust correction |
| `estimator_actor.c` | Use tunable params for KF/HKF innovation gating |

### Flight test results

| Test | alt_kp | pos_kd | vvel_damp | Duration | Overshoot | Max tilt | XY error | Notes |
|------|--------|--------|-----------|----------|-----------|----------|----------|-------|
| 50 | 0.12 | 0.20 | 0.55 | 8.2s | +103% | 44 deg | 0.41m | Growing oscillation, crash |
| 51 | 0.08 | 0.05 | 0.55 | 9.6s | +67% | 38 deg | 0.06m | Stable hover! End crash = sofa edge |
| 52 | 0.08 | 0.05 | 0.55 | 16.5s | +63% | 14 deg | 0.06m | **Best flight.** Landed successfully |
| 53 | 0.08 | 0.05 | 0.75 | 17.6s | +156% | 24 deg | 0.07m | vvel_damping experiment, much worse |

### Analysis

**Test 50** - pos_kd=0.20 with real flow (250x stronger than before).
Velocity damping produced large tilt commands. These tilts stole vertical
thrust, altitude dropped, PID overcorrected, more tilt. Growing
oscillation until crash latch at 44 deg.

**Test 51** - pos_kd reduced to 0.05. Stable hover for 9.6s, attitude
1-4 deg during hover. XY error 0.061m. Crash at end was a sudden
40 deg pitch excursion during landing - likely sofa edge confusing
the VL53L1x rangefinder.

**Test 52** - Same params as 51. Full flight including successful landing.
16.5s total. Best flight ever. XY error 0.061m. Altitude settled at
~0.74m (overshoot +63%), thrust hunting at 9.9/s. Y-axis drift of
0.25m without position correction (pos_kp=0).

**Test 53** - vvel_damping increased to 0.75. Overshoot jumped to +156%
(1.28m peak). The feedforward coupling made the climb ramp push harder.
Attitude stayed calm (max 24 deg) but drone traveled ~1m sideways.
Reverted to 0.55.

### Phase status

- **Phase 2a**: DONE. HKF validated on hardware. XY error 0.061m.
- **Phase 2b**: Pending. Needs altitude fix first.
- **Phase 3**: In progress. alt_kp=0.08 + climb_rate=0.2 not yet tested.

---

## Session 12 - 2026-02-23 - Unify pitch sign convention across HALs

Unified the pitch axis convention between Webots and Crazyflie HALs.
Both platforms now present identical sensor conventions to the shared
control code. No tuning parameter changes.

### Problem

The position_actor was changed to negate both roll and pitch when
publishing attitude setpoints (`.pitch = -pitch_cmd`), matching
Bitcraze controller_pid.c lines 251-252 (aerospace convention:
positive pitch = nose-UP). This broke the Webots sim because the
motor HAL had a compensating pitch negation (`float pitch = -cmd->pitch`)
that created a double-negation.

The root cause: the original Webots sensor HAL produced **positive
est.pitch = nose-DOWN** while the Crazyflie sensor HAL produced
**positive est.pitch = nose-UP**. The shared code worked by accident
because the position_actor's missing pitch negation and the motor
HAL's pitch negation cancelled each other out.

### Fix - sensor-side corrections

Moved all sensor polarity corrections to the sensor HAL, where they
belong. Each HAL now presents the same convention to shared code:

**Webots sensor HAL (`hal_sensors.c`)**:
- `accel[0] = +g*sin(pitch)` (sign-flipped from naive `-g*sin(pitch)`)
- `gyro[1] = -(float)gyro[1]` (pitch rate negated)
- `gyro[2]` unchanged (yaw is an actuator issue, see below)

This mirrors what the Crazyflie HAL already does for the BMI088:
- `accel[0] = -(accel_data.x * scale)` (X-axis negated, BMI088 points AFT)
- `gyro[1] = -(gyro_data.y * scale - bias)` (pitch rate negated to match)

**Webots motor HAL (`hal_motors.c`)**:
- Pitch negation removed (sensor-side makes it unnecessary)
- Yaw negation kept (`float yaw = -cmd->yaw`) - this compensates for
  the PROTO's opposite propeller spin directions, an actuator difference
  that cannot be sensor-corrected

**Position actor (`position_actor.c`)**:
- `.pitch = -pitch_cmd` (was `pitch_cmd`, now matches Bitcraze convention)
- `.roll = -roll_cmd` (unchanged, was already correct)

**Flow gyro compensation (`platform.c`)**:
- Changed from `+=` to `-=` for both axes, matching Bitcraze mm_flow.c
  velocity model. The session 3 sign flip (subtract -> add) was wrong
  given the gyro Y negation at line 1307.

### Design principle

- **Sensor HAL** corrects measurement polarity (pitch axis for both platforms)
- **Motor HAL** corrects actuator differences (yaw for Webots propeller spin)
- **Shared actors** use a single convention (aerospace: positive pitch = nose-UP)

### Verification (Webots sim)

| Profile | Noise | Max tilt | XY error | Verdict |
|---------|-------|----------|----------|---------|
| FIRST_TEST | 0 | 0.0 deg | 0.000m | OK |
| FIRST_TEST | 3 | 1.9 deg | 0.000m | OK |
| FULL_3D | 0 | 4.6 deg | 0.348m | OK |
| FULL_3D | 3 | ~4.7 deg | ~0.5m | OK (stochastic) |

Performance matches the original code (before the position_actor change).
FULL_3D navigates all 5 waypoints and lands cleanly at all noise levels.

### Code changes

| File | Change |
|------|--------|
| `hal/webots-crazyflie/hal_sensors.c` | Negate gyro[1], flip accel[0] sign |
| `hal/webots-crazyflie/hal_motors.c` | Remove pitch negation, keep yaw negation |
| `hal/webots-crazyflie/README.md` | Updated sign convention docs |
| `position_actor.c` | `.pitch = -pitch_cmd` (aerospace convention) |
| `hal/crazyflie-2.1plus/platform.c` | Fix flow gyro compensation signs |

### Impact on hardware tuning

No tuning parameter changes. The position_actor pitch negation and
flow gyro compensation fix affect Crazyflie hardware behavior. Next
hardware flight (test 50) will validate. If position control direction
is wrong, the flow compensation sign change is the likely cause.

---

## Session 11 - 2026-02-22 - Flow sensor bugs and safety fix (tests 44-49)

Discovered two critical flow sensor bugs that made position control blind
for all 49 flights, plus a post-crash safety bug. Three code fixes, no
tuning changes.

### Bug 1 - Missing /dt in flow velocity conversion (CRITICAL)

**File**: `hal/crazyflie-2.1plus/platform.c:1358`

The PMW3901 pixel-to-velocity conversion was:
```c
float vx_body = delta_x * HAL_FLOW_SCALE * height_m;       // WRONG
```
Should be:
```c
float vx_body = delta_x * HAL_FLOW_SCALE * height_m / dt;  // FIXED
```

The PMW3901 outputs pixel displacement per sample. Without dividing by dt
(~0.004s at 250Hz), velocities were **250x too small**. The HKF reported
0.017 m/s while the real drift was 0.25 m/s. Position control was
essentially running blind on accelerometer integration for all 49 tests.

This explains why increasing pos_kp from 0 to 0.10 across tests 44-48
barely helped - the corrections were based on nearly-zero position errors.

### Bug 2 - PMW3901 axis mapping missing (CRITICAL)

**File**: `hal/crazyflie-2.1plus/platform.c:1338`

Raw PMW3901 delta_x/delta_y were passed straight through to body-frame
velocity. But the sensor is mounted 90 deg rotated on the flow deck.
Bitcraze's `flowdeck_v1v2.c:94-95` applies:
```c
body_x = -sensor.deltaY;
body_y = -sensor.deltaX;
```

Our code had no axis mapping. This was invisible while bug 1 existed
because flow velocities were negligible. After fixing /dt, test 49
(pos_kp=0.03) showed 40.8 deg tilt and 2m drift into a wall in 3.9s -
the position controller was correcting in the wrong direction.

Fixed by adding Bitcraze's swap-and-negate before velocity computation.

### Bug 3 - No negative altitude emergency cutoff (SAFETY)

**File**: `altitude_actor.c:201`, `config.h:137`

The altitude emergency check only triggered at `> 2.0m` (positive).
After a crash, the KF altitude diverged to -26m (post-landing drift
from accelerometer bias with no rangefinder correction). The PID
commanded 99.2% thrust for 30+ seconds trying to "climb" from -26m.

The 10-second landing timeout never fired because the flight manager
never entered LANDING state (crash happened during FLYING).

Fixed by adding `EMERGENCY_ALTITUDE_MIN = -0.5f`. Negative altitude
is physically impossible - any reading below -0.5m triggers the crash
latch and zeros motors immediately.

Verified in test 48: post-landing KF drift was caught within 3 seconds
(vs 30+ seconds in test 47).

### Flight test results

| Test | pos_kp | Duration | Max tilt | XY drift (real) | XY (HKF) | Notes |
|------|--------|----------|----------|-----------------|----------|-------|
| 44 | 0.00 | ~6s | - | 2m (shelf crash) | - | Baseline, no position hold |
| 45 | 0.03 | 11.7s | 32 deg | 1.5m (shelf) | 0.20m | Slow backward drift |
| 46 | 0.05 | 7.6s | - | 1.5m (shelf) | 0.05m | HKF divergence clear |
| 47 | 0.08 | 8.1s | 25 deg | 1.0m (shelf) | 0.07m | **Safety bug found** - 99% thrust 30s on ground |
| 48 | 0.10 | 8.1s | 23 deg | 1.0m (wall) | 0.10m | Safety fix confirmed working |
| 49 | 0.03 | 3.9s | 41 deg | 2m (wall) | 0.18m | /dt fix but wrong axes - violent |

Tests 44-48 had the /dt bug (flow 250x too small). Test 49 had /dt
fixed but axes wrong. Test 50 (pending - battery charging) will be
the first flight with all three fixes.

### Key insight - why drift was invisible for 49 tests

All previous position control problems (session 3 "flow drift", session 5
"XY drift ~1m", session 7 "aggressive tilts", session 8 "drift getting
worse") trace back to the same root cause: the flow sensor data was 250x
too small to be useful. The HKF, position PD controller, and velocity
damping were all processing nearly-zero flow data. Every pos_kp tuning
attempt was futile - there was no real position information to correct from.

### Updated tuning plan

With correct flow data (both scale and axes), the tuning plan changes:

- **Phase 2b restarts from scratch** - pos_kp=0.03 with working flow is a
  completely different starting point. The stop condition (drift < 0.10m)
  is now actually achievable.
- **Phase 3 may improve too** - some altitude oscillation was likely caused
  by the position controller fighting phantom drift. With real position
  feedback, the system should be calmer overall.
- **HAL_FLOW_SCALE (0.0005)** may need calibration. Derived from Bitcraze's
  Npix=35, thetapix=0.717, FLOW_RESOLUTION=0.1 gives ~0.00205. Current
  value is 4x smaller. Not critical yet but may need adjustment if
  velocity magnitude is wrong after axis fix.

### Compile-time changes (all in hal/crazyflie-2.1plus/)

| File | Change |
|------|--------|
| `platform.c:1337-1343` | Added Bitcraze axis mapping (swap + negate) |
| `platform.c:1358-1359` | Added `/dt` to flow velocity conversion |
| `altitude_actor.c:201` | Added `est.altitude < EMERGENCY_ALTITUDE_MIN` |
| `config.h:137` | Added `EMERGENCY_ALTITUDE_MIN -0.5f` |

No tuning parameter changes. All gains at session 8 compiled defaults.

---

## Session 10 - 2026-02-19 - Fix altitude overshoot metric bug

The +65% altitude overshoot reported in session 9 (and +63% in earlier
analysis) was a **metric bug in flight_summary.py**, not real overshoot.

### The bug

`flight_summary.py` compared the global max altitude against the first
waypoint's target_z. For FULL_3D flights, the first waypoint is 0.60m but
the drone later climbs to 1.0m (WP3). The metric computed
`(0.98 - 0.60) / 0.60 = +63%` - measuring intentional navigation as
overshoot.

### The fix

Replaced global overshoot with per-waypoint phase detection. The tool now
tracks `(target_x, target_y, target_z)` changes to identify waypoint
phases and computes overshoot, steady-state error, and XY tracking error
per phase. Also fixed XY drift metric to measure distance from current
waypoint target instead of from origin.

### Verification (Webots, original gains)

| Profile | Noise | Per-WP Overshoot | Hover Alt | Thrust Osc |
|---------|-------|------------------|-----------|------------|
| FIRST_TEST | 0 | -6% (0.469m vs 0.50m) | 0.431m | 0 (0.0/s) |
| FIRST_TEST | 3 | -5% (0.475m vs 0.50m) | ~0.43m | 111 (7.9/s) |
| FULL_3D | 0 | -6% to -4% ascending | 0.53-0.93m | 0 (0.0/s) |
| FULL_3D | 3 | -4% to -0% ascending | variable | 975 (13.8/s) |

**There is no altitude overshoot in Webots.** Every ascending waypoint
undershoots by 4-12%. The 6-12% steady-state undershoot is real - the
integral term (KI=0.03) is too weak to fully close the gap against
velocity damping resistance.

### Impact on tuning plan

The 43-56% overshoot on hardware (sessions 7-8) is from single-waypoint
FIRST_TEST flights where the old metric was correct. Those numbers are
real and phase 3 still needs to address them. The metric fix only affects
multi-waypoint analysis (FULL_3D, ALTITUDE profiles).

A Webots gain retuning attempt (KP 0.30->0.20, KI 0.03->0.05,
damping 0.15->0.35) was tested and reverted - stronger damping caused
thrust oscillation at noise 3 (7.9/s FIRST_TEST, 13.8/s FULL_3D) and
worse steady-state hover (0.43m vs 0.47m). Original Webots gains retained.

---

## Session 9 - 2026-02-19 - Horizontal Kalman filter (simulation)

Implemented the horizontal KF from phase 1 of the tuning plan and validated
in Webots simulation (`tools/run_webots_sim.sh`).

### Code changes

- **Horizontal Kalman filter** - `fusion/horizontal_kf.c` / `.h`. Two
  independent 1D filters (X and Y), each with states [position, velocity,
  accel_bias]. Prediction uses world-frame acceleration (full rotation
  including yaw). Correction uses optical flow velocity (H=[0,1,0]).
  Innovation gating at 1.0 m/s rejects flow outliers.
- **Full rotation matrix** - estimator_actor.c now computes accel_world_x
  and accel_world_y in addition to accel_world_z. Requires cos/sin of yaw.
- **7 new tunable params** (IDs 48-54) - `hkf_q_position`, `hkf_q_velocity`,
  `hkf_q_bias`, `hkf_r_velocity`, `hkf_p0_position`, `hkf_p0_velocity`,
  `hkf_p0_bias`. Total param count: 55.
- **run_webots_sim.sh** - Automated Webots simulation: build, launch in
  fast mode, wait for flight, print hive log and flight analysis.

### Default HKF parameters (config.h)

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `hkf_q_position` | 0.0001 | Trust position model (slow drift OK) |
| `hkf_q_velocity` | 1.0 | Allow velocity to change fast |
| `hkf_q_bias` | 0.0001 | Slow accel bias drift |
| `hkf_r_velocity` | 0.01 | Flow velocity noise variance |
| `hkf_p0_position` | 1.0 | Start uncertain on position |
| `hkf_p0_velocity` | 1.0 | Start uncertain on velocity |
| `hkf_p0_bias` | 0.1 | Start uncertain on bias |

### Webots results (FULL_3D, noise=3, motor_lag=20ms)

| Metric | Value |
|--------|-------|
| Flight duration | 68s (60s flight + liftoff ramp) |
| Waypoints completed | 5/5 (1m x 1m box at 0.6-1.0m) |
| Position settle time | 5-9s per waypoint |
| Hover position RMS | X=0.076m, Y=0.066m |
| Hover velocity RMS | X=0.19 m/s, Y=0.14 m/s |
| Velocity jitter (sample-to-sample) | 0.005-0.006 m/s |
| Max XY from target | 1.9m (during 1m waypoint transitions) |
| Altitude overshoot | 65% global (metric bug - see session 10) |
| Max tilt | 4-5 deg |

### Assessment

The horizontal KF provides smooth velocity (jitter reduced to 0.005 m/s
from raw pixel noise). Position hold is 7-8 cm RMS at hover with pos_kp=0
and pos_kd=0.20 (damping only). The KF integrates position from
bias-corrected acceleration, so position drifts slower than raw flow
integration.

**Ready for hardware testing.** The simulation validates the filter math
and integration. Real-world flow deck noise characteristics differ from
Webots, so HKF_R_VELOCITY will likely need tuning via radio on the
Crazyflie. Next step is phase 2: enable pos_kp with filtered velocity.

---

## Session 8 - 2026-02-18 - Velocity damping tuning (tests 38-42)

Tuned pos_kd via radio while keeping pos_kp=0 (position correction disabled).
Test 42 was the 4th consecutive GO on the same power cycle - failed with
thrust=0 due to RESET race condition (fixed in commit f88103f).

### Compiled defaults

| Parameter | Value | Changed from |
|-----------|-------|-------------|
| `alt_kp` | 0.12 | 0.18 (session 7) |
| `vvel_damping` | 0.55 | 0.45 (session 7) |
| `pos_kp` | 0.0 | 0.14 (session 7 - disabled) |
| `pos_kd` | 0.20 | 0.16 (tuned via radio tests 40-41) |

### Results

| Test | Duration | Overshoot | Max tilt | XY drift | Thrust osc | Notes |
|------|----------|-----------|----------|----------|------------|-------|
| 38 | 11.2s | +56% | 28 deg | 0.33m | 6.3/s | KF drift +32m post-landing |
| 39 | 3.5s | -10% | 46 deg | 0.25m | 2.0/s | Short, near-crash tilt |
| 40 | 21.0s | +51% | 51 deg | 0.45m | 2.0/s | Longest flight, end drift 1.9m |
| 41 | 5.4s | +43% | 44 deg | 1.39m | 3.9/s | Worst drift |
| 42 | - | - | - | - | - | RESET race: 4th GO failed |

### Key observations

- **XY drift is bad and getting worse** across consecutive flights (0.25 -> 1.4m).
  Without pos_kp, drift accumulates unchecked. Velocity damping alone is
  insufficient.
- **Altitude overshoot consistent at 43-56%**, except test 39 (short flight,
  -10% - didn't reach target before crash detection).
- **Max tilt 44-51 deg** - near crash threshold. Driven by altitude
  oscillations, not position control (which is disabled).
- **Test 38 thrust oscillation** (6.3/s) indicates altitude PID at edge of
  stability. Tests 39-41 are calmer (2-4/s).
- **Test 42 RESET race** confirmed the bug fixed in today's commit.

---

## Session 7 - 2026-02-17 - Robustness fixes and altitude retuning (tests 19-37)

Major refactoring session. Replaced fixed hover thrust with liftoff
detection, added altitude robustness features, and retuned after
discovering altitude overshoot was much worse than session 4 predicted.

### Code changes (not tuning)

- **Liftoff detection** - Replaced fixed hover_thrust with auto-calibrating
  ramp (LIFTOFF_RAMP_RATE=0.4/s). Self-calibrates every flight.
- **Innovation gating** - Reject rangefinder readings > 0.3m from KF
  prediction. Prevents velocity spikes from sensor glitches.
- **Minimum airborne thrust** - 5% floor prevents attitude PIDs from
  losing authority during altitude PID undershoot.
- **Derivative-on-measurement** - PID refactored to avoid setpoint kick.
- **Separate yaw PID** - Yaw rate gains separated from roll/pitch
  (yaw_kp=0.12, yaw_ki=0.05 vs roll/pitch kp=0.020).
- **Attitude-priority mixing** - Motor mixer prioritizes attitude over
  thrust to prevent tumble during saturation.
- **Consecutive flight support** - Waypoint actor outer loop, multi-GO.

### Tuning changes

| Parameter | Old (session 4) | New | Reasoning |
|-----------|-----------------|-----|-----------|
| `alt_kp` | 0.18 | 0.12 | 0.18 caused 100-263% overshoot (tests 34-35) |
| `vvel_damping` | 0.45 | 0.55 | More braking but still overshoots 43-56% |
| `pos_kp` | 0.14 | **0.0** | Disabled - caused aggressive tilts (tests 34-36) |
| `pos_kd` | 0.16 | 0.20 | Velocity-only damping while pos_kp disabled |

### Results (tests 34-37 - altitude retuning)

| Test | Duration | Overshoot | Max tilt | XY drift | Notes |
|------|----------|-----------|----------|----------|-------|
| 34 | 5.6s | +103% | 22 deg | 0.49m | alt_kp=0.18 (too strong) |
| 35 | 11.3s | +263% | 39 deg | 0.23m | Went to 1.8m! |
| 36 | 3.1s | +6% | 47 deg | 0.30m | Near-crash, best altitude |
| 37 | 16.2s | +211% | 25 deg | 0.34m | Went to 1.6m |

Tests 34-37 led to reducing alt_kp from 0.18 to 0.12 and disabling
position control (pos_kp=0) as a safety measure.

---

## Session 6 - Cancelled (battery died)

Planned radio overrides (pos_kp=0.04, pos_kd=0.05, vvel_damping=0.55)
were never tested. Battery died before GO. Parameters were later updated
in compiled defaults during session 7-8.

---

## Session 5 - 2026-02-15 - Bug fixes and first flight with session 4 tuning

Fixed six bugs preventing flight with session 4 parameters. Achieved first
successful hover with updated altitude/position gains.

### Bug fixes

1. **False crash detection on ground** - KF altitude drifted to 2m while
   stationary (VL53L1x can't measure below ~40mm, drone sits at ~20mm).
   Gated crash detection on `target_altitude > 0` in altitude_actor.

2. **KF drift during countdown** - Estimator accumulated 30m+ drift during
   the armed countdown. Added estimator-only RESET at FLYING start in
   flight_manager_actor (just before motor/waypoint START).

3. **Motor START starvation** - motor_actor's hive_select had bus source
   (250Hz torque) before IPC sources. Bus always had new data, so START
   message was never checked. Reordered: IPC sources first, bus last.

4. **Mailbox pool exhaustion** - Pool was 32 entries for 12 actors.
   PREFLIGHT RESET burst (11 messages) plus FLYING start burst (3 messages)
   could exhaust the pool. Waypoint START silently failed. Increased pool
   to 48 and added error checking on all flight-critical hive_ipc_notify
   calls.

5. **Log spam filling SD card** - COMMS wake (every 1500 wakes), velocity
   source toggling (250Hz), and MIX motor saturation (250Hz) flooded the
   log. Demoted all to TRACE level.

6. **KF fresh-sample detection** - Changed from value-change detection to
   fresh-rangefinder detection. The VL53L1x runs at 40Hz vs 250Hz control
   loop; feeding the same sample repeatedly shrank KF covariance too fast.

### Other changes

| Parameter | Old | New | Reasoning |
|-----------|-----|-----|-----------|
| `armed_countdown_s` | 60 | 10 | Faster iteration during testing |
| `MAILBOX_ENTRY_POOL_SIZE` | 32 | 48 | Prevent IPC pool exhaustion |

### Results (test 17 - first successful hover with session 4 params)

| Metric | Value |
|--------|-------|
| Target altitude | 0.50 m |
| Peak altitude | 0.666 m |
| Altitude overshoot | 33% |
| Mean hover thrust | 78.9% |
| Max tilt | < 11 deg |
| XY drift | ~1 m (crashed into sofa during landing) |
| Flight duration | 6s hover + 8.5s landing = 14.5s total |
| Battery | 3.18V warning during flight |

### Results (test 18 - position hold disabled)

Zeroed pos_kp/pos_kd via radio to isolate altitude control from flow drift.

| Metric | Value |
|--------|-------|
| Peak altitude | 0.616 m |
| Altitude overshoot | 23% |
| Max tilt | < 8.6 deg (much calmer) |
| XY drift | ~1 m passive (no position correction) |

### Issues remaining

1. Altitude overshoot 23-33% (target <10%) - vvel_damping needs more tuning
2. Altitude oscillation - not settling at 0.5m within 6s flight window
3. XY drift - position hold at full gains pushes drone sideways (flow drift?)
4. Landing too slow - 8.5s descent from 0.6m

---

## Session 4 - 2026-02-12 - Altitude and position tuning

Post-flight analysis of session 3 test 8. Updated defaults for next flight.

### Changes

| Parameter | Old | New | Reasoning |
|-----------|-----|-----|-----------|
| `alt_kp` | 0.12 | 0.18 | Matched Webots, stronger error correction |
| `alt_ki` | 0.01 | 0.005 | Halved to reduce windup overshoot |
| `alt_omax` | 0.15 | 0.20 | Allow stronger PID corrections |
| `vvel_damping` | 0.25 | 0.45 | Primary fix: 1.8x braking force |
| `pos_kp` | 0.08 | 0.14 | Tighter position hold |
| `pos_kd` | 0.10 | 0.16 | More velocity damping for XY |
| `thrust_ramp_ms` | 500 | 750 | Slower ramp, less velocity at target |

Attitude/rate gains unchanged. Kalman filter unchanged.

### Targets

| Metric | Session 3 | Target |
|--------|-----------|--------|
| Altitude overshoot | 30% | <10% |
| Altitude std dev | 0.101 m | <0.05 m |
| Rise time | 1.68 s | ~1.5 s |
| Velocity at 0.5m | 0.30 m/s | <0.15 m/s |
| XY drift | 2.4 cm | <5 cm (tighter hold) |

### Results (test 9 - crash)

Crash at t=1.76s into flight. **Not a tuning issue.** Root cause: SD card
write blocking the scheduler. The hive log was opened at `/sd/hive.log`
(unbuffered SD path), and GPS valid/lost spam + COMMS RX logging generated
hundreds of synchronous SPI writes per second. An SD card latency spike
(108ms) caused a motor deadman timeout, zeroing thrust mid-climb at 0.35m.
The drone flipped and crashed.

The tuning parameters were never tested in actual hover. Retained for next
flight after fixing the SD card write path (ring buffer + DMA spin-wait).

---

## Session 3 - 2026-02-12 - First stable hover (tests 1-8)

Eight flights in one session. Fixed sensor axes, yaw, flow compensation,
and hover thrust calibration via radio.

### Changes

| Parameter | Old | New | Method |
|-----------|-----|-----|--------|
| Accel X | positive | negated | Code fix (platform.c) |
| Gyro Y | positive | negated | Code fix (platform.c) |
| Yaw PID output | positive | negated | Code fix (rate_actor.c) |
| Flow gyro compensation | subtract | add | Code fix (sign flip) |
| `hover_thrust` | 0.65 | 0.85 | Radio tuning during session |

All PID gains at session 2 values. Altitude gains at initial conservative
defaults:

| Parameter | Value |
|-----------|-------|
| `alt_kp` | 0.12 |
| `alt_ki` | 0.01 |
| `alt_omax` | 0.15 |
| `vvel_damping` | 0.25 |
| `pos_kp` | 0.08 |
| `pos_kd` | 0.10 |
| `thrust_ramp_ms` | 500 |

### Results (test 8 - first stable hover)

| Metric | Value |
|--------|-------|
| Mean hover altitude | 0.512 m (target 0.50) |
| Max altitude | 0.648 m |
| Altitude std dev | 0.101 m |
| Altitude overshoot | 51% (analyze_pid), ~30% during hover |
| Rise time (10-90%) | 1.68 s |
| Max tilt | 9.7 deg |
| XY drift | 2.4 cm total |
| Yaw drift | +51.4 deg / 10s |
| Mean hover thrust | 82.7% |

### Issues identified

1. Altitude overshoot - velocity 0.30 m/s at target, damping too weak
2. Altitude oscillation - integral windup during rise phase
3. XY drift - position gains deliberately conservative

---

## Session 2 - 2026-02-03 - Attitude/rate PID tuning

Iterated attitude and rate gains between crashes.

### Changes

| Parameter | Old | New | Result |
|-----------|-----|-----|--------|
| `att_kp` | 1.5 | 2.2 | Oscillated, overcorrected |
| `att_kd` | 0.08 | 0.12 | Oscillated |
| `rate_kp` | 0.018 | 0.024 | Oscillated |
| `rate_kd` | 0.001 | 0.002 | Oscillated |

Settled on midpoint values:

| Parameter | Value |
|-----------|-------|
| `att_kp` | 1.8 |
| `att_kd` | 0.10 |
| `rate_kp` | 0.020 |
| `rate_ki` | 0.001 |
| `rate_kd` | 0.0015 |

### Results

- Weak gains (1.5/0.018): tips to 45 deg in 0.7s, no recovery
- Strong gains (2.2/0.024): tips -41 deg, overcorrects to +64 deg, crash
- Midpoint gains: stable attitude, ready for altitude testing

---

## Session 1 - 2026-02-03 - Initial flight attempts

First power-on flights. Motor mixer and thrust calibration.

### Changes

- Fixed thrust bug (100% output instead of HAL_HOVER_THRUST)
- Disabled barometer (garbage readings blocked control pipeline)
- `HAL_HOVER_THRUST`: 0.38 -> 0.65 (bare-frame calibration)

### Results

Tests 1-2: immediate tumble/crash. Thrust too low, yaw spin from
incorrect gyro Z negation.
