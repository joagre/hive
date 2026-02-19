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
| `alt_kp` | 0.12 | Needs work (43-56% overshoot) |
| `alt_ki` | 0.005 | Needs work |
| `vvel_damping` | 0.55 | Needs work |
| `pos_kp` | **0.0** | **Disabled** - re-enable with HKF (phase 2) |
| `pos_kd` | 0.20 | Tuned session 8, damping-only |
| `max_tilt_angle` | 0.25 (14 deg) | OK |

### Architectural analysis - why we drifted

The fundamental problem was the asymmetry between vertical and horizontal
state estimation. Session 9 closed this gap with a horizontal KF.

**Altitude** has a proper 3-state Kalman filter:
- States: [altitude, velocity, accel_bias]
- Predict: accelerometer (250 Hz)
- Correct: rangefinder (40 Hz)
- Result: smooth velocity, bias-compensated, handles sensor dropouts

**Horizontal** now has a 3-state Kalman filter (session 9):
- States: [position, velocity, accel_bias] (same as altitude KF)
- Predict: world-frame accelerometer (250 Hz)
- Correct: optical flow velocity (100 Hz)
- Result: smooth velocity, bias-compensated, handles flow dropouts
- Sim-validated in Webots (session 9), awaiting hardware testing

Before session 9, horizontal had raw flow only (no filtering, no bias
estimation). At 0.5m hover, a single PMW3901 pixel delta = 0.25 m/s
went directly into the pos_kd damping term, producing 2.9 deg tilt
jitter per pixel. The horizontal KF eliminates this noise path.

### Phase 1 - Horizontal Kalman filter - DONE (session 9)

Implemented `fusion/horizontal_kf.c` / `.h` mirroring `altitude_kf.c`.
Two independent 1D filters (X and Y), each with states [position,
velocity, accel_bias]. Velocity measurement from optical flow. Innovation
gating at 1.0 m/s rejects flow outliers. Full rotation matrix in
estimator_actor.c provides world-frame acceleration for all three axes.

Sim-validated in Webots: 7-8 cm hover RMS, velocity jitter 0.005 m/s.
See session 9 for full results and default parameters.

### Phase 2 - Enable position hold and tune

**Goal** - XY drift < 0.10m during 6s FIRST_TEST flight.

With filtered velocity from the horizontal KF, re-enable pos_kp.
The control law is: `accel = pos_kp * error - pos_kd * velocity`.

**Approach** - Tune via radio, no reflash needed.

| Step | pos_kp | pos_kd | Rationale |
|------|--------|--------|-----------|
| 1 | 0.03 | 0.20 | Very gentle. At 0.3m drift: 0.5 deg tilt |
| 2 | 0.05 | 0.20 | If step 1 shows drift reduction |
| 3 | 0.08 | 0.20 | Moderate hold. At 0.3m drift: 1.4 deg tilt |
| 4 | 0.10 | 0.15 | Stronger hold if step 3 is good |

**Stop condition** - If max tilt exceeds 35 deg, back off pos_kp.
If drift < 0.10m, lock in the value and move to phase 3.

**Why position control failed before (tests 34-36)** - Two compounding
problems: altitude was wildly overshooting (up to 263%) creating large
attitude disturbances, AND the velocity fed into pos_kd was unfiltered
pixel noise. The horizontal KF fixes the second problem. The altitude
work in phase 3 addresses the first.

### Phase 3 - Tame altitude overshoot

**Goal** - Altitude overshoot < 10%, thrust oscillation < 1/s.

Current: alt_kp=0.12, alt_ki=0.005, vvel_damping=0.55, alt_omax=0.20.
Overshoot of 43-56% suggests the system cannot brake fast enough.

**Tuning steps** (via radio):

| Step | Parameter | Value | Rationale |
|------|-----------|-------|-----------|
| 1 | `vvel_damping` | 0.70 | Stronger altitude braking |
| 2 | `alt_omax` | 0.25 | Allow PID to brake harder (was clamped) |
| 3 | `alt_kp` | 0.08 | Softer correction if oscillation persists |

**Code change to consider** - Reduce LIFTOFF_CLIMB_RATE from 0.3 to
0.2 m/s. Slower climb ramp means less momentum to brake at target.
Requires reflash. Try tuning-only first.

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

### Architecture notes

**What we have vs what Bitcraze has:**

| Component | Hive pilot | Bitcraze firmware |
|-----------|-----------|-------------------|
| Attitude | Complementary filter | Extended Kalman Filter |
| Altitude | 3-state KF | Part of full EKF |
| Horizontal velocity | 3-state KF (session 9) | EKF-fused with accel |
| Horizontal position | KF-integrated (session 9) | EKF-fused |
| Position control | PD (no integral) | PID with feedforward |

The horizontal KF (phase 1, done) closed the biggest gap. Adding an
integral to position control (phase 4) closes the next. A full EKF
fusing all sensors would be ideal but is a much larger project.

**Current estimation data flow:**

```
                     Accelerometer (250 Hz)
                            |
                   Rotate to world frame
                     /      |      \
                    X       Y       Z
                    |       |       |
              ┌─────v─┐ ┌──v────┐ ┌v────────┐
              | Horiz  | | Horiz | | Altitude |
              | KF (X) | | KF(Y) | | KF       |
              └────┬───┘ └──┬────┘ └┬────────┘
  Flow vel ------->|------->|       |<------- Rangefinder
  (100 Hz)    position  position  altitude
              velocity  velocity  velocity
              bias      bias      bias
```

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
