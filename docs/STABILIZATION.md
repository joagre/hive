# Stabilization Roadmap

A practical, incremental path from current hover to acrobatic-ready flight control.

**Context:** This document is a prerequisite for [COPTER_ARENA_IDEA.md](COPTER_ARENA_IDEA.md).
Before attempting aggressive maneuvers, pursuit-evasion, or any form of aerial acrobatics,
the flight controller must have rock-solid stabilization. Quick, precise, noise-tolerant
attitude control is the foundation everything else builds on.

**Guiding principle:** Every improvement must make hover *better*, not worse. The path to
acrobatics goes through rock-solid hover. If a change degrades gentle flight, it's wrong.

---

## Current State

**What works:**
- Webots simulation: waypoint navigation, altitude hold, position hold
- Control cascade: sensor → estimator → altitude/position → attitude → rate → motor
- Complementary filter for attitude estimation
- PID controllers at each level
- 250 Hz control loop

**What's untested:**
- Real hardware hover (Crazyflie arriving tomorrow)
- Behavior under disturbances
- Noise handling with real sensors

**What's missing for acrobatics:**
- Euler angles break at large angles (gimbal lock)
- No feedforward (pure feedback control)
- Conservative tuning (slow response)
- Simple complementary filter (poor during acceleration)
- No actuator saturation handling

---

## Incremental Steps

Each step has:
- **Goal**: What capability we gain
- **Test (Webots)**: How to verify in simulation
- **Test (Hardware)**: How to verify on Crazyflie
- **Success criteria**: Measurable improvement
- **Regression check**: Hover must remain stable

---

### Step 0: Baseline — Stable Hardware Hover

**Goal:** Achieve stable hover on real Crazyflie hardware. This is the foundation.

**Why first:** Can't improve what we can't measure. Need a working baseline before
making changes.

**Work:**
- Tune existing PID gains for Crazyflie (may differ from Webots)
- Verify sensor data looks reasonable (no excessive noise, correct signs)
- Achieve hands-off hover for 30+ seconds

**Test (Webots):** Already works — waypoint navigation completes.

**Test (Hardware):**
- Takeoff to 0.5m, release controls
- Drone holds position ±20cm for 30 seconds
- No oscillations visible

**Success criteria:**
- Stable hover without human intervention
- Position drift < 20cm over 30 seconds
- No audible oscillation in motors

**Regression check:** N/A (this is the baseline)

---

### Step 1: Measurement and Logging

**Goal:** Understand what's actually happening. Add instrumentation to measure
control performance before changing anything.

**Why now:** You can't improve what you don't measure. Before tuning, we need
to see loop timing, sensor noise, control errors, actuator commands.

**Work:**
- Add optional high-rate logging to flight actors
- Log: timestamp, setpoint, measurement, error, output for each controller
- Log: raw sensor values, estimated state
- Post-flight analysis tools (Python scripts or similar)

**Test (Webots):**
- Run waypoint mission, collect logs
- Plot attitude error over time
- Plot rate controller tracking

**Test (Hardware):**
- Hover for 30 seconds, collect logs
- Analyze sensor noise spectrum
- Identify any oscillations or delays

**Success criteria:**
- Can produce time-series plots of all control loops
- Can measure RMS error for each controller
- Can identify dominant noise frequencies

**Regression check:** Logging is passive — hover behavior unchanged.

---

### Step 2: Rate Controller Tuning

**Goal:** Tighter, faster rate control. The rate controller is the innermost loop —
everything else depends on it.

**Why now:** Rate controller performance limits all outer loops. A sluggish rate
controller makes attitude control slow, which makes position control slow.

**Work:**
- Analyze rate controller logs from Step 1
- Increase P gain until oscillation, then back off 20%
- Tune D term for damping (derivative on measurement, not error)
- Verify anti-windup is working

**Test (Webots):**
- Command step changes in angular rate (0 → 90°/s → 0)
- Measure rise time, overshoot, settling time
- Target: <100ms rise time, <10% overshoot

**Test (Hardware):**
- Same step response test via software
- Or: gentle manual stick input, observe response crispness
- Listen for oscillation (buzzing = gains too high)

**Success criteria:**
- Rate step response: rise time < 100ms
- No audible oscillation during hover
- Tracking error reduced vs Step 1 baseline

**Regression check:**
- Hover stability same or better
- Position hold same or better

---

### Step 3: Attitude Controller Tuning

**Goal:** Faster attitude response while maintaining stability.

**Why now:** With a solid rate controller (Step 2), we can push attitude control harder.

**Work:**
- Analyze attitude logs — how fast do we track setpoint changes?
- Increase attitude P gain
- Verify rate setpoint limits are appropriate
- Consider feedforward: attitude error → expected rate

**Test (Webots):**
- Command attitude step (0° → 15° roll → 0°)
- Measure response time
- Target: <200ms to reach 90% of commanded angle

**Test (Hardware):**
- Push test: gently push drone, measure recovery time
- Should snap back to level quickly without oscillation

**Success criteria:**
- Attitude step response < 200ms to 90%
- Disturbance recovery < 500ms
- No oscillation after disturbance

**Regression check:**
- Hover stability same or better
- Position hold same or better

---

### Step 4: Estimator Improvements — Velocity

**Goal:** Better velocity estimation for tighter position control.

**Why now:** Current velocity is derived by differentiating position (noisy) or
integrating acceleration (drifts). Better velocity estimates help position control.

**Work:**
- Improve velocity estimation in complementary filter
- Options:
  - Lower cutoff frequency on derivative filter
  - Fuse optical flow (Crazyflie Flow deck)
  - Add simple Kalman filter for position/velocity
- Start with simplest improvement that helps

**Test (Webots):**
- Compare estimated velocity to ground truth
- Measure RMS velocity error during hover and movement

**Test (Hardware):**
- Position hold in presence of disturbance
- Measure position variance during hover

**Success criteria:**
- Velocity estimate RMS error < 0.1 m/s during hover
- Position hold tighter than Step 3

**Regression check:**
- Hover at least as stable as Step 3
- No new oscillation modes

---

### Step 5: Position Controller Tuning

**Goal:** Tighter position hold, faster response to position commands.

**Why now:** With better velocity estimates (Step 4), position controller can be
more aggressive.

**Work:**
- Increase position P gain
- Tune velocity damping term
- Verify max tilt limit is appropriate
- Consider feedforward: position error → expected velocity

**Test (Webots):**
- Command position step (0,0 → 1m,0 → 0,0)
- Measure response time and overshoot
- Target: <2s to reach position, <10% overshoot

**Test (Hardware):**
- Same position step test
- Or: move drone by hand, release, watch it return

**Success criteria:**
- Position step response < 2s
- Overshoot < 10%
- Steady-state error < 5cm

**Regression check:**
- Hover stability same or better

---

### Step 6: Altitude Controller Tuning

**Goal:** Precise altitude hold, smooth altitude changes.

**Why now:** Altitude is somewhat independent of XY. Can be tuned in parallel
or after position work.

**Work:**
- Analyze altitude logs
- Tune altitude PID for tighter hold
- Improve vertical velocity estimation
- Consider barometer fusion (if available)

**Test (Webots):**
- Command altitude step (1m → 1.5m → 1m)
- Measure response time, overshoot

**Test (Hardware):**
- Same altitude step test
- Verify no altitude oscillation during hover

**Success criteria:**
- Altitude step response < 1.5s
- Altitude hold ±5cm during hover
- No oscillation

**Regression check:**
- Hover stability same or better
- Position hold same or better

---

### Step 7: Extended Kalman Filter

**Goal:** Replace complementary filter with EKF for better state estimation,
especially during maneuvers.

**Why now:** Steps 2-6 have optimized the controllers assuming decent state estimates.
To go further (faster maneuvers, larger angles), we need better estimation.

**Work:**
- Implement quaternion-based EKF
- State: position, velocity, attitude (quaternion), gyro bias
- Measurements: accelerometer, gyroscope, position (GPS/flow)
- Start with conservative tuning (trust sensors less)
- Gradually tune process/measurement noise

**Test (Webots):**
- Compare EKF attitude to ground truth during aggressive waypoint mission
- Verify no divergence
- Measure attitude error RMS

**Test (Hardware):**
- Hover comparison: EKF vs complementary filter
- Should be at least as stable, ideally better
- Test with deliberate disturbances

**Success criteria:**
- EKF attitude error < complementary filter error
- No divergence over 60+ second flight
- Hover stability same or better

**Regression check:**
- CRITICAL: Hover must not degrade
- If EKF hurts hover, keep complementary filter, debug EKF

---

### Step 8: Quaternion Attitude Control

**Goal:** Attitude control that works at any angle, not just near hover.

**Why now:** EKF (Step 7) provides quaternion attitude. Now controllers can use it.
This removes the gimbal lock limitation.

**Work:**
- Modify attitude controller to use quaternion error
- Quaternion error → axis-angle → rate setpoint
- Works correctly at all orientations
- Test at large angles

**Test (Webots):**
- Command large attitude: 45° roll, hold, return to level
- Command 90° pitch (pointing up) — currently impossible with Euler
- Verify no singularity or weird behavior

**Test (Hardware):**
- Conservative: 30° roll/pitch commands, verify tracking
- Only attempt larger angles with safety setup (tether/net)

**Success criteria:**
- Stable control at 45° roll/pitch
- No singularity at 90° pitch (in simulation)
- Hover stability unchanged

**Regression check:**
- Hover must remain rock solid
- Small angle behavior unchanged

---

### Step 9: Feedforward Control

**Goal:** Anticipate required thrust/torque instead of just reacting to error.

**Why now:** For fast maneuvers, feedback alone is too slow. Feedforward uses
knowledge of desired trajectory to command actuators proactively.

**Work:**
- Add feedforward path to rate controller (desired angular acceleration → torque)
- Add feedforward to altitude (desired vertical acceleration → thrust)
- Add feedforward to attitude (desired rate → rate setpoint directly)
- Requires trajectory information (not just setpoint)

**Test (Webots):**
- Aggressive waypoint mission with feedforward enabled
- Compare tracking error: feedforward vs feedback-only
- Should see reduced lag

**Test (Hardware):**
- Position step response with feedforward
- Should be faster with less overshoot

**Success criteria:**
- Tracking error reduced 30%+ during dynamic maneuvers
- Step response faster than Step 5

**Regression check:**
- Hover unchanged (feedforward is zero during hover)
- Gentle maneuvers unchanged or improved

---

### Step 10: Actuator Modeling and Saturation

**Goal:** Know what the motors can actually deliver; handle saturation gracefully.

**Why now:** Aggressive maneuvers will saturate motors. Need to handle this
correctly — prioritize attitude over altitude, avoid integrator windup.

**Work:**
- Characterize motor response (thrust vs command, time constant)
- Implement mixer saturation handling
- Priority: attitude control > altitude control (don't flip to maintain height)
- Anti-windup on all integrators when saturated

**Test (Webots):**
- Command impossible maneuver (flip while climbing)
- Verify drone doesn't crash, recovers gracefully
- Verify attitude maintained even if altitude suffers

**Test (Hardware):**
- Push drone hard during hover
- Verify recovery even if motors momentarily saturate
- No flip or loss of control

**Success criteria:**
- Graceful degradation under saturation
- Attitude always prioritized
- No unexpected flips

**Regression check:**
- Normal flight unchanged
- Hover unchanged

---

## Future Steps (Beyond Basic Acrobatics)

These are out of scope for this document but noted for completeness:

- **Trajectory generation**: Minimum-snap polynomials, time-optimal paths
- **Flip maneuvers**: Open-loop + closed-loop hybrid control
- **Inverted flight**: Negative thrust handling
- **Multi-drone coordination**: For the arena concept

---

## Testing Philosophy

1. **Simulation first**: Every change tested in Webots before hardware
2. **Measure before and after**: Quantitative comparison, not just "feels better"
3. **Regression tests**: Hover stability checked at every step
4. **Conservative rollout**: New features can be disabled if problems arise
5. **Safety margins**: Hardware tests start gentle, increase gradually

---

## Hardware Safety

For Crazyflie testing:

- **Indoor, open space**: No furniture to crash into
- **Low altitude first**: Test at 0.3-0.5m before going higher
- **Kill switch ready**: Thumb on disarm at all times
- **Propeller guards**: Until confident in stability
- **Tether for aggressive tests**: When testing large angles or fast maneuvers

---

## Summary

| Step | Focus | Key Improvement |
|------|-------|-----------------|
| 0 | Baseline | Stable hardware hover |
| 1 | Measurement | Understand current performance |
| 2 | Rate control | Faster inner loop |
| 3 | Attitude control | Faster angle response |
| 4 | Velocity estimation | Better damping data |
| 5 | Position control | Tighter position hold |
| 6 | Altitude control | Precise height control |
| 7 | EKF | Better state estimation |
| 8 | Quaternion control | Large angle capability |
| 9 | Feedforward | Anticipate, don't just react |
| 10 | Saturation handling | Graceful limits |

Each step makes hover better while enabling the next level of aggressiveness.
The goal is not to rush to acrobatics, but to build a solid foundation where
acrobatics become natural extensions of already-excellent basic flight.
