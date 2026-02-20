# Future

Production gaps, deferred features, and estimator roadmap.

## Table of Contents

- [Production Gaps](#production-gaps)
- [Deferred Features](#deferred-features)
- [Estimator Evolution](#estimator-evolution)

---

## Production Gaps

What a production flight controller would need beyond this demonstration:

### Missing Error Handling

| Scenario                        | Current Behavior                                       | Production Requirement                            |
|---------------------------------|--------------------------------------------------------|---------------------------------------------------|
| Sensor read fails               | `hive_bus_read()` returns error, actor skips iteration | Watchdog timeout, switch to backup sensor or land |
| Bus publish fails               | Logs WARN, continues                                   | Trigger failsafe if persistent                    |
| Actor crashes                   | ONE_FOR_ALL restart of entire pipeline (supervisor)    | Restart + emergency landing if repeated           |
| Position source lost (flow/GPS) | Position control uses stale data                       | Hold last position and descend slowly             |
| IMU data invalid                | Garbage in, garbage out                                | Sanity checks, sensor voting, reject outliers     |

### Implemented Safety Features

The following safety features run on all platforms (Webots and STM32):

| Feature             | Location               | Behavior                                                                      |
|---------------------|------------------------|-------------------------------------------------------------------------------|
| Motor START gate    | motor_actor.c          | Motors stay OFF until flight_manager sends authorization                      |
| Crash latch         | altitude_actor.c       | Once attitude exceeds 45 deg, motors stay OFF until reboot                    |
| Attitude cutoff     | altitude_actor.c       | Motors off if roll or pitch >45 deg                                           |
| Altitude cutoff     | altitude_actor.c       | Motors off if altitude >2m                                                    |
| Landed detection    | altitude_actor.c       | Motors off when target <5cm and altitude <15cm                                |
| Liftoff ramp        | altitude_actor.c       | Physics-based thrust ramp until rangefinder detects liftoff                   |
| Motor deadman       | motor_actor.c          | Motors zeroed if no torque command within 50ms                                |
| Flight duration     | flight_manager_actor.c | Controlled landing after timeout (6-60s depending on flight profile)          |
| Landing timeout     | flight_manager_actor.c | Forces shutdown if landing detection fails (10s max)                          |
| Battery monitoring  | battery_actor.c        | 2 Hz voltage sampling, 3.2V warning, 3.0V critical with 5s debounce           |
| Low battery landing | flight_manager_actor.c | Emergency landing on debounced critical voltage (ARMED: disarm, FLYING: land) |

The following safety features are STM32-only (disabled in Webots):

| Feature       | Location               | Behavior                                                              |
|---------------|------------------------|-----------------------------------------------------------------------|
| Startup delay | flight_manager_actor.c | Flight blocked by ~3s calibration + 10s armed countdown (motors OFF) |

### Future Safety Features

The following may be added after initial flight testing:

- **Rate cutoff**: Motors off if angular rate exceeds threshold (e.g., >300 deg/s) - catches violent oscillation while within tilt limits
- **Accelerometer sanity check**: Motors off if acceleration readings are implausible
- **Sensor timeout**: Motors off if no sensor updates within expected interval

#### Bitcraze Supervisor (Reference)

The [Bitcraze supervisor](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/supervisor.c)
implements a comprehensive 11-state safety state machine. Key features
worth adopting:

- **Tumble detection** - Z-accel < 0.5g for 1000ms, or < -0.2g for 100ms (upside-down)
- **Free fall exception** - Suspends tumble check when all axes < 0.1g
- **Commander watchdog** - Stale setpoint detection (500ms warning, 2s shutdown)
- **Warning Level Out** - On excessive tilt: disables XY, locks roll/pitch to zero, keeps Z

The current pilot has basic cutoffs in altitude_actor.c (attitude >45 deg,
altitude >2m). A dedicated supervisor actor could centralize these checks
and add the Bitcraze-style features above.

### Missing Safety Features (Production Requirements)

- **Geofence** - No boundary limits, drone can fly away indefinitely
- **Arming/disarming** - No safety switch to prevent accidental motor start
- **Pre-flight checks** - No sensor validation before takeoff
- **Communication loss** - No failsafe if telemetry link drops

### Production Instrumentation Requirements

- **Error counters** - Per-actor `bus_read_fail_count` and `bus_publish_fail_count`, exposed via telemetry

## Deferred Features

Features intentionally omitted or deferred:

- **Unified EKF** - See [Estimator Evolution](#estimator-evolution) below
- **Acrobatic flight modes** - Requires EKF (complementary filter breaks at high angles)
- **Dynamic waypoints** - Upload waypoints via radio during flight
- **Mode switching** - Hover, land, return-to-home
- **Failsafe handling** (return-to-home, auto-land) - requires GPS and mission planning
- **Multiple vehicle types** - Single X-configuration quadcopter only

---

## Estimator Evolution

The current estimator uses three independent filters. This section documents
the path toward a unified EKF for acrobatic flight.

### Current Architecture (3 filters)

```
Accelerometer ──► Complementary Filter ──► roll, pitch, yaw
Gyroscope     ──┘

Accelerometer ──► Altitude KF (3-state) ──► altitude, vz, accel_bias_z
Rangefinder   ──┘

Accelerometer ──► Horizontal KF (3-state x2) ──► x, y, vx, vy, bias_x, bias_y
Optical Flow  ──┘
```

Total: 9 states across 3 filters, ~20 lines of matrix math each.

**Strengths**
- Simple, debuggable, each filter tuneable independently
- Complementary filter is computationally cheap (~10us per update)
- Adequate for gentle hover and slow waypoint navigation

**Comparison with Bitcraze firmware**

| Component | Hive pilot | Bitcraze firmware |
|-----------|-----------|-------------------|
| Attitude | Complementary filter | Extended Kalman Filter |
| Altitude | 3-state KF | Part of full EKF |
| Horizontal velocity | 3-state KF | EKF-fused with accel |
| Horizontal position | KF-integrated | EKF-fused |
| Position control | PD (no integral) | PID with feedforward |

**Limitation** - The complementary filter computes roll and pitch from the
gravity vector in accelerometer data. This assumes acceleration is dominated
by gravity, which breaks during aggressive maneuvers (>30 deg tilt, rapid
translation). The horizontal KF also uses a small-angle rotation matrix
that degrades at high tilt.

### Future Architecture (unified EKF)

A full Extended Kalman Filter would fuse all sensors in a single ~15-state
model:

```
State vector (15-16 states):
  Attitude:  quaternion (4) or rotation matrix
  Position:  x, y, z (3)
  Velocity:  vx, vy, vz (3)
  Gyro bias: bwx, bwy, bwz (3)
  Accel bias: bax, bay, baz (3)

Measurements:
  Accelerometer (250 Hz) - gravity + linear acceleration
  Gyroscope (250 Hz)     - angular rate (prediction step)
  Rangefinder (~40 Hz)   - altitude
  Optical flow (~100 Hz) - horizontal velocity
  Magnetometer (future)  - heading
```

**Why EKF matters for acrobatics** - The quaternion state tracks orientation
through arbitrary rotations (flips, rolls, inverted flight). No singularity
at 90 deg pitch, no gravity assumption for tilt estimation. Gyro bias
estimation compensates for sensor drift that would accumulate during extended
maneuvers.

### Implementation Plan

**Compile-time switch** - Add `ESTIMATOR_MODE` build flag:
- `ESTIMATOR_3FILTER` (default) - Current complementary + altitude KF + horizontal KF
- `ESTIMATOR_EKF` - Unified EKF

Both modes use the same actor interface (read sensor bus, publish state bus).
No changes to any other actor. The estimator actor already isolates all
fusion logic behind the state bus abstraction.

**What carries forward** - Inner loop tuning (rate PID, attitude PID) is
independent of the estimator. The PIDs consume the same state bus fields
regardless of which estimator produced them. Only the outer loops (position,
altitude) may need gain adjustments because EKF velocity estimates will have
different noise characteristics.

**Effort estimate** - 2-4 weeks for a working EKF:
- Quaternion math library (~200 lines)
- EKF predict/update with 15x15 covariance (~400 lines)
- Sensor measurement models (~100 lines each)
- Tuning Q/R matrices (iterative, data-driven)

