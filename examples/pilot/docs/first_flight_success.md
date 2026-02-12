# First Flight Test Results

Crazyflie 2.1+ running Hive pilot autopilot. First successful autonomous hover
achieved after 8 iterative test flights over a single session.

**Date** - 2026-02-12

**Hardware** - Crazyflie 2.1+ with Flow Deck v2 (PMW3901 + VL53L1x),
SD card deck, 3g plastic crash cage

**Flight profile** - `FLIGHT_PROFILE_FIRST_TEST` (hover at 0.5m for 6 seconds)

**Build** - `make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST ENABLE_SD=1`

## Successful Flight (Test 8) - Summary

| Metric | Value |
|--------|-------|
| Target altitude | 0.50 m |
| Mean hover altitude | 0.512 m |
| Max altitude | 0.648 m |
| Altitude std dev | 0.101 m |
| Max tilt | 9.7 deg |
| Max roll | 9.3 deg |
| Max pitch | 6.1 deg |
| Yaw drift | +51.4 deg over 10s |
| XY drift during hover | (-0.009, -0.022) m = 2.4 cm total |
| Mean hover thrust | 82.7% |
| Active flight duration | 10.3 s |
| Battery | 3.74V start, 3.73V end |
| On-board telemetry | 8875 samples at 25 Hz |

## Analysis Tool Output

### Hover Stability (`analyze_hover.py`)

```
Hover at 0.5m for 7.6s (190 samples)
  Altitude: mean=0.5121m, stddev=0.1012m
            range=[0.3045, 0.6481]m
  Velocity: mean=0.0069m/s, stddev=0.1772m/s
            range=[-0.4844, 0.5564]m/s
  Max tilt: 9.71deg (roll=9.33, pitch=3.51)
  XY drift: (-0.009, -0.022)m
```

### PID Performance (`analyze_pid.py`)

```
Target altitude:     0.5 m
Max altitude:        0.7564 m
Overshoot:           51.28%
Rise time (10-90%):  1.684 s
Settling time (5%):  None s
Steady-state error:  0.4807 m
RMS error:           0.4785 m
```

Note: steady-state error and RMS error are inflated because the tool computes
over the entire log (361s), including the long idle period after landing where
altitude is 0.0m against a 0.5m target.

### Flight Debug (`flight_debug.py`)

```
Duration: 361.7s
Max altitude: 0.756m at t=358.9s (target: 0.5m)
Max tilt: 69.2deg
XY drift: (-0.15, -0.08)m

Stable hover periods (alt 0.3-0.7m, tilt < 15deg):
  t=61.9-68.8s (6.9s, mean alt=0.529m)
```

Note: the 69.2 deg max tilt and 0.756m max altitude are from the post-landing
tip-over, not from the hover phase.

## Parameter Values

All parameters at compile-time defaults from `hal_config.h`, except
`hover_thrust` which was set to 0.85 via radio before flight.

### Control Gains

| Controller | KP | KI | KD | Notes |
|------------|-----|-----|-----|-------|
| Altitude | 0.12 | 0.01 | 0.0 | Conservative; velocity damping = 0.25 |
| Attitude | 1.8 | 0.0 | 0.10 | Tuned between weak (1.5) and oscillating (2.2) |
| Rate | 0.020 | 0.001 | 0.0015 | Tuned between weak (0.018) and oscillating (0.024) |

### Other Key Parameters

| Parameter | Value |
|-----------|-------|
| `hover_thrust` | 0.85 (0.65 default too low for loaded drone) |
| `vvel_damping` | 0.25 |
| `thrust_ramp_ms` | 500 |
| `flow_scale` | 0.0005 |
| `tof_max_range_mm` | 1300 |
| `emergency_tilt_limit` | 45 deg |
| `emergency_alt_max` | 2.0 m |
| `cf_alpha` | 0.995 |

## Issues Found and Fixed

### 1. Sensor axis mapping (tests 1-2)

The BMI088 IMU on the CF2.1+ has its X axis pointing aft (backward). The
original code used a 90-degree rotation transform from Bitcraze's
`sensors_bmi088_bmp388.c`, but the Hive estimator expects a different
coordinate convention.

**Fix** - Negate accel X and gyro Y (pitch axis). Keep gyro Z positive.

### 2. Yaw spin (test 1)

Gyro Z was negated in the HAL (treating it like the pitch axis). This created
positive feedback in the yaw PID loop, causing the drone to spin continuously
(-88 degrees accumulated in one test).

**Root cause** - Bitcraze negates the yaw PID *output* (controller_pid.c line
131), not the gyro *input*. Negating the input reverses the measurement while
the setpoint stays at zero, so the PID drives the error larger instead of
smaller.

**Fix** - Revert gyro Z negation in `platform.c`. Add output negation
(`cmd.yaw = -pid_update(...)`) in `rate_actor.c`.

### 3. Insufficient hover thrust (tests 1-3)

Default `HAL_HOVER_THRUST` of 0.65 was calibrated on the bare Crazyflie.
With the flow deck (~3g), SD card deck (~3g), and crash cage (~3g), the
drone is approximately 40-50% heavier (27g base + ~10-15g accessories).

**Observations** -
- 0.65: barely lifts off (skimming ground)
- 0.80: hovers ~10 cm above ground
- 0.85: reaches 0.5m target altitude

**Fix** - Updated `HAL_HOVER_THRUST` to 0.85 in `hal_config.h`.

### 4. Flow deck gyro compensation sign (tests 3-7)

The PMW3901 optical flow sensor sees rotational flow when the drone
pitches/rolls. This gyro-induced component must be subtracted to isolate
translational velocity. The original compensation was:

```c
vx_body -= sensors->gyro[1] * height_m;  // pitch rate
```

After negating gyro Y (fix #1), `sensors->gyro[1]` is the negated pitch rate.
The subtraction now *adds* the rotational component, doubling the error instead
of cancelling it. This caused severe lateral drift.

**Fix** - Change to `vx_body += sensors->gyro[1] * height_m` to account for
the negated sign.

## Iteration History

8 flight tests were needed to reach stable hover. Each test revealed one or
more issues that were fixed before the next attempt.

| Test | hover_thrust | Changes | Result |
|------|-------------|---------|--------|
| 1 | 0.65 | Initial flight attempt | Yaw spin, barely lifted |
| 2 | 0.65 | Yaw output negation fix | Stable yaw, too low altitude |
| 3 | 0.80 | Thrust tuned via radio | ~10 cm hover, lateral drift |
| 4 | 0.85 | Second GO without power cycle | Stuttering (stale state) |
| 5 | 0.85 | Fresh power cycle | Reached 0.5m, crashed into desk from drift |
| 6 | 0.85 | Flow gyro compensation fix | No thrust (GO during ST-Link halt) |
| 7 | 0.85 | Retry | No thrust (same issue) |
| 8 | 0.85 | Fresh power cycle | Stable hover at 0.51m, 2.4 cm drift |

### Progression of key metrics

| Test | Max alt | Roll range | Yaw range | Outcome |
|------|---------|------------|-----------|---------|
| 1 | 8.24 m | [-84, +84] deg | [-176, +179] deg | Tumble + yaw spin |
| 2 | 1.44 m | [-139, +37] deg | [-180, +180] deg | Low hover, crash |
| 3 | 0.62 m | [-22, +18] deg | [-180, +180] deg | Lateral drift into desk |
| 5 | 3.66 m | [-178, +160] deg | [-127, +173] deg | Reached target, desk crash |
| 8 | 0.65 m | [-9.3, +10.3] deg | [-0.2, +56.7] deg | Stable hover |

## Known Remaining Issues

1. **Altitude overshoot** - 51% overshoot on takeoff (0.76m peak vs 0.5m
   target). Could reduce `alt_kp` or increase `vvel_damping`.

2. **Yaw drift** - +51.4 degrees over 10 seconds. No magnetometer on CF2.1+
   and no yaw hold controller. Acceptable for altitude-only profile.

3. **Altitude oscillation** - Std dev 0.101m during hover. The altitude
   Kalman filter and PID gains could be further tuned.

4. **Repeated GO without power cycle** - Test 4 showed stuttering behavior
   when sending a second GO command after landing. Fresh power cycle resolves
   it. Root cause unclear (possibly stale estimator or PID integrator state
   not fully cleared by RESET).

## Data Files

On-board telemetry was logged to SD card at 25 Hz with 24 channels:
`time_ms, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, x, y, altitude,
vx, vy, vz, thrust, target_x, target_y, target_z, target_yaw, gyro_x,
gyro_y, gyro_z, accel_x, accel_y, accel_z`

Ground station telemetry was received via ESB radio at approximately 28 Hz
with 14 channels (10190 samples over the session).

Analysis tools used:
- `tools/analyze_hover.py` - Hover stability metrics
- `tools/analyze_pid.py` - PID response metrics (overshoot, rise time)
- `tools/flight_debug.py` - Flight timeline, crash detection, phase analysis
