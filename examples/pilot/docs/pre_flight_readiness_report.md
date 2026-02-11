# Pre-Flight Readiness Report: Webots -> Crazyflie 2.1+

Date: 2026-02-10

Comprehensive audit comparing the Webots simulation HAL
(`hal/webots-crazyflie/`) against the real hardware HAL
(`hal/crazyflie-2.1+/`) to identify issues that could affect
flight stability on the real Crazyflie.

## Items That Could Cause a Crash

### 1. Keep waypoints below ~1.0m (most important)

The VL53L1x rangefinder tops out at 1.3m and the barometer is disabled
on the real HAL. Above 1.3m, the Kalman filter runs on stale altitude
data and becomes overconfident. The `ALTITUDE` and `FULL_3D` profiles
have 1.2m waypoints - dangerously close to the ceiling. Use `FIRST_TEST`
(0.5m hover) for initial flights.

**Files** - `estimator_actor.c:324-336`, `hal/crazyflie-2.1+/hal_config.h:81`,
`hal/crazyflie-2.1+/platform.c:1257`

### 2. 8-bit motor PWM vs 16-bit in Bitcraze firmware

Only 256 thrust levels (`TIM2->ARR = 255`). At hover thrust 0.38, the
motor command is ~97/255. Each step is ~0.4% of full power. This is much
coarser than the Webots simulation and can cause limit-cycle altitude
oscillations. The Bitcraze firmware uses `ARR=65535`. Consider bumping
to 16-bit if oscillations appear.

**Files** - `hal/crazyflie-2.1+/platform.c:860-861`

### 3. Optical flow velocity not compensated for body rotation

Raw PMW3901 deltas include apparent motion from pitch/roll changes. The
Bitcraze firmware subtracts `gyro_rate * height / focal_length`. This
code does not. Every attitude correction injects false velocity into the
position estimate.

**Files** - `hal/crazyflie-2.1+/platform.c:1300-1301`

### 4. Motor mixer pitch sign difference

Webots negates pitch (`pitch = -cmd->pitch`) before mixing, real HAL
does not. Both use the same mixer equations otherwise. If this
compensates for a Webots model quirk it is fine, but if a sign bug was
hidden in simulation, pitch control will be reversed on real hardware.

**Verification** - Bench-test with props off: positive pitch command
should tilt nose down (increase front motor thrust).

**Files** - `hal/webots-crazyflie/hal_motors.c:28`,
`hal/crazyflie-2.1+/hal_motors.c:60-67`

### 5. Stale sensor data on I2C failure

When `bmi088_get_accel_data()` fails, the previous reading is silently
reused. No validity flag, no error counter. A single I2C glitch during
flight means the estimator runs on stale gyro/accel data.

**Files** - `hal/crazyflie-2.1+/platform.c:1241`

## Items That Will Cause Degraded Performance

### 6. PID gains tuned for different motor dynamics

Webots simulates a 20ms first-order motor lag and tunes gains to
compensate. The real HAL writes PWM directly - no software lag. The
`hal_config.h` gains are already lower for CF (e.g., attitude Kp 1.8 vs
2.5, rate Kp 0.020 vs 0.028), which is the right direction, but they
may still need real-world tuning.

| Parameter | Webots | Crazyflie | Notes |
|---|---|---|---|
| `HAL_HOVER_THRUST` | 0.553 | 0.38 | Different physics models |
| `HAL_ALT_PID_KP` | 0.18 | 0.12 | CF more conservative |
| `HAL_ALT_PID_KI` | 0.03 | 0.01 | CF much lower integral |
| `HAL_VVEL_DAMPING_GAIN` | 0.35 | 0.25 | CF less damping |
| `HAL_ATTITUDE_PID_KP` | 2.5 | 1.8 | CF notably lower |
| `HAL_RATE_PID_KP` | 0.028 | 0.020 | CF lower |

### 7. Position will drift - no absolute reference

Webots uses GPS-quality position. Real CF dead-reckons from optical flow
integration. Combined with finding #3 (rotation coupling) and no
integral term in the position PD controller (`position_actor.c:122`),
expect steady-state position offset and cumulative drift.

### 8. Yaw will drift continuously

No magnetometer, no runtime gyro bias correction. The complementary
filter corrects roll/pitch via accelerometer but has zero yaw correction
source. Keep flights short.

**Files** - `fusion/complementary_filter.c:82-84`,
`hal/crazyflie-2.1+/platform.c:1248-1251`

### 9. Kalman filter over-corrects with repeated stale measurements

The rangefinder updates at 40Hz but `altitude_kf_update()` runs every
4ms (250Hz). The same measurement is fed 6x as independent observations,
artificially shrinking covariance.

**Files** - `estimator_actor.c:369`

### 10. No battery monitoring during flight

Voltage is logged once before flight (`pilot.c:63`). No in-flight
low-voltage cutoff. As voltage sags under load, hover thrust becomes
insufficient.

## Safety/Robustness Gaps in Real HAL

### 11. No hardware watchdog

If MCU hangs, motors hold last PWM indefinitely. The 50ms deadman in
motor_actor helps only if the scheduler is alive.

### 12. No HardFault handler

Maps to `Default_Handler` (infinite loop with motors running).

**Files** - `hal/crazyflie-2.1+/startup_stm32f405.s:255`

### 13. I2C1 bus recovery loop is unbounded

If SDA stuck low, system hangs forever.

**Files** - `hal/crazyflie-2.1+/i2c_drv.c:334`

### 14. I2C1 clock timing too fast

Comment says 500 NOPs needed for 3us half-cycle, code uses 100 NOPs
(0.6us). May not properly reset VL53L1x.

**Files** - `hal/crazyflie-2.1+/platform.c:180-192`

## Minor / Not Urgent

- Gravity constant difference (9.81 vs 9.80665) - negligible
- Barometer initialized/calibrated but never used - wastes ~1s at startup
- `FLOW_SCALE = 0.0005f` may need in-flight calibration
- Landing detection threshold (0.08m) is near VL53L1x minimum range
- `cf_use_mag` tunable defaults to enabled but mag_valid is always false - harmless
- Complementary filter uses simple Euler integration (no cross-axis coupling) - acceptable at small angles
- PID derivative computed on error, not measurement - derivative kick on setpoint changes
- `platform_get_time_us()` has a race between reading ms and SysTick->VAL (sanity-checked downstream)
- GPIO AFR registers OR'd without clearing first (`platform.c:236,414`) - correct from reset but not after software reset

## Accelerometer Data Source Difference

Webots **synthesizes** accelerometer readings from known attitude by
projecting gravity. It does not include dynamic acceleration (vibration,
linear motion, centripetal forces). On real hardware, the BMI088 measures
total specific force. This means:

- The complementary filter sees cleaner, more idealized data in simulation
- Motor vibration (major issue for MEMS accelerometers) is absent in Webots
- Kalman filter tuning validated in simulation may be too aggressive for real hardware

## Sensor Rate Differences

| Sensor | Webots | Crazyflie | Notes |
|---|---|---|---|
| Gyro | 250 Hz (4ms step) | 1000 Hz ODR, 116 Hz BW | HW-filtered |
| Accelerometer | 250 Hz | 1600 Hz ODR | HW-filtered |
| Barometer | Not simulated | 25 Hz (disabled) | Prop wash issues |
| Rangefinder | 250 Hz | 40 Hz (VL53L1x) | 6x oversampled by KF |
| Optical flow | 250 Hz | ~100 Hz (PMW3901) | Variable rate |
| Control loop | 250 Hz | 250 Hz | Matched |

## Recommendation for First Flight

1. Use `FIRST_TEST` profile (0.5m hover only)
2. Bench-test motor mixing with props off - verify pitch/roll/yaw directions
3. Keep flight under 30 seconds to limit drift accumulation
4. Have a kill switch ready (the 50ms deadman is good but watchdog is missing)
5. Check battery voltage before each attempt
6. If altitude oscillates visibly, the 8-bit PWM resolution is the likely cause
