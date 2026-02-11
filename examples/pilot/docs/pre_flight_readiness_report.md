# Pre-Flight Readiness Report: Webots -> Crazyflie 2.1+

Date: 2026-02-10
Updated: 2026-02-11 (cross-referenced against Bitcraze crazyflie-firmware)

Comprehensive audit comparing the Webots simulation HAL
(`hal/webots-crazyflie/`) against the real hardware HAL
(`hal/crazyflie-2.1+/`) to identify issues that could affect
flight stability on the real Crazyflie.

Each finding has been cross-referenced against the Bitcraze
crazyflie-firmware source in `local/crazyflie-firmware/`.

## Items That Could Cause a Crash

### 1. Keep waypoints below ~1.0m (most important)

The VL53L1x rangefinder tops out at 1.3m and the barometer is disabled
on the real HAL. Above 1.3m, the Kalman filter runs on stale altitude
data and becomes overconfident. The `ALTITUDE` and `FULL_3D` profiles
have 1.2m waypoints - dangerously close to the ceiling. Use `FIRST_TEST`
(0.5m hover) for initial flights.

Bitcraze uses barometer as a backup altitude source (see finding #15),
which provides altitude sensing above rangefinder range. Hive has no
such fallback.

**Files** - `estimator_actor.c:324-336`, `hal/crazyflie-2.1+/hal_config.h:81`,
`hal/crazyflie-2.1+/platform.c:1257`

### 2. PWM frequency 1000x too slow (NEW - found during cross-reference)

Both Hive and Bitcraze use 8-bit PWM resolution (`ARR=255`), but the
prescaler differs dramatically:

| | Hive | Bitcraze |
|---|---|---|
| PSC | 999 | 0 |
| ARR | 255 | 255 |
| PWM frequency | 328 Hz | ~328 kHz |
| Resolution | 8-bit (256 levels) | 8-bit (256 levels) |

Bitcraze deliberately chose ~328 kHz because at lower frequencies the
PWM ripple affects the NCP702 voltage regulator on the Crazyflie PCB
(documented in `motors.h` line 46). The Hive pilot's 328 Hz PWM could
cause audible motor whine and voltage regulator instability.

**Files** - `hal/crazyflie-2.1+/platform.c:856-861`
**Bitcraze ref** - `src/drivers/interface/motors.h:44-50`,
`src/drivers/src/motors_def.c:29-48`

### 3. Optical flow velocity not compensated for body rotation

Confirmed missing. Bitcraze compensates in the EKF measurement model
(`mm_flow.c`), not in the deck driver. The key equations:

```
predictedNX = (dt * Npix / thetapix) * (Vx * R[2][2] / z - omega_y)
predictedNY = (dt * Npix / thetapix) * (Vy * R[2][2] / z + omega_x)
```

The `omega_y` / `omega_x` terms subtract gyro-induced rotational flow
from the raw sensor reading, isolating translational velocity. Hive's
code uses a simple `delta * FLOW_SCALE * height` with no gyro term.
Every attitude correction injects false velocity into the position
estimate.

Bitcraze also applies tilt compensation via `R[2][2]` (rotation matrix
element) to the translational term. Hive does not.

**Files** - `hal/crazyflie-2.1+/platform.c:1300-1301`
**Bitcraze ref** - `src/modules/src/kalman_core/mm_flow.c:79,92`

### 4. Motor mixer pitch sign - VERIFIED OK

~~Webots negates pitch, real HAL does not. Possible sign bug.~~

Cross-reference confirms the Crazyflie hardware HAL matches Bitcraze's
legacy mixer exactly (same signs, same motor layout). The Webots pitch
negation compensates for the simulator's different motor numbering
convention. Motor numbering, positions, rotation directions, and GPIO
pin assignments all match the Bitcraze reference:

| Motor | Position | Rotation | GPIO | Timer |
|---|---|---|---|---|
| M1 | Front-right | CCW | PA1 | TIM2_CH2 |
| M2 | Back-right | CW | PB11 | TIM2_CH4 |
| M3 | Back-left | CCW | PA15 | TIM2_CH1 |
| M4 | Front-left | CW | PB9 | TIM4_CH4 |

**Status** - No action needed. Bench-test still recommended to confirm.

**Bitcraze ref** - `src/modules/src/power_distribution_quadrotor.c:84-93`,
`src/drivers/src/motors_def.c:720-726`

### 5. Stale sensor data on I2C failure

When `bmi088_get_accel_data()` fails, the previous reading is silently
reused. No validity flag, no error counter. A single I2C glitch during
flight means the estimator runs on stale gyro/accel data.

Bitcraze has the same pattern in their sensor driver, but is protected
by their watchdog (see finding #11) and HardFault handler (see
finding #12) which limit the blast radius of failures.

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

### 8. Yaw will drift continuously - same as Bitcraze

No magnetometer, no runtime gyro bias correction. The complementary
filter corrects roll/pitch via accelerometer but has zero yaw correction
source.

Cross-reference confirms Bitcraze has the same limitation for stock
CF2.1+ without Lighthouse. Their default complementary estimator
(sensfusion6) is 6-DOF with no yaw correction. The Kalman estimator
only gets yaw corrections from the Lighthouse positioning system, not
the onboard magnetometer. Both Hive and Bitcraze accept yaw drift as a
known limitation. Keep flights short.

**Files** - `fusion/complementary_filter.c:82-84`,
`hal/crazyflie-2.1+/platform.c:1248-1251`
**Bitcraze ref** - `src/modules/src/sensfusion6.c:181-252`

### 9. Kalman filter over-corrects with repeated stale measurements

The rangefinder updates at 40Hz but `altitude_kf_update()` runs every
4ms (250Hz). The same measurement is fed 6x as independent observations,
artificially shrinking covariance.

Bitcraze solves this with a measurement queue: sensors enqueue data
asynchronously, and the Kalman filter dequeues and processes ONLY actual
new measurements. Prediction runs at 100 Hz (not 1000 Hz like the IMU),
and IMU data is subsampled via an accumulator before prediction.

**Files** - `estimator_actor.c:369`, `fusion/altitude_kf.c:239-242`
**Bitcraze ref** - `src/modules/src/estimator/estimator_kalman.c:301-367`,
`src/deck/drivers/src/zranger2.c:131-132`

### 10. No battery monitoring during flight

Voltage is logged once before flight and included in telemetry packets,
but there is no in-flight low-voltage protection. No actor checks
battery during flight to trigger an emergency landing.

Bitcraze has a comprehensive battery management system: dedicated PM
task at 10 Hz, two-tier voltage thresholds (3.2V warning, 3.0V critical),
5-second debouncing to avoid false triggers from voltage sag, automatic
system shutdown on critical low, and LED/sound alerts.

**Files** - `pilot.c:63`, `comms_actor.c:541`
**Bitcraze ref** - `src/hal/src/pm_stm32f4.c:397-501`

## Safety/Robustness Gaps in Real HAL

### 11. No hardware watchdog

If MCU hangs, motors hold last PWM indefinitely. The 50ms deadman in
motor_actor helps only if the scheduler is alive.

Bitcraze uses IWDG (Independent Watchdog) with a 100-353ms timeout,
fed every 80ms from the system task. It also detects and logs watchdog
resets on startup. This protects against the unbounded I2C loop in
finding #13 and other hangs.

**Bitcraze ref** - `src/drivers/src/watchdog.c:47-76`

### 12. No HardFault handler

Maps to `Default_Handler` (infinite loop with motors running).

Bitcraze has a full HardFault handler that: extracts all stacked
registers (R0-R3, R12, LR, PC, PSR), prints them via UART, reads
all ARM fault status registers, stops all motors, shows LED fault
pattern, and persists fault data to RAM for post-mortem. Separate
handlers for MemManage, BusFault, and UsageFault also call
`motorsStop()`.

**Files** - `hal/crazyflie-2.1+/startup_stm32f405.s:255`
**Bitcraze ref** - `src/drivers/src/nvic.c:87-154`

### 13. I2C bus recovery loop is unbounded - same as Bitcraze

The `i2cdrvdevUnlockBus()` function has an unbounded `while` loop on SDA
stuck low. Cross-reference confirms Bitcraze has the identical unbounded
loop. However, Bitcraze is protected by their IWDG watchdog which would
reset the system within ~350ms. Hive has no such safety net.

Note: Hive's separate I2C1 recovery in `platform.c:180` uses a bounded
`for (int i = 0; i < 9; i++)` loop, which is actually better than both
Bitcraze's and the I2C3 path.

**Files** - `hal/crazyflie-2.1+/i2c_drv.c:334`
**Bitcraze ref** - `src/drivers/src/i2c_drv.c:322-339`

### 14. I2C1 clock timing too fast

Comment says 500 NOPs needed for 3us half-cycle, code uses 100 NOPs
(0.6us), giving ~833 kHz clock - above I2C fast mode spec (400 kHz).

Bitcraze uses proper `sleepus(10)` timestamp-based delays (10us
half-cycle, giving 50 kHz clock, well within spec). Hive's I2C3 path
(Bitcraze-derived code) correctly uses `sleepus()`, but the I2C1 path
in `platform.c` uses fragile NOP-counting with incorrect count.

**Files** - `hal/crazyflie-2.1+/platform.c:180-192`
**Bitcraze ref** - `src/drivers/src/i2c_drv.c:72,332-338`,
`src/utils/src/sleepus.c:29-34`

## Estimator Architecture Differences

### 15. Complementary filter uses Euler integration, Bitcraze uses quaternions

Hive integrates gyro rates directly into Euler angles:
```
roll += gyro_x * dt
pitch += gyro_y * dt
```

Bitcraze uses Mahony AHRS algorithm operating in quaternion space. This
avoids cross-axis coupling errors, gimbal lock, and the singularity
issues of Euler angle integration. At small angles (<15 degrees, typical
for hover) the difference is negligible.

**Files** - `fusion/complementary_filter.c:82-84`
**Bitcraze ref** - `src/modules/src/sensfusion6.c:181-252`

### 16. Barometer disabled vs Bitcraze active at 50 Hz

Hive initializes BMP388 and calibrates it (~1s at startup) but then
discards all readings during flight.

Bitcraze reads the barometer at 50 Hz and fuses it into the Kalman
filter as a Z-altitude observation. The measurement noise parameter
(`measNoiseBaro`) naturally down-weights noisy barometer readings rather
than discarding them entirely. This provides altitude sensing above
rangefinder range and redundancy if the rangefinder fails.

**Files** - `hal/crazyflie-2.1+/platform.c:1254-1259`
**Bitcraze ref** - `src/hal/src/sensors_bmi088_bmp3xx.c:350-368`,
`src/modules/src/kalman_core/kalman_core.c:294-308`

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

## Minor / Not Urgent

- Gravity constant difference (9.81 vs 9.80665) - negligible
- Barometer calibration wastes ~1s at startup for unused data
- `FLOW_SCALE = 0.0005f` may need in-flight calibration
- Landing detection threshold (0.08m) is near VL53L1x minimum range
- `cf_use_mag` tunable defaults to enabled but mag_valid is always false - harmless
- PID derivative computed on error, not measurement - derivative kick on setpoint changes
- `platform_get_time_us()` has a race between reading ms and SysTick->VAL (sanity-checked downstream)
- GPIO AFR registers OR'd without clearing first (`platform.c:236,414`) - correct from reset but not after software reset

## Recommendation for First Flight

1. Use `FIRST_TEST` profile (0.5m hover only)
2. Bench-test motor mixing with props off - verify pitch/roll/yaw directions
3. Keep flight under 30 seconds to limit drift accumulation
4. Have a kill switch ready (the 50ms deadman is good but watchdog is missing)
5. Check battery voltage before each attempt
6. If altitude oscillates visibly, check PWM frequency first (finding #2)

## Cross-Reference Summary

| Finding | Hive | Bitcraze | Match? |
|---|---|---|---|
| PWM resolution | 8-bit (ARR=255) | 8-bit (ARR=255) | Yes |
| PWM frequency | 328 Hz (PSC=999) | ~328 kHz (PSC=0) | **NO** |
| Motor mixer signs | Legacy: same | Legacy: same | Yes |
| Motor numbering | M1-M4, FR/BR/BL/FL | M1-M4, FR/BR/BL/FL | Yes |
| Motor pin mapping | PA1/PB11/PA15/PB9 | PA1/PB11/PA15/PB9 | Yes |
| Flow gyro compensation | None | EKF measurement model | **NO** |
| Attitude estimation | Euler integration | Quaternion (Mahony) | **NO** |
| Yaw drift (no mag) | Yes | Yes (same limitation) | Yes |
| KF stale measurement | Corrects every cycle | Queue, new data only | **NO** |
| Battery monitoring | One-time log + telemetry | PM task, auto-shutdown | **NO** |
| HardFault handler | Infinite loop | Motor stop + diagnostics | **NO** |
| Watchdog | None | IWDG, 100-353ms | **NO** |
| I2C bus unlock loop | Unbounded (I2C3) | Unbounded (same code) | Yes |
| I2C1 bus recovery timing | 100 NOPs (0.6us) | sleepus(10) (10us) | **NO** |
| Barometer in estimator | Disabled | Active at 50 Hz | **NO** |
