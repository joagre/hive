# Pre-Flight Readiness Report: Webots -> Crazyflie 2.1+

Date: 2026-02-10
Updated: 2026-02-11 (cross-referenced against Bitcraze crazyflie-firmware)
Updated: 2026-02-11 (findings #2, #9, #11, #12, #14 fixed)
Updated: 2026-02-11 (findings #3, #5, #13 fixed)
Updated: 2026-02-11 (finding #10 fixed)

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

### 2. ~~PWM frequency 1000x too slow~~ FIXED

**Fixed in commit 0e2af17.** PSC changed from 999 to 0, giving ~328 kHz
PWM matching Bitcraze.

**Files** - `hal/crazyflie-2.1+/platform.c`
**Bitcraze ref** - `src/drivers/interface/motors.h:44-50`,
`src/drivers/src/motors_def.c:29-48`

### 3. ~~Optical flow velocity not compensated for body rotation~~ FIXED

**Fixed.** Gyro-induced rotational flow is now subtracted from the raw
pixel velocity before body-to-world rotation, matching the signs in
Bitcraze `mm_flow.c:79` (`-omega_y`) and `:92` (`+omega_x`). The
`R[2][2]` tilt compensation is skipped for now (cos(15 deg) = 0.966,
negligible at hover angles).

**Files** - `hal/crazyflie-2.1+/platform.c`
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

### 5. ~~Stale sensor data on I2C failure~~ FIXED

**Fixed.** Added `accel_valid` and `gyro_valid` flags to `sensor_data_t`.
HAL sets flags false before reading, true on success. Estimator checks
both flags at the top of `validate_sensors()` and skips the cycle if
either is false. Skipping one 4ms estimator cycle on I2C failure is
safer than running on stale accel/gyro data.

**Files** - `include/types.h`, `hal/crazyflie-2.1+/platform.c`,
`hal/webots-crazyflie/hal_sensors.c`, `estimator_actor.c`

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

### 9. ~~Kalman filter over-corrects with repeated stale measurements~~ FIXED

**Fixed in commit 64c1982.** Split into separate predict (every cycle)
and correct (only when a new altitude measurement arrives). Detects new
data by value change, matching Bitcraze's approach of gating corrections
on fresh measurements.

**Files** - `estimator_actor.c`
**Bitcraze ref** - `src/modules/src/estimator/estimator_kalman.c:301-367`,
`src/deck/drivers/src/zranger2.c:131-132`

### 10. ~~No battery monitoring during flight~~ FIXED

**Fixed.** Added `battery_actor` - dedicated LOW-priority actor that
samples `hal_power_get_battery()` at 2 Hz. Two-tier thresholds matching
Bitcraze (3.2V warning, 3.0V critical) with 10-reading debounce (5
seconds). On debounced critical, sends `NOTIFY_LOW_BATTERY` to
flight_manager, which cancels countdown (ARMED state) or initiates
emergency landing (FLYING state).

**Files** - `battery_actor.c`, `flight_manager_actor.c`
**Bitcraze ref** - `src/hal/src/pm_stm32f4.c:397-501`

## Safety/Robustness Gaps in Real HAL

### 11. ~~No hardware watchdog~~ FIXED

**Fixed in commit 0e2af17.** IWDG enabled with Bitcraze-matching config
(prescaler 32, reload 188, 100-353ms timeout). Fed every 80ms from
SysTick_Handler. Detects watchdog resets on startup via RCC flag.

**Files** - `hal/crazyflie-2.1+/platform.c`
**Bitcraze ref** - `src/drivers/src/watchdog.c:47-76`

### 12. ~~No HardFault handler~~ FIXED

**Fixed in commit 0e2af17.** HardFault handler extracts stacked
registers (R0-R3, R12, LR, PC, PSR) and ARM fault status registers
(CFSR, HFSR, BFAR) via SWO. Separate MemManage, BusFault, and
UsageFault handlers also stop motors. All handlers call
`platform_emergency_stop()` first, then loop without feeding the
watchdog (IWDG resets MCU within 100-353ms).

**Files** - `hal/crazyflie-2.1+/platform.c`
**Bitcraze ref** - `src/drivers/src/nvic.c:87-154`

### 13. ~~I2C bus recovery loop is unbounded~~ FIXED

**Fixed.** Changed the unbounded `while` loop to a bounded `for` loop
with a 9-iteration limit (I2C standard maximum clock cycles to release
any slave), matching the I2C1 recovery in `platform.c`.

**Files** - `hal/crazyflie-2.1+/i2c_drv.c`
**Bitcraze ref** - `src/drivers/src/i2c_drv.c:322-339`

### 14. ~~I2C1 clock timing too fast~~ FIXED

**Fixed in commit 0e2af17.** NOP loops replaced with
`platform_delay_us(3)` for accurate 3us half-cycle timing (~167 kHz
clock, well within I2C fast mode spec).

**Files** - `hal/crazyflie-2.1+/platform.c`
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
4. Have a kill switch ready (50ms deadman + IWDG watchdog provide backup)
5. Check battery voltage before each attempt
6. ~~If altitude oscillates visibly, check PWM frequency first (finding #2)~~ PWM fixed

## Cross-Reference Summary

| Finding | Hive | Bitcraze | Match? |
|---|---|---|---|
| PWM resolution | 8-bit (ARR=255) | 8-bit (ARR=255) | Yes |
| PWM frequency | ~328 kHz (PSC=0) | ~328 kHz (PSC=0) | Yes (FIXED) |
| Motor mixer signs | Legacy: same | Legacy: same | Yes |
| Motor numbering | M1-M4, FR/BR/BL/FL | M1-M4, FR/BR/BL/FL | Yes |
| Motor pin mapping | PA1/PB11/PA15/PB9 | PA1/PB11/PA15/PB9 | Yes |
| Flow gyro compensation | Velocity-domain subtraction | EKF measurement model | Yes (FIXED) |
| Attitude estimation | Euler integration | Quaternion (Mahony) | **NO** |
| Yaw drift (no mag) | Yes | Yes (same limitation) | Yes |
| KF stale measurement | Correct on new data only | Queue, new data only | Yes (FIXED) |
| Battery monitoring | 2 Hz actor, debounced landing | PM task, auto-shutdown | Yes (FIXED) |
| HardFault handler | Motor stop + register dump | Motor stop + diagnostics | Yes (FIXED) |
| Watchdog | IWDG, 100-353ms | IWDG, 100-353ms | Yes (FIXED) |
| I2C bus unlock loop | Bounded (9 cycles) | Unbounded (same code) | Better (FIXED) |
| I2C1 bus recovery timing | delay_us(3) (3us) | sleepus(10) (10us) | Yes (FIXED) |
| Barometer in estimator | Disabled | Active at 50 Hz | **NO** |
