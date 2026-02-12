# Tuning Log

Parameter changes and flight test results for the Crazyflie 2.1+
running the Hive pilot autopilot. Newest session first.

Hardware: Crazyflie 2.1+ with Flow Deck v2, SD card deck, 3g crash cage.

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
flight after fixing the SD card write path (ring buffer + DMA yield).

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
