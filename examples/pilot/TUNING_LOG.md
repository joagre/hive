# Tuning Log

Parameter changes and flight test results for the Crazyflie 2.1+
running the Hive pilot autopilot. Newest session first.

Hardware: Crazyflie 2.1+ with Flow Deck v2, SD card deck, 3g crash cage.

---

## Session 6 - Altitude tuning continued

Continuing from session 5. Set via radio before GO (these reset on reboot):

```bash
python3 tools/ground_station.py --set-param pos_kp 0.04
python3 tools/ground_station.py --set-param pos_kd 0.05
python3 tools/ground_station.py --set-param vvel_damping 0.55
```

| Parameter | Compiled default | Radio override | Reasoning |
|-----------|-----------------|----------------|-----------|
| `pos_kp` | 0.14 | 0.04 | Gentle position hold, avoid chasing flow drift |
| `pos_kd` | 0.16 | 0.05 | Gentle velocity damping for XY |
| `vvel_damping` | 0.45 | 0.55 | More altitude braking, untested (battery died) |

### Results

(pending)

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
