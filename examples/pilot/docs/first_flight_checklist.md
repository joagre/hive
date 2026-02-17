# Crazyflie 2.1+ First Flight Checklist

Step-by-step guide from hardware verification to first flight.

## Prerequisites

- [ ] Crazyflie 2.1+ assembled
- [ ] Battery charged (3.7V+)
- [ ] Debug adapter + ST-Link V3 connected
- [ ] Propellers available but **NOT installed yet**
- [ ] Clear indoor space (3m x 3m minimum)
- [ ] Safety glasses recommended

---

## Phase 1: Hardware Tests

Run the sensors_motors test to verify all hardware components.

```bash
cd examples/pilot/tests
make PLATFORM=crazyflie TEST=sensors_motors
make flash-crazyflie TEST=sensors_motors
./st-trace.sh -t 0    # No timeout, Ctrl-C to stop
```

### Required Tests (must pass)

- [ ] Boot & Clock: 168 MHz confirmed
- [ ] I2C scan: Found 0x18 (accel), 0x68 (gyro), 0x77 (baro)
- [ ] BMI088 Accel chip ID: 0x1E
- [ ] BMI088 Gyro chip ID: 0x0F
- [ ] BMP388 chip ID: 0x50
- [ ] Accelerometer data: X,Y near 0, Z near 1g (drone level)
- [ ] Gyroscope data: All axes near 0 (drone stationary)
- [ ] Barometer data: 95-106 kPa, 15-35 degC
- [ ] Motor test: All 4 motors spin (props OFF!)
- [ ] Radio: Battery voltage received

### Optional Tests (Flow deck)

- [ ] VL53L1x detected at 0x29
- [ ] PMW3901 product ID: 0x49
- [ ] ToF range reading valid

**STOP if any required test fails. Debug before proceeding.**

---

## Phase 2: Pilot Firmware - Props OFF

Flash the pilot firmware and verify control response without props.

```bash
cd examples/pilot
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST
make -f Makefile.crazyflie-2.1plus flash
```

### Motor Response Tests (props OFF)

Power on and let it calibrate (keep still and level during calibration).

- [ ] LED indicates armed state (solid on)
- [ ] Hold drone in hand, tilt forward -> rear motors speed up
- [ ] Tilt backward -> front motors speed up
- [ ] Tilt left -> right motors speed up
- [ ] Tilt right -> left motors speed up
- [ ] Rotate CW -> CCW motors (M1, M3) speed up
- [ ] Rotate CCW -> CW motors (M2, M4) speed up

If motor response is **inverted**, check:
- Motor wiring (swap wires to reverse direction)
- Motor position (M1-M4 in correct corners)
- Mixer signs in `hal_motors.c`

---

## Phase 3: Tethered Flight Test

First powered test with propellers installed.

### Setup

- [ ] Install propellers (CCW on M1/M3, CW on M2/M4)
- [ ] Verify prop rotation direction matches motor
- [ ] Attach tether to center of drone (string to fixed point above)
- [ ] Tether length: ~1m (allows hover but limits travel)
- [ ] Clear area of obstacles
- [ ] Battery freshly charged

### Pre-Flight

- [ ] Place drone on level surface
- [ ] Power on
- [ ] Wait for calibration (LED blinking -> solid)
- [ ] Stand clear (2m minimum)

### Flight Test (FLIGHT_PROFILE_FIRST_TEST: 6s hover at 0.5m)

**Timeline after power-on:**

| Time | Phase | Motors |
|------|-------|--------|
| 0-15s | Grace period (LED blinking) | OFF |
| 15-20s | Calibration (keep still!) | OFF |
| 20-80s | Startup delay (step back!) | OFF |
| 80s | Flight start | ON |
| 80-86s | Hover at 0.5m | ON |
| 86s+ | Landing | OFF |

**Motors stay OFF during the grace period and countdown.** Use this time to step back safely.

The drone will automatically:
1. Wait for flight authorization (10s armed countdown)
2. Ramp up thrust
3. Attempt to hover at 0.5m for 6 seconds
4. Land and disarm

**Observe**

- [ ] Drone lifts off
- [ ] Attitude is stable (not oscillating wildly)
- [ ] Altitude is roughly maintained
- [ ] Lands softly

### Common Issues

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Flips on takeoff | Wrong prop direction or position | Check prop placement |
| Oscillates rapidly | PID gains too high | Reduce rate PID gains |
| Drifts slowly | PID gains too low | Increase attitude PID gains |
| Can't lift off | Thrust too low | Increase HAL_HOVER_THRUST |
| Shoots up | Thrust too high | Decrease HAL_HOVER_THRUST |
| Altitude oscillates | Alt PID needs tuning | Adjust HAL_ALT_PID_* |

---

## Phase 4: Free Flight

Only proceed if tethered flight was stable.

### Setup

- [ ] Remove tether
- [ ] Ensure 3m x 3m clear space
- [ ] Soft landing surface (carpet/grass)
- [ ] Kill switch ready (power disconnect)

### Flight Profiles

```bash
# Conservative hover test (6s at 0.5m)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST

# Altitude changes (max 1.0m, within flow deck range)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_ALTITUDE

# Full 3D (requires Flow deck, max 1.0m)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_FULL_3D
```

### Tuning Sequence

If flight is unstable, tune in this order:

1. **Rate PID** (`HAL_RATE_PID_*`) - Controls rotation speed
   - Start low, increase until responsive but not oscillating

2. **Attitude PID** (`HAL_ATTITUDE_PID_*`) - Controls angle
   - Increase P until it holds level, add D to dampen

3. **Altitude PID** (`HAL_ALT_PID_*`) - Controls height
   - Tune after attitude is stable

4. **Position PID** (`HAL_POS_PID_*`) - Controls XY position (Flow deck only)
   - Tune last, requires stable altitude hold

---

## Emergency Procedures

### Unexpected Behavior
1. **Don't chase it** - let it crash
2. Disconnect battery immediately after landing
3. Check for damage before next attempt

### Runaway Drone
1. Cut power if accessible
2. Let battery drain if not
3. Block with soft object if heading toward people

### Post-Crash Checklist
- [ ] Props intact (no cracks or chips)
- [ ] Motors spin freely
- [ ] Frame not cracked
- [ ] Battery not puffy or damaged
- [ ] All connectors seated

---

## Success Criteria

First flight is successful if:

- [ ] Drone lifts off under control
- [ ] Hovers within +/-20cm of target altitude
- [ ] No violent oscillations
- [ ] Lands without crashing
- [ ] Repeatable (3 consecutive flights)

---

## PID Tuning Reference

Default values in `hal/crazyflie-2.1plus/hal_config.h` (conservative for first flights):

```c
// Rate (innermost loop)
HAL_RATE_PID_KP     0.018   // Conservative (Webots: 0.028)
HAL_RATE_PID_KI     0.001   // Conservative (Webots: 0.002)
HAL_RATE_PID_KD     0.001   // Conservative (Webots: 0.003)

// Attitude (middle loop)
HAL_ATTITUDE_PID_KP 1.5     // Conservative (Webots: 2.5)
HAL_ATTITUDE_PID_KI 0.0
HAL_ATTITUDE_PID_KD 0.08    // Conservative (Webots: 0.15)

// Altitude
HAL_ALT_PID_KP      0.12    // Conservative (Webots: 0.18)
HAL_ALT_PID_KI      0.01    // Conservative (Webots: 0.03)
HAL_ALT_PID_KD      0.0

// Vertical velocity damping
HAL_VVEL_DAMPING_GAIN 0.25  // Conservative (Webots: 0.35)

// Hover thrust (normalized 0.0-1.0)
HAL_HOVER_THRUST    0.38    // Calibrated with thrust test
```

**Note** - Crazyflie gains are intentionally conservative for first flights.
If drone is sluggish but stable, gradually increase toward Webots values.
HAL_HOVER_THRUST may need calibration - start at 0.38 and adjust.

Adjust in 10-20% increments. Either reflash after each change, or use runtime
parameter tuning (see below) for faster iteration.

**Runtime tuning alternative** - Use the ground station to tune gains without reflashing:
```bash
python3 tools/ground_station.py --set-param rate_kp 0.022
python3 tools/ground_station.py --set-param att_kp 1.8
python3 tools/ground_station.py --set-param vvel_damping 0.40
```
Once satisfied, update `hal_config.h` with final values for persistent defaults.

---

## Notes

Record observations after each flight:

```
Date: ___________
Flight #: ___________
Profile: ___________
Battery voltage: ___________

Observations:
_________________________________
_________________________________
_________________________________

Changes for next flight:
_________________________________
_________________________________
```
