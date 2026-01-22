# Webots Crazyflie HAL

Simulation HAL for the Bitcraze Crazyflie model in Webots.

## Quick Start

```bash
# From examples/pilot/
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots and start the simulation.

## Webots Device Names

| Device | Name | Type |
|--------|------|------|
| Motor 1 (rear-left) | `m1_motor` | RotationalMotor |
| Motor 2 (front-left) | `m2_motor` | RotationalMotor |
| Motor 3 (front-right) | `m3_motor` | RotationalMotor |
| Motor 4 (rear-right) | `m4_motor` | RotationalMotor |
| Gyroscope | `gyro` | Gyro |
| Inertial Unit | `inertial_unit` | InertialUnit |
| GPS | `gps` | GPS |

## Motor Layout

X-configuration matching Bitcraze firmware:

```
        Front
      M2(CW)  M3(CCW)
          \  /
           \/
           /\
          /  \
      M1(CCW) M4(CW)
        Rear
```

**Motor mixing (in hal_webots.c):**
```
M1 = thrust - roll + pitch + yaw  (rear-left, CCW)
M2 = thrust - roll - pitch - yaw  (front-left, CW)
M3 = thrust + roll - pitch + yaw  (front-right, CCW)
M4 = thrust + roll + pitch - yaw  (rear-right, CW)
```

**Motor velocity signs:** M1,M3 use negative velocity; M2,M4 use positive.

## Sensors

| Sensor | Source | Notes |
|--------|--------|-------|
| Accelerometer | Synthesized from gravity + inertial_unit | No accelerometer device in PROTO |
| Gyroscope | `gyro` device | Body-frame angular rates |
| Position | `gps` device | Perfect XYZ position |
| Attitude | `inertial_unit` device | Used internally for accel synthesis |

**Sensor noise:** Realistic noise is simulated with configurable levels:

| Level | Description | Use Case |
|-------|-------------|----------|
| 0 | No noise | Clean debugging |
| 1 | Low noise + drift | General testing |
| 2 | Moderate noise + drift | Stress testing |
| 3 | Flow deck realistic (default) | Pre-hardware testing |

Level 3 (default) matches real Crazyflie flow deck behavior: measurement noise
but no artificial drift accumulation (flow deck is ground-referenced).

Build with custom level: `make SENSOR_NOISE=0`

## Files

| File | Description |
|------|-------------|
| `hal_webots.c` | HAL implementation |
| `hal_config.h` | Platform-specific PID gains and thrust |

## Configuration

| Setting | Location | Description |
|---------|----------|-------------|
| PID gains, thrust | `hal_config.h` | Platform-specific tuning |
| Actors, buses, pools | `hive_config.mk` | Shared (pilot-determined) |
| Stack sizes | `Makefile` | Platform-specific (matches Crazyflie for realistic testing) |

## Differences from Hardware

| Feature | Webots | Hardware |
|---------|--------|----------|
| Position | GPS (perfect) | Flow deck (relative) or none |
| Altitude | GPS Z | Barometer or ToF |
| Accelerometer | Synthesized | Real MEMS sensor |
| Motor output | Velocity (rad/s) | PWM duty cycle |
| Time control | `hal_step()` + simulation | Real-time |

## Simulation Realism Features

### Motor Response Lag

Real motors have electrical and mechanical delay. The simulation models this
with a first-order filter (default 20ms time constant):

```bash
make MOTOR_LAG=30    # 30ms time constant
make MOTOR_LAG=0     # Instant response (ideal)
```

This affects rate controller tuning - gains that work in simulation should
transfer to real hardware since both have similar lag characteristics.

### Flow Deck Height Limits

The VL53L1x ToF sensor has limited range. Outside this range, position and
velocity become invalid (simulating real hardware behavior):

| Mode | Min | Max | Build Option |
|------|-----|-----|--------------|
| Short distance (default) | 0.05m | 1.3m | Default |
| Long distance | 0.05m | 4.0m | `make FLOW_MAX_HEIGHT=4.0` |

When outside range, `gps_valid` and `velocity_valid` become false, and the
estimator holds last known position.

## Key Parameters

- Time step: 4 ms (250 Hz)
- Max motor velocity: 100 rad/s
- Base thrust: 0.553
- Motor time constant: 20 ms (default)
- Flow deck range: 0.05 - 1.3 m (default)
