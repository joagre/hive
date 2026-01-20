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

**Sensor noise:** Realistic MEMS noise is simulated by default (0.02 m/s² accel,
0.001 rad/s gyro). Disable with `-DSENSOR_NOISE=0` for clean testing.

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

## Key Parameters

- Time step: 4 ms (250 Hz)
- Max motor velocity: 100 rad/s
- Base thrust: 0.553
