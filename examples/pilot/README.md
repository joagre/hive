# Pilot Example

Quadcopter waypoint navigation using hive actor runtime.

Supports two platforms:
- **Webots simulation** (default) - Crazyflie quadcopter in Webots simulator
- **STM32 hardware** - STEVAL-DRONE01 mini drone kit (requires hive STM32 port)

## What it does

Demonstrates 3D waypoint navigation with a Crazyflie quadcopter using 8 actors:

1. **Sensor actor** reads IMU/GPS from Webots, publishes to IMU bus
2. **Estimator actor** sensor fusion, computes velocities, publishes to state bus
3. **Altitude actor** reads target altitude from target bus, runs altitude PID
4. **Waypoint actor** manages 3D waypoint list (x, y, z, yaw), publishes to target bus
5. **Position actor** reads target XY/yaw from target bus, runs position PD
6. **Angle actor** runs angle PIDs, publishes rate setpoints
7. **Attitude actor** runs rate PIDs, publishes torque commands
8. **Motor actor** applies X-config mixer, enforces safety limits, writes to hardware

The drone flies a square pattern with altitude changes at each waypoint.

## Prerequisites

**For Webots simulation:**
- Webots installed (https://cyberbotics.com/)
- hive runtime built: `cd ../.. && make`

**For STM32 hardware:**
- ARM GCC: `apt install gcc-arm-none-eabi`
- ST-Link: `apt install stlink-tools`
- hive runtime ported to STM32 (context switch, scheduler, timers)

## Build and Run

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots and start the simulation.

### STM32 Hardware (STEVAL-DRONE01)

```bash
make -f Makefile.STEVAL-DRONE01        # Build firmware (requires hive STM32 port)
make -f Makefile.STEVAL-DRONE01 flash  # Flash to device
```

See `hal/STEVAL-DRONE01/README.md` for hardware details.

## Files

### Application Code

| File | Description |
|------|-------------|
| `pilot.c` | Main entry point, platform layer, bus setup |
| `sensor_actor.c/h` | Hardware sensor reading → IMU bus |
| `estimator_actor.c/h` | Sensor fusion → state bus |
| `altitude_actor.c/h` | Altitude PID → thrust |
| `waypoint_actor.c/h` | Waypoint manager → target bus |
| `position_actor.c/h` | Position PD → angle setpoints |
| `angle_actor.c/h` | Angle PIDs → rate setpoints |
| `attitude_actor.c/h` | Rate PIDs → torque commands |
| `motor_actor.c/h` | Mixer + safety: torque → motors → hardware |
| `pid.c/h` | Reusable PID controller |
| `types.h` | Shared data types (imu_data_t, motor_cmd_t, etc.) |
| `config.h` | Shared constants (PID gains, timing, limits) |

### Build System

| File | Description |
|------|-------------|
| `Makefile` | Webots simulation build |
| `Makefile.STEVAL-DRONE01` | STM32 hardware build (links hive + pilot + HAL) |

### Documentation

| File | Description |
|------|-------------|
| `README.md` | This file |
| `SPEC.md` | Detailed design specification |

### Directories

| Directory | Description |
|-----------|-------------|
| `hal/STEVAL-DRONE01/` | Hardware abstraction layer for STM32 drone |
| `controllers/pilot/` | Webots controller (symlink created by `make install`) |
| `worlds/` | Webots world files (hover_test.wbt) |

## Architecture

Eight actors connected via buses:

```mermaid
graph TB
    Sensor[Sensor] --> IMUBus([IMU Bus]) --> Estimator[Estimator] --> StateBus([State Bus])

    StateBus --> Waypoint[Waypoint] --> TargetBus([Target Bus])
    TargetBus --> Altitude[Altitude] --> ThrustBus([Thrust Bus]) --> Attitude[Attitude]
    TargetBus --> Position[Position]
    StateBus --> Altitude
    StateBus --> Position --> AngleSP([Angle SP Bus]) --> Angle[Angle]
    StateBus --> Angle --> RateSP([Rate SP Bus]) --> Attitude
    Attitude --> TorqueBus([Torque Bus]) --> Motor[Motor]
```

Platform layer (in pilot.c) provides hardware abstraction:
- `platform_read_imu()` - reads sensors (Webots or STM32)
- `platform_write_motors()` - writes motors (Webots or STM32)

pilot.c uses conditional compilation (`#ifdef PLATFORM_STEVAL_DRONE01`) to select
between Webots simulation and STM32 hardware at build time.

## Actor Priorities and Spawn Order

All actors run at CRITICAL priority. Spawn order determines execution order
within the same priority level (round-robin). Actors are spawned in data-flow
order to ensure each actor sees fresh data from upstream actors in the same step:

| Order | Actor     | Priority | Rationale |
|-------|-----------|----------|-----------|
| 1     | sensor    | CRITICAL | Reads hardware first |
| 2     | estimator | CRITICAL | Needs IMU, produces state estimate |
| 3     | altitude  | CRITICAL | Needs state, produces thrust |
| 4     | waypoint  | CRITICAL | Needs state, produces position targets |
| 5     | position  | CRITICAL | Needs target, produces angle setpoints |
| 6     | angle     | CRITICAL | Needs angle setpoints, produces rate setpoints |
| 7     | attitude  | CRITICAL | Needs state + thrust + rate setpoints |
| 8     | motor     | CRITICAL | Needs torque, writes hardware last |

## Control System

### PID Controllers (tuned in config.h)

| Controller | Kp   | Ki   | Kd    | Purpose |
|------------|------|------|-------|---------|
| Altitude   | 0.3  | 0.05 | 0     | Hold 1.0m height (PI + velocity damping) |
| Position   | 0.2  | -    | 0.1   | Hold XY position (PD, max tilt 0.35 rad) |
| Angle      | 4.0  | 0    | 0     | Level attitude (roll/pitch) |
| Yaw angle  | 4.0  | 0    | 0     | Heading hold (uses pid_update_angle for wrap-around) |
| Roll rate  | 0.02 | 0    | 0.001 | Stabilize roll |
| Pitch rate | 0.02 | 0    | 0.001 | Stabilize pitch |
| Yaw rate   | 0.02 | 0    | 0.001 | Stabilize yaw |

Altitude control uses measured vertical velocity for damping (Kv=0.15) instead
of differentiating position error. This provides smoother response with less noise.

Position control uses simple PD with velocity damping. Commands are transformed
from world frame to body frame based on current yaw. Heading hold is achieved
via yaw angle setpoint published to the angle actor, which uses `pid_update_angle()`
to handle the ±π wrap-around correctly.

### Waypoint Navigation

The waypoint actor manages a list of 3D waypoints and publishes the current target
to the target bus. Both altitude and position actors read from the target bus.

**Demo route (square pattern with gentle altitude changes):**
1. (0, 0, 1.0m) heading 0° - start at 1m
2. (1, 0, 1.2m) heading 0° - rise to 1.2m
3. (1, 1, 1.4m) heading 90° - rise to 1.4m, face east
4. (0, 1, 1.2m) heading 180° - drop to 1.2m, face south
5. (0, 0, 1.0m) heading 0° - return to 1m

**Arrival detection:** The drone must satisfy all conditions before advancing:
- XY position within 0.15m of waypoint
- Altitude within 0.15m of target
- Heading within 0.1 rad (~6°) of target
- Velocity below 0.1 m/s (nearly stopped)
- Hover at waypoint for 200ms

After completing the route, the drone loops back to the first waypoint and repeats forever.

### Motor Mixer (Platform-Specific, X Configuration)

Both platforms use X-configuration with the same motor layout, but mixer formulas differ
due to motor rotation directions:

```
        Front
      M2    M3
        \  /
         \/
         /\
        /  \
      M1    M4
        Rear
```

**Crazyflie (Webots simulation):** Matches Bitcraze firmware
```
M1 = thrust - roll + pitch + yaw  (rear-left, CCW)
M2 = thrust - roll - pitch - yaw  (front-left, CW)
M3 = thrust + roll - pitch + yaw  (front-right, CCW)
M4 = thrust + roll + pitch - yaw  (rear-right, CW)
```

**STEVAL-DRONE01 (STM32 hardware):** Different sign conventions
```
M1 = thrust + roll + pitch - yaw  (rear-left, CCW)
M2 = thrust + roll - pitch + yaw  (front-left, CW)
M3 = thrust - roll - pitch - yaw  (front-right, CCW)
M4 = thrust - roll + pitch + yaw  (rear-right, CW)
```

The mixer is selected at compile time via `#ifdef PLATFORM_STEVAL_DRONE01` in motor_actor.c.

## Main Loop

The main loop is minimal - all logic is in actors:

```c
while (wb_robot_step(TIME_STEP_MS) != -1) {
    hive_step();
}
```

Webots controls time via `wb_robot_step()`. Each call:
1. Blocks until Webots simulates TIME_STEP milliseconds
2. Returns, allowing `hive_step()` to run all actors once
3. Actors read sensors, compute, publish results
4. Loop repeats

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
