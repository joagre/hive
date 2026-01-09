# Pilot Example

Quadcopter hover example using hive actor runtime with Webots simulator.

## What it does

Demonstrates altitude-hold hover control with a Crazyflie quadcopter using 6 actors:

1. **Sensor actor** reads IMU/GPS from Webots, publishes to IMU bus
2. **Estimator actor** sensor fusion, computes vertical velocity, publishes to state bus
3. **Altitude actor** runs altitude PID, publishes thrust commands
4. **Angle actor** runs angle PIDs, publishes rate setpoints
5. **Attitude actor** runs rate PIDs, publishes torque commands
6. **Motor actor** applies mixer, enforces safety limits, writes to hardware

The drone rises to 1.0m altitude and holds position.

## Prerequisites

- Webots installed (https://cyberbotics.com/)
- hive runtime built: `cd ../.. && make`

## Build and Run

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots and start the simulation.

## Files

```
pilot.c              # Main loop, platform layer, bus setup
sensor_actor.c/h     # Hardware sensor reading → IMU bus
estimator_actor.c/h  # Sensor fusion → state bus
altitude_actor.c/h   # Altitude PID → thrust
angle_actor.c/h      # Angle PIDs → rate setpoints
attitude_actor.c/h   # Rate PIDs → torque commands
motor_actor.c/h      # Mixer + safety: torque → motors → hardware
pid.c/h              # Reusable PID controller
types.h              # Portable data types
config.h             # Shared constants (PID gains, timing)
```

## Architecture

Six actors connected via buses:

```
Sensor ──► IMU Bus ──► Estimator ──► State Bus ──► Altitude ──► Thrust Bus ─┐
                                          │                                  │
                                          ├──► Angle ──► Rate Setpoint Bus ──┤
                                          │                                  │
                                          └──► Attitude ◄────────────────────┘
                                                  │
                                                  ▼
                                            Torque Bus ──► Motor ──► Hardware
```

Platform layer (in pilot.c) provides hardware abstraction:
- `platform_read_imu()` - reads Webots sensors
- `platform_write_motors()` - writes Webots motors

To port to real hardware, replace these functions.

## Actor Priorities and Spawn Order

All actors run at CRITICAL priority. Spawn order determines execution order
within the same priority level (round-robin). Actors are spawned in data-flow
order to ensure each actor sees fresh data from upstream actors in the same step:

| Order | Actor     | Priority | Rationale |
|-------|-----------|----------|-----------|
| 1     | sensor    | CRITICAL | Reads hardware first |
| 2     | estimator | CRITICAL | Needs IMU, produces state estimate |
| 3     | altitude  | CRITICAL | Needs state, produces thrust |
| 4     | angle     | CRITICAL | Needs state, produces rate setpoints |
| 5     | attitude  | CRITICAL | Needs state + thrust + rate setpoints |
| 6     | motor     | CRITICAL | Needs torque, writes hardware last |

## Control System

### PID Controllers (tuned in config.h)

| Controller | Kp   | Ki   | Kd    | Purpose |
|------------|------|------|-------|---------|
| Altitude   | 0.3  | 0.05 | 0.15  | Hold 1.0m height |
| Angle      | 4.0  | 0    | 0.1   | Level attitude |
| Roll rate  | 0.02 | 0    | 0.001 | Stabilize roll |
| Pitch rate | 0.02 | 0    | 0.001 | Stabilize pitch |
| Yaw rate   | 0.02 | 0    | 0.001 | Stabilize yaw |

### Motor Mixer (in motor_actor, + Configuration)

```
        Front
          M1
           │
    M4 ────┼──── M2
           │
          M3
         Rear

M1 = thrust - pitch - yaw
M2 = thrust - roll  + yaw
M3 = thrust + pitch - yaw
M4 = thrust + roll  + yaw
```

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
| Motor 1 (front) | `m1_motor` | RotationalMotor |
| Motor 2 (right) | `m2_motor` | RotationalMotor |
| Motor 3 (rear) | `m3_motor` | RotationalMotor |
| Motor 4 (left) | `m4_motor` | RotationalMotor |
| Gyroscope | `gyro` | Gyro |
| Inertial Unit | `inertial_unit` | InertialUnit |
| GPS | `gps` | GPS |
