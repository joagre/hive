# Pilot Example Specification

A quadcopter autopilot example using the actor runtime with Webots simulator.

## Status

**Implemented:** Altitude-hold hover with attitude stabilization.

## Goals

1. **Demonstrate actor runtime** in a realistic embedded application
2. **Hover control** - maintain stable altitude and attitude
3. **Clean architecture** - portable control code with platform abstraction
4. **Webots integration** - Use `hive_step()` per simulation step

## Non-Goals (Future Work)

- Navigation (waypoint following)
- Position hold (XY GPS-based)
- Full state estimation (Kalman filter)
- Failsafe handling
- Parameter tuning UI
- Multiple vehicle types

---

## Architecture Overview

The implementation uses a single attitude actor with platform abstraction:

```
┌─────────────────────────────────────────────────────────────────┐
│                        WEBOTS SIMULATION                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │   IMU    │  │   GPS    │  │  Gyro    │  │  Motors (x4)     │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────▲─────────┘ │
│       │             │             │                  │           │
└───────┼─────────────┼─────────────┼──────────────────┼───────────┘
        │             │             │                  │
        ▼             ▼             ▼                  │
┌───────────────────────────────────────────────────────────────────┐
│                    WEBOTS PLATFORM LAYER                          │
│                                                                   │
│   platform_init()      - Initialize devices                       │
│   platform_read_imu()  - Read gyro, IMU, GPS → imu_data_t         │
│   platform_write_motors() - Write motor_cmd_t → Webots motors     │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
        │                                              ▲
        ▼                                              │
┌───────────────────────────────────────────────────────────────────┐
│                      MAIN INTEGRATION LOOP                        │
│                                                                   │
│   while (wb_robot_step(TIME_STEP) != -1) {                        │
│       platform_read_imu(&imu);                                    │
│       hive_bus_publish(g_imu_bus, &imu, sizeof(imu));             │
│       hive_step();                                                │
│       platform_write_motors();                                    │
│   }                                                               │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
        │                                              ▲
        ▼                                              │
┌───────────────────────────────────────────────────────────────────┐
│                        ACTOR RUNTIME                              │
│                                                                   │
│  ┌─────────────┐                           ┌────────────────────┐ │
│  │   IMU Bus   │ ─────────────────────────►│  ATTITUDE ACTOR    │ │
│  │  (sensor    │                           │                    │ │
│  │   data)     │                           │  PID Controllers:  │ │
│  └─────────────┘                           │  - Roll rate       │ │
│                                            │  - Pitch rate      │ │
│                                            │  - Yaw rate        │ │
│  ┌─────────────┐                           │  - Altitude        │ │
│  │ Motor Output│◄──────────────────────────│                    │ │
│  │  (global)   │                           │  Mixer → motors    │ │
│  └─────────────┘                           └────────────────────┘ │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

---

## Implementation Details

### Single File Design

All code is in `pilot.c` (~260 lines), organized into sections:

1. **Portable types** - `imu_data_t`, `motor_cmd_t`, `pid_state_t`
2. **Portable control code** - `pid_init()`, `pid_update()`, `mixer_update()`
3. **Platform abstraction** - `motor_output_set()` (global variable interface)
4. **Attitude actor** - Subscribes to IMU bus, runs PIDs, outputs motor commands
5. **Webots platform layer** - Device init, sensor reading, motor writing
6. **Main loop** - Webots integration with `hive_step()`

### Data Flow

```
Webots sensors → platform_read_imu() → imu_data_t → bus publish
                                                         │
                                                         ▼
                                              attitude_actor reads bus
                                                         │
                                                         ▼
                                              PID controllers compute
                                                         │
                                                         ▼
                                              mixer_update() → motor_cmd_t
                                                         │
                                                         ▼
                                              motor_output_set() → global
                                                         │
                                                         ▼
                              platform_write_motors() ← reads global
                                                         │
                                                         ▼
                                              wb_motor_set_velocity()
```

---

## Control Algorithms

### PID Controller

Standard discrete PID with anti-windup:

```c
float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    float p = pid->kp * error;

    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, -integral_max, integral_max);
    float i = pid->ki * pid->integral;

    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return clamp(p + i + d, -output_max, output_max);
}
```

### Tuned PID Gains

| Controller | Kp   | Ki   | Kd    | Output Max | Purpose |
|------------|------|------|-------|------------|---------|
| Roll rate  | 0.02 | 0    | 0.001 | 0.1        | Stabilize roll |
| Pitch rate | 0.02 | 0    | 0.001 | 0.1        | Stabilize pitch |
| Yaw rate   | 0.02 | 0    | 0.001 | 0.15       | Stabilize yaw |
| Altitude   | 0.3  | 0.05 | 0.15  | 0.15       | Hold 1.0m height |

Base thrust: 0.553 (approximate hover thrust for Webots Crazyflie model)

### Mixer (Crazyflie + Configuration)

```
        Front
          M1
           │
    M4 ────┼──── M2
           │
          M3
         Rear

M1 (front): thrust - pitch_torque - yaw_torque
M2 (right): thrust - roll_torque  + yaw_torque
M3 (rear):  thrust + pitch_torque - yaw_torque
M4 (left):  thrust + roll_torque  + yaw_torque
```

### Motor Velocity Signs

The Webots Crazyflie model requires specific velocity signs to cancel reaction torque:

- **M1, M3** (front, rear): Negative velocity
- **M2, M4** (right, left): Positive velocity

```c
static const float signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
```

---

## Webots Integration

### Simulation Loop

```c
while (wb_robot_step(TIME_STEP) != -1) {
    // 1. Read sensors from Webots
    platform_read_imu(&imu);

    // 2. Publish to bus for actors
    hive_bus_publish(g_imu_bus, &imu, sizeof(imu));

    // 3. Run actors (attitude actor computes motor commands)
    hive_step();

    // 4. Apply motor commands to Webots
    platform_write_motors();
}
```

### Key Parameters

- `TIME_STEP = 4` ms (250 Hz control rate)
- `MOTOR_MAX_VELOCITY = 100.0` rad/s
- Target altitude: 1.0 m

### Webots Device Names

| Device | Name | Type |
|--------|------|------|
| Motor 1 (front) | `m1_motor` | RotationalMotor |
| Motor 2 (right) | `m2_motor` | RotationalMotor |
| Motor 3 (rear) | `m3_motor` | RotationalMotor |
| Motor 4 (left) | `m4_motor` | RotationalMotor |
| Gyroscope | `gyro` | Gyro |
| Inertial Unit | `inertial_unit` | InertialUnit |
| GPS | `gps` | GPS |

---

## Portability

### Platform Abstraction

To port to real hardware, replace three functions:

```c
// Initialize hardware
int platform_init(void);

// Read sensors into portable struct
void platform_read_imu(imu_data_t *imu);

// Write motor commands from global
void platform_write_motors(void);
```

### Portable Code

The following code is hardware-independent:

- `pid_init()`, `pid_update()` - Generic PID controller
- `mixer_update()` - Crazyflie-specific but no hardware deps
- `attitude_actor()` - Uses only bus API and portable types

---

## File Structure

```
examples/pilot/
    SPEC.md              # This specification
    README.md            # Usage instructions
    pilot.c              # Complete hover controller
    Makefile             # Build rules
    worlds/
        hover_test.wbt   # Webots world file
    controllers/
        pilot/           # Webots controller directory (installed here)
```

---

## Testing Results

### Hover Behavior

1. Drone starts at 0.5m altitude (world file setting)
2. Altitude PID commands increased thrust
3. Drone rises with some initial oscillation
4. Settles at 1.0m within ~3 seconds
5. Maintains stable hover at 1.0m ± 0.05m

### Console Output

```
Pilot started - hover mode
alt=0.01 thrust=0.75 | roll=  0.0 pitch=  0.0
alt=0.02 thrust=0.75 | roll=  0.0 pitch=  0.0
alt=1.41 thrust=0.35 | roll=  0.0 pitch=  0.0
...
alt=1.05 thrust=0.55 | roll=  0.0 pitch=  0.0
alt=1.04 thrust=0.55 | roll=  0.0 pitch=  0.0
alt=1.01 thrust=0.55 | roll=  0.0 pitch=  0.0
```

---

## Future Extensions

1. **Position hold** - Add XY GPS feedback, position PID
2. **Waypoint navigation** - Mission actor, path planning
3. **Sensor fusion** - Complementary filter for better attitude estimation
4. **Failsafe** - Motor failure detection, emergency landing
5. **Telemetry** - Logging, MAVLink output
6. **RC input** - Manual control override
7. **Multiple actors** - Separate altitude and attitude actors
