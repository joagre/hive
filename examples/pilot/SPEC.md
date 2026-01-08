# Pilot Example Specification

A quadcopter autopilot example using the actor runtime with Webots simulator.

## Status

**Implemented:**
- Altitude-hold hover with attitude stabilization
- Step 1: Motor actor (safety, watchdog)
- Step 2: Separate altitude actor (outer/inner loop split)

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

Three actors connected via buses:

```
┌─────────────────────────────────────────────────────────────────────┐
│                         WEBOTS SIMULATION                           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌───────────────────┐   │
│  │   IMU    │  │   GPS    │  │  Gyro    │  │   Motors (x4)     │   │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └─────────▲─────────┘   │
└───────┼─────────────┼─────────────┼──────────────────┼──────────────┘
        │             │             │                  │
        ▼             ▼             ▼                  │
┌─────────────────────────────────────────────────────────────────────┐
│                     PLATFORM LAYER (pilot.c)                        │
│   platform_read_imu() ──► imu_data_t                                │
│   platform_write_motors() ◄── motor_cmd_t                           │
└─────────────────────────────────────────────────────────────────────┘
        │                                              ▲
        ▼                                              │
┌─────────────────────────────────────────────────────────────────────┐
│                          ACTOR RUNTIME                              │
│                                                                     │
│  ┌───────────┐     ┌─────────────────┐     ┌───────────────────┐   │
│  │  IMU Bus  │────►│ ALTITUDE ACTOR  │────►│   Thrust Bus      │   │
│  │           │     │ (altitude PID)  │     │                   │   │
│  │           │     └─────────────────┘     └─────────┬─────────┘   │
│  │           │                                       │             │
│  │           │     ┌─────────────────┐               │             │
│  │           │────►│ ATTITUDE ACTOR  │◄──────────────┘             │
│  │           │     │ (rate PIDs)     │                             │
│  └───────────┘     │ (mixer)         │                             │
│                    └────────┬────────┘                             │
│                             │                                      │
│                             ▼                                      │
│                    ┌─────────────────┐     ┌───────────────────┐   │
│                    │   Motor Bus     │────►│   MOTOR ACTOR     │   │
│                    │                 │     │ (safety/watchdog) │   │
│                    └─────────────────┘     └─────────┬─────────┘   │
│                                                      │             │
└──────────────────────────────────────────────────────┼─────────────┘
                                                       │
                                                       ▼
                                              platform_write_motors()
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
    pilot.c              # Main loop, platform layer, bus setup
    altitude_actor.c/h   # Outer loop: altitude PID → thrust
    attitude_actor.c/h   # Inner loop: rate PIDs → motor commands
    motor_actor.c/h      # Safety: watchdog, limits → hardware
    pid.c/h              # Reusable PID controller
    types.h              # Portable data types
    config.h             # Shared constants
    Makefile             # Build with auto-deps
    SPEC.md              # This specification
    README.md            # Usage instructions
    worlds/
        hover_test.wbt   # Webots world file
    controllers/
        pilot/           # Webots controller (installed here)
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

## Architecture Evolution Roadmap

The example will evolve incrementally toward a clean multi-actor design.
Each step maintains a working system while improving separation of concerns.

### Target Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         ACTOR ARCHITECTURE                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────┐      ┌──────────┐      ┌──────────┐                  │
│  │  Sensor  │─────►│ Estimator│─────►│  State   │                  │
│  │  Actor   │      │  Actor   │      │   Bus    │                  │
│  └──────────┘      └──────────┘      └────┬─────┘                  │
│   Read HW           Fuse data             │                        │
│   Publish raw       Publish estimate      │                        │
│                                           ▼                        │
│  ┌──────────┐      ┌──────────┐      ┌──────────┐                  │
│  │ Setpoint │─────►│ Altitude │─────►│ Attitude │                  │
│  │  Actor   │      │  Actor   │      │  Actor   │                  │
│  └──────────┘      └──────────┘      └────┬─────┘                  │
│   RC input          Outer loop            │ Inner loop             │
│   Waypoints         Z control             │ Rate control           │
│   Commands                                ▼                        │
│                                      ┌──────────┐                  │
│                                      │  Motor   │                  │
│                                      │  Actor   │                  │
│                                      └──────────┘                  │
│                                       Safety                       │
│                                       Write HW                     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### Actor Responsibilities (Target)

| Actor | Input | Output | Priority | Responsibility |
|-------|-------|--------|----------|----------------|
| **Sensor** | Hardware | Raw Bus | CRITICAL | Read IMU/GPS, timestamp, publish |
| **Estimator** | Raw Bus | State Bus | CRITICAL | Sensor fusion, state estimate |
| **Setpoint** | RC/Mission | Setpoint Bus | NORMAL | Generate target state |
| **Altitude** | State + Setpoint | Thrust Setpoint | HIGH | Outer loop (~50Hz) |
| **Attitude** | State + Thrust | Motor Bus | CRITICAL | Inner loop (250Hz) |
| **Motor** | Motor Bus | Hardware | CRITICAL | Safety, limits, write PWM |

### Step 1: Motor Actor ✓

Separate motor output into dedicated actor with safety features.

```
Attitude Actor ──► Motor Bus ──► Motor Actor ──► Hardware
```

**Features:** Subscribe to motor bus, enforce limits, watchdog, platform write.

### Step 2: Separate Altitude Actor ✓

Split outer loop (altitude) from inner loop (attitude).

```
IMU Bus ──► Altitude Actor ──► Thrust Bus ──► Attitude Actor ──► Motor Bus
            (altitude PID)                    (rate PIDs only)
```

**Benefits:** Clear separation, different rates possible, easier tuning.

### Step 3: Sensor Actor

Move sensor reading from main loop into actor.

**Before:**
```
Main Loop: read sensors ──► IMU Bus
```

**After:**
```
Main Loop: hive_step() only
Sensor Actor: read sensors ──► IMU Bus
```

**Note:** Requires careful handling of Webots timing integration.

### Step 4: Estimator Actor

Add sensor fusion between raw sensors and controllers.

**Before:**
```
Sensor Actor ──► IMU Bus (raw) ──► Controllers
```

**After:**
```
Sensor Actor ──► Raw Bus ──► Estimator Actor ──► State Bus ──► Controllers
                             (complementary filter)
```

**Benefits:**
- Cleaner sensor data
- Filtered attitude estimate
- Altitude rate estimation

### Step 5: Setpoint Actor

Add command generation actor.

**Before:**
```
Altitude Actor has hardcoded target_altitude = 1.0m
```

**After:**
```
Setpoint Actor ──► Setpoint Bus ──► Altitude Actor
(generates commands)                (tracks setpoint)
```

**Future extensions:**
- RC input handling
- Waypoint mission execution
- Mode switching (hover, land, etc.)

---

## Future Extensions

1. **Position hold** - Add XY GPS feedback, position PID
2. **Waypoint navigation** - Mission actor, path planning
3. **Sensor fusion** - Complementary filter for better attitude estimation
4. **Failsafe** - Motor failure detection, emergency landing
5. **Telemetry** - Logging, MAVLink output
6. **RC input** - Manual control override
