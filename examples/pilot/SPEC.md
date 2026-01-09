# Pilot Example Specification

A quadcopter autopilot example using the actor runtime with Webots simulator.

## Status

**Implemented:**
- Altitude-hold hover with attitude stabilization
- Step 1: Motor actor (mixer, safety, watchdog)
- Step 2: Separate altitude actor (altitude/rate split)
- Step 3: Sensor actor (hardware abstraction)
- Mixer moved to motor_actor (platform-specific code in one place)

## Goals

1. **Showcase the actor runtime** - demonstrate benefits of actor-based design for embedded systems
2. **Improve the runtime** - if problems arise (latency, scheduling), fix the runtime, don't work around it
3. **Beautiful code** - show how an autopilot can be elegantly written using actors
4. **Hover control** - maintain stable altitude and attitude
5. **Clean architecture** - portable control code with platform abstraction
6. **Webots integration** - use `hive_step()` per simulation step

The pilot serves dual purposes: a real-world stress test that exposes runtime weaknesses, and a showcase of clean actor-based embedded design.

## Design Decisions

### Why buses instead of IPC?

- **Buses** provide latest-value semantics - subscribers get current state, not history
- **IPC notify** queues every message - slow consumers would process stale data
- For control loops, you always want the *latest* sensor reading, not a backlog

### Why max_entries=1?

All buses use `max_entries=1` (single entry, latest value only):

```c
hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
cfg.max_entries = 1;  // Latest value only - correct for real-time control
```

- Control loops need current state, not history
- If a subscriber is slow, it should skip stale data, not queue it
- Larger buffers would cause processing of outdated sensor readings
- This matches how real flight controllers handle sensor data

## Non-Goals (Future Work)

- Navigation (waypoint following)
- Position hold (XY GPS-based)
- Full state estimation (Kalman filter)
- Failsafe handling
- Parameter tuning UI
- Multiple vehicle types

---

## Architecture Overview

Four actors connected via buses:

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
│  ┌─────────────────┐                                               │
│  │  SENSOR ACTOR   │                                               │
│  │ (platform_read) │                                               │
│  └────────┬────────┘                                               │
│           │                                                        │
│           ▼                                                        │
│  ┌───────────┐     ┌─────────────────┐     ┌───────────────────┐   │
│  │  IMU Bus  │────►│ ALTITUDE ACTOR  │────►│   Thrust Bus      │   │
│  │           │     │ (altitude PID)  │     │                   │   │
│  │           │     └─────────────────┘     └─────────┬─────────┘   │
│  │           │                                       │             │
│  │           │     ┌─────────────────┐               │             │
│  │           │────►│ ATTITUDE ACTOR  │◄──────────────┘             │
│  │           │     │ (rate PIDs)     │                             │
│  └───────────┘     └────────┬────────┘                             │
│                             │                                      │
│                             ▼                                      │
│                    ┌─────────────────┐     ┌───────────────────┐   │
│                    │   Torque Bus    │────►│   MOTOR ACTOR     │   │
│                    │                 │     │ (mixer+safety)    │   │
│                    └─────────────────┘     └─────────┬─────────┘   │
│                                                      │             │
└──────────────────────────────────────────────────────┼─────────────┘
                                                       │
                                                       ▼
                                              platform_write_motors()
```

---

## Implementation Details

### Multi-File Design

Code is split into focused modules:

| File | Purpose |
|------|---------|
| `pilot.c` | Main loop, platform layer, bus setup |
| `sensor_actor.c/h` | Reads hardware, publishes to IMU bus |
| `altitude_actor.c/h` | Altitude PID → thrust |
| `attitude_actor.c/h` | Rate PIDs → torque commands |
| `motor_actor.c/h` | Mixer + safety: torque → motors → hardware |
| `pid.c/h` | Reusable PID controller |
| `types.h` | Portable data types |
| `config.h` | All tuning parameters and constants |

### Data Flow

```
Webots sensors
       │
       ▼
platform_read_imu() ◄── called by sensor_actor
       │
       ▼
   IMU Bus
       │
       ├──────────────────────┐
       ▼                      ▼
Altitude Actor          Attitude Actor
(altitude PID)            (rate PIDs)
       │                      │
       ▼                      │
  Thrust Bus ─────────────────┘
                              │
                              ▼
                        Torque Bus
                              │
                              ▼
                        Motor Actor
                     (mixer + safety)
                              │
                              ▼
                platform_write_motors()
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

### Mixer (in motor_actor, Crazyflie + Configuration)

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

The main loop is minimal - all logic lives in actors:

```c
while (wb_robot_step(TIME_STEP_MS) != -1) {
    hive_step();
}
```

Each `hive_step()` runs all ready actors once:
1. Sensor actor reads hardware, publishes to IMU bus
2. Altitude actor reads IMU bus, publishes thrust
3. Attitude actor reads IMU + thrust, publishes torque commands
4. Motor actor applies mixer, writes to hardware

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

To port to real hardware, replace the platform functions in `pilot.c`:

```c
// Initialize hardware (sensors, motors)
int platform_init(void);

// Read sensors into portable struct (called by sensor_actor)
void platform_read_imu(imu_data_t *imu);

// Write motor commands (called by motor_actor)
void platform_write_motors(const motor_cmd_t *cmd);
```

Actors receive platform functions via init:
- `sensor_actor_init(bus, platform_read_imu)`
- `motor_actor_init(bus, platform_write_motors)`

### Portable Code (no hardware deps)

- `pid.c/h` - Generic PID controller
- `altitude_actor.c/h` - Uses only bus API
- `attitude_actor.c/h` - Uses only bus API
- `types.h` - Data structures
- `config.h` - Tuning parameters

---

## File Structure

```
examples/pilot/
    pilot.c              # Main loop, platform layer, bus setup
    sensor_actor.c/h     # Hardware sensor reading → IMU bus
    altitude_actor.c/h   # Altitude PID → thrust
    attitude_actor.c/h   # Rate PIDs → torque commands
    motor_actor.c/h      # Mixer + safety: torque → motors → hardware
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
│   RC input          Altitude              │ Rate                   │
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
| **Altitude** | State + Setpoint | Thrust Setpoint | HIGH | Altitude PID (~50Hz) |
| **Attitude** | State + Thrust | Torque Bus | CRITICAL | Rate PIDs (250Hz) |
| **Motor** | Torque Bus | Hardware | CRITICAL | Mixer, safety, limits, write PWM |

### Step 1: Motor Actor ✓

Separate motor output into dedicated actor with mixer and safety features.

```
Attitude Actor ──► Torque Bus ──► Motor Actor ──► Hardware
                                  (mixer+safety)
```

**Features:** Subscribe to torque bus, apply mixer, enforce limits, watchdog, platform write.

### Step 2: Separate Altitude Actor ✓

Split altitude control from rate control.

```
IMU Bus ──► Altitude Actor ──► Thrust Bus ──► Attitude Actor ──► Torque Bus
            (altitude PID)                    (rate PIDs only)
```

**Benefits:** Clear separation, different rates possible, easier tuning.

### Step 3: Sensor Actor ✓

Move sensor reading from main loop into actor.

```
Main Loop: hive_step() only
Sensor Actor: platform_read_imu() ──► IMU Bus
```

**Benefits:** Main loop is now just `hive_step()`, all logic in actors.

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

## Memory Requirements

### Code Size (Flash/ROM)

| Component | Text (code) |
|-----------|-------------|
| Pilot application | ~4 KB |
| Hive runtime | ~27 KB |
| **Total** | **~31 KB** |

Note: STM32 builds will differ slightly. Webots platform layer adds ~1.5 KB.

### RAM (Static Memory)

With default `hive_static_config.h`, the runtime uses ~1.2 MB RAM (mostly the 1 MB stack arena). This example needs far less.

**Minimal configuration for hover example:**

| Resource | Used | Default | Minimal |
|----------|------|---------|---------|
| Actors | 4 | 64 | 4 |
| Buses | 3 | 32 | 4 |
| Stack per actor | 1 KB | 64 KB | 1 KB |
| Stack arena | 4 KB | 1 MB | 4 KB |
| Mailbox entries | ~4 | 256 | 8 |
| Message data | ~4 | 256 | 8 |
| Timer entries | 0 | 64 | 4 |
| Link entries | 0 | 128 | 4 |
| Monitor entries | 0 | 128 | 4 |

**Result: ~8 KB RAM** vs default ~1.2 MB

This fits comfortably on small STM32 chips (e.g., STM32F103 with 20 KB RAM).

---

## Future Extensions

1. **Position hold** - Add XY GPS feedback, position PID
2. **Waypoint navigation** - Mission actor, path planning
3. **Sensor fusion** - Complementary filter for better attitude estimation
4. **Failsafe** - Motor failure detection, emergency landing
5. **Telemetry** - Logging, MAVLink output
6. **RC input** - Manual control override
