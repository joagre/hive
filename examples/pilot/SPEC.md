# Pilot Example Specification

A quadcopter autopilot example using the actor runtime with Webots simulator.

## Goals

1. **Demonstrate actor runtime** in a realistic embedded application
2. **Hover control** - maintain stable altitude and attitude
3. **Clean architecture** - show how actors decompose a control system
4. **Webots integration** - Option A: `rt_run_until_idle()` per simulation step

## Non-Goals (Future Work)

- Navigation (waypoint following)
- Position hold (GPS-based)
- Full state estimation (Kalman filter)
- Failsafe handling
- Parameter tuning UI
- Multiple vehicle types

---

## Architecture Overview

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
│                      MAIN INTEGRATION LOOP                        │
│                                                                   │
│   1. wb_robot_step(TIME_STEP)                                     │
│   2. Read Webots sensors → publish to buses                       │
│   3. rt_run_until_idle()  → run all ready actors                  │
│   4. Read motor commands from bus → write to Webots               │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
        │                                              ▲
        ▼                                              │
┌───────────────────────────────────────────────────────────────────┐
│                        ACTOR RUNTIME                              │
│                                                                   │
│  ┌─────────────┐      ┌─────────────┐      ┌─────────────┐       │
│  │   IMU Bus   │      │ Setpoint Bus│      │  Motor Bus  │       │
│  │  (sensor    │      │ (attitude   │      │ (motor      │       │
│  │   data)     │      │  targets)   │      │  commands)  │       │
│  └──────┬──────┘      └──────┬──────┘      └──────▲──────┘       │
│         │                    │                    │               │
│         ▼                    ▼                    │               │
│  ┌────────────────────────────────────────────────┴─────┐        │
│  │                  ATTITUDE ACTOR                       │        │
│  │                                                       │        │
│  │  - Subscribes: IMU bus, setpoint bus                  │        │
│  │  - Runs PID controllers (roll, pitch, yaw, thrust)    │        │
│  │  - Publishes: motor commands to motor bus             │        │
│  └───────────────────────────────────────────────────────┘        │
│                                                                   │
│  ┌───────────────────────────────────────────────────────┐        │
│  │                  SETPOINT ACTOR                        │        │
│  │                                                        │        │
│  │  - Generates hover setpoint (roll=0, pitch=0, yaw=0)   │        │
│  │  - Publishes: setpoint bus                             │        │
│  │  - Future: receives navigation commands                │        │
│  └────────────────────────────────────────────────────────┘        │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

---

## Webots Integration Model

### Simulation Loop (Option A)

The main loop is controlled by Webots, not the actor runtime:

```
initialize:
    wb_robot_init()
    rt_init()
    create buses (imu_bus, setpoint_bus, motor_bus)
    spawn actors (attitude_actor, setpoint_actor)

loop:
    while wb_robot_step(TIME_STEP) != -1:

        1. READ SENSORS
           - Read IMU from Webots (accelerometer, gyro)
           - Read current motor speeds (for feedback, optional)
           - Publish sensor data to imu_bus

        2. RUN ACTORS
           - Call rt_run_until_idle()
           - All ready actors execute until blocked/idle
           - Attitude actor reads IMU, computes PIDs, publishes motor commands

        3. WRITE ACTUATORS
           - Read motor commands from motor_bus
           - Write to Webots motor devices

cleanup:
    rt_cleanup()
    wb_robot_cleanup()
```

### Why This Model

- **Webots controls time**: Simulation time advances only on `wb_robot_step()`
- **No real-time timers**: Actor timers would fire based on wall-clock, not simulation time
- **Deterministic**: Same inputs produce same outputs (no timing races)
- **Simple**: No threading, no synchronization

### TIME_STEP Selection

- Webots `TIME_STEP` is in milliseconds
- Typical values: 1, 2, 4, 8 ms
- For hover control: 4-8 ms is sufficient (125-250 Hz effective rate)
- Recommendation: **TIME_STEP = 4** (250 Hz)

---

## Actor Specifications

### Attitude Actor

**Purpose:** Stabilize the quadcopter attitude using PID control.

**Priority:** RT_PRIO_CRITICAL (must run every step)

**Subscriptions:**
- `imu_bus` - receives IMU sensor data
- `setpoint_bus` - receives desired attitude

**Publications:**
- `motor_bus` - publishes motor commands

**Behavior:**
1. Read latest IMU data from imu_bus (non-blocking)
2. Read latest setpoint from setpoint_bus (non-blocking)
3. Compute PID outputs for roll, pitch, yaw
4. Compute thrust (constant for hover, or altitude PID)
5. Publish motor commands to motor_bus
6. Yield (return control to scheduler)

**Control Loops:**
- Roll rate PID: gyro_x → roll_torque
- Pitch rate PID: gyro_y → pitch_torque
- Yaw rate PID: gyro_z → yaw_torque
- Thrust: constant value for hover (future: altitude PID)

**State:**
- PID integrators (roll_i, pitch_i, yaw_i)
- Previous errors (for derivative term)
- Last update timestamp (for dt calculation, if needed)

---

### Setpoint Actor

**Purpose:** Generate attitude setpoints for the attitude controller.

**Priority:** RT_PRIO_NORMAL

**Subscriptions:** None (for hover)

**Publications:**
- `setpoint_bus` - publishes desired attitude

**Behavior (Hover Mode):**
1. Publish constant setpoint: roll=0, pitch=0, yaw=0, thrust=hover_thrust
2. Yield

**Future Extensions:**
- Subscribe to navigation actor for position-based setpoints
- Subscribe to RC input for manual control
- Mode switching (hover, position hold, waypoint)

---

## Bus Specifications

### IMU Bus

**Purpose:** Distribute IMU sensor data from Webots to actors.

**Configuration:**
- consume_after_reads: 0 (persist until overwritten)
- max_age_ms: 0 (no expiry)
- max_entries: 1 (only latest value matters)

**Message Format:**
```
imu_data:
    float accel_x, accel_y, accel_z    // m/s^2, body frame
    float gyro_x, gyro_y, gyro_z       // rad/s, body frame
    float roll, pitch, yaw             // radians (from Webots IMU)
    uint64_t timestamp                 // simulation time in microseconds
```

**Publisher:** Main loop (after reading Webots sensors)

**Subscribers:** Attitude actor

---

### Setpoint Bus

**Purpose:** Distribute attitude setpoints to controllers.

**Configuration:**
- consume_after_reads: 0
- max_age_ms: 0
- max_entries: 1

**Message Format:**
```
setpoint_data:
    float roll                         // radians, desired roll angle
    float pitch                        // radians, desired pitch angle
    float yaw                          // radians, desired yaw angle (or rate)
    float thrust                       // 0.0-1.0, normalized thrust
```

**Publisher:** Setpoint actor

**Subscribers:** Attitude actor

---

### Motor Bus

**Purpose:** Distribute motor commands from controller to actuators.

**Configuration:**
- consume_after_reads: 0
- max_age_ms: 0
- max_entries: 1

**Message Format:**
```
motor_data:
    float motor[4]                     // 0.0-1.0, normalized motor speeds
                                       // [0]=front-left, [1]=front-right
                                       // [2]=rear-left,  [3]=rear-right
```

**Publisher:** Attitude actor (via mixer logic)

**Subscribers:** Main loop (reads and writes to Webots)

---

## Control Algorithms

### PID Controller

Standard discrete PID with anti-windup:

```
pid_update(pid, setpoint, measurement, dt):
    error = setpoint - measurement

    // Proportional
    p_term = Kp * error

    // Integral with anti-windup
    pid.integral += error * dt
    pid.integral = clamp(pid.integral, -integral_max, integral_max)
    i_term = Ki * pid.integral

    // Derivative (on measurement to avoid derivative kick)
    derivative = (measurement - pid.prev_measurement) / dt
    d_term = -Kd * derivative

    pid.prev_measurement = measurement

    output = p_term + i_term + d_term
    return clamp(output, output_min, output_max)
```

### Rate Controller (Inner Loop)

For hover, we use rate control (gyro feedback):

```
roll_torque  = pid_update(roll_pid,  0.0, gyro_x, dt)
pitch_torque = pid_update(pitch_pid, 0.0, gyro_y, dt)
yaw_torque   = pid_update(yaw_pid,   0.0, gyro_z, dt)
```

**Why rate control for hover:**
- Simpler than angle control
- Webots IMU provides attitude, but rate control is more stable
- Angle control can be added as outer loop later

### Mixer (Crazyflie + Configuration)

Convert torques to motor commands:

```
        Front
          M1 (CW)
           |
   M4 -----+------ M2
  (CCW)    |      (CCW)
          M3 (CW)
         Rear

M1 (front): thrust - pitch_torque - yaw_torque
M2 (right): thrust - roll_torque  + yaw_torque
M3 (rear):  thrust + pitch_torque - yaw_torque
M4 (left):  thrust + roll_torque  + yaw_torque
```

**Sign convention:**
- Positive roll → right side down → M4 speeds up, M2 slows down
- Positive pitch → nose down → M3 speeds up, M1 slows down
- Positive yaw → CCW rotation → CW motors (M1,M3) speed up

**Note:** Signs may need verification against actual Webots Crazyflie behavior.

---

## Timing Model

### No Actor Timers

For this example, we do NOT use `rt_timer_after()` or `rt_timer_every()`:

- Webots controls simulation time
- Real-time timers would be meaningless
- Loop rate is determined by `wb_robot_step(TIME_STEP)`

### Implicit Rate

- All actors run once per Webots step
- Effective rate = 1000 / TIME_STEP Hz
- Example: TIME_STEP=4 → 250 Hz

### Delta Time (dt)

For PID calculations:
- Option 1: `dt = TIME_STEP / 1000.0` (constant, simple)
- Option 2: Track simulation time from Webots `wb_robot_get_time()`

Recommendation: Use constant dt for simplicity.

---

## Webots Model: Crazyflie

We use the **Crazyflie** quadcopter model included with Webots.

### Why Crazyflie

- Small and simple (~27g real weight)
- Well-documented in Webots
- Fast dynamics (good for testing control loops)
- Open-source hardware/software (real Crazyflie exists)

### Crazyflie Specifications

**Physical:**
- Mass: ~27g (Webots model may differ slightly)
- Size: 92mm motor-to-motor diagonal
- 4 brushed coreless motors

**Motor Layout (top view, + configuration):**
```
        Front
          M1 (CW)
           |
   M4 -----+------ M2
  (CCW)    |      (CCW)
          M3 (CW)
         Rear
```

**Note:** Crazyflie uses + configuration, not X. Mixer formulas differ.

### Webots Device Names

| Device | Webots Name | Type | Notes |
|--------|-------------|------|-------|
| Motor 1 (front) | `m1_motor` | RotationalMotor | CW rotation |
| Motor 2 (right) | `m2_motor` | RotationalMotor | CCW rotation |
| Motor 3 (rear) | `m3_motor` | RotationalMotor | CW rotation |
| Motor 4 (left) | `m4_motor` | RotationalMotor | CCW rotation |
| Gyroscope | `gyro` | Gyro | Angular rates (rad/s) |
| Accelerometer | `accelerometer` | Accelerometer | Linear accel (m/s^2) |
| Inertial Unit | `inertial unit` | InertialUnit | Roll, pitch, yaw (rad) |
| GPS | `gps` | GPS | Position (m), optional |

### Coordinate System

Webots Crazyflie uses:
- **X:** Forward (nose direction)
- **Y:** Left
- **Z:** Up
- **Roll:** Rotation around X (positive = right side down)
- **Pitch:** Rotation around Y (positive = nose down)
- **Yaw:** Rotation around Z (positive = counterclockwise from above)

### Mixer (+ Configuration)

For + configuration, the mixer is different from X:

```
M1 (front): thrust - pitch_torque - yaw_torque
M2 (right): thrust - roll_torque  + yaw_torque
M3 (rear):  thrust + pitch_torque - yaw_torque
M4 (left):  thrust + roll_torque  + yaw_torque
```

**Sign convention:**
- Positive roll torque → roll right → M4 up, M2 down
- Positive pitch torque → pitch forward (nose down) → M3 up, M1 down
- Positive yaw torque → yaw CCW → M1,M3 up (CW motors), M2,M4 down

### Motor Velocity Range

- Webots Crazyflie motors use velocity control
- Typical range: 0 to ~600 rad/s
- We normalize to 0.0-1.0 and scale to actual range

### Initial PID Gains (Crazyflie-tuned)

Starting values based on Crazyflie dynamics:

| Controller | Kp | Ki | Kd | Notes |
|------------|-----|-----|-----|-------|
| Roll rate  | 0.4 | 0.05 | 0.005 | Crazyflie is agile |
| Pitch rate | 0.4 | 0.05 | 0.005 | Same as roll |
| Yaw rate   | 0.2 | 0.02 | 0.0 | Less aggressive |

**Hover thrust:** ~0.5 - 0.6 (depends on Webots gravity setting)

### Webots World Setup

The world file should include:
- Crazyflie robot with controller set to our pilot
- Ground plane (floor)
- Appropriate gravity (default -9.81 m/s^2)
- Reasonable simulation step (basicTimeStep in WorldInfo)

**WorldInfo settings:**
```
WorldInfo {
  basicTimeStep 4      # 4ms = 250 Hz
  gravity 0 0 -9.81
}
```

---

## File Structure

```
examples/pilot/
    SPEC.md              # This specification
    pilot.c              # Main integration loop
    attitude_actor.c     # Attitude controller
    attitude_actor.h
    setpoint_actor.c     # Setpoint generator
    setpoint_actor.h
    pid.c                # PID controller implementation
    pid.h
    types.h              # Shared data types (imu_data, motor_data, etc.)
    Makefile             # Build rules
    worlds/              # Webots world files
        hover_test.wbt
    controllers/         # Webots controller directory (symlink or copy)
```

---

## Runtime API Extension

### rt_run_until_idle()

New API needed for simulation integration:

**Signature:**
```
rt_status rt_run_until_idle(void);
```

**Behavior:**
1. Poll for I/O events (non-blocking, timeout=0)
2. Run all runnable actors until none are runnable
3. Return immediately (do not block on epoll)

**Returns:**
- `RT_SUCCESS` - at least one actor executed
- `RT_ERR_WOULDBLOCK` - no actors were runnable (idle)

**Does NOT:**
- Block waiting for I/O events
- Advance simulation time
- Call epoll_wait with timeout > 0

---

## Testing Plan

### Test 1: Stable Hover

**Setup:** Spawn drone at 1m altitude, motors off

**Expected:**
- Drone falls briefly
- Controllers engage, motors spin up
- Drone stabilizes and hovers
- Oscillations dampen within 2-3 seconds

**Pass Criteria:**
- Altitude stable within +/- 0.1m after 5 seconds
- Roll/pitch within +/- 5 degrees
- No crashes

### Test 2: Disturbance Rejection

**Setup:** Hover, then apply impulse force

**Expected:**
- Drone deflects
- Controllers correct
- Returns to hover

### Test 3: Motor Failure (Future)

**Setup:** Hover, disable one motor

**Expected:** Graceful degradation or controlled descent

---

## Future Extensions

1. **Altitude hold:** Add barometer, altitude PID
2. **Position hold:** Add GPS, position PID, navigator actor
3. **Waypoint navigation:** Mission actor, path planning
4. **Sensor fusion:** AHRS actor with complementary filter
5. **Failsafe:** Watchdog actor, emergency landing
6. **Telemetry:** Logging actor, MAVLink output
7. **RC input:** Manual control override

---

## Open Questions

1. ~~**Which Webots drone model?**~~ **RESOLVED:** Crazyflie (+ configuration)

2. **Mixer in attitude actor or separate?** Keeping it in attitude actor is simpler for hover.

3. **Angle vs rate control?** Starting with rate control (simpler). Add angle outer loop if needed.

4. **How to handle motor saturation?** Clamp individual motors? Priority-based mixing?

5. **Simulation time access:** Should actors have access to simulation time? (via bus message timestamp)
