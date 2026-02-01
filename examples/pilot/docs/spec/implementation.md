# Implementation

Implementation details, control algorithms, portability, and reference information.

## Table of Contents

- [Implementation Details](#implementation-details)
- [Control Algorithms](#control-algorithms)
- [Main Loop](#main-loop)
- [Portability](#portability)
- [File Structure](#file-structure)
- [Testing Results](#testing-results)
- [Memory Requirements](#memory-requirements)

---

## Implementation Details

### Multi-File Design

Code is split into focused modules:

| File | Purpose |
|------|---------|
| `pilot.c` | Main loop, bus setup, actor spawn |
| `sensor_actor.c/h` | Reads sensors via HAL, publishes to sensor bus |
| `estimator_actor.c/h` | Sensor fusion -> state bus |
| `altitude_actor.c/h` | Altitude PID -> thrust |
| `waypoint_actor.c/h` | Waypoint navigation -> position target bus |
| `position_actor.c/h` | Position PD -> attitude setpoints |
| `attitude_actor.c/h` | Attitude PIDs -> rate setpoints |
| `rate_actor.c/h` | Rate PIDs -> torque commands |
| `motor_actor.c/h` | Output: torque -> HAL -> motors |
| `flight_manager_actor.c/h` | Startup delay, flight window cutoff |
| `comms_actor.c/h` | Radio telemetry (Crazyflie only) |
| `telemetry_logger_actor.c/h` | CSV logging for PID tuning (to /sd or /tmp) |
| `pid.c/h` | Reusable PID controller |
| `types.h` | Portable data types |
| `config.h` | Configuration constants (timing, thresholds) |
| `math_utils.h` | Math utilities (CLAMPF, LPF macros, normalize_angle) |
| `notifications.h` | IPC notification tags (NOTIFY_FLIGHT_START, etc.) |
| `flight_profiles.h` | Waypoint definitions per flight profile |
| `tools/*.py` | PID tuning and telemetry analysis tools |

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

PID gains are platform-specific, defined in each `hal/<platform>/hal_config.h`.
The control cascade uses:

- **Altitude** - PI + velocity damping (Kv) for smooth response
- **Position** - PD with velocity damping, max tilt limited
- **Attitude** - P controller for roll/pitch/yaw angles
- **Rate** - PD controller for angular rates

### Mixer

Each HAL implements its own X-configuration mixer in `hal_write_torque()`.
See `hal/<platform>/README.md` for motor layout and mixing equations.

---

## Main Loop

The main loop is minimal - all logic lives in actors:

```c
// Simulation (Webots)
while (hal_step()) {
    hive_advance_time(HAL_TIME_STEP_US);
    hive_run_until_blocked();
}

// Real-time (STM32)
hive_run();
```

Control rate is 250 Hz (4ms time step). Each step triggers the control chain:
sensor -> estimator -> altitude/waypoint/position -> attitude -> rate -> motor.

---

## Portability

### Hardware Abstraction Layer (HAL)

All hardware access goes through `hal/hal.h`. Each platform provides its own
implementation of this interface:

```c
// Platform lifecycle
int hal_init(void);        // Initialize hardware
void hal_cleanup(void);    // Release resources
bool hal_self_test(void);  // Verify sensors respond (returns true if OK)
void hal_calibrate(void);  // Calibrate sensors
void hal_arm(void);        // Enable motor output
void hal_disarm(void);     // Disable motor output

// Sensor interface (called by sensor_actor)
void hal_read_sensors(sensor_data_t *sensors);

// Motor interface (called by motor_actor)
void hal_write_torque(const torque_cmd_t *cmd);

// ESB radio interface (called by comms_actor, HAL_HAS_RADIO only)
int hal_esb_init(void);
int hal_esb_send(const void *data, size_t len);
bool hal_esb_tx_ready(void);
void hal_esb_poll(void);

// Power interface (HAL_HAS_RADIO only)
float hal_power_get_battery(void);

// Simulation time (only for SIMULATED_TIME builds)
bool hal_step(void);  // Advance simulation, returns false when done
```

Actors use the HAL directly - no function pointers needed:
- `sensor_actor.c` calls `hal_read_sensors()`
- `motor_actor.c` calls `hal_write_torque()`
- `comms_actor.c` calls `hal_esb_*()` (Crazyflie only)

### Startup Sequence

The HAL startup sequence in `pilot.c` is:

```c
hal_init();       // Initialize hardware
hal_self_test();  // Verify sensors respond
hal_calibrate();  // Calibrate sensors (keep drone still and level)
hal_arm();        // Enable motor output
```

If any step fails, the program aborts before starting the actor runtime.

### LED Feedback (Hardware Platforms)

Real hardware platforms (Crazyflie) provide LED feedback during startup:

| Pattern | Stage | Meaning |
|---------|-------|---------|
| 1-3 slow blinks | init | Progress indicator (1=start, 2=IMU OK, 3=all OK) |
| 3-5 fast blinks | init | Hardware init failed (count indicates component) |
| 6-9 fast blinks | self-test | Sensor self-test failed (count indicates sensor) |
| 10 fast blinks | calibrate | Level warning (drone tilted >6 deg) |
| Slow blink | calibrate | Calibration in progress |
| LED off | calibrate | Calibration complete |
| LED on | armed | Ready to fly |
| Slow continuous blink | any | Fatal error (stuck in loop) |

See `hal/<platform>/README.md` for detailed LED patterns per platform.

### Supported Platforms

| Platform | Build | Details |
|----------|-------|---------|
| Webots simulation | `make` | Uses `-DSIMULATED_TIME` |
| Crazyflie 2.1+ | `make -f Makefile.crazyflie-2.1plus` | See `hal/crazyflie-2.1plus/README.md` |

All hardware differences are encapsulated in the HAL. Actor code is identical
across platforms. The only compile-time difference is `SIMULATED_TIME` which
controls the main loop (simulation vs real-time).

### Portable Code

All actor code is platform-independent. Actors use:
- Bus API for inter-actor communication
- HAL API for hardware access (abstracted)

| File | Dependencies |
|------|--------------|
| `sensor_actor.c/h` | HAL (hal_read_sensors) + bus API |
| `estimator_actor.c/h` | Bus API only |
| `altitude_actor.c/h` | Bus API only |
| `waypoint_actor.c/h` | IPC + bus API |
| `position_actor.c/h` | Bus API only |
| `attitude_actor.c/h` | Bus API only |
| `rate_actor.c/h` | Bus API only |
| `motor_actor.c/h` | HAL (hal_write_torque) + IPC + bus API |
| `flight_manager_actor.c/h` | IPC only (no bus) |
| `comms_actor.c/h` | HAL (hal_esb_*, hal_power_*) + bus API (Crazyflie only) |
| `pid.c/h` | Pure C, no runtime deps |
| `types.h` | Data structures |
| `config.h` | Tuning parameters |

---

## File Structure

```
examples/pilot/
    pilot.c              # Main loop, bus setup, supervisor config
    sensor_actor.c       # Reads sensors via HAL -> sensor bus
    estimator_actor.c    # Sensor fusion -> state bus
    altitude_actor.c     # Altitude PID -> thrust
    waypoint_actor.c     # Waypoint navigation -> position target bus
    position_actor.c     # Position PD -> attitude setpoints
    attitude_actor.c     # Attitude PIDs -> rate setpoints
    rate_actor.c         # Rate PIDs -> torque commands
    motor_actor.c        # Output: torque -> HAL -> motors
    flight_manager_actor.c # Startup delay, flight window cutoff
    comms_actor.c        # Radio telemetry (Crazyflie only)
    telemetry_logger_actor.c # CSV logging (to /sd or /tmp)
    pid.c                # Reusable PID controller
    stack_profile.c      # Stack usage profiling
    include/             # Header files
        *_actor.h        # Actor interfaces
        types.h          # Portable data types
        config.h         # Configuration constants
        math_utils.h     # Math macros
        notifications.h  # IPC notification tags
        flight_profiles.h # Waypoint definitions
        pid.h            # PID controller interface
        pilot_buses.h    # Bus handle struct
    fusion/
        complementary_filter.c/h  # Portable attitude estimation
    tools/
        analyze_pid.py       # PID metrics analysis (overshoot, settling time)
        plot_telemetry.py    # 6-panel telemetry visualization
        plot_flight.py       # Full flight summary with 3D trajectory
        ground_station.py    # Radio telemetry receiver (Crazyflie)
        README.md            # Tools documentation
    Makefile                 # Webots simulation build
    Makefile.crazyflie-2.1plus  # Crazyflie 2.1+ build
    hive_config.mk           # Shared Hive memory config
    README.md            # Usage instructions
    spec/                # This specification
        README.md        # Specification overview
        design.md        # Design decisions
        implementation.md # This file
        evolution.md     # Architecture evolution
    docs/
        first_flight_checklist.md  # Hardware bring-up and flight checklist
    worlds/
        hover_test.wbt   # Webots world file
    controllers/
        pilot/           # Webots controller (installed here)
    hal/
        hal.h                # Common HAL interface
        webots-crazyflie/    # Webots simulation HAL
        crazyflie-2.1plus/      # Crazyflie 2.1+ HAL (STM32F405)
            bringup/         # Hardware bring-up test firmware
            tests/           # HAL test firmware
```

---

## Testing Results

### Hover Behavior

1. Drone starts at 0.5m altitude (world file setting)
2. Altitude PID commands increased thrust
3. Drone rises with some initial oscillation
4. Settles at 1.0m within ~3 seconds
5. Maintains stable hover at 1.0m +/- 0.05m

### Console Output

```
10-11 actors spawned (see Actor Counts table)
[ALT] tgt=1.00 alt=0.01 vvel=0.00 thrust=0.750
[ALT] tgt=1.00 alt=0.05 vvel=0.12 thrust=0.720
...
[WPT] Advancing to waypoint 1: (1.0, 0.0, 1.2) yaw=0 deg
[ALT] tgt=1.20 alt=1.02 vvel=0.05 thrust=0.560
[POS] tgt=(1.0,0.0) x=0.12 y=0.00 pitch=5.2 roll=0.0
...
```

## Memory Requirements

### STM32 Builds

| Platform | Flash | RAM | MCU |
|----------|-------|-----|-----|
| Crazyflie 2.1+ | ~63 KB | ~120 KB | STM32F405 (1 MB / 192 KB) |

### Configuration Split

Hive memory settings are split between shared and platform-specific files:

| File | Contents |
|------|----------|
| `hive_config.mk` | Shared settings: actors, buses, pools, message size, supervisor limits |
| `Makefile.<platform>` | Platform-specific: stack sizes, flash layout |

The shared settings in `hive_config.mk` are determined by the pilot application
(10-11 actors, 7 buses, pool sizes for supervision) and are identical across all
platforms. Only stack sizes vary based on available RAM.

Key memory optimizations in `hive_config.mk`:
- Supervisor pool reduced to 1 supervisor with 12 children (saves ~23 KB vs default)
- Pool sizes tuned for pilot's actual usage patterns

### Measured Stack Usage

Build with `STACK_PROFILE=1` to measure actual stack usage via watermarking.
Results from x86-64 Linux (Webots simulation) with 4KB stacks:

| Actor | Used | Usage | Notes |
|-------|------|-------|-------|
| sensor | 2056 | 50.2% | Highest - HAL sensor structs |
| tlog | 1912 | 46.7% | CSV formatting + file I/O |
| estimator | 1480 | 36.1% | Kalman filter + complementary filter state |
| altitude | 1304 | 31.8% | PID state + landing logic |
| waypoint | 1288 | 31.4% | Waypoint list + arrival detection |
| flight_mgr | 1192 | 29.1% | Log file management + IPC |
| supervisor | 1128 | 27.5% | Child management overhead |
| rate | 792 | 19.3% | Rate PIDs |
| attitude | 760 | 18.6% | Attitude PIDs |
| position | 664 | 16.2% | Position PD |
| motor | 504 | 12.3% | Lowest - simple output |

All actors fit comfortably in 4KB with ~50% headroom (50.2% peak).
ARM Cortex-M may differ slightly due to calling conventions.
