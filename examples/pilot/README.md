# Pilot Example

Quadcopter waypoint navigation using hive actor runtime.

Supports three platforms:
- **Webots simulation** (default) - Crazyflie quadcopter in Webots simulator
- **Crazyflie 2.1+** - Bitcraze nano quadcopter (~39 KB flash, ~140 KB RAM)
- **STEVAL-DRONE01** - ST mini drone kit (~60 KB flash, ~57 KB RAM)

## What it does

Demonstrates waypoint navigation with a quadcopter using 9 actors:

1. **Sensor actor** reads raw sensors via HAL, publishes to sensor bus
2. **Estimator actor** runs complementary filter, computes velocities, publishes to state bus
3. **Altitude actor** reads target altitude from position target bus, runs altitude PID
4. **Waypoint actor** waits for START signal, manages waypoint list, publishes to position target bus
5. **Position actor** reads target XY/yaw from position target bus, runs position PD
6. **Attitude actor** runs attitude PIDs, publishes rate setpoints
7. **Rate actor** runs rate PIDs, publishes torque commands
8. **Motor actor** reads torque bus, writes to hardware via HAL (checks for STOP signal)
9. **Supervisor actor** handles startup delay (60s), sends START/STOP notifications

**Webots:** Flies a square pattern with altitude changes at each waypoint (full 3D navigation with GPS).

**Crazyflie 2.1+:** With Flow deck v2, uses optical flow for XY positioning and ToF for altitude.
Without Flow deck, hovers and changes altitude only. 60-second startup delay before flight.

**STEVAL-DRONE01:** Hovers and changes altitude only (no GPS, so XY position fixed at origin).
60-second startup delay before flight.

**Safety features (all platforms):** Emergency cutoff on excessive tilt (>45°), excessive
altitude (>2m), or touchdown. Flight duration limited by supervisor (10s/40s/60s per profile).

## Prerequisites

**For Webots simulation:**
- Webots installed (https://cyberbotics.com/)

**For Crazyflie 2.1+ / STEVAL-DRONE01:**
- ARM GCC: `apt install gcc-arm-none-eabi`
- ST-Link: `apt install stlink-tools`
- Debug adapter (Bitcraze debug adapter for Crazyflie, or ST-Link for STEVAL)

## Build and Run

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots and start the simulation.

### STM32 Hardware (Crazyflie 2.1+, STEVAL-DRONE01)

```bash
make -f Makefile.<platform>        # Build firmware
make -f Makefile.<platform> flash  # Flash via debug adapter
make -f Makefile.<platform> clean  # Clean build artifacts
```

See `hal/<platform>/README.md` for hardware details, pin mapping, and flight profiles.

## Files

### Application Code

| File | Description |
|------|-------------|
| `pilot.c` | Main entry point, bus setup, actor spawn |
| `sensor_actor.c/h` | Reads sensors via HAL → sensor bus |
| `estimator_actor.c/h` | Sensor fusion → state bus |
| `altitude_actor.c/h` | Altitude PID → thrust |
| `waypoint_actor.c/h` | Waypoint manager → position target bus |
| `position_actor.c/h` | Position PD → attitude setpoints |
| `attitude_actor.c/h` | Attitude PIDs → rate setpoints |
| `rate_actor.c/h` | Rate PIDs → torque commands |
| `motor_actor.c/h` | Output: torque → HAL → motors |
| `supervisor_actor.c/h` | Startup delay, flight window cutoff |
| `pid.c/h` | Reusable PID controller |
| `fusion/complementary_filter.c/h` | Portable attitude estimation (accel+gyro fusion) |
| `types.h` | Shared data types (sensor_data_t, state_estimate_t, etc.) |
| `config.h` | Configuration constants (timing, thresholds, bus config) |
| `math_utils.h` | Math macros (CLAMPF, LPF, NORMALIZE_ANGLE) |
| `notifications.h` | IPC notification tags (NOTIFY_FLIGHT_START, etc.) |
| `flight_profiles.h` | Waypoint definitions per flight profile |

### Build System

| File | Description |
|------|-------------|
| `Makefile` | Webots simulation build |
| `Makefile.crazyflie-2.1+` | Crazyflie 2.1+ build (STM32F405, 168 MHz) |
| `Makefile.STEVAL-DRONE01` | STEVAL-DRONE01 build (STM32F401, 84 MHz) |
| `hive_config.mk` | Shared Hive memory config (included by all Makefiles) |

### Configuration

| File | Description |
|------|-------------|
| `config.h` | Configuration constants (timing, thresholds, bus config) |
| `math_utils.h` | Math macros (CLAMPF, LPF, NORMALIZE_ANGLE) |
| `notifications.h` | IPC notification tags (NOTIFY_FLIGHT_START, etc.) |
| `flight_profiles.h` | Waypoint definitions per flight profile |
| `hive_config.mk` | Shared Hive memory limits (actors, buses, pools) |
| `hal/<platform>/hal_config.h` | Platform-specific PID gains and thrust |

Hive memory settings (actors, buses, pool sizes) are determined by the pilot
application and shared across all platforms via `hive_config.mk`. Only stack
sizes differ per platform based on available RAM.

### Documentation

| File | Description |
|------|-------------|
| `README.md` | This file |
| `SPEC.md` | Detailed design specification |

### Directories

| Directory | Description |
|-----------|-------------|
| `hal/` | Hardware abstraction layer (see `hal/<platform>/README.md`) |
| `controllers/pilot/` | Webots controller (symlink created by `make install`) |
| `worlds/` | Webots world files |

## Architecture

Nine actors connected via buses:

```mermaid
graph TB
    Sensor[Sensor] --> SensorBus([Sensor Bus]) --> Estimator[Estimator] --> StateBus([State Bus])

    StateBus --> Waypoint[Waypoint] --> PositionTargetBus([Position Target Bus])
    PositionTargetBus --> Altitude[Altitude] --> ThrustBus([Thrust Bus]) --> Rate[Rate]
    PositionTargetBus --> Position[Position]
    StateBus --> Altitude
    StateBus --> Position --> AttitudeSP([Attitude SP Bus]) --> Attitude[Attitude]
    StateBus --> Attitude --> RateSP([Rate SP Bus]) --> Rate
    Rate --> TorqueBus([Torque Bus]) --> Motor[Motor]
```

Hardware Abstraction Layer (HAL) provides platform independence:
- `hal_read_sensors()` - reads sensors (called by sensor_actor)
- `hal_write_torque()` - writes motors with mixing (called by motor_actor)

Actor code is identical across platforms. See `hal/<platform>/README.md` for
hardware-specific details.

## Actor Priorities and Spawn Order

All actors run at CRITICAL priority. Spawn order determines execution order
within the same priority level (round-robin). Actors are spawned in data-flow
order to ensure each actor sees fresh data from upstream actors in the same step:

| Order | Actor     | Priority | Rationale |
|-------|-----------|----------|-----------|
| 1     | sensor    | CRITICAL | Reads hardware first |
| 2     | estimator | CRITICAL | Needs sensors, produces state estimate |
| 3     | altitude  | CRITICAL | Needs state, produces thrust |
| 4     | waypoint  | CRITICAL | Needs state + START signal, produces position targets |
| 5     | position  | CRITICAL | Needs target, produces attitude setpoints |
| 6     | attitude  | CRITICAL | Needs attitude setpoints, produces rate setpoints |
| 7     | rate      | CRITICAL | Needs state + thrust + rate setpoints |
| 8     | motor     | CRITICAL | Needs torque + STOP signal, writes hardware last |
| 9     | supervisor| CRITICAL | Sends START to waypoint, STOP to motor after flight window |

## Control System

### PID Controllers

Gains are tuned per platform in `hal/<platform>/hal_config.h`. The control
cascade is: altitude → position → attitude → rate → motors.

- **Altitude:** PI with velocity damping (tracks target altitude)
- **Position:** PD with velocity damping (tracks target XY, max tilt limited)
- **Attitude:** P controller for roll/pitch/yaw angles
- **Rate:** PD controller for angular rates

### Waypoint Navigation

The waypoint actor manages a list of waypoints and publishes the current target
to the position target bus. Both altitude and position actors read from this bus.

Routes depend on flight profile (`FLIGHT_PROFILE=N` at build time) and platform
capabilities. See `hal/<platform>/README.md` for available flight profiles.

**Arrival detection:** Position within 0.15m, heading within 0.1 rad, velocity below 0.05 m/s.
After completing the route, the drone loops back to the first waypoint.

### Motor Mixer

Each HAL implements its own X-configuration mixer in `hal_write_torque()`.
See `hal/<platform>/README.md` for motor layout and mixing equations.

## Main Loop

The main loop is minimal - all logic is in actors:

```c
while (hal_step()) {
    hive_advance_time(HAL_TIME_STEP_US);  // Advance simulation time, fire due timers
    hive_run_until_blocked();              // Run actors until all blocked
}
```

Webots controls time via `hal_step()` (which wraps `wb_robot_step()`). Each call:
1. Blocks until Webots simulates TIME_STEP milliseconds
2. Returns, `hive_advance_time()` fires due timers
3. `hive_run_until_blocked()` runs all ready actors
4. Actors read sensors, compute, publish results
5. Loop repeats

