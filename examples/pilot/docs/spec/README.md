# Pilot Example Specification

A complete quadcopter autopilot using the actor runtime. Features cascaded PID control,
sensor fusion (complementary filter + altitude KF + horizontal KF), pub-sub data flow,
and fail-safe supervision. Supports Webots simulation (default) and STM32 hardware
(Crazyflie 2.1+).

> In this document, "pilot" refers to the autopilot software, not a human pilot.

## Documents

| Document | Description |
|----------|-------------|
| [design.md](design.md) | Goals, design decisions, architecture overview |
| [implementation.md](implementation.md) | Implementation details, control algorithms, portability |
| [evolution.md](evolution.md) | Production gaps, architecture evolution roadmap, future work |
| [../tunable_radio_params.md](../tunable_radio_params.md) | Runtime parameter tuning specification |

## Quick Reference

### Actor Counts

| Category | Count | Notes |
|----------|-------|-------|
| Flight-critical workers | 8 | sensor, estimator, waypoint, altitude, position, attitude, rate, motor |
| Flight manager | 1 | Coordinates startup/shutdown, TRANSIENT restart |
| Battery monitor | 1 | 2 Hz voltage sampling, emergency landing, TEMPORARY restart |
| Logger | 1 | Hive log sync + CSV telemetry (to /sd or /tmp) |
| Supervisor | 1 | Monitors all children |
| Optional (Crazyflie) | +1 | comms_actor for radio telemetry |
| **Total** | 12-13 | 11-12 children + 1 supervisor |

### Platform Differences

| Feature | Webots | Crazyflie 2.1+ |
|---------|--------|----------------|
| `comms_actor` | No | Yes |
| `logger_actor` | Yes (/tmp) | Optional (/sd, ENABLE_SD=1) |
| Position source | Simulated GPS | Optical flow (Flow deck v2) |
| Altitude source | Simulated rangefinder | ToF sensor (Flow deck v2) |
| Flash logging | No | Yes |
| Radio telemetry | No | Yes |

### Timing Terminology

| Term | Definition |
|------|------------|
| **Control period** | 4ms (250 Hz), the interval between control loop iterations |
| **Tick** | One control period |
| **Simulation step** | `HAL_TIME_STEP_US`, equals control period in Webots |

**Clock source** - The sensor_actor runs a 4ms periodic timer and is the only actor with a timer. All other actors are event-driven; they block on bus reads and wake when new data arrives. This creates a natural data-flow pipeline: sensor publishes -> estimator reacts -> controllers react -> motor reacts, all within one tick.

> **Disclaimer** - This is a demonstration autopilot, not production-ready flight software. See [Production Gaps](evolution.md#production-gaps) for what a real flight controller would need.
