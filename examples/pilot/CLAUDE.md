# CLAUDE.md - Pilot Project

This file provides guidance to Claude Code when working with the pilot autopilot.

## Project Overview

Quadcopter autopilot using the Hive actor runtime. 11-12 actors in a supervised control pipeline with cascaded PID control, sensor fusion, and telemetry logging.

**Platforms:**
- Crazyflie 2.1+ hardware (STM32F405 + nRF51822)
- Webots simulation

**Actor count:** 11-12 (10-11 children + 1 supervisor)
- 8 flight-critical workers: sensor, estimator, waypoint, altitude, position, attitude, rate, motor
- 1 flight manager (TRANSIENT - normal exit = mission complete)
- 1 telemetry logger (to /sd or /tmp)
- 1 supervisor
- +1 comms actor (Crazyflie only, radio telemetry)

## Build Commands

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots
make                    # Build and auto-install to Webots
make clean              # Clean build
make STACK_PROFILE=1    # Build with stack profiling
```

### Crazyflie 2.1+

```bash
# Ground test mode (motors disabled, safe for testing)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_GROUND_TEST

# Indoor flight (conservative limits)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_INDOOR

# With SD card logging
make -f Makefile.crazyflie-2.1plus ENABLE_SD=1

# Clean
make -f Makefile.crazyflie-2.1plus clean
```

## Flash and Debug Workflow

**CRITICAL:** st-flash leaves CPU halted. You MUST run st-trace.sh after every flash.

```bash
# Flash firmware
make -f Makefile.crazyflie-2.1plus flash

# MANDATORY: Reset CPU and view debug output
./tools/st-trace.sh -t 30    # 30s timeout
./tools/st-trace.sh -t 0     # No timeout (Ctrl-C to stop)
```

Without running st-trace.sh after flashing, the drone appears dead (no LEDs, no response).

## Ground Station

```bash
# Activate Python venv
source tests/.venv/bin/activate

# Receive real-time telemetry
python3 tools/ground_station.py --uri radio://0/80/2M

# Log telemetry to CSV
python3 tools/ground_station.py --uri radio://0/80/2M -o flight.csv

# Download flash log after flight
python3 tools/ground_station.py --download-log flight.log
```

## Telemetry Analysis

CSV telemetry is written to `/tmp/tlog.csv` (Webots) or `/sd/tlog.csv` (Crazyflie with SD).

```bash
# PID metrics (overshoot, settling time, etc.)
python3 tools/analyze_pid.py /tmp/tlog.csv

# 6-panel visualization
python3 tools/plot_telemetry.py /tmp/tlog.csv

# Full flight summary with 3D trajectory
python3 tools/plot_flight.py /tmp/tlog.csv

# Hover stability analysis
python3 tools/analyze_hover.py /tmp/tlog.csv

# Flight debugging (timeline, crash detection)
python3 tools/flight_debug.py /tmp/tlog.csv
```

## Flight Profiles

Build-time selection via `FLIGHT_PROFILE=`:

| Profile | Motors | Duration | Use Case |
|---------|--------|----------|----------|
| `FLIGHT_PROFILE_GROUND_TEST` | Disabled | N/A | ESB/telemetry testing |
| `FLIGHT_PROFILE_INDOOR` | Enabled | 40s | Indoor flight |
| `FLIGHT_PROFILE_OUTDOOR` | Enabled | 60s | Outdoor flight |

## Key Files

### Actors
- `pilot.c` - Main entry, bus setup, supervisor config
- `sensor_actor.c` - Reads sensors via HAL, 4ms timer (250Hz)
- `estimator_actor.c` - Kalman filter + complementary filter
- `altitude_actor.c` - Altitude PID, landing detection
- `waypoint_actor.c` - Waypoint navigation
- `position_actor.c` - Position PD controller
- `attitude_actor.c` - Attitude PIDs
- `rate_actor.c` - Rate PIDs
- `motor_actor.c` - Output to HAL, 50ms deadman timeout
- `flight_manager_actor.c` - Startup delay, flight coordination
- `comms_actor.c` - Radio telemetry (Crazyflie only, event-driven RX)
- `telemetry_logger_actor.c` - CSV logging at 25Hz

### HAL
- `hal/hal.h` - Common HAL interface
- `hal/crazyflie-2.1plus/` - Crazyflie HAL (STM32F405)
- `hal/crazyflie-2.1plus/hal_syslink.c` - Syslink UART with DMA and IDLE interrupt
- `hal/webots-crazyflie/` - Webots simulation HAL

### Configuration
- `config.h` - Timing constants, thresholds
- `hal/*/hal_config.h` - Platform-specific PID gains, thrust
- `hive_config.mk` - Shared Hive memory config
- `include/flight_profiles.h` - Waypoint definitions

### Tools
- `tools/ground_station.py` - Radio telemetry receiver
- `tools/st-trace.sh` - SWO trace viewer
- `tools/analyze_pid.py` - PID metrics analysis
- `tools/plot_telemetry.py` - Telemetry visualization

## Architecture Notes

### Event-Driven Radio RX

The comms_actor uses `hive_event_wait()` for interrupt-driven radio reception:
- UART IDLE interrupt signals HAL event when syslink packet arrives
- No polling - actor sleeps until data available
- HAL handles protocol framing (CRTP header), application sees clean payload

### Bus Configuration

All buses use `max_entries=1` (latest value only). Control loops need current state, not history.

### Supervision

ONE_FOR_ALL strategy - if any flight-critical actor crashes, all restart together. This ensures consistent pipeline state. Comms and telemetry logger use TEMPORARY restart (crash doesn't affect flight).

### Motor Safety

- 50ms deadman timeout in motor_actor (zeros motors if no command)
- Attitude cutoff at 45 degrees
- Altitude cutoff at 2m
- Thrust ramp on takeoff (0.5s)

## ESB Radio Protocol

The nRF51 is PRX (receiver), Crazyradio is PTX (transmitter):
- Ground station polls, drone responds via ACK payload
- Telemetry piggybacks on ACK packets
- ESB limit is 32 bytes, HAL uses 1 byte for framing
- Maximum application payload is 30 bytes
- Protocol framing handled by HAL, not application code

## Documentation

- `README.md` - Usage instructions
- `docs/spec/` - Design specification (design.md, implementation.md, evolution.md)
- `docs/first_flight_checklist.md` - Hardware bring-up guide
- `hal/*/README.md` - Platform-specific details
- `tools/README.md` - Analysis tools documentation
- `tests/README.md` - Test documentation
