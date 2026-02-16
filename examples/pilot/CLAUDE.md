# CLAUDE.md - Pilot Project

This file provides guidance to Claude Code when working with the pilot autopilot.

## Project Overview

Quadcopter autopilot using the Hive actor runtime. 12-13 actors in a supervised control pipeline with cascaded PID control, sensor fusion, battery monitoring, and telemetry logging.

**Platforms:**
- Crazyflie 2.1+ hardware (STM32F405 + nRF51822)
- Webots simulation

**Actor count:** 12-13 (11-12 children + 1 supervisor)
- 8 flight-critical workers: sensor, estimator, waypoint, altitude, position, attitude, rate, motor
- 1 flight manager (TRANSIENT - normal exit = mission complete)
- 1 battery monitor (TEMPORARY - 2 Hz voltage sampling, emergency landing)
- 1 logger (hive log sync + CSV telemetry to /sd or /tmp)
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

# First flight test (6s hover at 0.5m)
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST

# With SD card logging
make -f Makefile.crazyflie-2.1plus ENABLE_SD=1

# Clean
make -f Makefile.crazyflie-2.1plus clean
```

## Flash and Debug Workflow

**CRITICAL:** st-flash leaves CPU halted. You MUST run st-trace.sh after every flash.

**ST-Link tools location:** `local/stlink/build/bin/` (st-flash, st-trace, st-info)

```bash
# Flash firmware (use full path - st-flash is not in PATH)
local/stlink/build/bin/st-flash --connect-under-reset write \
  examples/pilot/build_crazyflie-2.1plus/pilot_crazyflie-2.1plus.bin 0x8000000

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

# Start flight (sends GO command, 10s countdown then flight)
python3 tools/ground_station.py --go
```

## Runtime Parameter Tuning

Parameters can be tuned live via radio without reflashing. This eliminates the build-flash-wait-test cycle during PID tuning.

```bash
# List all tunable parameters with current values
python3 tools/ground_station.py --list-params

# Get a specific parameter
python3 tools/ground_station.py --get-param rate_kp

# Set a parameter (takes effect immediately on next control loop)
python3 tools/ground_station.py --set-param rate_kp 0.025
python3 tools/ground_station.py --set-param att_kp 2.0
python3 tools/ground_station.py --set-param vvel_damping 0.50
```

**Tunable Parameters (45 total):**

| Category | Parameters |
|----------|------------|
| Rate PID | `rate_kp`, `rate_ki`, `rate_kd`, `rate_imax`, `rate_omax_roll`, `rate_omax_pitch`, `rate_omax_yaw` |
| Attitude PID | `att_kp`, `att_ki`, `att_kd`, `att_imax`, `att_omax` |
| Altitude PID | `alt_kp`, `alt_ki`, `alt_kd`, `alt_imax`, `alt_omax`, `vvel_damping` |
| Emergency | `emergency_tilt_limit`, `emergency_alt_max` |
| Landing | `landing_descent_rate`, `landing_velocity_gain` |
| Position | `pos_kp`, `pos_kd`, `max_tilt_angle` |
| Complementary Filter | `cf_alpha`, `cf_mag_alpha`, `cf_use_mag`, `cf_accel_thresh_lo`, `cf_accel_thresh_hi` |
| Altitude Kalman Filter | `kf_q_altitude`, `kf_q_velocity`, `kf_q_bias`, `kf_r_altitude`, `kf_p0_altitude`, `kf_p0_velocity`, `kf_p0_bias` |
| Horizontal Velocity | `hvel_filter_alpha` |
| Waypoints | `wp_tolerance_xy`, `wp_tolerance_z`, `wp_tolerance_yaw`, `wp_tolerance_vel`, `wp_hover_time_s` |

**Safety:** All values are validated before application. Invalid values (out of range) are rejected.

**Thread Safety:** ARM Cortex-M4 guarantees atomic 32-bit aligned reads/writes. Actors read current values each control loop iteration.

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
| `FLIGHT_PROFILE_GROUND_TEST` | Disabled | 60s | ESB/telemetry testing |
| `FLIGHT_PROFILE_FIRST_TEST` | Enabled | 6s | First flight (hover at 0.5m) |
| `FLIGHT_PROFILE_ALTITUDE` | Enabled | 20s | Altitude waypoints (0.5-1.0m) |
| `FLIGHT_PROFILE_FULL_3D` | Enabled | 10s | Full 3D waypoint navigation |

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
- `battery_actor.c` - Battery voltage monitoring (2 Hz), emergency landing
- `comms_actor.c` - Radio telemetry (Crazyflie only, event-driven RX)
- `logger_actor.c` - Hive log sync + CSV telemetry at 25Hz

### HAL
- `hal/hal.h` - Common HAL interface
- `hal/crazyflie-2.1plus/` - Crazyflie HAL (STM32F405)
- `hal/crazyflie-2.1plus/hal_syslink.c` - Syslink UART with DMA and IDLE interrupt
- `hal/crazyflie-2.1plus/hal_debug.c` - Debug output (SWO + early log buffer)
- `hal/crazyflie-2.1plus/early_log.c` - Captures hal_printf() before hive log ready
- `hal/webots-crazyflie/` - Webots simulation HAL

### Configuration
- `config.h` - Timing constants, thresholds
- `hal/*/hal_config.h` - Platform-specific PID gains, thrust (compile-time defaults)
- `hive_config.mk` - Shared Hive memory config
- `include/flight_profiles.h` - Waypoint definitions
- `include/tunable_params.h` - Runtime-tunable parameter definitions
- `tunable_params.c` - Parameter validation and initialization

### Tools
- `tools/ground_station.py` - Radio telemetry receiver
- `tools/st-trace.sh` - SWO trace viewer
- `tools/analyze_pid.py` - PID metrics analysis
- `tools/plot_telemetry.py` - Telemetry visualization

## Architecture Notes

### Early Log Buffer (Crazyflie only)

HAL init runs before hive_log_file_open(), so HIVE_LOG_* can't capture early messages.
The early log buffer solves this:

1. `hal_printf()` writes to both SWO trace (if debugger connected) and early log buffer
2. After `hive_log_file_open()`, `hal_flush_early_log()` replays buffered messages to hive log
3. Buffer is then disabled (further hal_printf goes only to SWO)

This ensures HAL init messages appear in the log file even though the file wasn't open yet.
On Webots, `hal_flush_early_log()` is a no-op (printf goes to stdout, no buffering needed).

### Event-Driven Radio RX

The comms_actor uses `hive_event_wait()` for interrupt-driven radio reception:
- UART IDLE interrupt signals HAL event when syslink packet arrives
- No polling - actor sleeps until data available
- HAL handles protocol framing (CRTP header), application sees clean payload

### Bus Configuration

All buses use `max_entries=1` (latest value only). Control loops need current state, not history.

### Supervision

ONE_FOR_ALL strategy - if any flight-critical actor crashes, all restart together. This ensures consistent pipeline state. Battery, comms, and logger use TEMPORARY restart (crash doesn't affect flight).

### Motor Safety

- **START gate** - Motors stay OFF until flight_manager sends authorization (after 10s countdown)
- **Crash latch** - Once attitude exceeds 45 degrees, motors stay OFF until reboot (only active when target altitude > 0)
- 50ms deadman timeout in motor_actor (zeros motors if no command)
- Attitude cutoff at 45 degrees
- Altitude cutoff at 2m
- Thrust ramp on takeoff (0.75s)
- 10-second landing timeout (forces shutdown if landing detection fails)
- Battery monitoring at 2 Hz with 5-second debounce (3.0V critical triggers emergency landing)

## ESB Radio Protocol

The nRF51 is PRX (receiver), Crazyradio is PTX (transmitter):
- Ground station polls, drone responds via ACK payload
- Telemetry piggybacks on ACK packets
- ESB limit is 32 bytes, HAL uses 1 byte for framing
- Maximum application payload is 30 bytes
- Protocol framing handled by HAL, not application code

**Command IDs (in comms_actor.c):**
- `0x03-0x04` - Telemetry packets (tlog_state, tlog_sensors)
- `0x10-0x12` - Log download commands
- `0x20` - GO command (start flight)
- `0x30-0x32` - Parameter tuning (set, get, list)
- `0x33-0x35` - Parameter responses (ack, value, list chunk)

## Local Dependencies

Paths are relative to repo root (`/home/jocke/projects/hive`):

- `local/stlink/build/bin/` - ST-Link tools (st-flash, st-trace, st-info)
- `local/crazyflie-firmware/` - Bitcraze firmware source (reference for deck drivers, SPI config)

## Documentation

- `README.md` - Usage instructions
- `docs/spec/` - Design specification (design.md, implementation.md, evolution.md)
- `docs/tunable_radio_params.md` - Runtime parameter tuning specification
- `docs/first_flight_checklist.md` - Hardware bring-up guide
- `hal/*/README.md` - Platform-specific details
- `tools/README.md` - Analysis tools documentation
- `tests/README.md` - Test documentation
