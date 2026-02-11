# Evolution

Production gaps, architecture evolution roadmap, and future extensions.

## Table of Contents

- [Production Gaps](#production-gaps)
- [Deferred Features](#deferred-features)
- [Architecture Evolution Roadmap](#architecture-evolution-roadmap)
- [Future Extensions](#future-extensions)

---

## Production Gaps

What a production flight controller would need beyond this demonstration:

### Missing Error Handling

| Scenario | Current Behavior | Production Requirement |
|----------|------------------|------------------------|
| Sensor read fails | `hive_bus_read()` returns error, actor skips iteration | Watchdog timeout, switch to backup sensor or land |
| Bus publish fails | Error ignored | Log error, trigger failsafe |
| Actor crashes | Runtime notifies linked actors | Auto-restart or emergency landing |
| GPS signal lost | Position control uses stale data | Hold last position, descend slowly, or return-to-home |
| IMU data invalid | Garbage in, garbage out | Sanity checks, sensor voting, reject outliers |

### Implemented Safety Features

The following safety features run on all platforms (Webots and STM32):

| Feature | Location | Behavior |
|---------|----------|----------|
| Motor START gate | motor_actor.c | Motors stay OFF until flight_manager sends authorization |
| Crash latch | altitude_actor.c | Once attitude exceeds 45 deg, motors stay OFF until reboot |
| Attitude cutoff | altitude_actor.c | Motors off if roll or pitch >45 deg |
| Altitude cutoff | altitude_actor.c | Motors off if altitude >2m |
| Landed detection | altitude_actor.c | Motors off when target <5cm and altitude <15cm |
| Thrust ramp | altitude_actor.c | Gradual thrust increase over 0.5 seconds on takeoff |
| Motor deadman | motor_actor.c | Motors zeroed if no torque command within 50ms |
| Flight duration | flight_manager_actor.c | Controlled landing after timeout (6-60s depending on flight profile) |
| Landing timeout | flight_manager_actor.c | Forces shutdown if landing detection fails (10s max) |
| Battery monitoring | battery_actor.c | 2 Hz voltage sampling, 3.2V warning, 3.0V critical with 5s debounce |
| Low battery landing | flight_manager_actor.c | Emergency landing on debounced critical voltage (ARMED: disarm, FLYING: land) |

The following safety features are STM32-only (disabled in Webots):

| Feature | Location | Behavior |
|---------|----------|----------|
| Startup delay | flight_manager_actor.c | Flight blocked for 60 seconds after boot (motors OFF) |

### Future Safety Features

The following may be added after initial flight testing:

- **Rate cutoff**: Motors off if angular rate exceeds threshold (e.g., >300 deg/s) - catches violent oscillation while within tilt limits
- **Accelerometer sanity check**: Motors off if acceleration readings are implausible
- **Sensor timeout**: Motors off if no sensor updates within expected interval

#### Bitcraze Supervisor Features (Reference)

The [Bitcraze crazyflie-firmware supervisor](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/supervisor.c)
implements a comprehensive state machine with safety features. Local copy at `local/crazyflie-firmware/`.

**State Machine (11 states)**

```
NotInitialized → PreFlChecksNotPassed → PreFlChecksPassed → ReadyToFly
                                                               ↓
                     Locked ← Reset ← Landed ← Flying ← (armed)
                       ↑                ↓         ↓
                       └──── Crashed ←──┴── WarningLevelOut
                                              (excessive tilt)
```

| State | Description |
|-------|-------------|
| PreFlChecksNotPassed | Waiting for system initialization |
| PreFlChecksPassed | Ready to arm (sensors OK) |
| ReadyToFly | Armed, motors can spin |
| Flying | Thrust above idle threshold |
| Landed | Thrust below idle for >2s |
| WarningLevelOut | Excessive tilt - XY disabled, roll/pitch locked to zero |
| Crashed | Crash detected, requires explicit recovery |
| Locked | Emergency stop triggered, reboot required |
| Reset | Transitional state back to PreFlChecksPassed |

**Safety Features**

| Feature | Description | Thresholds/Timeouts | Priority |
|---------|-------------|---------------------|----------|
| **Commander watchdog** | Stale setpoint detection | Warning 500ms, shutdown 2000ms | High |
| **Emergency stop** | Multiple trigger sources | Parameter, localization watchdog (1000ms), external request | High |
| **Tumble detection** | Tilt angle + duration | 0.5g Z-accel (~60°) for 1000ms, or -0.2g (upside-down) for 100ms | High |
| **Free fall exception** | Suspends tumble check | All axes < 0.1g | Medium |
| **Is Flying detection** | Motor thrust monitoring | Any motor > idle thrust, 2000ms hysteresis | Medium |
| **Preflight timeout** | Armed-but-not-flying limit | 30s default (configurable) | Medium |
| **Landing timeout** | Landed state duration | 3s default (configurable) | Low |
| **Crash recovery** | Explicit recovery required | Blocked if still tumbled | Low |
| **Warning Level Out** | Excessive tilt handling | Disables XY, locks roll/pitch to zero, keeps Z control | Medium |

**Configurable Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `supervisor.tmblChckEn` | 1 | Enable/disable tumble check |
| `supervisor.prefltTimeout` | 30000ms | Preflight timeout duration |
| `supervisor.landedTimeout` | 3000ms | Landing timeout duration |
| `stabilizer.stop` | 0 | Emergency stop (non-zero = stop) |

**Tumble Detection Details**

The tumble check uses accelerometer Z-axis to detect unsafe orientations:
- **Tilt threshold**: Z-accel < 0.5g (~60° from vertical) for 1000ms → tumbled
- **Upside-down threshold**: Z-accel < -0.2g for 100ms → tumbled immediately
- **Free fall exception**: If |X|, |Y|, |Z| all < 0.1g, tumble timer resets (valid flight)

**NOT in Bitcraze supervisor** (was incorrectly listed):
- Estimator health / Kalman covariance checking - not implemented in supervisor.c

**Implementation approach**: These would be implemented in a dedicated supervisor
actor (not in HAL) that:
1. Subscribes to sensor_bus and state_bus
2. Monitors attitude, thrust, and flight state
3. Can override motor commands via motor_bus or direct IPC
4. Manages arming state transitions
5. Implements tumble detection using accelerometer data

The current pilot has basic cutoffs in altitude_actor.c (attitude >45°, altitude >2m).
A supervisor actor would centralize these checks and add the additional features above.

### Missing Safety Features (Production Requirements)

- **Geofence**: No boundary limits - drone can fly away indefinitely
- **Battery monitoring**: ~~No low-voltage warning or auto-land~~ IMPLEMENTED - battery_actor monitors at 2 Hz with debounced emergency landing
- **Arming/disarming**: No safety switch to prevent accidental motor start
- **Pre-flight checks**: No sensor validation before takeoff
- **Communication loss**: No failsafe if telemetry link drops

### Why These Are Omitted

This example focuses on demonstrating the actor runtime architecture, not building a safe drone. Adding proper failsafes would obscure the core concepts (actors, buses, control loops) with error handling code.

For a production system, each actor should:
1. Validate inputs before processing
2. Handle bus read/write failures
3. Implement timeouts for expected data
4. Report health status to a flight manager actor
5. Respond to emergency stop commands

### Production Instrumentation Requirements

The following instrumentation should be added for production flight software:

**Error Counters (per actor)**
- `bus_read_fail_count` - incremented when `hive_bus_read()` returns error
- `bus_publish_fail_count` - incremented when `hive_bus_publish()` returns error
- Counters exposed via telemetry or debug interface

**Motor Deadman Watchdog** - Implemented
- Motor actor uses `hive_select()` with `MOTOR_DEADMAN_TIMEOUT_MS` timeout (50ms)
- If no torque command received within timeout, all motors are zeroed
- Protects against controller actor crash leaving motors at last commanded value
- Timeout of 50ms (~12 control cycles at 250Hz) provides margin while remaining safe
- Logs warning on timeout: `[MOTOR] Deadman timeout - zeroing motors`

## Deferred Features

Features intentionally omitted from this demonstration:

- Unified EKF (single filter for position/velocity/attitude) - separate estimators used instead: altitude Kalman filter + complementary filter for attitude
- Failsafe handling (return-to-home, auto-land) - requires GPS and mission planning
- Multiple vehicle types - single X-configuration quadcopter

**Note:** Runtime parameter tuning is now implemented. See `docs/tunable_radio_params.md`.

---

## Architecture Evolution Roadmap

This section documents how the architecture evolved from a monolithic design to the current
multi-actor implementation. Steps 1-11 are complete; Step 12 is future work.

### Future Architecture (Aspirational)

This simplified diagram shows the end-goal architecture including future features (Step 12).
The "Setpoint Actor" would replace the current Waypoint + Flight Manager pattern with a
unified setpoint source supporting RC input and mode switching.

```mermaid
graph LR
    subgraph Sensing
        Sensor[Sensor Actor<br/>Read HW]
        Estimator[Estimator Actor<br/>Fuse data]
    end

    subgraph Control
        Setpoint[Setpoint Actor<br/>RC/Waypoints]
        Altitude[Altitude Actor<br/>Z control]
        Attitude[Attitude Actor<br/>Attitude control]
        Rate[Rate Actor<br/>Rate control]
    end

    subgraph Output
        Motor[Motor Actor<br/>Safety + Write HW]
    end

    subgraph Telemetry
        Comms[Comms Actor<br/>Radio TX]
    end

    Sensor --> Estimator --> StateBus([State Bus])
    Setpoint --> Altitude
    StateBus --> Altitude --> Attitude --> Rate --> Motor
    StateBus -.-> Comms
```

### Step 1: Motor Actor
Separate motor output into dedicated actor.

```
Rate Actor ──► Torque Bus ──► Motor Actor ──► HAL ──► Hardware
                               (output)         (mixer)
```

**Features** - Subscribe to torque bus, call HAL for motor output. Mixer is in HAL.

> **Motor authority** - The motor actor is the sole writer to motor outputs. STOP notifications from flight manager override all torque commands and force outputs to zero. This single point of control prevents conflicting motor commands.

### Step 2: Separate Altitude Actor
Split altitude control from rate control.

```
Sensor Bus ──► Altitude Actor ──► Thrust Bus ──► Rate Actor ──► Torque Bus
            (altitude PID)                    (rate PIDs only)
```

**Benefits** Clear separation, different rates possible, easier tuning.

### Step 3: Sensor Actor
Move sensor reading from main loop into actor.

```
Main Loop: hal_step() + hive_advance_time() + hive_run_until_blocked()
Sensor Actor: timer ──► hal_read_sensors() ──► Sensor Bus
```

**Benefits** Main loop is minimal, all logic in timer-driven actors.

### Step 4: Attitude Actor
Add attitude angle control between altitude and rate control.

**Before**
```
Sensor Bus ──► Rate Actor (rate PIDs with hardcoded 0.0 setpoints)
```

**After**
```
Sensor Bus ──► Attitude Actor ──► Rate Setpoint Bus ──► Rate Actor
            (attitude PIDs)                          (rate PIDs)
```

**Benefits**
- Cascaded control (proper drone architecture)
- Attitude controller generates rate setpoints
- Rate controller tracks those setpoints
- Easier to tune each layer independently

### Step 5: Estimator Actor
Add sensor fusion between raw sensors and controllers.

**Before**
```
Sensor Actor ──► Sensor Bus ──► Controllers
```

**After**
```
Sensor Actor ──► Sensor Bus ──► Estimator Actor ──► State Bus ──► Controllers
                             (Kalman + complementary)
```

**Implementation**
- Altitude Kalman filter (`fusion/altitude_kf.c`) for altitude/velocity estimation
- Complementary filter (`fusion/complementary_filter.c`) for attitude estimation
- Fuses accelerometer and gyroscope for roll/pitch estimation
- Fuses magnetometer for yaw (when available)
- Webots: synthesizes accelerometer from gravity + inertial_unit angles

**Benefits**
- Controllers use state estimate, not raw sensors
- Derived values (velocities) computed in one place
- Fusion algorithm is portable (same code on all platforms)
- HALs are simpler (just raw sensor reads)

### Step 6: Position Actor
Add horizontal position hold and heading hold.

**Before**
```
Attitude Actor uses hardcoded 0.0 attitude setpoints
```

**After**
```
State Bus ──► Position Actor ──► Attitude Setpoint Bus ──► Attitude Actor
              (position PD)       (roll, pitch, yaw)    (attitude PIDs)
```

**Implementation**
- Simple PD controller: position error -> attitude command
- Velocity damping: reduces overshoot
- Max tilt limit: 0.35 rad (~20 deg) for safety
- Sign conventions match Bitcraze Webots controller
- Heading hold: publishes target yaw from waypoint actor
- Angle wrap-around: `pid_update_angle()` handles +/-pi discontinuity
- World-to-body frame transformation based on current yaw

**Benefits**
- Drone holds XY position and heading
- Returns to target when displaced or rotated
- Takes shortest rotation path (never rotates >180 deg)

### Step 7: Waypoint Actor
Add waypoint navigation with platform-specific routes.

**Before**
```
Altitude Actor uses hardcoded TARGET_ALTITUDE
Position Actor uses hardcoded TARGET_X, TARGET_Y, TARGET_YAW
```

**After**
```
                              ┌──► Altitude Actor (reads z)
State Bus ──► Waypoint Actor ──► Position Target Bus
              (navigation)       (x, y, z, yaw)
                              └──► Position Actor (reads x, y, yaw)
```

**Implementation**
- Manages list of waypoints (platform-specific)
- Publishes current target to position target bus
- Altitude actor reads target altitude from position target bus
- Position actor reads target XY and yaw from position target bus
- Monitors state bus for arrival detection
- Arrival requires: altitude, heading within tolerance, velocity below threshold
- Hovers briefly at each waypoint before advancing
- Loops forever: returns to first waypoint after completing route

**Platform-specific routes**
- **Webots** - 3D waypoints with square pattern and altitude changes
- **STM32 hardware** - Platform-dependent (see flight profiles in README.md)
- **First flight test** - Hover at 0.5m briefly, then land (safe tethered test)

**Benefits**
- Decouples waypoint logic from both position and altitude control
- Both actors read targets from bus (no hardcoded values)
- World-to-body frame transformation handles arbitrary headings
- Easy to extend with mission planning

### Step 8: Flight Manager Actor
Add centralized startup coordination and safety cutoff.

**Before**
```
Startup delay and flight window in motor_actor
Waypoint actor starts immediately
```

**After**
```
Flight Manager ──► START ──► Motor Actor (enables output)
               ──► START ──► Waypoint Actor (begins navigation)
                              │
                              v (flight begins)
               ──► LANDING ──► Altitude Actor
                              │
               ◄── LANDED ◄───┘ (touchdown detected)
                              │
               ──► STOP ────► Motor Actor
                              │
                              v (motors zeroed)
```

**Implementation**
- Handles 60-second startup delay (hardware only, motors stay OFF)
- Opens log file (erases flash sector on STM32)
- Sends START notification to motor actor (enables motor output)
- Sends START notification to waypoint actor to begin flight
- Periodic log sync every 4 seconds
- Flight duration per profile (10s/40s/60s)
- Sends LANDING notification to altitude actor
- Waits for LANDED notification (touchdown detected)
- Sends STOP notification to motor actor
- Closes log file

**Benefits**
- Centralized safety timing (not scattered across actors)
- Clear flight authorization flow
- Waypoint actor blocks until flight manager authorizes flight
- Easy to add pre-flight checks in one place

### Step 9: Comms Actor
Add radio telemetry for ground station logging (Crazyflie only).

**Before**
```
No real-time flight data logging during flight
```

**After**
```
State Bus ──┬──► Comms Actor ──► HAL Radio ──► Crazyradio 2.0 ──► Ground Station
Sensor Bus ─┤      (event-driven)
Thrust Bus ─┘
```

**Implementation**
- Subscribes to sensor, state, and thrust buses (not position target bus)
- Uses event-driven RX via `hive_event_wait(hal_esb_get_rx_event())` - no polling
- UART IDLE interrupt signals HAL event when packet arrives
- Sends binary packets over syslink protocol

**Packet limits** - ESB max payload is 32 bytes. HAL uses 1 byte for framing,
so max application payload is 30 bytes.

**Design choice** - Prioritize attitude, rates, and thrust in telemetry packets. Position
targets are omitted to keep packets small - use Webots CSV telemetry for position
control tuning. Each packet includes a 32-bit timestamp (ms since boot).
- Two operating modes:
  - Flight mode: Sends telemetry packets at 100Hz (alternating types at 50Hz each)
  - Download mode: Transfers flash log file to ground station on request
- Telemetry packet types (17 bytes payload each):
  - Type 0x01: Attitude/rates (timestamp, gyro XYZ, roll/pitch/yaw)
  - Type 0x02: Position (timestamp, altitude, velocities, thrust, battery voltage)
- Log download packet types:
  - Type 0x10: CMD_REQUEST_LOG (ground -> drone)
  - Type 0x11: LOG_CHUNK (drone -> ground, 27 bytes data)
  - Type 0x12: LOG_COMPLETE (drone -> ground)
- Runs at LOW priority so control loops run first each cycle
- Radio send blocks ~370us (37 bytes * 10 bits/byte / 1Mbaud)
- Uses TEMPORARY restart (crash/exit doesn't trigger restarts of flight-critical actors)

**Ground station commands (from examples/pilot directory)**
```bash
pip install cflib
./tools/ground_station.py -o flight.csv        # Receive telemetry
./tools/ground_station.py --download-log log.txt  # Download log file (plain text)
cat log.txt                                     # View log directly
```

**Benefits**
- Real-time flight data for debugging and analysis
- Post-flight log download (no physical access required)
- Separate from flash logging (higher rate, no flash wear)
- Non-intrusive (LOW priority doesn't affect control loops)

### Step 10: Logger Actor
Add CSV logging for PID tuning and flight analysis.

**Before**
```
No structured data export for analysis
```

**After**
```
State Bus ──┬──► Logger ──► /sd/tlog.csv or /tmp/tlog.csv
Sensor Bus ─┤      (25Hz)
Thrust Bus ─┤
Position Target Bus ─┘
```

**Implementation**
- Subscribes to sensor, state, thrust, and position target buses
- Writes CSV at 25Hz with all flight data
- Storage selected at runtime via `hive_file_mount_available()`:
  - Prefers `/sd` (Crazyflie with SD card deck, build with `ENABLE_SD=1`)
  - Falls back to `/tmp` (Webots simulation)
  - Exits gracefully if no storage available
- Filename: `tlog.csv` (8.3 compatible for SD card)
- Runs at LOW priority, TEMPORARY restart (not flight-critical)
- Flushes to disk every second (25 samples)

**CSV columns**
- `time_ms`: Timestamp since flight start
- `roll,pitch,yaw`: Attitude angles (rad)
- `roll_rate,pitch_rate,yaw_rate`: Angular rates (rad/s)
- `x,y,altitude`: Position (m)
- `vx,vy,vz`: Velocities (m/s)
- `thrust`: Thrust command (normalized)
- `target_x,target_y,target_z,target_yaw`: Position targets
- `gyro_x,gyro_y,gyro_z`: Raw gyro (rad/s)
- `accel_x,accel_y,accel_z`: Raw accel (m/s^2)

**Usage**
```bash
# Run simulation (make now auto-installs to Webots)
make
webots worlds/hover_test.wbt

# Analyze PID performance
python3 tools/analyze_pid.py /tmp/tlog.csv

# Visualize telemetry (6-panel plot)
python3 tools/plot_telemetry.py /tmp/tlog.csv

# Full flight summary with 3D trajectory
python3 tools/plot_flight.py /tmp/tlog.csv
```

**Benefits**
- Data-driven PID tuning (vs blind iteration)
- Visualize oscillations, overshoot, settling time
- Quantitative metrics: rise time, settling time, RMS error
- Compare before/after gain changes
- Non-intrusive (LOW priority doesn't affect control loops)

### Step 11: Battery Actor
Add in-flight battery monitoring with debounced emergency landing.

**Before**
```
Battery voltage logged once at startup, included in telemetry packets.
No autonomic action on low voltage.
```

**After**
```
Battery Actor ──► (2 Hz timer) ──► hal_power_get_battery()
                                       │
                                       v (voltage < 3.0V, 10 consecutive readings)
                    ──► LOW_BATTERY ──► Flight Manager
                                           │
                                           v (ARMED: disarm, FLYING: land)
```

**Implementation**
- 2 Hz sampling via periodic timer (500ms interval)
- Two-tier thresholds matching Bitcraze pm_stm32f4.c:
  - WARNING: 3.2V - log WARN once
  - CRITICAL: 3.0V - 10 consecutive readings (5s debounce)
- On debounced critical: sends NOTIFY_LOW_BATTERY to flight_manager (once)
- Flight manager handles LOW_BATTERY in two states:
  - ARMED: cancel countdown, disarm motors, return to IDLE
  - FLYING: initiate controlled landing (same path as flight timer expiry)
- Handles NOTIFY_RESET from flight_manager (resets debounce state)
- LOW priority, TEMPORARY restart, pool_block=true

**Debounce rationale** - Motor load causes voltage sag of 0.2-0.5V during
aggressive maneuvers. Without debouncing, transient sag would trigger false
critical alerts. 10 consecutive readings at 2 Hz (5 seconds) matches Bitcraze.

**On simulation** - `hal_power_get_battery()` returns 4.2V always, so the
actor runs harmlessly (never triggers). This exercises the code path without
affecting simulation behavior.

**Benefits**
- Prevents crash from depleted battery (controlled landing instead)
- Matches Bitcraze's proven thresholds and debounce timing
- Decoupled from control loop (LOW priority, IPC to flight_manager)
- Flight_manager decides action based on current state (no motor commands from battery actor)

### Step 12 (Future): RC Input / Mode Switching

**Future extensions**
- RC input handling (manual override)
- Takeoff/landing sequences
- Mode switching (hover, land, follow-me, etc.)
- Dynamic waypoint updates via telemetry

---

## Future Extensions

1. **Mission planning** - Load waypoints from file, complex routes
2. **Sensor fusion** - [DONE] Altitude Kalman filter + attitude complementary filter (estimator actor)
3. **Failsafe** - Motor failure detection, emergency landing
4. **Telemetry** - [DONE] Radio telemetry implemented (Crazyflie only)
5. **RC input** - Manual control override
6. **Setpoint actor** - Altitude command generation, mode switching
7. **Bus retention on subscribe** - Late subscribers immediately receive most recent value (useful after supervisor restart). Current behavior: wait for next publish cycle, acceptable for high-frequency buses (< 4ms delay at 250Hz)
8. **SD card logging** - [DONE] High-volume telemetry logging via Micro SD Card Deck (SPI, FAT32). Build with `ENABLE_SD=1`. See `hal/crazyflie-2.1plus/README.md` for details
