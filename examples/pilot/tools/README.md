# Pilot Tools

Tools for debugging, analyzing, and communicating with Crazyflie hardware.

## Debug Tools

### st-trace.sh

SWO trace viewer for viewing debug output from Crazyflie firmware via ST-Link.
Uses a locally built stlink-tools (system version 1.8.0 has bugs with SWO).

```bash
# View SWO output with 30s timeout
./st-trace.sh -t 30

# No timeout (Ctrl-C to stop)
./st-trace.sh -t 0

# Default 70s timeout (enough for full bringup test)
./st-trace.sh
```

**Building stlink-tools** (if not already built):
```bash
cd $HIVE_ROOT/local/stlink
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make && make install
```

## Telemetry Tools

Python scripts for analyzing flight telemetry, tuning PID controllers, and communicating with Crazyflie hardware.

## Requirements

- Python 3
- numpy
- matplotlib
- cflib (for ground_station.py only - `pip install cflib`)

## Tools

### analyze_pid.py

Analyze PID controller performance from telemetry data. Computes metrics for altitude and position controllers:

- Overshoot percentage
- Rise time (10% to 90%)
- Settling time (within 5% of target)
- Steady-state error
- RMS tracking error

```bash
# Analyze all controllers
python3 analyze_pid.py /tmp/pilot_telemetry.csv

# Altitude controller only
python3 analyze_pid.py /tmp/pilot_telemetry.csv --altitude

# Position controller only
python3 analyze_pid.py /tmp/pilot_telemetry.csv --position

# Analyze specific time window (first 10 seconds)
python3 analyze_pid.py /tmp/pilot_telemetry.csv --time 0 10

# Output as JSON (for scripting)
python3 analyze_pid.py /tmp/pilot_telemetry.csv --json
```

### plot_telemetry.py

Generate 6-panel telemetry plots showing:

- Altitude response
- Vertical velocity & thrust
- Attitude angles (roll, pitch, yaw)
- Angular rates
- Horizontal position (X, Y)
- Raw accelerometer data

```bash
# Show interactive plot
python3 plot_telemetry.py /tmp/pilot_telemetry.csv

# Save to file
python3 plot_telemetry.py /tmp/pilot_telemetry.csv -o output.png

# Limit time range
python3 plot_telemetry.py /tmp/pilot_telemetry.csv --xlim 0 20

# Plot only altitude response
python3 plot_telemetry.py /tmp/pilot_telemetry.csv --altitude

# Plot only position response
python3 plot_telemetry.py /tmp/pilot_telemetry.csv --position

# Custom title
python3 plot_telemetry.py /tmp/pilot_telemetry.csv --title "Test Flight 1"
```

### plot_flight.py

Generate full flight summary with 3D trajectory visualization:

- 3D trajectory plot
- Altitude profile
- Horizontal position (XY)
- Attitude angles
- Thrust command
- Velocities
- Flight statistics

```bash
# Show interactive plot with stats
python3 plot_flight.py /tmp/pilot_telemetry.csv

# Save to file
python3 plot_flight.py /tmp/pilot_telemetry.csv -o flight_report.png

# Print statistics only (no plot)
python3 plot_flight.py /tmp/pilot_telemetry.csv --stats

# Custom title
python3 plot_flight.py /tmp/pilot_telemetry.csv --title "Waypoint Test"
```

### analyze_hover.py

Analyze hover stability from telemetry data. Detects hover phases and computes metrics for filter tuning (KF vs LPF comparison):

- Altitude standard deviation
- Vertical velocity noise
- Maximum tilt angle
- Position drift

```bash
# Analyze hover phases
python3 analyze_hover.py /tmp/pilot_telemetry.csv

# Compare two flights (e.g., KF vs LPF)
python3 analyze_hover.py kf_flight.csv lpf_flight.csv --compare

# Analyze specific time window
python3 analyze_hover.py /tmp/pilot_telemetry.csv --time 5 15

# Output as JSON
python3 analyze_hover.py /tmp/pilot_telemetry.csv --json

# Adjust hover detection tolerance
python3 analyze_hover.py /tmp/pilot_telemetry.csv --tolerance 0.3
```

### flight_debug.py

Debug flight telemetry - show timeline and detect issues:

- Failed takeoff detection
- Crash/flip detection (tilt > 45deg)
- Position drift analysis
- Thrust saturation warnings
- Stable hover period identification

```bash
# Quick flight analysis
python3 flight_debug.py /tmp/pilot_telemetry.csv

# Verbose timeline (0.5s intervals)
python3 flight_debug.py /tmp/pilot_telemetry.csv --verbose

# Analyze specific time window
python3 flight_debug.py /tmp/pilot_telemetry.csv --time 0 10

# Custom timeline interval
python3 flight_debug.py /tmp/pilot_telemetry.csv --interval 1.0
```

### ground_station.py

Receive real-time telemetry and download flight logs from Crazyflie hardware via Crazyradio 2.0. Decodes binary telemetry packets (attitude/rates and position/altitude) and optionally logs to CSV.

```bash
# Display real-time telemetry to stdout
python3 ground_station.py

# Log telemetry to CSV file
python3 ground_station.py -o flight.csv

# Quiet mode (log only, no console output)
python3 ground_station.py -o flight.csv --quiet

# Download binary flight log from drone flash storage
python3 ground_station.py --download-log log.bin

# Use custom radio URI (default: radio://0/80/2M)
python3 ground_station.py --uri radio://0/80/2M
```

**Telemetry packet types**
- Attitude (0x01): timestamp, gyro XYZ, roll/pitch/yaw
- Position (0x02): timestamp, altitude, vz, vx, vy, thrust, battery_mv

**Log download** - Sends CMD_REQUEST_LOG command to drone, receives binary log chunks, and saves to file. Use with `tools/decode_log.py` (in hive root) to decode the binary log format.

## PID Tuning Workflow

### Webots Simulation

1. Run a flight test in Webots (telemetry logged to `/tmp/pilot_telemetry.csv`)

2. Analyze current performance:
   ```bash
   python3 tools/analyze_pid.py /tmp/pilot_telemetry.csv
   ```

3. Visualize the response:
   ```bash
   python3 tools/plot_telemetry.py /tmp/pilot_telemetry.csv --xlim 0 15
   ```

4. Adjust gains in `hal/*/hal_config.h` (altitude) or `config.h` (position)

5. Rebuild and test:
   ```bash
   make && make install
   # Reset Webots simulation
   ```

6. Repeat until satisfied with response characteristics

### Crazyflie Hardware

1. Flash the drone and connect Crazyradio 2.0

2. Start telemetry logging:
   ```bash
   python3 tools/ground_station.py -o flight.csv
   ```

3. Perform flight test, then Ctrl+C to stop logging

4. Analyze and visualize (same as Webots workflow):
   ```bash
   python3 tools/analyze_pid.py flight.csv
   python3 tools/plot_telemetry.py flight.csv
   ```

5. Alternatively, download the onboard flash log after flight:
   ```bash
   python3 tools/ground_station.py --download-log flight.bin
   ```

## Telemetry CSV Format

The telemetry logger produces CSV files with these columns:

| Column | Description |
|--------|-------------|
| time_ms | Timestamp in milliseconds |
| roll, pitch, yaw | Attitude angles (radians) |
| roll_rate, pitch_rate, yaw_rate | Angular rates (rad/s) |
| x, y, altitude | Position (meters) |
| vx, vy, vz | Velocity (m/s) |
| thrust | Thrust command (0-1) |
| target_x, target_y, target_z, target_yaw | Waypoint targets |
| gyro_x, gyro_y, gyro_z | Raw gyroscope (rad/s) |
| accel_x, accel_y, accel_z | Raw accelerometer (m/s^2) |
