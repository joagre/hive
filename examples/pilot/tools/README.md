# Pilot Telemetry Tools

Python scripts for analyzing flight telemetry and tuning PID controllers.

## Requirements

- Python 3
- numpy
- matplotlib

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

## PID Tuning Workflow

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
| accel_x, accel_y, accel_z | Raw accelerometer (m/s²) |
