# Pilot Example

Minimal quadcopter example using hive actor runtime with Webots simulator.

## What it does

1. Main loop reads IMU sensors from Webots
2. Publishes sensor data to a bus
3. Calls `hive_step()` to run actors
4. Sensor actor reads from bus and prints roll/pitch/yaw every second

## Prerequisites

- Webots installed (https://cyberbotics.com/)
- hive runtime built: `cd ../.. && make`

## Build and Run

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots.

## Files

- `pilot.c` - Main loop and sensor actor (~90 lines)
- `Makefile` - Build configuration
- `worlds/hover_test.wbt` - Webots world with Crazyflie
