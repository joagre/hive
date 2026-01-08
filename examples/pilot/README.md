# Pilot Example

Quadcopter hover example using hive actor runtime with Webots simulator.

## What it does

Demonstrates altitude-hold hover control with a Crazyflie quadcopter:

1. Main loop reads IMU and GPS sensors from Webots
2. Publishes sensor data to a bus
3. Calls `hive_step()` to run the attitude actor
4. Attitude actor runs PID controllers for roll, pitch, yaw, and altitude
5. Motor commands are applied to Webots motors

The drone rises to 1.0m altitude and holds position.

## Prerequisites

- Webots installed (https://cyberbotics.com/)
- hive runtime built: `cd ../.. && make`

## Build and Run

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make
make install
```

Then open `worlds/hover_test.wbt` in Webots and start the simulation.

## Files

- `pilot.c` - Complete hover controller (~260 lines)
- `Makefile` - Build configuration
- `worlds/hover_test.wbt` - Webots world with Crazyflie

## Architecture

The code is organized for portability to real hardware:

```
┌─────────────────────────────────────────────────────────────┐
│                    PORTABLE CONTROL CODE                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ PID Control │  │    Mixer    │  │   Attitude Actor    │  │
│  │  (generic)  │  │ (Crazyflie) │  │ (roll/pitch/yaw/alt)│  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
                            │
                    Platform abstraction
                            │
┌─────────────────────────────────────────────────────────────┐
│                   WEBOTS PLATFORM LAYER                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │ platform_   │  │ platform_   │  │  platform_write_    │  │
│  │ init()      │  │ read_imu()  │  │  motors()           │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

To port to real hardware, replace the platform layer functions.

## Control System

### PID Controllers

| Controller | Kp   | Ki   | Kd    | Purpose |
|------------|------|------|-------|---------|
| Roll rate  | 0.02 | 0    | 0.001 | Stabilize roll |
| Pitch rate | 0.02 | 0    | 0.001 | Stabilize pitch |
| Yaw rate   | 0.02 | 0    | 0.001 | Stabilize yaw |
| Altitude   | 0.3  | 0.05 | 0.15  | Hold 1.0m height |

### Motor Mixer (+ Configuration)

```
        Front
          M1
           │
    M4 ────┼──── M2
           │
          M3
         Rear

M1 = thrust - pitch - yaw
M2 = thrust - roll  + yaw
M3 = thrust + pitch - yaw
M4 = thrust + roll  + yaw
```

Motor velocity signs: M1,M3 negative; M2,M4 positive (for torque cancellation).

## How Webots Controllers Work

Understanding the Webots execution model is crucial for writing controllers.

### Process Model

Webots runs your controller as a **separate process**. When you start a simulation:

1. Webots spawns your executable (`pilot`)
2. Your `main()` runs
3. Webots and your controller communicate via IPC

### The Step Function

The key function is `wb_robot_step(TIME_STEP)`. It synchronizes your controller with the simulator:

```
Your controller                    Webots simulator
───────────────                    ────────────────
main() starts
  │
  ├─► wb_robot_init()  ──────────► "Controller connected"
  │
  ├─► enable sensors
  │
  └─► while loop:
        │
        ├─► wb_robot_step(4) ────► BLOCKS here
        │         │                    │
        │         │                    ▼
        │         │               Simulate 4ms of physics
        │         │               Update sensor values
        │         │               Apply motor commands
        │         │                    │
        │         ◄────────────────────┘ Returns when done
        │
        ├─► read sensors (values from previous step)
        ├─► your logic (hive_step runs actors)
        ├─► set motors (applied on next step)
        │
        └─► loop back to wb_robot_step()
```

### Key Points

1. **`wb_robot_step()` blocks** until Webots advances simulation by TIME_STEP milliseconds
2. **Sensors read values from the previous step** (there's a 1-step delay)
3. **Motor commands take effect on the next step** (another 1-step delay)
4. **Return value of -1** means simulation ended (user pressed stop)
5. **If your code is slow**, simulation slows down to wait for you

### Timing

With `TIME_STEP=4`:
- Your loop runs **250 times per simulated second**
- If your code takes <4ms real time, simulation runs at real-time speed
- If your code takes >4ms, simulation runs slower than real-time

### Integration with Hive Runtime

We use `hive_step()` instead of `hive_run()`:

```c
while (wb_robot_step(TIME_STEP) != -1) {
    // 1. Read sensors from Webots, publish to bus
    platform_read_imu(&imu);
    hive_bus_publish(g_imu_bus, &imu, sizeof(imu));

    // 2. Run each ready actor exactly once
    hive_step();

    // 3. Apply motor commands to Webots
    platform_write_motors();
}
```

This works because:
- **Webots controls time**, not the hive scheduler
- **No hive timers needed** - the loop rate is determined by `wb_robot_step()`
- **Actors yield after processing** - `hive_step()` runs each once and returns
- **Deterministic** - same inputs produce same outputs (no timing races)
