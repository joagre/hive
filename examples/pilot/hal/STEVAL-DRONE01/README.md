# STEVAL-DRONE01 Hardware Abstraction Layer

Platform layer for the STMicroelectronics STEVAL-DRONE01 mini drone kit.

This HAL provides drivers for STM32F401, enabling the pilot example to run on real hardware instead of the Webots simulator.

## Quick Start

```bash
# Build full firmware (from examples/pilot/)
cd ..
make -f Makefile.STEVAL-DRONE01
make -f Makefile.STEVAL-DRONE01 flash

# Or build just the HAL library (from this directory)
make          # Build libhal.a
make clean    # Remove build artifacts
```

## Hardware Validation Tests

Before running the full pilot firmware, use the standalone test programs in
`vendor/` to verify hardware connectivity.

```bash
cd vendor
make                    # Build sensor_motor_test (default)
make TEST=main          # Build WHO_AM_I register test
make flash              # Flash to device (press reset after)
make clean              # Clean build
```

The `sensor_motor_test` reads all sensors and spins motors briefly.
Feedback is via LED blinks and serial output (115200 baud on P7 header):
- 1 blink = starting
- 2 blinks = sensors initialized
- 3 blinks = reading sensor data (prints values to serial)
- 4 blinks = motors test starting (REMOVE PROPS!)
- Fast blink = success
- Slow blink = failure

Serial output shows sensor initialization status and readings:
```
========================================
STEVAL-FCU001V1 Sensor + Motor Test
========================================

Starting... (1 blink)
Initializing SPI bus...
  SPI bus OK
Initializing LSM6DSL accelerometer...
  Accelerometer OK
...
Sample 1:
  Accel: X=10 Y=-5 Z=1002 mg
  Gyro:  X=100 Y=-50 Z=20 mdps
  Mag:   X=150 Y=-80 Z=400 mGauss
  Press: 1013.2 hPa
...
```

## Integration with Pilot

This HAL links with `pilot.c` and the hive runtime. The platform API
(`platform_stm32f4.h`) provides the same interface as the Webots HAL.

Build the complete firmware from `examples/pilot/`:
```bash
make -f Makefile.STEVAL-DRONE01
```

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F401CCU6 | - | ARM Cortex-M4, 84 MHz, DSP+FPU |
| IMU | LSM6DSL | SPI2 | 6-axis accel + gyro |
| Magnetometer | LIS2MDL | SPI2 | 3-axis compass |
| Barometer | LPS22HB | SPI2 | Pressure/altitude |
| Bluetooth | SPBTLE-RF | SPI3 | BLE 4.1 for RC (not implemented) |
| Motors | 85x20mm | TIM4 PWM | 3.7V brushed, x4 |

## Specifications

**Flight Controller (STEVAL-FCU001V1):**
- STM32F401CCU6 @ 84 MHz (Cortex-M4F)
- 256 KB Flash, 64 KB RAM
- Hardware FPU for fast sensor fusion

**Sensors (all on SPI2):**
- LSM6DSL: +/-2/4/8/16g accel, +/-250/500/1000/2000 dps gyro
- LIS2MDL: +/-50 gauss magnetometer
- LPS22HB: 260-1260 hPa barometer

**Motors:**
- Type: Brushed DC, 85x20mm
- Voltage: 3.7V (direct LiPo drive)
- 2x CW, 2x CCW rotation
- Propellers: 65mm

**Battery:**
- LiPo 3.7V / 600mAh
- Max discharge: 30C (18A)

## Architecture

```
+---------------------------------------------------------------+
|                   pilot.c + hive runtime                      |
|                (Actor-based Flight Controller)                |
|         Sensor fusion in fusion/complementary_filter.c        |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                       hal_stm32.c                             |
|         (HAL Interface: hal_read_sensors, hal_write_torque)   |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                   platform_stm32f4.c                          |
|        (Platform layer using ST HAL + BSP drivers)            |
|  +----------+  +----------+  +----------+  +----------+       |
|  |  LSM6DSL |  |  LIS2MDL |  |  LPS22HB |  |  motors  |       |
|  | (BSP/SPI)|  | (BSP/SPI)|  | (BSP/SPI)|  |  (TIM4)  |       |
|  +----------+  +----------+  +----------+  +----------+       |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                    vendor/ (ST Drivers)                       |
|  +------------------+  +------------------+  +-------------+  |
|  | STM32F4xx_HAL    |  | BSP/STEVAL_FCU001|  | BSP/sensors |  |
|  | (HAL framework)  |  | (board support)  |  | (lsm6dsl,..)| |
|  +------------------+  +------------------+  +-------------+  |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                    Low-Level HAL                              |
|  +----------+  +----------+  +----------+  +----------+       |
|  |  system  |  |   tim4   |  |  motors  |  |  linker  |       |
|  |  config  |  |  (PWM)   |  |  (ctrl)  |  |   .ld    |       |
|  +----------+  +----------+  +----------+  +----------+       |
+---------------------------------------------------------------+
```

## File Overview

### HAL Layer

| File | Description |
|------|-------------|
| `hal_stm32.c` | HAL interface (hal_read_sensors, hal_write_torque) |
| `hal_config.h` | Platform-specific PID gains and thrust |
| `platform_stm32f4.h/c` | Platform-specific sensor reading and motor control |

### Configuration

| Setting | Location | Description |
|---------|----------|-------------|
| PID gains, thrust | `hal_config.h` | Platform-specific tuning |
| Actors, buses, pools | `hive_config.mk` | Shared (pilot-determined) |
| Stack sizes | `Makefile.STEVAL-DRONE01` | Platform-specific (RAM-dependent) |

### Motor Drivers

| File | Description |
|------|-------------|
| `motors.h/c` | Motor control abstraction (arm, disarm, set speeds) |
| `tim4.h/c` | TIM4 PWM driver for motor control |

### System Layer

| File | Description |
|------|-------------|
| `system_config.h/c` | Peripheral clock enables (TIM4, USART, etc.) |
| `gpio_config.h/c` | GPIO pin configuration helpers |
| `syscalls.c` | Newlib stubs for bare-metal (_read, _write, _sbrk) |
| `startup_stm32f401.s` | Vector table, Reset_Handler, C runtime init |
| `stm32f401_flash.ld` | Memory layout (256K Flash, 64K RAM) |
| `Makefile` | Build libhal.a static library |

### Debug Serial

| File | Description |
|------|-------------|
| `usart1.h/c` | USART1 driver for debug output via P7 header |

Serial debug is enabled by default. Output goes to USART1 at 115200 baud.
Connect a USB-to-serial adapter to the P7 header (pin 2 = TX, pin 3 = RX).

To disable serial debug for production, change `HIVE_LOG_LEVEL` in both
Makefiles from `HIVE_LOG_LEVEL_INFO` to `HIVE_LOG_LEVEL_NONE`.

### Vendor Drivers (vendor/)

| Directory | Description |
|-----------|-------------|
| `Drivers/STM32F4xx_HAL_Driver/` | ST HAL framework |
| `Drivers/CMSIS/` | ARM CMSIS headers |
| `Drivers/BSP/STEVAL_FCU001_V1/` | Board support package |
| `Drivers/BSP/lsm6dsl/` | LSM6DSL IMU driver |
| `Drivers/BSP/lis2mdl/` | LIS2MDL magnetometer driver |
| `Drivers/BSP/lps22hb/` | LPS22HB barometer driver |
| `sensor_motor_test.c` | Hardware validation test |
| `main.c` | WHO_AM_I register test |
| `Makefile` | Build hardware tests |

## Pin Mapping

### SPI2 (All Sensors via BSP)
```
PB13 - SPI2_SCK
PB14 - SPI2_MISO
PB15 - SPI2_MOSI
PA8  - LSM6DSL_CS
PB12 - LIS2MDL_CS
PB10 - LPS22HB_CS
```

### TIM4 PWM (Motors)
```
PB6  - TIM4_CH1 (M1, rear-left, CCW)   → Connector P1
PB7  - TIM4_CH2 (M2, front-left, CW)   → Connector P2
PB8  - TIM4_CH3 (M3, front-right, CCW) → Connector P4
PB9  - TIM4_CH4 (M4, rear-right, CW)   → Connector P5
```

Note: Board connectors are labeled P1, P2, P4, P5 (no P3).

### USART1 (Debug Serial)
```
PA9  - USART1_TX  (P7 header pin 2)
PA10 - USART1_RX  (P7 header pin 3)
```

### Misc
```
PB5  - LED (LD1)
PA0  - User button
```

## Differences from Webots Simulation

| Feature | Webots | STEVAL-DRONE01 |
|---------|--------|----------------|
| Position | GPS (perfect) | None (need external tracking) |
| Attitude | Synthesized from inertial unit | Raw IMU -> complementary filter |
| Altitude | GPS Z | Barometer (relative only) |
| Heading | Synthesized yaw | Magnetometer |
| Timing | Simulation step | Real-time (hive scheduler) |
| Fusion | Same portable code (fusion/) | Same portable code (fusion/) |

## Building

### Requirements

```bash
# ARM GCC toolchain
sudo apt install gcc-arm-none-eabi

# ST-Link tools (for flashing)
sudo apt install stlink-tools

# OpenOCD (optional, for debugging)
sudo apt install openocd
```

### Build Commands

```bash
# Build HAL library only (from this directory)
make          # Build libhal.a
make clean    # Remove build artifacts
make help     # Show targets

# Build full firmware (from examples/pilot/)
make -f Makefile.STEVAL-DRONE01           # Build pilot_STEVAL-DRONE01.elf
make -f Makefile.STEVAL-DRONE01 flash     # Flash to device
make -f Makefile.STEVAL-DRONE01 debug     # Start GDB session
```

## HAL API

The HAL provides a platform-independent interface used by pilot actors:

```c
#include "hal/hal.h"

// Initialization
hal_init();         // Initialize hardware
hal_calibrate();    // Calibrate sensors (keep drone still and level)
hal_arm();          // Arm motors

// Sensor reading (called by sensor_actor)
sensor_data_t sensors;
hal_read_sensors(&sensors);   // Raw accel, gyro, mag, baro (no GPS)

// Motor output (called by motor_actor)
torque_cmd_t cmd = {.thrust = 0.5f, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f};
hal_write_torque(&cmd);       // HAL applies mixer internally

// Shutdown
hal_disarm();
```

Key differences from Webots:
- No GPS - `sensors.gps_valid` is always false
- Sensor fusion done in portable code (`fusion/complementary_filter.c`)
- Altitude is relative (barometer-based), not absolute

## Motor Layout

X-configuration quadcopter with brushed DC motors:

```
             Front
         M2(CW)    M3(CCW)
          P2  \    /  P4
               \  /
                \/
                /\
               /  \
          P1  /    \  P5
         M1(CCW)  M4(CW)
             Rear
```

**Motor to connector mapping:**

| Motor | Position | Rotation | Connector | Pin |
|-------|----------|----------|-----------|-----|
| M1 | rear-left | CCW | P1 | PB6 (TIM4_CH1) |
| M2 | front-left | CW | P2 | PB7 (TIM4_CH2) |
| M3 | front-right | CCW | P4 | PB8 (TIM4_CH3) |
| M4 | rear-right | CW | P5 | PB9 (TIM4_CH4) |

**Motor mixing (in hal_stm32.c):**
```
M1 = thrust - roll - pitch + yaw
M2 = thrust - roll + pitch - yaw
M3 = thrust + roll + pitch + yaw
M4 = thrust + roll - pitch - yaw
```

**To reverse motor direction:** Flip the 2-wire motor connector (reverses polarity).

## LED Feedback

The LED on PB5 provides status during initialization:

| Pattern | Meaning |
|---------|---------|
| 1 blink | Starting initialization |
| 2 blinks | Sensors initialized |
| 3 blinks | All hardware initialized |
| Slow continuous blink | Fatal error |

## Flight Profiles

Select flight profile when building:

```bash
make -f Makefile.STEVAL-DRONE01 FLIGHT_PROFILE=1  # FIRST_TEST (default)
make -f Makefile.STEVAL-DRONE01 FLIGHT_PROFILE=2  # ALTITUDE
make -f Makefile.STEVAL-DRONE01 FLIGHT_PROFILE=3  # FULL_3D
```

| Profile | Description | Duration |
|---------|-------------|----------|
| 1 (FIRST_TEST) | Hover at 0.5m, then land | 10s |
| 2 (ALTITUDE) | Altitude changes: 0.5m -> 1.0m -> 1.5m | 40s |
| 3 (FULL_3D) | 3D waypoints (no GPS, so altitude-only) | 60s |

## Calibration

Before flight, `hal_calibrate()` performs:

1. **Gyro bias** - Average 500 samples while stationary
2. **Barometer reference** - Average 50 samples for ground level

**Important:** Keep the drone still and level during calibration!

## Flashing Notes

After `st-flash write`, the board may not auto-reset. Press the reset button
manually or run `st-flash reset` after flashing.

## Resources

- [STEVAL-DRONE01 Product Page](https://www.st.com/en/evaluation-tools/steval-drone01.html)
- [STSW-FCU001 Firmware](https://www.st.com/en/embedded-software/stsw-fcu001.html)
- [LSM6DSL Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsl.pdf)
- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
- [LPS22HB Datasheet](https://www.st.com/resource/en/datasheet/lps22hb.pdf)
