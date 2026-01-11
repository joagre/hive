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
Feedback is via LED blinks (no serial output):
- 1 blink = starting
- 2 blinks = sensors initialized
- 3 blinks = reading sensor data
- 4 blinks = motors test starting (REMOVE PROPS!)
- Fast blink = success
- Slow blink = failure

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
| MCU | STM32F401CEU6 | - | ARM Cortex-M4, 84 MHz, DSP+FPU |
| IMU | LSM6DSL | SPI2 | 6-axis accel + gyro |
| Magnetometer | LIS2MDL | SPI2 | 3-axis compass |
| Barometer | LPS22HB | SPI2 | Pressure/altitude |
| Bluetooth | SPBTLE-RF | SPI3 | BLE 4.1 for RC (not implemented) |
| Motors | 85x20mm | TIM4 PWM | 3.7V brushed, x4 |

## Specifications

**Flight Controller (STEVAL-FCU001V1):**
- STM32F401CEU6 @ 84 MHz (Cortex-M4F)
- 512 KB Flash, 96 KB RAM
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
| `platform_stm32f4.h/c` | Platform-specific sensor reading and motor control |

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
| `stm32f401_flash.ld` | Memory layout (512K Flash, 96K RAM) |
| `Makefile` | Build libhal.a static library |

### Debug (Optional, Not Implemented)

| File | Description |
|------|-------------|
| `usart1.h/c` | USART1 driver (not compiled - P7 header available on board) |

Note: Serial debug output requires adding `usart1.c` to Makefile, connecting
`syscalls.c` `_write()` to USART, and changing `HIVE_LOG_LEVEL`.

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
PB6  - TIM4_CH1 (M1)
PB7  - TIM4_CH2 (M2)
PB8  - TIM4_CH3 (M3)
PB9  - TIM4_CH4 (M4)
```

### USART1 (Debug Serial - Not Implemented)
```
PA9  - USART1_TX  (directly to P7 header pin 2)
PA10 - USART1_RX  (directly to P7 header pin 3)
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

X-configuration quadcopter:

```
           Front
         M2    M3
     (CW)  \  /  (CCW)
            \/
            /\
     (CCW) /  \ (CW)
         M1    M4
           Rear

Motor mixing:
  M1 = throttle + roll + pitch - yaw
  M2 = throttle + roll - pitch + yaw
  M3 = throttle - roll - pitch - yaw
  M4 = throttle - roll + pitch + yaw
```

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
