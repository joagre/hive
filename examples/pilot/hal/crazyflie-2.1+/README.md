# Crazyflie 2.1+ Hardware Abstraction Layer

Platform layer for the Bitcraze Crazyflie 2.1+ nano quadcopter.

This HAL provides drivers for STM32F405, enabling the pilot example to run on real hardware instead of the Webots simulator.

## Quick Start

```bash
# Build full firmware (from examples/pilot/)
cd ..
make -f Makefile.crazyflie-2.1+
make -f Makefile.crazyflie-2.1+ flash

# Or build just the HAL library (from this directory)
make          # Build libhal.a
make clean    # Remove build artifacts
```

## Integration with Pilot

This HAL links with `pilot.c` and the hive runtime. The platform API
(`platform_crazyflie.h`) provides the same interface as the Webots HAL.

Build the complete firmware from `examples/pilot/`:
```bash
make -f Makefile.crazyflie-2.1+
```

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F405RG | - | ARM Cortex-M4, 168 MHz, DSP+FPU |
| IMU | BMI088 | I2C3 | 6-axis accel + gyro (addr: 0x18/0x68) |
| Barometer | BMP388 | I2C3 | Pressure/altitude (addr: 0x77) |
| Flow sensor | PMW3901 | SPI1 | Optical flow (Flow deck v2) |
| ToF sensor | VL53L1x | I2C3 | Height measurement (Flow deck v2) |
| Motors | 7x16mm | TIM2 PWM | Brushed coreless, x4 |
| Radio | nRF51822 | USART6 | Syslink to Crazyradio PA |
| LED | Blue | PD2 | Status indicator |

## Specifications

**Flight Controller (Crazyflie 2.1+):**
- STM32F405RG @ 168 MHz (Cortex-M4F)
- 1 MB Flash, 192 KB RAM + 64 KB CCM
- Hardware FPU for fast sensor fusion
- 8 MHz HSE crystal

**Sensors:**
- BMI088: 16-bit accel (+/-24g) + 16-bit gyro (+/-2000 dps) on I2C3 (addr: 0x18/0x68)
- BMP388: 24-bit barometer on I2C3 (addr: 0x77)

**Optional Flow Deck v2:**
- PMW3901: Optical flow sensor (80x80 pixels)
- VL53L1x: Time-of-Flight ranging (up to 4m)

**Motors:**
- Type: Brushed coreless, 7x16mm
- Voltage: 3.0V nominal (single-cell LiPo)
- 2x CCW (M1, M3), 2x CW (M2, M4)
- Propellers: 45mm

**Battery:**
- LiPo 3.7V / 250mAh
- Weight: ~27g (without Flow deck)

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
|                     hal_crazyflie.c                           |
|         (HAL Interface: hal_read_sensors, hal_write_torque)   |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                  platform_crazyflie.c                         |
|        (Platform layer with I2C/SPI callbacks)                |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                    Vendor Drivers                             |
|  +----------+  +----------+  +----------+  +----------+       |
|  | BMI08x   |  |   BMP3   |  | PMW3901  |  | VL53L1x  |       |
|  |  Bosch   |  |   Bosch  |  | Bitcraze |  |    ST    |       |
|  +----------+  +----------+  +----------+  +----------+       |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                         CMSIS                                 |
|  +------------------+  +------------------+                   |
|  | stm32f4xx.h      |  | core_cm4.h       |                   |
|  | (register defs)  |  | (Cortex-M4)      |                   |
|  +------------------+  +------------------+                   |
+---------------------------------------------------------------+
```

## File Overview

### HAL Layer

| File | Description |
|------|-------------|
| `hal_crazyflie.c` | HAL interface (hal_read_sensors, hal_write_torque) |
| `hal_radio.c` | Radio communication via syslink to nRF51822 |
| `hal_config.h` | Platform-specific PID gains and thrust |
| `platform_crazyflie.h/c` | Platform-specific sensor reading and motor control |

### Configuration

| Setting | Location | Description |
|---------|----------|-------------|
| PID gains, thrust | `hal_config.h` | Platform-specific tuning |
| Actors, buses, pools | `hive_config.mk` | Shared (pilot-determined) |
| Stack sizes | `Makefile.crazyflie-2.1+` | Platform-specific (RAM-dependent) |

### Vendor Drivers (vendor/)

Uses official vendor APIs for reliability. All under permissive licenses
(BSD-3-Clause or MIT). See `THIRD_PARTY_LICENSES.md` for details.

| Directory | Driver | License | Description |
|-----------|--------|---------|-------------|
| `vendor/bosch/bmi08x/` | BMI08x API | BSD-3-Clause | BMI088 IMU (accel + gyro) |
| `vendor/bosch/bmp3/` | BMP3 API | BSD-3-Clause | BMP388 barometer |
| `vendor/st/vl53l1x/` | VL53L1x ULD | BSD-3-Clause | ToF ranging sensor |
| `vendor/bitcraze/pmw3901/` | PMW3901 | MIT | Optical flow sensor |

### Motor Driver

| File | Description |
|------|-------------|
| `motors.h/c` | Motor control abstraction (arm, disarm, set speeds) |

### System Layer

| File | Description |
|------|-------------|
| `syscalls.c` | Newlib stubs for bare-metal (_read, _write, _sbrk) |
| `startup_stm32f405.s` | Vector table, Reset_Handler, C runtime init |
| `stm32f405_flash.ld` | Memory layout (1 MB Flash, 192 KB RAM) |
| `Makefile` | Build libhal.a static library |

### CMSIS Headers (vendor/CMSIS/)

| File | Description |
|------|-------------|
| `Device/ST/STM32F4xx/Include/stm32f4xx.h` | STM32F4 peripheral register definitions |
| `Include/core_cm4.h` | ARM Cortex-M4 core definitions |

## Pin Mapping

### I2C3 (BMI088, BMP388, VL53L1x)
```
PA8  - I2C3_SCL
PC9  - I2C3_SDA

BMI088 Accelerometer: 0x18
BMI088 Gyroscope: 0x68
BMP388 Barometer: 0x77
VL53L1x ToF: 0x29
```

### SPI1 (PMW3901 Flow Sensor)
```
PA5  - SPI1_SCK
PA6  - SPI1_MISO
PA7  - SPI1_MOSI
PB4  - PMW3901_CS (Flow deck v2, DECK_GPIO_IO3)
```

### TIM2 PWM (Motors)
```
PA0  - TIM2_CH1 (M1, front-left, CCW)
PA1  - TIM2_CH2 (M2, front-right, CW)
PA2  - TIM2_CH3 (M3, rear-right, CCW)
PA3  - TIM2_CH4 (M4, rear-left, CW)
```

### USART6 (nRF51 Syslink)
```
PC6  - USART6_TX
PC7  - USART6_RX
PA4  - TXEN (flow control from nRF51)
```

### Misc
```
PD2  - Blue LED
```

## Differences from Webots Simulation

| Feature | Webots | Crazyflie 2.1+ |
|---------|--------|----------------|
| Position | GPS (perfect) | Flow deck (relative) or none |
| Attitude | Synthesized from inertial unit | Raw IMU -> complementary filter |
| Altitude | GPS Z | Barometer or ToF (relative only) |
| Heading | Synthesized yaw | Gyro integration (no magnetometer) |
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

### Debug Adapter

Flashing requires a debug adapter connected to the Crazyflie's 0.05" debug header:
- Bitcraze debug adapter (recommended)
- Generic SWD adapter (ST-Link, J-Link)

### Build Commands

```bash
# Build HAL library only (from this directory)
make          # Build libhal.a
make clean    # Remove build artifacts

# Build full firmware (from examples/pilot/)
make -f Makefile.crazyflie-2.1+           # Build pilot_crazyflie-2.1+.elf
make -f Makefile.crazyflie-2.1+ flash     # Flash to device
make -f Makefile.crazyflie-2.1+ clean     # Clean build
```

## HAL API

The HAL provides a platform-independent interface used by pilot actors:

```c
#include "hal/hal.h"

// Initialization
hal_init();         // Initialize hardware
hal_self_test();    // Verify sensors respond (returns true if OK)
hal_calibrate();    // Calibrate sensors (keep drone still and level)
hal_arm();          // Arm motors

// Sensor reading (called by sensor_actor)
sensor_data_t sensors;
hal_read_sensors(&sensors);   // Raw accel, gyro, baro (+ flow if deck present)

// Motor output (called by motor_actor)
torque_cmd_t cmd = {.thrust = 0.5f, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f};
hal_write_torque(&cmd);       // HAL applies mixer internally

// Shutdown
hal_disarm();
hal_cleanup();    // Release hardware resources
```

Key differences from Webots:
- No magnetometer - heading uses gyro integration only
- Position requires Flow deck (otherwise unavailable)
- Altitude is relative (barometer or ToF), not absolute

## Motor Layout

X-configuration quadcopter with brushed coreless motors:

```
             Front
         M1(CCW)  M2(CW)
             +--+
             |  |
             +--+
         M4(CW)  M3(CCW)
             Rear
```

**Motor to pin mapping:**

| Motor | Position | Rotation | Pin |
|-------|----------|----------|-----|
| M1 | front-left | CCW | PA0 (TIM2_CH1) |
| M2 | front-right | CW | PA1 (TIM2_CH2) |
| M3 | rear-right | CCW | PA2 (TIM2_CH3) |
| M4 | rear-left | CW | PA3 (TIM2_CH4) |

**Motor mixing (in hal_crazyflie.c):**
```
M1 = thrust - roll + pitch + yaw
M2 = thrust + roll + pitch - yaw
M3 = thrust + roll - pitch + yaw
M4 = thrust - roll - pitch - yaw
```

**To reverse motor direction:** Swap the two motor wires at the connector.

## Self-Test

After initialization, `hal_self_test()` verifies all sensors respond:

1. **Required:** BMI088 (gyro/accel), BMP388 (barometer)
2. **Optional:** Flow deck sensors (PMW3901, VL53L1x) - only tested if deck was detected

Returns `true` if all required sensors pass, `false` otherwise.

## Calibration

Before flight, `hal_calibrate()` performs:

1. **Level check** - Verify accelerometer reads approximately level (warns if tilted >6°)
2. **Gyro bias** - Average 500 samples while stationary
3. **Barometer reference** - Average 50 samples for ground level
4. **Height offset** (Flow deck only) - Measure ground clearance for accurate height

**Important:** Keep the drone still and level on a flat surface during calibration!

## LED Feedback

The blue LED on PD2 provides status during startup. Progress blinks (slow, 200ms)
indicate successful stages, while error blinks (fast, 100ms) indicate failures:

| Pattern | Stage | Meaning |
|---------|-------|---------|
| 1 blink | init | Starting initialization |
| 2 blinks | init | IMU initialized |
| 3 blinks | init | All hardware initialized |
| 3 fast blinks | init | IMU initialization failed |
| 4 fast blinks | init | Barometer initialization failed |
| 5 fast blinks | init | Motor initialization failed |
| 6 fast blinks | self-test | IMU self-test failed |
| 7 fast blinks | self-test | Barometer self-test failed |
| 8 fast blinks | self-test | Flow sensor self-test failed |
| 9 fast blinks | self-test | ToF sensor self-test failed |
| 10 fast blinks | calibrate | Level warning (drone tilted >6 deg) |
| Slow blink | calibrate | Calibration in progress (keep still!) |
| LED off | calibrate | Calibration complete |
| LED on | armed | Motors armed, ready to fly |
| Slow continuous blink | any | Fatal error |

## Flight Profiles

Select flight profile when building:

```bash
make -f Makefile.crazyflie-2.1+ FLIGHT_PROFILE=1  # FIRST_TEST (default)
make -f Makefile.crazyflie-2.1+ FLIGHT_PROFILE=2  # ALTITUDE
make -f Makefile.crazyflie-2.1+ FLIGHT_PROFILE=3  # FULL_3D (requires Flow deck)
```

| Profile | Description | Duration |
|---------|-------------|----------|
| 1 (FIRST_TEST) | Hover at 0.5m, then land | 10s |
| 2 (ALTITUDE) | Altitude changes: 0.5m -> 1.0m -> 1.5m | 40s |
| 3 (FULL_3D) | 3D waypoints (requires Flow deck) | 60s |

## Flashing Notes

After `st-flash write`, press the reset button on the Crazyflie or run
`st-flash reset` to start the firmware.

## Resources

- [Crazyflie 2.1 Product Page](https://www.bitcraze.io/products/crazyflie-2-1/)
- [Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BMP388 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
- [PMW3901 Datasheet](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [VL53L1x Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
