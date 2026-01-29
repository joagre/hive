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

## Reference Implementation

**The Bitcraze crazyflie-firmware repository is the normative truth for this platform.**

When implementing sensor drivers, initialization sequences, or hardware behavior,
always refer to the Bitcraze firmware as the authoritative reference:
- https://github.com/bitcraze/crazyflie-firmware (STM32 main firmware)
- https://github.com/bitcraze/crazyflie2-nrf-firmware (NRF51 power management firmware)

Key learnings from the Bitcraze implementation:
- Sensor configuration (BMI088, BMP388, VL53L1x, PMW3901) matches their settings
- The VL53L1x I2C address may be reprogrammed from 0x29 to 0x6A
- Deck EEPROM detection uses the NRF51 1-Wire bus, not STM32 GPIO
- Syslink communication at 1Mbaud requires DMA for reliable operation
- Motor pins use TIM2 (M1, M2, M3) and TIM4 (M4), not all on TIM2

## Design Decisions: Polling vs DMA/Interrupts

This HAL uses a mix of polling and DMA, chosen based on timing requirements:

| Peripheral | Bitcraze Reference | This HAL | Rationale |
|------------|-------------------|----------|-----------|
| **Radio (UART 1Mbaud)** | DMA | DMA | Required - polling causes overrun at 1Mbaud |
| **Motors (PWM)** | Timer hardware | Timer hardware | Same - no CPU involvement for PWM output |
| **I2C sensors** | Interrupt + DMA | Polling | Acceptable - see analysis below |
| **SPI sensors** | Varies | Polling | Acceptable - short transactions |

### I2C Polling Analysis

The sensor actor is the timer-driven "heartbeat" that publishes to the bus and
wakes up the control pipeline. It runs at high priority, not in the background.
This means I2C polling time is part of the critical path.

**Timing budget at 500Hz control rate:**
- Total cycle time: 2.0 ms
- I2C sensor read (BMI088 + BMP388): ~0.5 ms (25% of budget)
- Remaining for estimator + controller + mixer: ~1.5 ms

**Why polling is acceptable:**
1. 1.5 ms is sufficient for Kalman filter + PID loops on Cortex-M4F with FPU
2. Simpler code = fewer bugs during initial flight testing
3. No race conditions or interrupt priority issues
4. Easier to debug with deterministic execution

**When to consider DMA/interrupts:**
- Control rate > 1kHz (budget shrinks to 1ms, I2C alone takes half)
- Adding more I2C sensors (flow deck sensors, additional IMUs)
- Profiling shows I2C is the bottleneck

**Current recommendation:** Keep polling for initial flight testing. Optimize
later if profiling shows it's needed.

## Design Decisions: HAL vs Application Responsibilities

This HAL intentionally omits certain features that Bitcraze implements, because
they belong at the application level rather than the HAL:

### Low-Pass Filters

**Bitcraze:** Applies 2nd-order Butterworth filters in the sensor driver (80Hz
gyro cutoff, 30Hz accel cutoff).

**This HAL:** Provides raw sensor data without filtering.

**Rationale:** The pilot firmware uses a Kalman filter for state estimation,
which inherently filters sensor noise as part of its prediction/update cycle.
Adding filters in the HAL would:
- Double-filter the data, adding latency without benefit
- Make the Kalman filter's noise model inaccurate
- Hide sensor characteristics from the estimator

If filtering is needed for a different application (e.g., complementary filter),
it should be implemented in that application, not the HAL.

### Supervisor / Arming State Machine

**Bitcraze:** Complex supervisor module with state machine:
- Pre-flight checks state
- Armed state
- Flying state
- Tumble detection
- Emergency stop
- Commander watchdog (500ms warning, 2000ms shutdown)
- Crash recovery

**This HAL:** Simple arming in `hal_arm()`:
- Checks `initialized` flag
- Checks `calibrated` flag
- Enables motor PWM output

**Rationale:** Flight safety logic belongs in the application layer, not HAL:
- Tumble detection requires attitude estimation (application has this)
- Commander watchdog depends on command source (radio, autonomous, etc.)
- Emergency stop policy is application-specific
- State machine complexity doesn't belong in hardware abstraction

The pilot firmware should implement these checks in its own supervisor actor
if needed. The HAL just provides the primitive: "enable/disable motor output".

### What hal_arm() Checks

| Check | HAL | Application Should Add |
|-------|-----|------------------------|
| Hardware initialized | ✓ | - |
| Sensors calibrated | ✓ | - |
| Attitude valid | - | ✓ (estimator converged) |
| Not tumbled | - | ✓ (roll/pitch < threshold) |
| Battery OK | - | ✓ (voltage > minimum) |
| Command source valid | - | ✓ (radio link, autonomous) |
| Watchdog not expired | - | ✓ (recent setpoint) |

## Integration with Pilot

This HAL links with `pilot.c` and the hive runtime. The platform API
(`platform_crazyflie.h`) provides the same interface as the Webots HAL.

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F405RG | - | ARM Cortex-M4, 168 MHz, DSP+FPU |
| IMU | BMI088 | I2C3 | 6-axis accel + gyro (addr: 0x18/0x69) |
| Barometer | BMP388 | I2C3 | Pressure/altitude (addr: 0x77) |
| Flow sensor | PMW3901 | SPI1 | Optical flow (Flow deck v2) |
| ToF sensor | VL53L1x | I2C1 | Height measurement (Flow deck v2, addr: 0x29) |
| SD card | Micro SD | SPI3 | Optional Micro SD Card Deck |
| Motors | 7x16mm | TIM2/TIM4 PWM | Brushed coreless, x4 |
| Radio | nRF51822 | USART6 | Syslink to Crazyradio PA |
| LED | Blue | PD2 | Status indicator |

## Specifications

**Flight Controller (Crazyflie 2.1+)**
- STM32F405RG @ 168 MHz (Cortex-M4F)
- 1 MB Flash, 192 KB RAM + 64 KB CCM
- Hardware FPU for fast sensor fusion
- 8 MHz HSE crystal

**Sensors (On-board, I2C3)**
- BMI088: 16-bit accel (+/-24g) + 16-bit gyro (+/-2000 dps) (addr: 0x18/0x69)
- BMP388: 24-bit barometer (addr: 0x77)

**Optional Flow Deck v2 (Expansion connector)**
- PMW3901: Optical flow sensor on SPI1 (80x80 pixels)
- VL53L1x: Time-of-Flight ranging on I2C1 (addr: 0x29, up to 4m)

**Motors**
- Type: Brushed coreless, 7x16mm
- Voltage: 3.0V nominal (single-cell LiPo)
- 2x CCW (M1, M3), 2x CW (M2, M4)
- Propellers: 45mm

**Battery**
- LiPo 3.7V / 250mAh
- Weight: ~27g (without Flow deck)

## Flash Layout

The STM32F405RG has 1 MB flash organized in sectors of varying sizes:

| Sector | Size | Address | Usage |
|--------|------|---------|-------|
| 0 | 16 KB | 0x08000000 | Firmware |
| 1 | 16 KB | 0x08004000 | Firmware |
| 2 | 16 KB | 0x08008000 | Firmware |
| 3 | 16 KB | 0x0800C000 | Firmware |
| 4 | 64 KB | 0x08010000 | Firmware |
| 5 | 128 KB | 0x08020000 | Firmware |
| 6 | 128 KB | 0x08040000 | Firmware |
| 7 | 128 KB | 0x08060000 | Firmware |
| **8** | **128 KB** | **0x08080000** | **Flight log (`/log`)** |
| 9-11 | 384 KB | 0x080A0000 | Reserved |

**Firmware region** - Sectors 0-7 (512 KB) - Limited by linker script
**Data region** - Sector 8 (128 KB) - Possible flight log during flight. The SD
card and/or radio is a better log solution though.

The linker script (`stm32f405_flash.ld`) limits firmware to 512 KB to prevent
accidentally overwriting the log sector. Current firmware is ~63 KB.

**Log file lifecycle**
1. `hive_log_file_open("/log")` erases sector 8 (takes ~1 second)
2. Writes to an 8 KB ring buffer, flushed to flash periodically
3. `hive_log_file_close()` flushes remaining data
4. After flight, download via radio using `ground_station.py --download-log`

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
(BSD-3-Clause or MIT). See top-level `THIRD_PARTY_LICENSES` for details.

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

### I2C3 (On-board sensors: BMI088, BMP388)
```
PA8  - I2C3_SCL
PC9  - I2C3_SDA

BMI088 Accelerometer: 0x18
BMI088 Gyroscope: 0x69
BMP388 Barometer: 0x77
```

### I2C1 (Expansion connector: VL53L1x on Flow deck)
```
PB6  - I2C1_SCL
PB7  - I2C1_SDA

VL53L1x ToF: 0x29
```

### SPI1 (PMW3901 Flow Sensor)
```
PA5  - SPI1_SCK
PA6  - SPI1_MISO
PA7  - SPI1_MOSI
PB4  - PMW3901_CS (Flow deck v2, DECK_GPIO_IO3)
```

### TIM2/TIM4 PWM (Motors)
```
PA1  - TIM2_CH2 (M1, front-left, CCW)
PB11 - TIM2_CH4 (M2, front-right, CW)
PA15 - TIM2_CH1 (M3, rear-right, CCW)
PB9  - TIM4_CH4 (M4, rear-left, CW)
```

### USART6 (nRF51 Syslink)
```
PC6  - USART6_TX
PC7  - USART6_RX
PA4  - TXEN (flow control from nRF51)
```

**Note**: The nRF51 handles multiple functions via syslink:
- Radio communication (Crazyradio PA)
- Power management and battery monitoring
- **Deck EEPROM detection** (1-Wire on NRF51 GPIO 8, NOT STM32 PC11)

Deck detection requires sending syslink commands to the NRF51:
- `SYSLINK_OW_SCAN (0x20)` - Get number of detected decks
- `SYSLINK_OW_GETINFO (0x21)` - Get deck serial number
- `SYSLINK_OW_READ (0x22)` - Read deck EEPROM (VID/PID)

### SPI3 (Micro SD Card Deck)
```
PB3  - SPI3_SCK  (AF6)
PB4  - SPI3_MISO (AF6)
PB5  - SPI3_MOSI (AF6)
PB6  - SD_CS     (GPIO output)
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

**Motor to pin mapping (Bitcraze reference)**

| Motor | Position | Rotation | Pin | Timer |
|-------|----------|----------|-----|-------|
| M1 | front-left | CCW | PA1 | TIM2_CH2 |
| M2 | front-right | CW | PB11 | TIM2_CH4 |
| M3 | rear-right | CCW | PA15 | TIM2_CH1 |
| M4 | rear-left | CW | PB9 | TIM4_CH4 |

**Motor mixing (in hal_crazyflie.c)**
```
M1 = thrust - roll + pitch + yaw
M2 = thrust + roll + pitch - yaw
M3 = thrust + roll - pitch + yaw
M4 = thrust - roll - pitch - yaw
```

**To reverse motor direction** - Swap the two motor wires at the connector.

## Sensor Configuration

The sensor configuration matches the Bitcraze crazyflie-firmware for compatibility:

**BMI088 IMU**
| Parameter | Value |
|-----------|-------|
| Accel range | +/-24g |
| Accel ODR | 1600 Hz |
| Accel bandwidth | OSR4 (4x oversampling) |
| Gyro range | +/-2000 dps |
| Gyro ODR | 1000 Hz |
| Gyro bandwidth | 116 Hz |

**BMP388 Barometer**
| Parameter | Value |
|-----------|-------|
| Pressure oversampling | 8x |
| Temperature oversampling | None |
| ODR | 50 Hz |
| IIR filter coefficient | 3 |

**VL53L1x ToF (Flow deck)**
| Parameter | Value |
|-----------|-------|
| Distance mode | Short |
| Timing budget | 20 ms |
| Inter-measurement | 25 ms |

**Startup sequence**
1. 1000 ms delay for sensor power stabilization
2. Initialize I2C3 (on-board sensors)
3. Initialize SPI1 (PMW3901)
4. Initialize BMI088 with soft reset
5. Initialize BMP388 with soft reset
6. Initialize Flow deck sensors on I2C1 (if present)

## Self-Test

After initialization, `hal_self_test()` verifies all sensors respond:

1. **Required** - BMI088 (gyro/accel), BMP388 (barometer)
2. **Optional** - Flow deck sensors (PMW3901, VL53L1x) - only tested if deck was detected

Returns `true` if all required sensors pass, `false` otherwise.

## Calibration

Before flight, `hal_calibrate()` performs (matching Bitcraze firmware):

1. **Level check** - Verify accelerometer reads approximately level (warns if tilted >6 deg)
2. **Gyro bias** - Average 512 samples with variance check; retries until drone is stationary
3. **Accelerometer scale** - Average 200 samples to compute magnitude-based scale correction
4. **Barometer reference** - Average 50 samples for ground level
5. **Height offset** (Flow deck only) - Measure ground clearance for accurate height

**Important** - Keep the drone still and level on a flat surface during calibration!

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
| 2 (ALTITUDE) | Altitude changes: 0.5m -> 0.8m -> 1.2m -> 0.8m | 40s |
| 3 (FULL_3D) | 3D waypoints, max 1.2m (requires Flow deck) | 60s |

**Note** - All profiles keep altitude <=1.2m to stay within flow deck range (VL53L1x: 1.3m max).

## Flashing Notes

After `st-flash write`, press the reset button on the Crazyflie or run
`st-flash reset` to start the firmware.

## SD Card Support (Optional)

The [Micro SD Card Deck](https://www.bitcraze.io/products/micro-sd-card-deck/)
enables file storage on a FAT-formatted SD card via SPI3.

### Building with SD Support

```bash
make -f Makefile.crazyflie-2.1+ ENABLE_SD=1
make -f Makefile.crazyflie-2.1+ ENABLE_SD=1 flash
```

### Usage

Files on the SD card are accessed via the `/sd` mount point:

```c
int fd;
hive_file_open("/sd/flight_log.bin", HIVE_O_WRONLY | HIVE_O_CREAT, 0644, &fd);
hive_file_write(fd, data, sizeof(data), &written);
hive_file_close(fd);
```

### Architecture

The SD card driver uses a layered architecture:

| Layer | File | Description |
|-------|------|-------------|
| Protocol | `src/hal/stm32/spi_sd.c` | SD card SPI protocol (generic) |
| Interface | `src/hal/stm32/spi_ll.h` | Low-level SPI function declarations |
| Hardware | `spi_ll_sd.c` | Board-specific SPI3 implementation |
| Filesystem | `lib/fatfs/` | FatFS library (FAT12/16/32, exFAT) |

### Configuration

Mount table settings in `hive_mounts.c`:

```c
HIVE_SD_SPI_ID   = 3      // SPI3
HIVE_SD_CS_PORT  = 1      // GPIOB
HIVE_SD_CS_PIN   = 6      // PB6
```

### Checking Availability

Use `hive_file_mount_available()` to check if the SD card is present before opening:

```c
if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
    hive_file_open("/sd/flight.bin", HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);
} else {
    hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);  // Fallback
}
```

### Memory Impact

| Component | Flash | RAM |
|-----------|-------|-----|
| Mount table dispatch | ~0.5 KB | ~0.1 KB |
| FatFS library | ~10 KB | ~0.5 KB |
| FatFS FIL structs (4x) | - | ~2.4 KB |
| SPI driver | ~1 KB | ~0.1 KB |
| **Total** | **~12 KB** | **~3 KB** |

Without SD (`ENABLE_SD=0`): Only ~0.5 KB flash overhead for mount table.

### Limitations

- **No hot-plug** - SD card presence is checked at init only. If the card is
  removed mid-flight, writes fail with `HIVE_ERR_IO`.
- **No auto-mkdir** - Parent directories must exist. Use flat paths like
  `/sd/flight_001.bin`.
- **No file listing** - Use sequential filenames; no `readdir()` API.

### Differences from Flash Virtual Files

| Feature | Flash (`/log`) | SD Card (`/sd`) |
|---------|----------------|-----------------|
| HIVE_O_RDWR | Not supported | Supported |
| HIVE_O_TRUNC required | Yes (erase sector) | No |
| Multiple open files | No (single writer) | Yes (up to HIVE_MAX_SD_FILES) |
| Speed | Fast (internal flash) | Slower (SPI interface) |
| Capacity | 128 KB (sector 8) | Card dependent (GB) |

## Bringup Tests

The `bringup/` directory contains standalone firmware for hardware verification.
Run these tests before the full pilot firmware to verify each hardware component
works correctly. See `bringup/README.md` for details.

```bash
cd bringup
make && make flash
./st-trace.sh  # View test output
```

## Resources

**Bitcraze Reference (normative truth)**
- [Crazyflie Firmware](https://github.com/bitcraze/crazyflie-firmware) - STM32 main firmware
- [NRF51 Firmware](https://github.com/bitcraze/crazyflie2-nrf-firmware) - Power management firmware
- [Syslink Protocol](https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/protocols/syslink/)

**Hardware Documentation**
- [Crazyflie 2.1 Product Page](https://www.bitcraze.io/products/crazyflie-2-1/)
- [Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/)
- [Micro SD Card Deck](https://www.bitcraze.io/products/micro-sd-card-deck/)

**Sensor Datasheets**
- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BMP388 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
- [PMW3901 Datasheet](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [VL53L1x Datasheet](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
