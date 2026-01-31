# Crazyflie 2.1+ Hardware Abstraction Layer

Platform layer for the Bitcraze Crazyflie 2.1+ nano quadcopter.

Uses DMA/interrupt-based I2C for reliable sensor communication.

## Quick Start

```bash
# Build HAL library (from this directory)
make          # Build libhal.a
make clean    # Remove build artifacts

# Run sensor/motor test
cd ../../../tests
make PLATFORM=crazyflie TEST=sensors_motors crazyflie
make PLATFORM=crazyflie TEST=sensors_motors flash-crazyflie
../tools/st-trace.sh  # View output
```

## Key Features

| Feature | Implementation |
|---------|----------------|
| **I2C3 driver** | DMA + interrupt (Bitcraze-derived) |
| **BMI088 self-test** | Hardware self-test via `bmi088_perform_*_selftest()` |
| **Gyro calibration** | Variance-based stillness detection |
| **Accel calibration** | 200 samples (matches Bitcraze) |
| **VL53L1x init** | Soft reset + robust boot sequence |
| **Vendor drivers** | Bitcraze-modified Bosch drivers |

## Reference Implementation

**The Bitcraze crazyflie-firmware repository is the normative truth for this platform.**

Local copy available at `local/crazyflie-firmware/` for reference.

Key resources:
- https://github.com/bitcraze/crazyflie-firmware (STM32 main firmware)
- https://github.com/bitcraze/crazyflie2-nrf-firmware (NRF51 power management)

## Design Decisions

### I2C Driver Strategy

| Bus | Purpose | Driver | Rationale |
|-----|---------|--------|-----------|
| **I2C3** | On-board sensors (BMI088, BMP388) | DMA + interrupt | Reliable under load, matches Bitcraze |
| **I2C1** | Flow deck (VL53L1x) | Polling | Infrequent reads (~40Hz), simpler |
| **SPI1** | Flow deck (PMW3901) + SD card deck | Polling | Shared bus, CS-based selection |

The DMA-based I2C3 driver (`i2c_drv.c`, `i2cdev.c`) is derived from the Bitcraze
firmware, providing proven reliability for the high-frequency sensor reads.

### Self-Test

This HAL uses BMI088's built-in hardware self-test:

```c
// Gyro self-test
bmi088_perform_gyro_selftest(&result, &dev);  // Triggers internal test
if (result != BMI088_SELFTEST_PASS) { /* fail */ }

// Accel self-test
bmi088_perform_accel_selftest(&result, &dev);
```

This verifies the sensor's internal mechanics, not just I2C communication.
After self-test, sensors are re-initialized since the test modifies configuration.

### Calibration

**Gyro calibration** uses variance-based stillness detection:

```c
// Collect 512 samples, compute mean and variance
for (i = 0; i < 512; i++) {
    gyro_sum[axis] += sample;
    gyro_sum_sq[axis] += sample * sample;
}
variance = (sum_sq / n) - (mean * mean);

// Reject if variance > 0.0001 (rad/s)^2 (~0.01 rad/s std dev)
if (variance > GYRO_VARIANCE_THRESHOLD) {
    return -1;  // Drone not still, retry
}
```

**Accel calibration** measures gravity magnitude:

```c
// 200 samples, compute average magnitude
magnitude = sqrt(ax^2 + ay^2 + az^2);
s_accel_scale = GRAVITY / avg_magnitude;  // Correction factor
```

### VL53L1x Initialization

The VL53L1x ToF sensor requires careful initialization:

1. **Soft reset** - Clean state after power-up
2. **Boot wait** - Poll firmware status register
3. **Config writes** - 91 register writes with inter-write delays
4. **Validation ranging** - Start/stop ranging to verify operation

The I2C1 driver includes delays between writes to prevent bus errors during
the lengthy configuration sequence.

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F405RG | - | ARM Cortex-M4, 168 MHz, DSP+FPU |
| IMU | BMI088 | I2C3 | 6-axis accel + gyro (addr: 0x18/0x69) |
| Barometer | BMP388 | I2C3 | Pressure/altitude (addr: 0x77) |
| Flow sensor | PMW3901 | SPI1 | Optical flow (Flow deck v2) |
| ToF sensor | VL53L1x | I2C1 | Height measurement (Flow deck v2, addr: 0x29) |
| Motors | 7x16mm | TIM2/TIM4 PWM | Brushed coreless, x4 |
| LED | Blue | PD2 | Status indicator |

## Architecture

```
+---------------------------------------------------------------+
|                      Test Application                         |
|              (test_sensors_motors.c or pilot.c)               |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                     HAL Interface                             |
|   hal_init.c, hal_sensors.c, hal_motors.c, hal_time.c        |
+---------------------------------------------------------------+
                              |
                              v
+---------------------------------------------------------------+
|                      platform.c                               |
|     (I2C3 DMA driver + I2C1/SPI1 polling + callbacks)        |
+---------------------------------------------------------------+
                              |
        +---------------------+---------------------+
        |                     |                     |
        v                     v                     v
+---------------+     +---------------+     +---------------+
|   i2c_drv.c   |     | vendor/bosch  |     | vendor/st     |
|   i2cdev.c    |     |  bmi088/bmp3  |     |   vl53l1x     |
| (DMA/int I2C) |     | (sensor APIs) |     |  (ToF API)    |
+---------------+     +---------------+     +---------------+
```

## File Overview

### HAL Layer

| File | Description |
|------|-------------|
| `hal_init.c` | init, cleanup, arm, disarm |
| `hal_sensors.c` | hal_read_sensors (wrapper to platform) |
| `hal_motors.c` | hal_write_torque (mixer, PWM output) |
| `hal_time.c` | hal_delay_ms, hal_get_time_ms |
| `hal_led.c` | hal_led_on/off/toggle |
| `hal_debug.c` | hal_debug_init, hal_printf |

### Platform Layer

| File | Description |
|------|-------------|
| `platform.c` | Sensor init, calibration, self-test, I2C/SPI callbacks |
| `platform.h` | Platform API declarations |
| `i2c_drv.c` | DMA/interrupt I2C3 driver (Bitcraze-derived) |
| `i2cdev.c` | I2C device abstraction layer |
| `debug_swo.c` | SWO debug output with printf support |
| `hal_syslink.c` | NRF51 radio communication (interrupt RX, polled TX) for ESB telemetry |
| `spi_ll_sd.c` | SD card low-level SPI driver (shares SPI1 with flow deck) |

### Vendor Drivers

| Directory | Driver | Source | Description |
|-----------|--------|--------|-------------|
| `vendor/bosch/` | BMI088, BMP3 | Bitcraze | IMU and barometer |
| `vendor/bitcraze/pmw3901/` | PMW3901 | Bitcraze | Optical flow |
| `vendor/st/vl53l1x/` | VL53L1x ULD | ST/mbed port | ToF ranging |
| `vendor/STM32F4xx_StdPeriph_Driver/` | StdPeriph | ST | GPIO, I2C, DMA, etc. |
| `vendor/CMSIS/` | CMSIS | ARM | Core and device headers |

## Pin Mapping

### I2C3 (On-board sensors - DMA)
```
PA8  - I2C3_SCL
PC9  - I2C3_SDA

BMI088 Accelerometer: 0x18
BMI088 Gyroscope:     0x69
BMP388 Barometer:     0x77
```

### I2C1 (Flow deck VL53L1x - Polling)
```
PB6  - I2C1_SCL
PB7  - I2C1_SDA

VL53L1x ToF: 0x29
```

### SPI1 (Flow deck + SD card deck - Polling)
```
PA5  - SPI1_SCK   (shared)
PA6  - SPI1_MISO  (shared)
PA7  - SPI1_MOSI  (shared)
PB4  - PMW3901_CS (flow deck)
PC12 - SD_CS      (SD card deck, IO4 on expansion)
```

### TIM2/TIM4 PWM (Motors)
```
PA1  - TIM2_CH2 (M1, front-left, CCW)
PB11 - TIM2_CH4 (M2, front-right, CW)
PA15 - TIM2_CH1 (M3, rear-right, CCW)
PB9  - TIM4_CH4 (M4, rear-left, CW)
```

### Debug
```
PB3  - SWO (trace output)
PD2  - Blue LED
```

## Sensor Configuration

Matches Bitcraze crazyflie-firmware settings:

**BMI088 IMU**
| Parameter | Value |
|-----------|-------|
| Accel range | +/-24g |
| Accel ODR | 1600 Hz |
| Accel bandwidth | OSR4 |
| Gyro range | +/-2000 dps |
| Gyro ODR | 1000 Hz |
| Gyro bandwidth | 116 Hz |

**BMP388 Barometer**
| Parameter | Value |
|-----------|-------|
| Pressure oversampling | 4x |
| Temperature oversampling | 2x |
| ODR | 25 Hz |
| IIR filter | Coeff 3 |

**VL53L1x ToF (Flow deck)**
| Parameter | Value |
|-----------|-------|
| Distance mode | Short (up to 1.3m) |
| Timing budget | 20 ms |
| Inter-measurement | 25 ms (~40 Hz) |

## Self-Test Sequence

1. **BMI088 Gyro Hardware Self-Test**
   - Triggers sensor's internal test mechanism
   - Re-initializes gyro after test (config modified)

2. **BMI088 Accel Hardware Self-Test**
   - Triggers sensor's internal test mechanism
   - Re-initializes accel after test

3. **Sensor Read Verification**
   - Reads accel, gyro, baro to verify I2C works
   - Prints raw values for debugging

## Calibration Sequence

**Prerequisites**: Drone must be still and level on a flat surface.

1. **Gyro Bias Calibration** (512 samples, ~1 second)
   - Computes mean and variance for each axis
   - Rejects if variance > 0.0001 (rad/s)^2 (drone moving)
   - Stores bias for runtime subtraction

2. **Accelerometer Scale Calibration** (200 samples)
   - Measures gravity magnitude
   - Computes scale factor: `GRAVITY / measured_magnitude`
   - Corrects for sensor scaling errors

3. **Barometer Reference** (50 samples)
   - Averages pressure readings
   - Sets ground-level reference for altitude

4. **Post-Calibration Verify**
   - Reads all sensors to confirm they still work

## Flow Deck Support

The Flow deck v2 is auto-detected during initialization:

1. **PMW3901** (SPI1) - Probes for optical flow sensor
2. **VL53L1x** (I2C1) - Soft reset, boot wait, sensor init

If flow deck is not present, initialization continues without it.

**Height filtering**: Readings > HAL_TOF_MAX_RANGE_MM (1300mm) are discarded as outliers.

### Flow Deck Integration

When the flow deck is present, `platform_read_sensors()` integrates flow data to provide
position and velocity information:

1. **ToF height** - VL53L1x provides ground-relative altitude (m)
2. **Optical flow** - PMW3901 pixel deltas converted to body-frame velocity using:
   ```
   velocity = pixel_delta * HAL_FLOW_SCALE * height
   ```
3. **World-frame rotation** - Body velocity rotated using integrated yaw from gyro
4. **Position integration** - World-frame velocity integrated to pseudo-GPS position

The flow integration provides:
- `sensors->velocity_x/y` - World-frame velocity (m/s)
- `sensors->velocity_valid` - True when flow + height valid
- `sensors->gps_x/y/z` - Integrated position (m), z = ToF height
- `sensors->gps_valid` - True when ToF height valid

**Configuration** (in `hal_config.h`):
- `HAL_FLOW_SCALE` - Pixel-to-velocity conversion (0.0005)
- `HAL_TOF_MAX_RANGE_MM` - Maximum valid ToF reading (1300mm)

## SD Card Deck Support

The Micro SD Card Deck shares SPI1 with the Flow deck:

- **SPI1** - Shared bus (PA5/PA6/PA7)
- **PC12** - SD card CS (IO4 on expansion connector)

Both decks can be used simultaneously - the SPI bus is shared with CS-based selection.

**Requirements:**
- Micro SD Card Deck attached to expansion connector
- FAT32-formatted SD card inserted
- Build with `ENABLE_SD=1`

**Limitations:**
- 8.3 filenames only (no long filenames) to save ~2KB flash
- Examples: `data.bin`, `log00001.txt`, `config.ini`

**Test:**
```bash
cd ../../../tests
make PLATFORM=crazyflie TEST=sd ENABLE_SD=1
make flash-crazyflie TEST=sd
../tools/st-trace.sh
```

Typical performance: Write ~450 KB/s, Read ~1000 KB/s.

## LED Feedback

| Pattern | Meaning |
|---------|---------|
| 1 slow blink | Init starting |
| 2 slow blinks | Init passed, starting self-test |
| 3 slow blinks | Self-test passed, starting calibration |
| Slow blink (continuous) | Calibration in progress |
| 4 slow blinks | Calibration done |
| 5 slow blinks | Sensor test done |
| LED solid on | All tests passed |
| Fast blinks (10x) | Sensor data out of range |
| Continuous fast blink | Fatal error |

## Building

### Requirements

```bash
# ARM GCC toolchain
sudo apt install gcc-arm-none-eabi

# ST-Link tools
sudo apt install stlink-tools
```

### Build Commands

```bash
# Build HAL library
make          # Creates build/libhal.a
make clean    # Remove build artifacts

# Build and run sensor test
cd ../../../tests
make PLATFORM=crazyflie TEST=sensors_motors crazyflie
make PLATFORM=crazyflie TEST=sensors_motors flash-crazyflie
```

### Debug Output

Use SWO trace to view debug output:

```bash
# From examples/pilot/tools/
./st-trace.sh

# Or directly:
st-trace -c 168
```

## Debug Hardware Setup

### Required Equipment

1. **Crazyflie 2.1+** - Target platform
2. **Bitcraze Debug Adapter Kit** - Connects to the 0.05" debug header
3. **ST-Link V2/V3** - For flashing and SWO debug output

### Connections

```
Crazyflie 2.1+          Debug Adapter          ST-Link V2/V3
+--------------+        +------------+        +----------+
| Debug Header |------->| 10-pin     |------->| 20-pin   |
| (0.05" pitch)|        | to 20-pin  |        | SWD+SWO  |
+--------------+        +------------+        +----------+
                                                   |
                                                   v
                                              USB to PC
                                              (view with st-trace)
```

### Debug Header Pinout (Crazyflie 10-pin)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | VCC | 3.0V reference |
| 2 | SWDIO | SWD data |
| 3 | GND | Ground |
| 4 | SWCLK | SWD clock |
| 5 | GND | Ground |
| 6 | SWO | Trace output (PB3) |
| 7 | GND | Ground |
| 8 | PA9 | USART1_TX (unused) |
| 9 | GND | Ground |
| 10 | PA10 | USART1_RX (unused) |

### SWO Debug Output

Debug output uses SWO (Serial Wire Output) via ITM. This is one-way only.

```bash
# View trace output (recommended)
../../../tools/st-trace.sh

# With timeout
../../../tools/st-trace.sh -t 30   # 30 second timeout
../../../tools/st-trace.sh -t 0    # No timeout (Ctrl-C to stop)

# Or directly (if stlink-tools installed)
st-trace -c 168
```

**Note**: System stlink-tools v1.8.0 has SWO bugs. Use the locally built version
via `st-trace.sh`, or build from `local/stlink/` if needed.

## NRF51 Architecture

The Crazyflie has two processors. Understanding this is important for deck detection.

```
                      +-----------+
Expansion Deck ------>| NRF51     |<------ Syslink (1Mbaud) ------>| STM32
EEPROM (DS28E05)      | GPIO 8    |         USART6 (PC6/PC7)       | Main CPU
                      +-----------+
```

**Key points:**
- Deck EEPROM (1-Wire) is connected to **NRF51**, not STM32
- STM32 cannot directly read deck EEPROM
- Deck detection uses sensor probing instead (VL53L1x + PMW3901 = Flow deck)
- NRF51 handles: battery monitoring, 1-Wire protocol, radio, power management

## Chip ID Reference

Use these to verify sensor communication:

| Sensor | Bus | Address | Register | Expected |
|--------|-----|---------|----------|----------|
| BMI088 Accel | I2C3 | 0x18 | 0x00 | 0x1E |
| BMI088 Gyro | I2C3 | 0x69 | 0x00 | 0x0F |
| BMP388 | I2C3 | 0x77 | 0x00 | 0x50 |
| VL53L1x | I2C1 | 0x29* | 0x010F | 0xEACC |
| PMW3901 | SPI1 | - | 0x00 | 0x49 |

*VL53L1x may be at alternate address - see below.

## Expected Sensor Values

At rest on a level surface:

| Sensor | Axis/Reading | Expected | Tolerance |
|--------|--------------|----------|-----------|
| Accelerometer | X, Y | ~0 g | +/- 0.1 g |
| Accelerometer | Z | ~+1 g | 0.9-1.1 g |
| Gyroscope | X, Y, Z | ~0 deg/s | +/- 5 deg/s |
| Barometer | Pressure | 95-106 kPa | varies with altitude |
| Barometer | Temperature | 15-35 C | room temp |
| VL53L1x | Range | 0-2000 mm | depends on surface |

## VL53L1x I2C Address

The VL53L1x default address is **0x29**, but Bitcraze firmware may reassign it.
The HAL scans multiple addresses automatically:

| Address | Notes |
|---------|-------|
| 0x29 | Default |
| 0x6A | Observed on some units |
| 0x30-0x32 | Bitcraze reassignment range |
| 0x52 | Alternative address |

Detection uses Model ID register (0x010F) which returns 0xEACC for VL53L1x.

## Motor Layout

```
         FRONT
     M1(CCW)  M2(CW)
         +--+
         |  |
         +--+
     M4(CW)  M3(CCW)
         REAR
```

| Motor | Position | Rotation | Pin | Timer |
|-------|----------|----------|-----|-------|
| M1 | Front-left | CCW | PA1 | TIM2_CH2 |
| M2 | Front-right | CW | PB11 | TIM2_CH4 |
| M3 | Rear-right | CCW | PA15 | TIM2_CH1 |
| M4 | Rear-left | CW | PB9 | TIM4_CH4 |

## Troubleshooting

### No SWO output

1. Use `../../../tools/st-trace.sh` (system st-trace v1.8.0 has bugs)
2. Verify SWO pin connected (pin 6 on debug header)
3. Try `st-flash reset` to reset target
4. Check ST-Link firmware is up to date

### I2C devices not found

**On-board sensors (I2C3 - PA8/PC9):**
- Check I2C3 pull-ups present on board
- Verify PA8 (SCL) and PC9 (SDA) not shorted
- Try power cycling the board

**Flow deck sensors (I2C1 - PB6/PB7):**
- Ensure deck is fully seated in expansion headers
- Check I2C1 pull-ups on expansion connector
- VL53L1x is on I2C1, not I2C3

### VL53L1x sensor init failed

The VL53L1x requires careful I2C timing. This HAL includes:
- Soft reset before init
- Boot state polling with timeout
- Inter-write delays during config

If init still fails, check:
- Flow deck properly seated on expansion connector
- I2C1 pull-ups present (deck should have them)

### Gyro variance too high

During calibration, if variance exceeds threshold:
- Keep drone completely still
- Place on flat, stable surface
- Wait for vibrations to settle

### Accel self-test shows x=0 y=0 z=0

This can occur immediately after hardware self-test due to timing.
The subsequent sensor reads should show correct values.

### Sensor data out of range

- Accelerometer: Ensure board is level
- Gyroscope: Ensure board is stationary
- Barometer: Check altitude, verify no pressure leaks
- ToF: Ensure clear line of sight to surface

### Motor doesn't spin

1. **Remove propellers first!**
2. Check motor connector seated properly
3. Verify battery charged (>3.0V)
4. Try higher duty cycle (20%)

## Resources

**Local Reference**
- `crazyflie_firmware_log.txt` - Console output from stock Bitcraze firmware (via cfclient)
- `local/crazyflie-firmware/` - Local copy of Bitcraze firmware source

**Bitcraze Reference**
- [Crazyflie Firmware](https://github.com/bitcraze/crazyflie-firmware)
- [Flow deck v2](https://www.bitcraze.io/products/flow-deck-v2/)

**Sensor Datasheets**
- [BMI088](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [BMP388](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp388/)
- [PMW3901](https://www.pixart.com/products-detail/10/PMW3901MB-TXQT)
- [VL53L1x](https://www.st.com/en/imaging-and-photonics-solutions/vl53l1x.html)
