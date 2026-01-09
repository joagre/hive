# STEVAL-DRONE01 Hardware Abstraction Layer

Platform layer for the STMicroelectronics STEVAL-DRONE01 mini drone kit.

This HAL provides bare-metal drivers for STM32F401, enabling the pilot example to run on real hardware instead of the Webots simulator.

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F401CEU6 | - | ARM Cortex-M4, 84 MHz, DSP+FPU |
| IMU | LSM6DSL | SPI1 | 6-axis accel + gyro |
| Magnetometer | LIS2MDL | I2C1 | 3-axis compass |
| Barometer | LPS22HD | I2C1 | Pressure/altitude |
| Bluetooth | SPBTLE-RF | SPI2 | BLE 4.1 for RC |
| Motors | 85x20mm | TIM4 PWM | 3.7V brushed, x4 |

## Specifications

**Flight Controller (STEVAL-FCU001V1):**
- STM32F401CEU6 @ 84 MHz (Cortex-M4F)
- 512 KB Flash, 96 KB RAM
- Hardware FPU for fast sensor fusion

**Sensors:**
- LSM6DSL: ±2/4/8/16g accel, ±250/500/1000/2000 dps gyro
- LIS2MDL: ±50 gauss magnetometer
- LPS22HD: 260-1260 hPa barometer

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
┌─────────────────────────────────────────────────────────────────┐
│                         main.c                                   │
│                    (PID Flight Controller)                       │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                       platform.h/c                               │
│              (400Hz Control Loop, Sensor Fusion)                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ attitude │  │ lsm6dsl  │  │ lis2mdl  │  │ lps22hd  │        │
│  │ (filter) │  │  (IMU)   │  │  (mag)   │  │ (baro)   │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Peripheral Drivers                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │   spi1   │  │   i2c1   │  │  motors  │  │  usart1  │        │
│  │          │  │  (TODO)  │  │  (TODO)  │  │  (TODO)  │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Low-Level HAL                                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │  system  │  │   gpio   │  │  startup │  │  linker  │        │
│  │  config  │  │  config  │  │   .s     │  │   .ld    │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
└─────────────────────────────────────────────────────────────────┘
```

## File Overview

### Application Layer

| File | Description |
|------|-------------|
| `main.c` | Example PID flight controller with altitude hold |
| `platform.h/c` | 400Hz control loop, sensor reading, motor output |
| `attitude.h/c` | Complementary filter for roll/pitch/yaw estimation |

### Sensor Drivers

| File | Sensor | Interface | Status |
|------|--------|-----------|--------|
| `lsm6dsl.h/c` | LSM6DSL 6-axis IMU | SPI1 | **Integrated** |
| `lis2mdl.h/c` | LIS2MDL magnetometer | I2C1 | **Integrated** |
| `lps22hd.h/c` | LPS22HD barometer | I2C1 | **Integrated** |
| `motors.h/c` | Brushed DC motors | TIM4 PWM | **Integrated** |

### Peripheral Drivers

| File | Peripheral | Description | Status |
|------|------------|-------------|--------|
| `spi1.h/c` | SPI1 | Mode 3, 10.5MHz for LSM6DSL | **Complete** |
| `i2c1.h/c` | I2C1 | 400kHz Fast Mode for LIS2MDL, LPS22HD | **Complete** |
| `tim4.h/c` | TIM4 | 20kHz PWM for motors (CH3/CH4 on PB8/PB9) | **Complete** |
| `usart1.h/c` | USART1 | Debug serial output | TODO |

### System Layer

| File | Description |
|------|-------------|
| `system_config.h/c` | Clock tree (84MHz), SysTick (1ms), DWT (µs timing) |
| `gpio_config.h/c` | Pin configuration for all peripherals |
| `startup_stm32f401.s` | Vector table, Reset_Handler, C runtime init |
| `stm32f401_flash.ld` | Memory layout (512K Flash, 96K RAM) |
| `Makefile` | ARM GCC build system |

## Pin Mapping

### SPI1 (LSM6DSL IMU)
```
PA4  - LSM6DSL_CS (GPIO output)
PA5  - SPI1_SCK
PA6  - SPI1_MISO
PA7  - SPI1_MOSI
```

### I2C1 (Magnetometer, Barometer)
```
PB6  - I2C1_SCL
PB7  - I2C1_SDA
```

### TIM4 PWM (Motors)
```
Default configuration (2 motors, I2C1 compatible):
  PB8  - TIM4_CH3 (M3 front-right)
  PB9  - TIM4_CH4 (M4 rear-right)

Full 4-motor configuration (requires Port D):
  PD12 - TIM4_CH1 (M1 rear-left)
  PD13 - TIM4_CH2 (M2 front-left)
  PD14 - TIM4_CH3 (M3 front-right)
  PD15 - TIM4_CH4 (M4 rear-right)

Note: PB6/PB7 (TIM4_CH1/CH2) conflict with I2C1.
Use motors_init_full(NULL, true) for 4-motor support.
```

### USART1 (Debug Serial)
```
PA9  - USART1_TX
PA10 - USART1_RX
```

### Misc
```
PC13 - LED (directly controlled GPIO)
PA0  - User button (directly controlled GPIO)
```

## Clock Configuration

```
HSE (16MHz) ──► PLL (×336/16/4) ──► SYSCLK (84MHz)
                                         │
                    ┌────────────────────┼────────────────────┐
                    ▼                    ▼                    ▼
               AHB (84MHz)         APB1 (42MHz)         APB2 (84MHz)
               DMA, GPIO           I2C1, TIM4            SPI1, USART1
                                   USART2, TIM2-5        TIM1
```

- Flash: 2 wait states, prefetch + caches enabled
- SysTick: 1ms interrupt for system tick
- DWT: Cycle counter for microsecond timing

## Differences from Webots Simulation

| Feature | Webots | STEVAL-DRONE01 |
|---------|--------|----------------|
| Position | GPS (perfect) | None (need external tracking) |
| Attitude | Inertial unit (fused) | Raw IMU (need sensor fusion) |
| Altitude | GPS Z | Barometer (relative only) |
| Heading | Direct yaw | Magnetometer (calibration needed) |
| Timing | Simulation step | Real-time (hardware timers) |

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
make          # Build firmware → build/pilot.elf
make flash    # Flash to device via ST-Link
make debug    # Start GDB debug session
make clean    # Remove build artifacts
make size     # Show memory usage
make help     # Show all targets
```

### Memory Usage

Typical footprint:
- Flash: ~20KB (code + constants)
- RAM: ~8KB (stack + heap + BSS)

## API Reference

### System Configuration

```c
#include "system_config.h"

system_init();              // Initialize clocks + SysTick + DWT
system_get_tick();          // Milliseconds since boot
system_delay_ms(100);       // Sleep (uses WFI)
system_get_us();            // Microsecond timestamp (DWT)
system_delay_us(10);        // Busy-wait microseconds
```

### GPIO

```c
#include "gpio_config.h"

gpio_init_all();            // Configure all peripheral pins
gpio_led_on();              // Turn on status LED
gpio_led_toggle();          // Toggle LED
gpio_button_read();         // Read user button
```

### SPI1 (LSM6DSL)

```c
#include "spi1.h"

spi1_init(SPI1_SPEED_10_5MHZ);   // Initialize for LSM6DSL
uint8_t rx = spi1_transfer(tx);  // Full-duplex byte transfer
spi1_transfer_buf(tx, rx, len);  // Buffered transfer
```

### I2C1 (LIS2MDL, LPS22HD)

```c
#include "i2c1.h"

i2c1_init(I2C1_SPEED_400KHZ);           // Initialize 400kHz Fast Mode
i2c1_write_reg(addr, reg, value);       // Write single register
i2c1_read_reg(addr, reg, &value);       // Read single register
i2c1_read_regs(addr, reg, buf, len);    // Burst read multiple registers
i2c1_probe(addr);                        // Check if device present
i2c1_reset();                            // Bus recovery (stuck SDA)
```

### LSM6DSL (IMU)

```c
#include "lsm6dsl.h"

lsm6dsl_init(NULL);                    // Default: ±4g, ±500dps, 416Hz
lsm6dsl_read_all(&accel, &gyro);       // Burst read (m/s², rad/s)
lsm6dsl_read_temp();                   // Temperature (°C)
```

### LIS2MDL (Magnetometer)

```c
#include "lis2mdl.h"

lis2mdl_init(NULL);                    // Default: 50Hz, continuous
lis2mdl_read(&mag);                    // Read in microtesla
float heading = lis2mdl_heading_tilt_compensated(&mag, roll, pitch);
```

### LPS22HD (Barometer)

```c
#include "lps22hd.h"

lps22hd_init(NULL);                    // Default: 50Hz
lps22hd_set_reference(lps22hd_read_pressure());  // Set ground level
float alt = lps22hd_read_altitude();   // Meters above ground
```

### TIM4 (Motor PWM)

```c
#include "tim4.h"

tim4_init(NULL);                       // Default: 20kHz, CH3/CH4 only
tim4_set_duty(TIM4_CH3, 0.5f);         // Set 50% duty cycle
tim4_set_all(duties);                  // Set all 4 channels at once
tim4_enable();                         // Enable PWM output
tim4_disable();                        // Disable output
```

### Motors

```c
#include "motors.h"

motors_init(NULL);                     // Default: 20kHz, CH3/CH4 only
motors_init_full(NULL, true);          // All 4 motors on PD12-PD15
motors_arm();                          // Enable PWM output
motors_set(&cmd);                      // Set speeds (0.0-1.0)
motors_emergency_stop();               // Immediate stop + disarm
```

### Attitude Filter

```c
#include "attitude.h"

attitude_init(NULL, NULL);             // Default: alpha=0.98
attitude_update(accel, gyro, dt);      // Fuse sensors (call at 400Hz)
attitude_update_mag(mag);              // Optional yaw correction (50Hz)
attitude_get(&att);                    // Get roll, pitch, yaw (radians)
```

### Platform

```c
#include "platform.h"

platform_callbacks_t callbacks = {
    .on_init = my_init,
    .on_control = my_control,          // Called at 400Hz
    .on_state_change = my_state_cb
};

platform_init(&callbacks);
platform_calibrate();                  // Keep drone still!
platform_arm();
platform_run();                        // Never returns
```

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

## Control Loop Timing

```
400 Hz (2.5ms) ─┬─► Read IMU (LSM6DSL)
                ├─► Update attitude filter
                ├─► Run PID controllers
                └─► Output to motors

 50 Hz (20ms) ──┬─► Read magnetometer (LIS2MDL)
                └─► Read barometer (LPS22HD)
```

## Calibration

Before flight, `platform_calibrate()` performs:

1. **Gyro bias** - Average 500 samples while stationary
2. **Barometer reference** - Average 50 samples for ground level
3. **Attitude init** - Set initial roll/pitch from accelerometer

**Important:** Keep the drone still and level during calibration!

## Tuning PID Gains

The default PID gains in `main.c` are starting points:

```c
#define ROLL_KP     2.0f
#define PITCH_KP    2.0f
#define YAW_KP      1.0f
#define ALT_KP      0.5f
#define THROTTLE_HOVER  0.5f
```

Tuning procedure:
1. Start with very low gains (0.5)
2. Increase P until oscillation, then reduce by 30%
3. Add D to dampen oscillation
4. Add I only if needed for steady-state error
5. Tune altitude last

## Resources

- [STEVAL-DRONE01 Product Page](https://www.st.com/en/evaluation-tools/steval-drone01.html)
- [Datasheet (PDF)](https://www.mouser.com/datasheet/2/389/steval-drone01-1500114.pdf)
- [STSW-FCU001 Firmware](https://www.st.com/en/embedded-software/stsw-fcu001.html)
- [LSM6DSL Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsl.pdf)
- [LIS2MDL Datasheet](https://www.st.com/resource/en/datasheet/lis2mdl.pdf)
- [LPS22HD Datasheet](https://www.st.com/resource/en/datasheet/lps22hd.pdf)

## TODO

### Completed
- [x] LSM6DSL driver (SPI) - **Fully integrated**
- [x] LIS2MDL driver (I2C) - **Fully integrated**
- [x] LPS22HD driver (I2C) - **Fully integrated**
- [x] Motor driver (TIM4) - **Fully integrated**
- [x] Complementary filter for attitude
- [x] Platform init and main loop
- [x] Example main.c with PID control
- [x] Makefile for ARM GCC build
- [x] Linker script (stm32f401_flash.ld)
- [x] Startup code (startup_stm32f401.s)
- [x] System clock configuration (84MHz)
- [x] GPIO pin configuration
- [x] SPI1 peripheral driver
- [x] I2C1 peripheral driver (400kHz Fast Mode)
- [x] TIM4 PWM driver (20kHz, 10-bit resolution)

### Remaining
- [ ] USART1 debug output (optional)
- [ ] hive runtime port for STM32F4
