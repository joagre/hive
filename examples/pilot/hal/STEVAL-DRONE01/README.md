# STEVAL-DRONE01 Hardware Abstraction Layer

Platform layer for the STMicroelectronics STEVAL-DRONE01 mini drone kit.

## Hardware Overview

| Component | Part Number | Interface | Description |
|-----------|-------------|-----------|-------------|
| MCU | STM32F401 | - | ARM Cortex-M4, 84 MHz, DSP+FPU |
| IMU | LSM6DSL | SPI1 | 6-axis accel + gyro |
| Magnetometer | LIS2MDL | I2C | 3-axis compass |
| Barometer | LPS22HD | I2C | Pressure/altitude |
| Bluetooth | SPBTLE-RF | SPI2 | BLE 4.1 for RC |
| Motors | 85x20mm | TIM4 PWM | 3.7V brushed, x4 |

## Specifications

**Flight Controller (STEVAL-FCU001V1):**
- STM32F401 @ 84 MHz (Cortex-M4F)
- 256 KB Flash, 64 KB RAM
- Hardware FPU for fast sensor fusion

**Sensors:**
- LSM6DSL: ±2/4/8/16g accel, ±125/250/500/1000/2000 dps gyro
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

## Pin Mapping

```
Motors (TIM4 PWM):
  M1 (rear-left)   - TIM4_CH1
  M2 (front-left)  - TIM4_CH2
  M3 (front-right) - TIM4_CH3
  M4 (rear-right)  - TIM4_CH4

Sensors:
  LSM6DSL - SPI1 (accel/gyro)
  LIS2MDL - I2C1 (magnetometer)
  LPS22HD - I2C1 (barometer)

Bluetooth:
  SPBTLE-RF - SPI2

Debug:
  UART1 - Serial debug
  JTAG  - Programming/debug
```

## Differences from Webots Simulation

| Feature | Webots | STEVAL-DRONE01 |
|---------|--------|----------------|
| Position | GPS (perfect) | None (need external tracking) |
| Attitude | Inertial unit (fused) | Raw IMU (need sensor fusion) |
| Altitude | GPS Z | Barometer (relative only) |
| Heading | Direct yaw | Magnetometer (calibration needed) |
| Timing | Simulation step | Real-time (hardware timers) |

## Sensor Fusion Requirements

The Webots `inertial_unit` provides pre-fused roll/pitch/yaw. On real hardware, the estimator actor must implement:

1. **Complementary filter** or **Kalman filter** for attitude
2. **Gyro integration** for angular rates
3. **Magnetometer fusion** for yaw (with tilt compensation)
4. **Barometer filtering** for altitude hold

## Driver Files

| File | Sensor | Interface | Purpose |
|------|--------|-----------|---------|
| `lsm6dsl.h/c` | LSM6DSL | SPI1 | 6-axis IMU for attitude estimation (roll/pitch/rates) |
| `lis2mdl.h/c` | LIS2MDL | I2C1 | 3-axis magnetometer for heading (yaw) |
| `lps22hd.h/c` | LPS22HD | I2C1 | Barometer for altitude hold |
| `motors.h/c` | - | TIM4 PWM | Motor output with arm/disarm safety |
| `attitude.h/c` | - | - | Complementary filter for sensor fusion |

All drivers are skeleton implementations with TODO placeholders for STM32 HAL integration.

### LSM6DSL (IMU)

6-axis accelerometer + gyroscope via SPI.

```c
lsm6dsl_init(NULL);                    // Use default config (±4g, ±500dps, 416Hz)
lsm6dsl_read_all(&accel, &gyro);       // Burst read both sensors
// accel: m/s², gyro: rad/s
```

Key features:
- Configurable full-scale (±2/4/8/16g accel, ±250/500/1000/2000 dps gyro)
- Configurable ODR (12.5Hz to 1.66kHz)
- Burst read for efficient data acquisition
- Temperature sensor

### LIS2MDL (Magnetometer)

3-axis magnetometer via I2C for heading estimation.

```c
lis2mdl_init(NULL);                    // Use default config (50Hz, continuous)
lis2mdl_read(&mag);                    // Read in microtesla
float heading = lis2mdl_heading_tilt_compensated(&mag, roll, pitch);
```

Key features:
- Temperature compensation
- Hard-iron calibration support (offset registers + software)
- Tilt-compensated heading calculation
- Low-pass filter option

### LPS22HD (Barometer)

Pressure sensor via I2C for altitude hold.

```c
lps22hd_init(NULL);                    // Use default config (50Hz)
lps22hd_set_reference(lps22hd_read_pressure());  // Set ground level
float alt = lps22hd_read_altitude();   // Meters above ground
```

Key features:
- Configurable ODR (1-75Hz) + one-shot mode
- Low-pass filter for noise reduction
- Barometric altitude calculation
- Reference pressure for relative altitude

**Note:** Barometer provides relative altitude only. Weather changes cause drift.

### Motors

TIM4 PWM output for 4 brushed DC motors.

```c
motors_init(NULL);                     // Use default config (20kHz PWM)
motors_arm();                          // Enable PWM output
motors_set(&cmd);                      // Set motor speeds (0.0-1.0)
motors_emergency_stop();               // Immediate stop + disarm
```

Key features:
- Arm/disarm safety (motors won't spin until armed)
- Normalized input (0.0-1.0) with configurable PWM range
- Emergency stop function
- Individual motor control for testing

Motor layout (X configuration):
```
        Front
      M2    M3
        \  /
         \/
         /\
        /  \
      M1    M4
        Rear
```

### Attitude (Complementary Filter)

Sensor fusion for attitude estimation from IMU and magnetometer.

```c
attitude_init(NULL, NULL);             // Use default config (alpha=0.98)

// Main loop (e.g., 400Hz)
lsm6dsl_read_all(&accel, &gyro);
attitude_update(accel, gyro, dt);      // Fuse accel + gyro

// Optional: Add magnetometer for yaw (e.g., 50Hz)
lis2mdl_read(&mag);
attitude_update_mag(mag);

// Get results
attitude_t att;
attitude_get(&att);                    // roll, pitch, yaw in radians
```

Key features:
- Complementary filter: `angle = alpha * gyro + (1-alpha) * accel`
- Configurable filter coefficient (default alpha=0.98)
- Accelerometer validity check (rejects data during high-g maneuvers)
- Optional magnetometer fusion for yaw (tilt-compensated)
- Angle normalization to [-PI, PI]

**Note:** This is a simple complementary filter suitable for basic flight. For aggressive maneuvers or precision applications, consider an Extended Kalman Filter (EKF) or Mahony/Madgwick filter.

## Resources

- [STEVAL-DRONE01 Product Page](https://www.st.com/en/evaluation-tools/steval-drone01.html)
- [Datasheet (PDF)](https://www.mouser.com/datasheet/2/389/steval-drone01-1500114.pdf)
- [STSW-FCU001 Firmware](https://www.st.com/en/embedded-software/stsw-fcu001.html)

## TODO

- [x] LSM6DSL driver skeleton (SPI)
- [x] LIS2MDL driver skeleton (I2C)
- [x] LPS22HD driver skeleton (I2C)
- [x] Motor PWM driver skeleton (TIM4)
- [x] Complementary filter for attitude estimation
- [ ] STM32 HAL integration (implement TODO placeholders)
- [ ] Platform init and main loop
- [ ] hive runtime port for STM32F4
