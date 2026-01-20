# Crazyflie 2.1+ Hardware Test Programs

Standalone test programs for bare-metal hardware verification.
These use direct register access (no HAL or Hive runtime).

For HAL validation tests, see `examples/pilot/tests/`.

## Programs

### thrust_test

Runs all 4 motors at equal thrust for calibration.

**Purpose:** Find the hover thrust value for `HAL_BASE_THRUST` in hal_config.h.

**Procedure:**
1. REMOVE PROPELLERS!
2. Build and flash: `make thrust_test && make flash-thrust`
3. Watch motors spin for 5 seconds
4. Edit `TEST_THRUST` in thrust_test.c (start at 0.15, increase by 0.05)
5. Repeat until drone starts to lift
6. Set `HAL_BASE_THRUST` to ~90% of liftoff thrust

### sensor_motor_test

Tests each motor individually and detects rotation direction using the BMI088 gyroscope.

**Purpose:** Verify motor wiring and rotation direction matches expected configuration.

**Procedure:**
1. REMOVE PROPELLERS!
2. Build and flash: `make sensor_motor_test && make flash-sensor`
3. Watch LED and motors:
   - N blinks = motor N is about to spin
   - After spin: 1 blink = CCW, 2 blinks = CW, 3 blinks = unclear
4. After all 4 motors: all motors spin together for balance test

### baro_test

Tests the BMP388 barometer sensor over I2C.

**Purpose:** Verify BMP388 barometer communication and pressure readings.

**Procedure:**
1. Build and flash: `make baro_test && make flash-baro`
2. Watch LED for feedback
3. Test reads pressure for 5 seconds and validates readings

### flow_test

Tests the Flow Deck v2 sensors: PMW3901 optical flow (SPI) and VL53L1x ToF (I2C).

**Purpose:** Verify Flow Deck v2 is connected and sensors respond correctly.

**Procedure:**
1. Attach Flow Deck v2 to the Crazyflie
2. Build and flash: `make flow_test && make flash-flow`
3. Watch LED for feedback
4. Place drone over a textured surface for best flow readings

## Build

```bash
make thrust_test          # Build thrust test
make sensor_motor_test    # Build sensor/motor test
make baro_test            # Build barometer test
make flow_test            # Build flow deck test
make all                  # Build all tests
make clean                # Clean
```

## Flash

Requires ST-Link debugger (Bitcraze debug adapter).

```bash
make flash-thrust         # Flash thrust_test
make flash-sensor         # Flash sensor_motor_test
make flash-baro           # Flash baro_test
make flash-flow           # Flash flow_test
```

## LED Feedback

The blue LED on PC4 provides status feedback (no serial output needed).

### thrust_test

| Pattern | Meaning |
|---------|---------|
| 2 slow blinks | Starting - secure the drone! |
| 3 quick blinks | Motors initialized |
| Fast blink (5 Hz) | Motors running at TEST_THRUST |
| 5 slow blinks | Test complete |
| Continuous fast | Error: motor init failed |

### sensor_motor_test

| Pattern | Meaning |
|---------|---------|
| 2 slow blinks | Starting |
| 3 quick blinks | Motors OK |
| 4 quick blinks | Gyro OK |
| N blinks | Testing motor N (1-4) |
| Fast blink | Motor spinning |
| 1 slow blink | CCW rotation detected |
| 2 slow blinks | CW rotation detected |
| 3 slow blinks | Rotation unclear |
| 10 fast blinks | All motors test |
| 1 slow blink (after all) | Balanced - good! |
| 2 slow blinks (after all) | Net CCW rotation |
| 3 slow blinks (after all) | Net CW rotation |
| Continuous slow | Test complete |
| Continuous fast | Motor init failed |
| Continuous medium | Gyro init failed |

### baro_test

| Pattern | Meaning |
|---------|---------|
| 2 slow blinks | Starting |
| 3 quick blinks | I2C initialized |
| 4 quick blinks | BMP388 chip ID verified |
| 5 quick blinks | Calibration data read |
| Fast blink | Reading sensor data |
| LED on 1s | Valid pressure readings |
| Continuous slow | Test complete |
| Continuous fast | I2C init failed |
| Continuous medium | Chip ID mismatch |

### flow_test

| Pattern | Meaning |
|---------|---------|
| 2 slow blinks | Starting |
| 3 quick blinks | SPI initialized |
| 4 quick blinks | PMW3901 chip ID verified |
| 5 quick blinks | I2C initialized |
| 6 quick blinks | VL53L1x chip ID verified |
| Fast blink | Reading sensors |
| 7 blinks | PMW3901 test passed |
| 8 blinks | VL53L1x test passed |
| 9 blinks | Both sensors passed |
| Continuous slow | Test complete |
| Continuous fast | SPI init failed |
| Continuous medium | PMW3901 not detected |
| Double blink | VL53L1x not detected |

## Expected Motor Configuration

```
        Front
    M1(CCW)  M2(CW)
        +--+
        |  |
        +--+
    M4(CW)  M3(CCW)
        Rear

TIM2 PWM mapping:
  M1 (front-left, CCW)  -> TIM2_CH1 (PA0)
  M2 (front-right, CW)  -> TIM2_CH2 (PA1)
  M3 (rear-right, CCW)  -> TIM2_CH3 (PA2)
  M4 (rear-left, CW)    -> TIM2_CH4 (PA3)
```

**Diagonal pairs must have same rotation:**
- M1 + M3 = both CCW
- M2 + M4 = both CW

If rotation is wrong, the motor connector can be reversed (swap the two wires).

## Hardware Notes

- **MCU:** STM32F405RG @ 168 MHz
- **Motors:** Brushed coreless, TIM2 PWM (PA0-PA3)
- **IMU:** BMI088 on I2C3 (PA8/PC9, accel addr: 0x18, gyro addr: 0x68)
- **Barometer:** BMP388 on I2C3 (PA8/PC9, addr: 0x77)
- **Flow Deck v2 (optional):**
  - PMW3901 optical flow on SPI1 (PA5/PA6/PA7, CS: PB12)
  - VL53L1x ToF on I2C3 (addr: 0x29)
- **LED:** Blue LED on PC4

See `../README.md` for detailed pin mappings and hardware specifications.
