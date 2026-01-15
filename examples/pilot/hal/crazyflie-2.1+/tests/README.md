# Crazyflie 2.1+ Test Programs

Standalone test programs for verifying the Crazyflie 2.1+ HAL hardware.

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

## Build

```bash
make thrust_test          # Build thrust test
make sensor_motor_test    # Build sensor/motor test
make all                  # Build both
make clean                # Clean
```

## Flash

Requires ST-Link debugger (Bitcraze debug adapter).

```bash
make flash-thrust         # Flash thrust_test
make flash-sensor         # Flash sensor_motor_test
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
  M1 (front-left, CCW)  → TIM2_CH1 (PA0)
  M2 (front-right, CW)  → TIM2_CH2 (PA1)
  M3 (rear-right, CCW)  → TIM2_CH3 (PA2)
  M4 (rear-left, CW)    → TIM2_CH4 (PA3)
```

**Diagonal pairs must have same rotation:**
- M1 + M3 = both CCW
- M2 + M4 = both CW

If rotation is wrong, the motor connector can be reversed (swap the two wires).

## Hardware Notes

- **MCU:** STM32F405RG @ 168 MHz
- **Motors:** Brushed coreless, TIM2 PWM at ~328 kHz
- **Gyro:** BMI088 on SPI1 (PA5/PA6/PA7), CS on PB4
- **LED:** Blue LED on PC4

These tests are self-contained and don't use the full HAL or Hive runtime.
