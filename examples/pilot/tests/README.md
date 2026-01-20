# HAL Validation Tests

Platform-independent tests for validating the HAL (Hardware Abstraction Layer).

These tests use the HAL API and can be built for any platform that implements
the HAL interface (Crazyflie 2.1+, Webots simulation, etc.).

For bare-metal hardware tests (direct register access), see
`hal/crazyflie-2.1+/tests/`.

## Tests

### hal_test

Validates the complete HAL API stack used by the pilot application.

**Test sequence:**
1. `hal_init()` - Initialize all hardware
2. `hal_self_test()` - Verify sensors respond
3. `hal_calibrate()` - Calibrate sensors (keep still and level!)
4. `hal_arm()` - Arm motors
5. Sensor loop - Read sensors for 5 seconds
6. Motor test - Brief motor spin (2 seconds)
7. `hal_disarm()` - Disarm motors
8. `hal_cleanup()` - Cleanup

## Build

### Crazyflie 2.1+

```bash
make PLATFORM=crazyflie
make flash-crazyflie
```

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make PLATFORM=webots
```

Then copy `build_webots/hal_test` to a Webots controller directory.

## LED Feedback (Crazyflie)

On hardware platforms with an LED, the test provides visual feedback:

| Pattern | Meaning |
|---------|---------|
| 1-3 blinks | hal_init() progress (from HAL) |
| 2 blinks | hal_init() passed, starting self-test |
| 3 blinks | hal_self_test() passed, starting calibration |
| Slow blink | Calibration in progress (from HAL) |
| 4 blinks | Calibration done, starting sensor read |
| Fast blink | Sensor read loop (5 seconds) |
| 5 blinks | Sensor test done, starting motor test |
| Medium blink | Motors running (2 seconds) |
| 6 blinks | Motor test done |
| LED on solid | All tests passed! |

### Error Patterns

| Pattern | Meaning |
|---------|---------|
| 3-5 fast blinks | hal_init() failed |
| 6-9 fast blinks | hal_self_test() failed |
| 10 fast blinks | Sensor data out of range |
| Continuous slow | Fatal error |

## Webots Notes

- LED functions are no-ops (no visual feedback)
- Delays advance simulation time via `wb_robot_step()`
- Test runs to completion in simulation time

## Adding New Platforms

To add support for a new platform:

1. Implement the HAL interface in `hal/<platform>/`:
   - `hal_init()`, `hal_cleanup()`, `hal_self_test()`, `hal_calibrate()`
   - `hal_arm()`, `hal_disarm()`
   - `hal_read_sensors()`, `hal_write_torque()`
   - `hal_delay_ms()`, `hal_get_time_ms()`
   - `hal_led_on()`, `hal_led_off()`, `hal_led_toggle()`

2. Add a build section to the Makefile for the new platform

3. Test with `make PLATFORM=<new_platform>`
