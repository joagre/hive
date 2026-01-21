/**
 * HAL validation test (platform-independent)
 *
 * Tests the complete HAL API stack used by the pilot application.
 * Can be built for any platform that implements the HAL interface.
 *
 * Test sequence:
 *   1. hal_init()      - Initialize all hardware
 *   2. hal_self_test() - Verify sensors respond
 *   3. hal_calibrate() - Calibrate sensors (keep still and level!)
 *   4. hal_arm()       - Arm motors
 *   5. Sensor loop     - Read sensors for 5 seconds
 *   6. Motor test      - Brief motor spin (2 seconds)
 *   7. hal_disarm()    - Disarm motors
 *   8. hal_cleanup()   - Cleanup
 *
 * LED feedback (hardware platforms with LED):
 *   1-3 blinks         = hal_init() progress (from HAL)
 *   2 blinks           = hal_init() passed, starting self-test
 *   3 blinks           = hal_self_test() passed, starting calibration
 *   Slow blink         = Calibration in progress (from HAL)
 *   4 blinks           = Calibration done, starting sensor read
 *   Fast blink         = Sensor read loop (5 seconds)
 *   5 blinks           = Sensor test done, starting motor test
 *   Medium blink       = Motors running (2 seconds)
 *   6 blinks           = Motor test done
 *   LED on solid       = All tests passed!
 *
 * Error patterns:
 *   3-5 fast blinks    = hal_init() failed (from HAL)
 *   6-9 fast blinks    = hal_self_test() failed (from HAL)
 *   10 fast blinks     = Sensor data out of range
 *   Continuous slow    = Fatal error
 *
 * Usage:
 *   Crazyflie: make PLATFORM=crazyflie && make flash-crazyflie
 *   Webots:    make PLATFORM=webots && run in simulation
 */

#include "hal/hal.h"
#include "types.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Console output for platforms without LED (Webots)
#ifdef SIMULATED_TIME
#define LOG(fmt, ...) printf("[HAL_TEST] " fmt "\n", ##__VA_ARGS__)
#else
#define LOG(fmt, ...) ((void)0)
#endif

// ============================================================================
// Test Configuration
// ============================================================================

#define SENSOR_TEST_DURATION_MS 5000 // Read sensors for 5 seconds
#define MOTOR_TEST_DURATION_MS 2000  // Run motors for 2 seconds
#define MOTOR_TEST_THRUST 0.15f      // Low thrust for safety

// ============================================================================
// LED Helpers (use HAL functions)
// ============================================================================

static void test_blink(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        hal_led_on();
        hal_delay_ms(on_ms);
        hal_led_off();
        hal_delay_ms(off_ms);
    }
}

static void error_blink_forever(int on_ms, int off_ms) {
    while (1) {
        hal_led_toggle();
        hal_delay_ms(on_ms > 0 ? on_ms : off_ms);
    }
}

// ============================================================================
// Main Test
// ============================================================================

int main(void) {
    // ========================================================================
    // Phase 1: hal_init()
    // ========================================================================

    // Note: We can't blink before hal_init() since GPIO isn't initialized yet.
    // hal_init() will show its own LED feedback (1-3 blinks for progress).

    LOG("Phase 1: hal_init()");
    if (hal_init() != 0) {
        LOG("FAIL: hal_init() returned error");
        // hal_init() already showed error blinks, just loop forever
        error_blink_forever(100, 100);
    }
    LOG("PASS: hal_init()");

    // 2 blinks = init passed, starting self-test
    hal_delay_ms(500);
    test_blink(2, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 2: hal_self_test()
    // ========================================================================

    LOG("Phase 2: hal_self_test()");
    if (!hal_self_test()) {
        LOG("FAIL: hal_self_test() returned false");
        // hal_self_test() already showed error blinks (6-9)
        error_blink_forever(100, 100);
    }
    LOG("PASS: hal_self_test()");

    // 3 blinks = self-test passed, starting calibration
    hal_delay_ms(500);
    test_blink(3, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 3: hal_calibrate()
    // ========================================================================

    LOG("Phase 3: hal_calibrate()");
    // hal_calibrate() will show slow blink during calibration
    // IMPORTANT: Keep drone still and level during this phase!
    hal_calibrate();
    LOG("PASS: hal_calibrate()");

    // 4 blinks = calibration done, starting sensor test
    hal_delay_ms(500);
    test_blink(4, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 4: Sensor Read Test
    // ========================================================================

    LOG("Phase 4: Sensor read test (%d ms)", SENSOR_TEST_DURATION_MS);
    // Read sensors for SENSOR_TEST_DURATION_MS, fast blink LED
    sensor_data_t sensors;
    uint32_t start_time = hal_get_time_ms();
    uint32_t last_blink = start_time;
    int read_count = 0;

    while ((hal_get_time_ms() - start_time) < SENSOR_TEST_DURATION_MS) {
        hal_read_sensors(&sensors);
        read_count++;

        // Fast blink (10 Hz)
        if ((hal_get_time_ms() - last_blink) >= 50) {
            hal_led_toggle();
            last_blink = hal_get_time_ms();
        }

        hal_delay_ms(4); // ~250 Hz sample rate
    }

    hal_led_off();
    LOG("Read %d sensor samples", read_count);

    // Verify we got reasonable data
    // Accel Z should be approximately -9.8 m/s^2 (gravity)
    // Gyro should be near zero if stationary
    bool accel_ok = (sensors.accel[2] < -5.0f && sensors.accel[2] > -15.0f);
    bool gyro_ok = (sensors.gyro[0] > -1.0f && sensors.gyro[0] < 1.0f &&
                    sensors.gyro[1] > -1.0f && sensors.gyro[1] < 1.0f &&
                    sensors.gyro[2] > -1.0f && sensors.gyro[2] < 1.0f);

    LOG("Accel Z: %.2f m/s^2 (%s)", sensors.accel[2], accel_ok ? "OK" : "FAIL");
    LOG("Gyro: [%.3f, %.3f, %.3f] rad/s (%s)", sensors.gyro[0], sensors.gyro[1],
        sensors.gyro[2], gyro_ok ? "OK" : "FAIL");

    if (!accel_ok || !gyro_ok) {
        LOG("FAIL: Sensor data out of range");
        // Sensor values out of range - show error
        test_blink(10, 50, 50); // 10 fast blinks = sensor data error
        error_blink_forever(300, 300);
    }
    LOG("PASS: Sensor read test");

    // 5 blinks = sensor test passed, starting motor test
    hal_delay_ms(500);
    test_blink(5, 200, 200);
    hal_delay_ms(1000); // Extra delay before motors

    // ========================================================================
    // Phase 5: Motor Test
    // ========================================================================

    LOG("Phase 5: Motor test (%d ms at %.0f%% thrust)", MOTOR_TEST_DURATION_MS,
        MOTOR_TEST_THRUST * 100);
    hal_arm();

    // Run motors at low thrust for MOTOR_TEST_DURATION_MS
    torque_cmd_t cmd = {
        .thrust = MOTOR_TEST_THRUST, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f};

    start_time = hal_get_time_ms();
    last_blink = start_time;

    while ((hal_get_time_ms() - start_time) < MOTOR_TEST_DURATION_MS) {
        hal_write_torque(&cmd);

        // Medium blink (5 Hz)
        if ((hal_get_time_ms() - last_blink) >= 100) {
            hal_led_toggle();
            last_blink = hal_get_time_ms();
        }

        hal_delay_ms(4);
    }

    // Stop motors
    cmd.thrust = 0.0f;
    hal_write_torque(&cmd);
    hal_disarm();

    hal_led_off();
    LOG("PASS: Motor test");

    // 6 blinks = motor test done
    hal_delay_ms(500);
    test_blink(6, 200, 200);

    // ========================================================================
    // Phase 6: Cleanup
    // ========================================================================

    LOG("Phase 6: hal_cleanup()");
    hal_cleanup();

    // ========================================================================
    // All Tests Passed!
    // ========================================================================

    LOG("========================================");
    LOG("ALL TESTS PASSED!");
    LOG("========================================");

    // Solid LED = success
    hal_led_on();

    // Stay in success state forever
    while (1) {
        // Could add a slow heartbeat blink here if preferred
        hal_delay_ms(1000);
    }

    return 0;
}
