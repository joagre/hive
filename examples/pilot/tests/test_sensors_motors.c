/**
 * Sensors and Motors Test (platform-independent)
 *
 * Tests the complete pilot HAL API stack: sensors, motors, calibration,
 * and timing functions. Can be built for any platform that implements
 * the HAL interface.
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
 *   Crazyflie: make PLATFORM=crazyflie TEST=sensors_motors
 *              make flash-crazyflie TEST=sensors_motors
 *   Webots:    make PLATFORM=webots TEST=sensors_motors
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"
#include "types.h"

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Public Interface (for test_main.c)
// ============================================================================

int test_sensors_motors_run(bool standalone);

// ============================================================================
// Test Configuration
// ============================================================================

#define SENSOR_TEST_DURATION_MS 5000 // Read sensors for 5 seconds
#define MOTOR_TEST_DURATION_MS 2000  // Run motors for 2 seconds
#define MOTOR_TEST_THRUST 0.15f      // Low thrust for safety

// ============================================================================
// LED Helpers (use pilot HAL functions)
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
// Test Implementation
// ============================================================================

/**
 * Run the sensors and motors test.
 *
 * @param standalone If true, initializes HAL/Hive and handles cleanup.
 *                   If false, assumes caller has already initialized.
 * @return 0 on success, -1 on failure
 */
int test_sensors_motors_run(bool standalone) {
    if (standalone) {
        // Initialize pilot HAL for LED, timing, and debug output
        hal_debug_init();

        // Note: We can't blink before hal_init() since GPIO isn't initialized yet.
        // hal_init() will show its own LED feedback (1-3 blinks for progress).
        if (hal_init() != 0) {
            return -1;
        }

        // Initialize Hive runtime for logging
        hive_status_t status = hive_init();
        if (HIVE_FAILED(status)) {
            return -1;
        }
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Sensors and Motors Test (Pilot HAL)");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("Phase 1: hal_init() PASS");

    // 2 blinks = init passed, starting self-test
    hal_delay_ms(500);
    test_blink(2, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 2: hal_self_test()
    // ========================================================================

    HIVE_LOG_INFO("Phase 2: hal_self_test()");
    if (!hal_self_test()) {
        HIVE_LOG_ERROR("hal_self_test() FAIL");
        if (standalone) {
            hive_cleanup();
        }
        return -1;
    }
    HIVE_LOG_INFO("hal_self_test() PASS");

    // 3 blinks = self-test passed, starting calibration
    hal_delay_ms(500);
    test_blink(3, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 3: hal_calibrate()
    // ========================================================================

    HIVE_LOG_INFO("Phase 3: hal_calibrate()");
    // hal_calibrate() will show slow blink during calibration
    // IMPORTANT: Keep drone still and level during this phase!
    hal_calibrate();
    HIVE_LOG_INFO("hal_calibrate() PASS");

    // 4 blinks = calibration done, starting sensor test
    hal_delay_ms(500);
    test_blink(4, 200, 200);
    hal_delay_ms(500);

    // ========================================================================
    // Phase 4: Sensor Read Test
    // ========================================================================

    HIVE_LOG_INFO("Phase 4: Sensor read test (%d ms)", SENSOR_TEST_DURATION_MS);

    // Read first sample while drone is still level/stationary
    sensor_data_t first_sensors;
    hal_read_sensors(&first_sensors);

    // Continue reading for SENSOR_TEST_DURATION_MS to test sustained operation
    sensor_data_t sensors;
    uint32_t start_time = hal_get_time_ms();
    uint32_t last_blink = start_time;
    int read_count = 1; // Already read one

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
    HIVE_LOG_INFO("Read %d sensor samples", read_count);

    // Verify FIRST sample (when drone was still level/stationary)
    // Accel Z should be approximately +9.8 m/s^2 when level
    // (accelerometer measures reaction force against gravity)
    // Gyro should be near zero if stationary
    bool accel_ok =
        (first_sensors.accel[2] > 5.0f && first_sensors.accel[2] < 15.0f);
    bool gyro_ok =
        (first_sensors.gyro[0] > -1.0f && first_sensors.gyro[0] < 1.0f &&
         first_sensors.gyro[1] > -1.0f && first_sensors.gyro[1] < 1.0f &&
         first_sensors.gyro[2] > -1.0f && first_sensors.gyro[2] < 1.0f);

    HIVE_LOG_INFO("First sample - Accel Z: %.2f m/s^2 (%s)",
                  first_sensors.accel[2], accel_ok ? "OK" : "FAIL");
    HIVE_LOG_INFO("First sample - Gyro: [%.3f, %.3f, %.3f] rad/s (%s)",
                  first_sensors.gyro[0], first_sensors.gyro[1],
                  first_sensors.gyro[2], gyro_ok ? "OK" : "FAIL");

    if (!accel_ok || !gyro_ok) {
        HIVE_LOG_ERROR("Sensor data out of range");
        if (standalone) {
            test_blink(10, 50, 50); // 10 fast blinks = sensor data error
            hive_cleanup();
        }
        return -1;
    }
    HIVE_LOG_INFO("Sensor read test PASS");

    // 5 blinks = sensor test passed, starting motor test
    hal_delay_ms(500);
    test_blink(5, 200, 200);
    hal_delay_ms(1000); // Extra delay before motors

    // ========================================================================
    // Phase 5: Motor Test
    // ========================================================================

    HIVE_LOG_INFO("Phase 5: Motor test (%d ms at %.0f%% thrust)",
                  MOTOR_TEST_DURATION_MS, MOTOR_TEST_THRUST * 100);
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
    HIVE_LOG_INFO("Motor test PASS");

    // 6 blinks = motor test done
    hal_delay_ms(500);
    test_blink(6, 200, 200);

    // ========================================================================
    // Phase 6: Cleanup
    // ========================================================================

    HIVE_LOG_INFO("Phase 6: hal_cleanup()");
    hal_cleanup();

    // ========================================================================
    // All Tests Passed!
    // ========================================================================

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  ALL TESTS PASSED!");
    HIVE_LOG_INFO("========================================");

    if (standalone) {
        hive_cleanup();
    }

    return 0;
}

// ============================================================================
// Standalone Entry Point
// ============================================================================

#ifndef TEST_MAIN_BUILD
int main(void) {
    int result = test_sensors_motors_run(true);

    if (result != 0) {
        error_blink_forever(100, 100);
    }

    // Solid LED = success, stay forever
    hal_led_on();
    while (1) {
        hal_delay_ms(1000);
    }

    return 0;
}
#endif
