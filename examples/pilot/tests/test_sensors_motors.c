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
 *
 * Motor test is disabled by default for safety. Enable with:
 *   make PLATFORM=crazyflie TEST=sensors_motors ENABLE_MOTOR_TEST=1
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"
#include "platform.h"
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

// Set ENABLE_MOTOR_TEST=1 to enable motor test (disabled by default for safety)
#ifndef ENABLE_MOTOR_TEST
#define ENABLE_MOTOR_TEST 0
#endif

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

__attribute__((unused)) static void error_blink_forever(int on_ms, int off_ms) {
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

    // Check for flow deck
    bool has_flow = platform_has_flow_deck();
    if (has_flow) {
        HIVE_LOG_INFO("Flow deck detected");
    } else {
        HIVE_LOG_INFO("No flow deck detected");
    }

    // Read first sample while drone is still level/stationary
    sensor_data_t first_sensors;
    hal_read_sensors(&first_sensors);

    // Read flow deck if present
    int16_t first_flow_x = 0, first_flow_y = 0;
    uint16_t first_height = 0;
    if (has_flow) {
        platform_read_flow(&first_flow_x, &first_flow_y);
        platform_read_height(&first_height);
    }

    // Continue reading for SENSOR_TEST_DURATION_MS to test sustained operation
    sensor_data_t sensors;
    uint32_t start_time = hal_get_time_ms();
    uint32_t last_blink = start_time;
    uint32_t last_print = start_time;
    int read_count = 1; // Already read one
    int16_t flow_x = 0, flow_y = 0;
    uint16_t height_mm = 0;

    while ((hal_get_time_ms() - start_time) < SENSOR_TEST_DURATION_MS) {
        hal_read_sensors(&sensors);
        if (has_flow) {
            platform_read_flow(&flow_x, &flow_y);
            platform_read_height(&height_mm);
        }
        read_count++;

        // Print sensor values every 500ms
        if ((hal_get_time_ms() - last_print) >= 500) {
            HIVE_LOG_INFO("Accel: [%.2f, %.2f, %.2f] Gyro: [%.3f, %.3f, %.3f]",
                          sensors.accel[0], sensors.accel[1], sensors.accel[2],
                          sensors.gyro[0], sensors.gyro[1], sensors.gyro[2]);
            HIVE_LOG_INFO("Baro: P=%.1f hPa T=%.2f C", sensors.pressure_hpa,
                          sensors.baro_temp_c);
            if (has_flow) {
                HIVE_LOG_INFO("Flow: dx=%d dy=%d Height: %u mm", flow_x, flow_y,
                              height_mm);
                // Also show integrated values from hal_read_sensors()
                HIVE_LOG_INFO("Integrated: pos=(%.3f, %.3f, %.3f) vel=(%.3f, "
                              "%.3f) valid=%d,%d",
                              sensors.gps_x, sensors.gps_y, sensors.gps_z,
                              sensors.velocity_x, sensors.velocity_y,
                              sensors.gps_valid, sensors.velocity_valid);
            }
            last_print = hal_get_time_ms();
        }

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

    // Check flow deck values if present
    if (has_flow) {
        HIVE_LOG_INFO("First sample - Flow: dx=%d dy=%d Height: %u mm",
                      first_flow_x, first_flow_y, first_height);
        // Height should be reasonable (not 0, not maxed out)
        if (first_height == 0 || first_height > 4000) {
            HIVE_LOG_WARN("Height reading may be invalid: %u mm", first_height);
        }
        // Check integrated values from hal_read_sensors()
        HIVE_LOG_INFO("First sample - Integrated pos: (%.3f, %.3f, %.3f) m",
                      first_sensors.gps_x, first_sensors.gps_y,
                      first_sensors.gps_z);
        HIVE_LOG_INFO(
            "First sample - Integrated vel: (%.3f, %.3f) m/s, valid=%d",
            first_sensors.velocity_x, first_sensors.velocity_y,
            first_sensors.velocity_valid);
        HIVE_LOG_INFO("First sample - GPS valid: %d", first_sensors.gps_valid);
    }

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
    // Phase 5: Motor Test (Individual motors for CW/CCW inspection)
    // ========================================================================

#if ENABLE_MOTOR_TEST
    // Motor mapping from hal_config.h (rotation viewed from above):
    // M1 (motor[0]) - Front Right (CCW)
    // M2 (motor[1]) - Back Right (CW)
    // M3 (motor[2]) - Back Left (CCW)
    // M4 (motor[3]) - Front Left (CW)
    static const char *motor_names[] = {
        "M1 Front-Right (CCW)", "M2 Back-Right (CW)", "M3 Back-Left (CCW)",
        "M4 Front-Left (CW)"};

    HIVE_LOG_INFO("Phase 5: Individual motor test (%.0f%% thrust each)",
                  MOTOR_TEST_THRUST * 100);
    HIVE_LOG_INFO("Each motor runs for %d ms - verify CW/CCW rotation",
                  MOTOR_TEST_DURATION_MS);

    hal_arm();

    // Test each motor individually
    for (int m = 0; m < 4; m++) {
        HIVE_LOG_INFO("Starting: %s", motor_names[m]);

        motor_cmd_t cmd = MOTOR_CMD_ZERO;
        cmd.motor[m] = MOTOR_TEST_THRUST;

        start_time = hal_get_time_ms();
        last_blink = start_time;

        while ((hal_get_time_ms() - start_time) < MOTOR_TEST_DURATION_MS) {
            platform_write_motors(&cmd);

            // Blink pattern: number of blinks = motor number (1-4)
            if ((hal_get_time_ms() - last_blink) >= 100) {
                hal_led_toggle();
                last_blink = hal_get_time_ms();
            }

            hal_delay_ms(4);
        }

        // Stop this motor
        cmd.motor[m] = 0.0f;
        platform_write_motors(&cmd);
        hal_led_off();

        HIVE_LOG_INFO("Stopped: %s", motor_names[m]);

        // Pause between motors (except after last)
        if (m < 3) {
            HIVE_LOG_INFO("Pause 2s before next motor...");
            hal_delay_ms(2000);
        }
    }

    // All motors test
    HIVE_LOG_INFO("All motors together for %d ms...", MOTOR_TEST_DURATION_MS);
    {
        motor_cmd_t cmd = {.motor = {MOTOR_TEST_THRUST, MOTOR_TEST_THRUST,
                                     MOTOR_TEST_THRUST, MOTOR_TEST_THRUST}};

        start_time = hal_get_time_ms();
        last_blink = start_time;

        while ((hal_get_time_ms() - start_time) < MOTOR_TEST_DURATION_MS) {
            platform_write_motors(&cmd);

            if ((hal_get_time_ms() - last_blink) >= 50) {
                hal_led_toggle();
                last_blink = hal_get_time_ms();
            }

            hal_delay_ms(4);
        }
    }

    // Stop all motors
    motor_cmd_t stop_cmd = MOTOR_CMD_ZERO;
    platform_write_motors(&stop_cmd);
    hal_disarm();

    hal_led_off();
    HIVE_LOG_INFO("Motor test PASS");

    // 6 blinks = motor test done
    hal_delay_ms(500);
    test_blink(6, 200, 200);
#else
    HIVE_LOG_INFO("Phase 5: Motor test SKIPPED (ENABLE_MOTOR_TEST=0)");
#endif

    // ========================================================================
    // All Tests Passed!
    // ========================================================================

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  ALL TESTS PASSED!");
    HIVE_LOG_INFO("========================================");

    if (standalone) {
        HIVE_LOG_INFO("Cleanup: hal_cleanup()");
        hal_cleanup();
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
