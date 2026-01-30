/**
 * Thrust Calibration Test
 *
 * Tests motor thrust to find the HAL_BASE_THRUST value needed for hover.
 * Run multiple times at different thrust levels to find where the Crazyflie
 * barely stays on the ground (light on skids = correct base thrust).
 *
 * Usage:
 *   1. Set THRUST_TEST_VALUE below to desired test thrust (start at 0.30)
 *   2. Build: make PLATFORM=crazyflie TEST=thrust
 *   3. Flash: make flash-crazyflie TEST=thrust
 *   4. Observe: Does the Crazyflie lift off?
 *      - No movement: increase thrust by 0.02
 *      - Light on skids: good base thrust found!
 *      - Lifts off: decrease thrust by 0.01
 *   5. Update HAL_BASE_THRUST in hal_config.h with the found value
 *
 * Safety:
 *   - REMOVE PROPELLERS for initial testing!
 *   - Once props are on, test in a safe area with clearance
 *   - Keep hands clear - motors will spin at significant speed
 *   - Test runs for 5 seconds then stops automatically
 *
 * LED feedback:
 *   2 blinks = Init complete, starting calibration
 *   3 blinks = Calibration done, motors starting in 3 seconds
 *   Slow blink = Motors running at test thrust
 *   Solid LED = Test complete
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"
#include "types.h"

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Test Configuration - CHANGE THIS VALUE
// ============================================================================

// Thrust value to test (0.0 to 1.0)
// Start at 0.30 and increase in 0.02 steps until drone is light on skids
// Typical Crazyflie hover thrust: 0.35-0.45 depending on battery charge
#ifndef THRUST_TEST_VALUE
#define THRUST_TEST_VALUE 0.35f
#endif

// Test duration in milliseconds
#define THRUST_TEST_DURATION_MS 5000

// Delay before motors start (seconds) - gives time to place drone
#define MOTOR_START_DELAY_S 3

// ============================================================================
// LED Helpers
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
// Test Implementation (standalone only - not for test_main.c)
// ============================================================================

static int run_thrust_test(void) {
    hal_debug_init();

    if (hal_init() != 0) {
        return -1;
    }

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        return -1;
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Thrust Calibration Test");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("Test thrust: %.2f (%.0f%%)", THRUST_TEST_VALUE,
                  THRUST_TEST_VALUE * 100.0f);
    HIVE_LOG_INFO("Duration: %d ms", THRUST_TEST_DURATION_MS);

    // 2 blinks = init done
    hal_delay_ms(500);
    test_blink(2, 200, 200);
    hal_delay_ms(500);

    // Self-test
    HIVE_LOG_INFO("Running self-test...");
    if (!hal_self_test()) {
        HIVE_LOG_ERROR("Self-test FAILED");
        hive_cleanup();
        return -1;
    }
    HIVE_LOG_INFO("Self-test PASSED");

    // Calibrate
    HIVE_LOG_INFO("Calibrating sensors (keep still and level)...");
    hal_calibrate();
    HIVE_LOG_INFO("Calibration done");

    // 3 blinks = calibration done, motors starting soon
    hal_delay_ms(500);
    test_blink(3, 200, 200);

    // Countdown before motor start
    HIVE_LOG_INFO("Motors starting in %d seconds...", MOTOR_START_DELAY_S);
    HIVE_LOG_INFO("Place drone on flat surface NOW!");
    for (int i = MOTOR_START_DELAY_S; i > 0; i--) {
        HIVE_LOG_INFO("  %d...", i);
        hal_delay_ms(1000);
    }

    // Arm and run motors
    HIVE_LOG_INFO("MOTORS STARTING at %.2f thrust!", THRUST_TEST_VALUE);
    hal_arm();

    torque_cmd_t cmd = {
        .thrust = THRUST_TEST_VALUE, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f};

    uint32_t start_time = hal_get_time_ms();
    uint32_t last_blink = start_time;
    uint32_t last_log = start_time;

    while ((hal_get_time_ms() - start_time) < THRUST_TEST_DURATION_MS) {
        hal_write_torque(&cmd);

        // Slow blink (2 Hz) while motors running
        if ((hal_get_time_ms() - last_blink) >= 250) {
            hal_led_toggle();
            last_blink = hal_get_time_ms();
        }

        // Log every second
        if ((hal_get_time_ms() - last_log) >= 1000) {
            uint32_t elapsed = hal_get_time_ms() - start_time;
            HIVE_LOG_INFO("Running at %.2f thrust... %lu ms / %d ms",
                          THRUST_TEST_VALUE, (unsigned long)elapsed,
                          THRUST_TEST_DURATION_MS);
            last_log = hal_get_time_ms();
        }

        hal_delay_ms(4); // ~250 Hz update rate
    }

    // Stop motors
    cmd.thrust = 0.0f;
    hal_write_torque(&cmd);
    hal_disarm();
    hal_led_off();

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Test complete!");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("Tested thrust: %.2f", THRUST_TEST_VALUE);
    HIVE_LOG_INFO("Observations:");
    HIVE_LOG_INFO("  - No movement: increase to %.2f",
                  THRUST_TEST_VALUE + 0.02f);
    HIVE_LOG_INFO("  - Light on skids: GOOD! Use %.2f as HAL_BASE_THRUST",
                  THRUST_TEST_VALUE);
    HIVE_LOG_INFO("  - Lifted off: decrease to %.2f",
                  THRUST_TEST_VALUE - 0.01f);
    HIVE_LOG_INFO("Update hal/crazyflie-2.1plus/hal_config.h with found value");

    hal_cleanup();
    hive_cleanup();

    return 0;
}

// ============================================================================
// Entry Point
// ============================================================================

int main(void) {
    int result = run_thrust_test();

    if (result != 0) {
        error_blink_forever(100, 100);
    }

    // Solid LED = success
    hal_led_on();
    while (1) {
        hal_delay_ms(1000);
    }

    return 0;
}
