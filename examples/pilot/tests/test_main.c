/**
 * Combined Test Runner
 *
 * Runs all applicable pilot tests in sequence. Primarily intended for use
 * as a "dummy pilot" in Webots simulation.
 *
 * Platform behavior:
 *   - Webots: Runs sensors_motors test only (flash/SD are hardware-specific)
 *   - Crazyflie: Can run all tests sequentially (typically flashed individually)
 *
 * Test sequence:
 *   1. Initialize HAL and Hive runtime
 *   2. Run sensors_motors test
 *   3. Run flash test (Crazyflie only)
 *   4. Run SD test (Crazyflie only)
 *   5. Report summary
 *
 * Usage:
 *   Webots:    make PLATFORM=webots TEST=main
 *   Crazyflie: make PLATFORM=crazyflie TEST=main
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// External Test Functions
// ============================================================================

// From test_sensors_motors.c
extern int test_sensors_motors_run(bool standalone);

// From test_flash.c (Crazyflie only)
extern int test_flash_run(bool standalone);

// From test_sd.c (Crazyflie only)
// Returns: 0 = success, -1 = failure, 1 = skipped
extern int test_sd_run(bool standalone);

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

static void error_blink_forever(void) {
    while (1) {
        hal_led_toggle();
        hal_delay_ms(300);
    }
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main(void) {
    int result;
    int passed = 0;
    int failed = 0;
    int skipped = 0;

    // Initialize pilot HAL
    hal_debug_init();
    if (hal_init() != 0) {
        error_blink_forever();
    }

    // Initialize Hive runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        error_blink_forever();
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Combined Test Runner");
    HIVE_LOG_INFO("========================================");

    // 1 blink = starting combined test
    test_blink(1, 300, 300);
    hal_delay_ms(500);

    // ========================================================================
    // Test 1: Sensors and Motors
    // ========================================================================

    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("--- Running: sensors_motors ---");

    result = test_sensors_motors_run(false);
    if (result == 0) {
        HIVE_LOG_INFO("sensors_motors: PASSED");
        passed++;
    } else {
        HIVE_LOG_ERROR("sensors_motors: FAILED");
        failed++;
    }

    hal_delay_ms(1000);

    // ========================================================================
    // Test 2: Flash Storage (Crazyflie only)
    // ========================================================================

#ifdef HIVE_PLATFORM_STM32
    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("--- Running: flash ---");

    result = test_flash_run(false);
    if (result == 0) {
        HIVE_LOG_INFO("flash: PASSED");
        passed++;
    } else {
        HIVE_LOG_ERROR("flash: FAILED");
        failed++;
    }

    hal_delay_ms(1000);
#else
    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("--- Skipping: flash (not available on this platform) ---");
    skipped++;
#endif

    // ========================================================================
    // Test 3: SD Card Storage (Crazyflie only)
    // ========================================================================

#ifdef HIVE_PLATFORM_STM32
    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("--- Running: sd ---");

    result = test_sd_run(false);
    if (result == 0) {
        HIVE_LOG_INFO("sd: PASSED");
        passed++;
    } else if (result > 0) {
        HIVE_LOG_INFO("sd: SKIPPED (SD not available)");
        skipped++;
    } else {
        HIVE_LOG_ERROR("sd: FAILED");
        failed++;
    }

    hal_delay_ms(1000);
#else
    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("--- Skipping: sd (not available on this platform) ---");
    skipped++;
#endif

    // ========================================================================
    // Summary
    // ========================================================================

    HIVE_LOG_INFO("");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  TEST SUMMARY");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Passed:  %d", passed);
    HIVE_LOG_INFO("  Failed:  %d", failed);
    HIVE_LOG_INFO("  Skipped: %d", skipped);
    HIVE_LOG_INFO("========================================");

    if (failed > 0) {
        HIVE_LOG_ERROR("  SOME TESTS FAILED!");
    } else {
        HIVE_LOG_INFO("  ALL TESTS PASSED!");
    }

    // Cleanup
    hive_cleanup();

    if (failed > 0) {
        // Slow blink = some tests failed
        while (1) {
            hal_led_toggle();
            hal_delay_ms(500);
        }
    } else {
        // Solid LED = all tests passed
        hal_led_on();
        while (1) {
            hal_delay_ms(1000);
        }
    }

    return 0;
}
