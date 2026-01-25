/**
 * Flash Storage Test (Crazyflie 2.1+ only)
 *
 * Tests the Hive file API with flash-backed storage.
 * Uses the /log virtual path which maps to internal flash sector 8.
 *
 * WARNING: This test erases /log! Any existing log data will be lost.
 *
 * Test sequence:
 *   1. Initialize Hive runtime (includes file subsystem)
 *   2. Open /log for writing (erases flash sector)
 *   3. Write test pattern (4KB)
 *   4. Close and sync
 *   5. Re-open /log for reading
 *   6. Read back with pread and verify
 *   7. Report results
 *
 * LED feedback:
 *   2 blinks    = Starting flash test
 *   Fast blink  = Writing/reading in progress
 *   LED on      = All tests passed!
 *   Slow blink  = Test failed
 *
 * Usage:
 *   make PLATFORM=crazyflie TEST=flash
 *   make flash-crazyflie TEST=flash
 */

#include "hal/hal.h"
#include "hive_file.h"
#include "hive_log.h"
#include "hive_runtime.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
// Test Configuration
// ============================================================================

#define TEST_PATTERN_SIZE 4096
#define BLOCK_SIZE 256

// ============================================================================
// Test Data
// ============================================================================

static uint8_t s_test_pattern[TEST_PATTERN_SIZE];
static uint8_t s_read_buffer[BLOCK_SIZE];

// Generate reproducible test pattern
static void generate_pattern(void) {
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        // Mix of patterns to catch various failure modes
        s_test_pattern[i] = (uint8_t)((i * 7 + 0x5A) ^ (i >> 4));
    }
}

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
// Main Test
// ============================================================================

int main(void) {
    hive_status_t status;
    int fd = -1;
    size_t bytes;

    // Initialize pilot HAL for LED and timing
    hal_debug_init();
    if (hal_init() != 0) {
        error_blink_forever();
    }

    // Initialize Hive runtime (includes file subsystem and logging)
    status = hive_init();
    if (HIVE_FAILED(status)) {
        error_blink_forever();
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Flash Storage Test (Hive File API)");
    HIVE_LOG_INFO("========================================");

    // 2 blinks = starting test
    test_blink(2, 200, 200);
    hal_delay_ms(500);

    // Generate test pattern
    HIVE_LOG_INFO("Generating test pattern (%d bytes)...", TEST_PATTERN_SIZE);
    generate_pattern();

    // ========================================================================
    // Phase 1: Open and Write
    // ========================================================================

    HIVE_LOG_INFO("Opening /log for writing (erases flash)...");
    uint32_t start_time = hal_get_time_ms();

    status = hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("hive_file_open() failed: %s", HIVE_ERR_STR(status));
        hive_cleanup();
        error_blink_forever();
    }

    uint32_t erase_time = hal_get_time_ms() - start_time;
    HIVE_LOG_INFO("Flash erased and opened (fd=%d, %u ms)", fd, erase_time);

    // Write test pattern in blocks
    HIVE_LOG_INFO("Writing %d bytes in %d-byte blocks...", TEST_PATTERN_SIZE,
                  BLOCK_SIZE);

    start_time = hal_get_time_ms();
    int blocks = TEST_PATTERN_SIZE / BLOCK_SIZE;
    size_t total_written = 0;

    for (int b = 0; b < blocks; b++) {
        status = hive_file_write(fd, &s_test_pattern[b * BLOCK_SIZE],
                                 BLOCK_SIZE, &bytes);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("Write failed at block %d: %s", b,
                           HIVE_ERR_STR(status));
            hive_file_close(fd);
            hive_cleanup();
            error_blink_forever();
        }
        total_written += bytes;

        // Blink LED during write
        if (b % 4 == 0) {
            hal_led_toggle();
        }
    }
    hal_led_off();

    uint32_t write_time = hal_get_time_ms() - start_time;
    HIVE_LOG_INFO("Write OK (%u ms)", write_time);

    // Sync and close
    HIVE_LOG_INFO("Syncing to flash...");
    start_time = hal_get_time_ms();
    status = hive_file_sync(fd);
    uint32_t sync_time = hal_get_time_ms() - start_time;

    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Sync failed: %s", HIVE_ERR_STR(status));
        hive_file_close(fd);
        hive_cleanup();
        error_blink_forever();
    }
    HIVE_LOG_INFO("Sync OK (%u ms)", sync_time);

    status = hive_file_close(fd);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_WARN("Close failed: %s", HIVE_ERR_STR(status));
    }

    // ========================================================================
    // Phase 2: Re-open and Verify
    // ========================================================================

    HIVE_LOG_INFO("Re-opening /log for reading...");
    status = hive_file_open("/log", HIVE_O_RDONLY, 0, &fd);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("hive_file_open(RDONLY) failed: %s",
                       HIVE_ERR_STR(status));
        hive_cleanup();
        error_blink_forever();
    }

    HIVE_LOG_INFO("Reading and verifying %d bytes...", TEST_PATTERN_SIZE);

    start_time = hal_get_time_ms();
    int errors = 0;
    int first_error_offset = -1;

    for (int b = 0; b < blocks; b++) {
        size_t offset = b * BLOCK_SIZE;

        // Use pread to read at specific offset
        status = hive_file_pread(fd, s_read_buffer, BLOCK_SIZE, offset, &bytes);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("Read failed at offset %zu: %s", offset,
                           HIVE_ERR_STR(status));
            hive_file_close(fd);
            hive_cleanup();
            error_blink_forever();
        }

        if (bytes != BLOCK_SIZE) {
            HIVE_LOG_ERROR("Short read at offset %zu: got %zu, expected %d",
                           offset, bytes, BLOCK_SIZE);
            hive_file_close(fd);
            hive_cleanup();
            error_blink_forever();
        }

        // Verify block
        for (int i = 0; i < BLOCK_SIZE; i++) {
            int data_offset = b * BLOCK_SIZE + i;
            if (s_read_buffer[i] != s_test_pattern[data_offset]) {
                if (errors == 0) {
                    first_error_offset = data_offset;
                }
                errors++;
            }
        }

        // Blink LED during read
        if (b % 4 == 0) {
            hal_led_toggle();
        }
    }
    hal_led_off();

    uint32_t read_time = hal_get_time_ms() - start_time;

    if (errors > 0) {
        HIVE_LOG_ERROR("Verification failed: %d errors, first at offset %d",
                       errors, first_error_offset);
        HIVE_LOG_ERROR("  Expected: 0x%02X, Got: 0x%02X",
                       s_test_pattern[first_error_offset],
                       s_read_buffer[first_error_offset % BLOCK_SIZE]);
        hive_file_close(fd);
        hive_cleanup();
        error_blink_forever();
    }
    HIVE_LOG_INFO("Read and verify OK (%u ms)", read_time);

    hive_file_close(fd);

    // ========================================================================
    // Results
    // ========================================================================

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  FLASH TEST PASSED!");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Path:       /log");
    HIVE_LOG_INFO("  Size:       %d bytes", TEST_PATTERN_SIZE);
    HIVE_LOG_INFO("  Erase time: %u ms", erase_time);
    HIVE_LOG_INFO("  Write time: %u ms (%u bytes/sec)", write_time,
                  write_time > 0 ? (TEST_PATTERN_SIZE * 1000) / write_time : 0);
    HIVE_LOG_INFO("  Sync time:  %u ms", sync_time);
    HIVE_LOG_INFO("  Read time:  %u ms (%u bytes/sec)", read_time,
                  read_time > 0 ? (TEST_PATTERN_SIZE * 1000) / read_time : 0);

    // Cleanup Hive runtime
    hive_cleanup();

    // Solid LED = success
    hal_led_on();

    // Idle forever
    while (1) {
        hal_delay_ms(1000);
    }

    return 0;
}
