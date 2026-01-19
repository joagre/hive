// Logging test - Verifies HIVE_LOG_* macros and log file API
//
// Tests:
// 1. hive_log_file_open creates log file
// 2. HIVE_LOG_* macros write to file
// 3. hive_log_file_sync flushes to disk
// 4. hive_log_file_close finalizes file
// 5. Log file contains valid binary entries (magic bytes)

#include "hive_runtime.h"
#include "hive_log.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name)               \
    do {                              \
        printf("  PASS: %s\n", name); \
        tests_passed++;               \
    } while (0)
#define TEST_FAIL(name)               \
    do {                              \
        printf("  FAIL: %s\n", name); \
        tests_failed++;               \
    } while (0)

// Test file path
static const char *TEST_LOG_FILE = "/tmp/hive_logging_test.log";

// Log entry header (from hive_log.h)
// Magic: 0x4C47 ("LG"), followed by seq, timestamp, len, level, reserved
#define LOG_MAGIC 0x4C47
#define LOG_HEADER_SIZE 12

static void run_logging_tests(void *args, const hive_spawn_info *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Clean up any previous test file
    unlink(TEST_LOG_FILE);

    // ========================================================================
    // Test 1: Open log file
    // ========================================================================
    printf("\nTest 1: Open log file\n");
    {
        hive_status status = hive_log_file_open(TEST_LOG_FILE);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", HIVE_ERR_STR(status));
            TEST_FAIL("hive_log_file_open");
            hive_exit();
        } else {
            TEST_PASS("hive_log_file_open succeeds");
        }
    }

    // ========================================================================
    // Test 2: Write log entries using macros
    // ========================================================================
    printf("\nTest 2: Write log entries using HIVE_LOG_* macros\n");
    {
        // Write entries at different levels
        HIVE_LOG_INFO("Test info message: %d", 42);
        HIVE_LOG_WARN("Test warning message: %s", "caution");
        HIVE_LOG_ERROR("Test error message: %d + %d = %d", 1, 2, 3);

        // These may be compiled out depending on HIVE_LOG_LEVEL
        HIVE_LOG_DEBUG("Test debug message");
        HIVE_LOG_TRACE("Test trace message");

        TEST_PASS("HIVE_LOG_* macros execute without crash");
    }

    // ========================================================================
    // Test 3: Sync log file
    // ========================================================================
    printf("\nTest 3: Sync log file\n");
    {
        hive_status status = hive_log_file_sync();
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", HIVE_ERR_STR(status));
            TEST_FAIL("hive_log_file_sync");
        } else {
            TEST_PASS("hive_log_file_sync succeeds");
        }
    }

    // ========================================================================
    // Test 4: Close log file
    // ========================================================================
    printf("\nTest 4: Close log file\n");
    {
        hive_status status = hive_log_file_close();
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", HIVE_ERR_STR(status));
            TEST_FAIL("hive_log_file_close");
        } else {
            TEST_PASS("hive_log_file_close succeeds");
        }
    }

    // ========================================================================
    // Test 5: Verify log file exists and has content
    // ========================================================================
    printf("\nTest 5: Verify log file exists and has content\n");
    {
        int fd = open(TEST_LOG_FILE, O_RDONLY);
        if (fd < 0) {
            TEST_FAIL("log file does not exist");
        } else {
            // Check file size
            off_t size = lseek(fd, 0, SEEK_END);
            lseek(fd, 0, SEEK_SET);

            if (size <= 0) {
                TEST_FAIL("log file is empty");
            } else {
                printf("    Log file size: %ld bytes\n", (long)size);
                TEST_PASS("log file has content");
            }
            close(fd);
        }
    }

    // ========================================================================
    // Test 6: Verify binary format (magic bytes)
    // ========================================================================
    printf("\nTest 6: Verify binary format (magic bytes)\n");
    {
        int fd = open(TEST_LOG_FILE, O_RDONLY);
        if (fd < 0) {
            TEST_FAIL("cannot open log file for verification");
        } else {
            uint8_t header[LOG_HEADER_SIZE];
            ssize_t nread = read(fd, header, LOG_HEADER_SIZE);

            if (nread < LOG_HEADER_SIZE) {
                TEST_FAIL("log file too small for header");
            } else {
                // Check magic bytes (little-endian: 0x47, 0x4C = "LG")
                uint16_t magic = header[0] | (header[1] << 8);
                if (magic == LOG_MAGIC) {
                    printf("    Magic: 0x%04X (correct)\n", magic);

                    // Extract other fields for display
                    uint16_t seq = header[2] | (header[3] << 8);
                    uint32_t timestamp = header[4] | (header[5] << 8) |
                                         (header[6] << 16) | (header[7] << 24);
                    uint16_t len = header[8] | (header[9] << 8);
                    uint8_t level = header[10];

                    printf(
                        "    First entry: seq=%u, ts=%u us, len=%u, level=%u\n",
                        seq, timestamp, len, level);

                    TEST_PASS("binary format has correct magic");
                } else {
                    printf("    Magic: 0x%04X (expected 0x%04X)\n", magic,
                           LOG_MAGIC);
                    TEST_FAIL("incorrect magic bytes");
                }
            }
            close(fd);
        }
    }

    // ========================================================================
    // Test 7: Count log entries
    // ========================================================================
    printf("\nTest 7: Count log entries\n");
    {
        int fd = open(TEST_LOG_FILE, O_RDONLY);
        if (fd < 0) {
            TEST_FAIL("cannot open log file");
        } else {
            int entry_count = 0;
            uint8_t header[LOG_HEADER_SIZE];

            while (read(fd, header, LOG_HEADER_SIZE) == LOG_HEADER_SIZE) {
                uint16_t magic = header[0] | (header[1] << 8);
                if (magic != LOG_MAGIC) {
                    break;
                }

                uint16_t len = header[8] | (header[9] << 8);
                entry_count++;

                // Skip payload
                if (len > 0) {
                    lseek(fd, len, SEEK_CUR);
                }
            }

            printf("    Found %d log entries\n", entry_count);

            // We wrote at least 3 entries (INFO, WARN, ERROR)
            if (entry_count >= 3) {
                TEST_PASS("found expected log entries");
            } else {
                TEST_FAIL("too few log entries");
            }

            close(fd);
        }
    }

    // ========================================================================
    // Test 8: Reopen and append (verify can open again)
    // ========================================================================
    printf("\nTest 8: Reopen log file\n");
    {
        // Opening again should work (will truncate on Linux)
        hive_status status = hive_log_file_open(TEST_LOG_FILE);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", HIVE_ERR_STR(status));
            TEST_FAIL("cannot reopen log file");
        } else {
            HIVE_LOG_INFO("Entry after reopen");
            hive_log_file_sync();
            hive_log_file_close();
            TEST_PASS("reopen and write succeeds");
        }
    }

    // Clean up test file
    unlink(TEST_LOG_FILE);

    hive_exit();
}

int main(void) {
    printf("=== Logging (hive_log) Test Suite ===\n");

    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    actor_id runner;
    if (HIVE_FAILED(hive_spawn(run_logging_tests, NULL, NULL, NULL, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n",
           tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
