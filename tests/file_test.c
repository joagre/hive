#include "hive_runtime.h"
#include "hive_file.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>

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
static const char *TEST_FILE = "/tmp/hive_file_test.tmp";

static void run_file_tests(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // ========================================================================
    // Test 1: Open file for writing (create)
    // ========================================================================
    printf("\nTest 1: Open file for writing (create)\n");
    int fd = -1;
    {
        hive_status_t status = hive_file_open(
            TEST_FILE, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644, &fd);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_open for write");
        } else if (fd < 0) {
            TEST_FAIL("got invalid fd");
        } else {
            TEST_PASS("open file for writing");
        }
    }

    // ========================================================================
    // Test 2: Write to file
    // ========================================================================
    printf("\nTest 2: Write to file\n");
    {
        const char *data = "Hello, RT File System!";
        size_t len = strlen(data);
        size_t actual = 0;

        hive_status_t status = hive_file_write(fd, data, len, &actual);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_write");
        } else if (actual != len) {
            printf("    Wrote %zu/%zu bytes\n", actual, len);
            TEST_FAIL("incomplete write");
        } else {
            TEST_PASS("write to file");
        }
    }

    // ========================================================================
    // Test 3: Sync file to disk
    // ========================================================================
    printf("\nTest 3: Sync file to disk\n");
    {
        hive_status_t status = hive_file_sync(fd);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_sync");
        } else {
            TEST_PASS("sync file to disk");
        }
    }

    // ========================================================================
    // Test 4: Close file
    // ========================================================================
    printf("\nTest 4: Close file\n");
    {
        hive_status_t status = hive_file_close(fd);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_close");
        } else {
            TEST_PASS("close file");
        }
    }

    // ========================================================================
    // Test 5: Open file for reading
    // ========================================================================
    printf("\nTest 5: Open file for reading\n");
    {
        hive_status_t status = hive_file_open(TEST_FILE, HIVE_O_RDONLY, 0, &fd);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_open for read");
        } else {
            TEST_PASS("open file for reading");
        }
    }

    // ========================================================================
    // Test 6: Read from file
    // ========================================================================
    printf("\nTest 6: Read from file\n");
    {
        char buf[64] = {0};
        size_t actual = 0;

        hive_status_t status =
            hive_file_read(fd, buf, sizeof(buf) - 1, &actual);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_read");
        } else if (strcmp(buf, "Hello, RT File System!") != 0) {
            printf("    Read: '%s'\n", buf);
            TEST_FAIL("data mismatch");
        } else {
            TEST_PASS("read from file");
        }
    }

    // ========================================================================
    // Test 7: pread (read at offset without changing position)
    // ========================================================================
    printf("\nTest 7: pread (read at offset)\n");
    {
        char buf[16] = {0};
        size_t actual = 0;

        // Read "RT" starting at offset 7
        hive_status_t status = hive_file_pread(fd, buf, 2, 7, &actual);
        if (HIVE_FAILED(status)) {
            printf("    Error: %s\n", status.msg ? status.msg : "unknown");
            TEST_FAIL("hive_file_pread");
        } else if (strncmp(buf, "RT", 2) != 0) {
            printf("    Read: '%s' (expected 'RT')\n", buf);
            TEST_FAIL("pread data mismatch");
        } else {
            TEST_PASS("pread at offset");
        }
    }

    // Close the read fd
    hive_file_close(fd);

    // ========================================================================
    // Test 8: pwrite (write at offset)
    // ========================================================================
    printf("\nTest 8: pwrite (write at offset)\n");
    {
        hive_status_t status = hive_file_open(TEST_FILE, HIVE_O_RDWR, 0, &fd);
        if (HIVE_FAILED(status)) {
            TEST_FAIL("open for pwrite");
        } else {
            size_t actual = 0;
            // Overwrite "RT" with "XX" at offset 7
            status = hive_file_pwrite(fd, "XX", 2, 7, &actual);
            if (HIVE_FAILED(status)) {
                printf("    Error: %s\n", status.msg ? status.msg : "unknown");
                TEST_FAIL("hive_file_pwrite");
            } else {
                // Verify by reading back
                char buf[64] = {0};
                hive_file_pread(fd, buf, sizeof(buf) - 1, 0, &actual);
                if (strncmp(buf + 7, "XX", 2) == 0) {
                    TEST_PASS("pwrite at offset");
                } else {
                    printf("    Read back: '%s'\n", buf);
                    TEST_FAIL("pwrite verification failed");
                }
            }
            hive_file_close(fd);
        }
    }

    // ========================================================================
    // Test 9: Open non-existent file without HIVE_O_CREAT fails
    // ========================================================================
    printf("\nTest 9: Open non-existent file fails\n");
    {
        hive_status_t status = hive_file_open(
            "/tmp/nonexistent_rt_test_file_xyz.tmp", HIVE_O_RDONLY, 0, &fd);
        if (HIVE_FAILED(status)) {
            TEST_PASS("open non-existent file fails");
        } else {
            hive_file_close(fd);
            TEST_FAIL("should fail to open non-existent file");
        }
    }

    // ========================================================================
    // Test 10: Close invalid fd
    // ========================================================================
    printf("\nTest 10: Close invalid fd\n");
    {
        hive_status_t status = hive_file_close(-1);
        if (HIVE_FAILED(status)) {
            TEST_PASS("close invalid fd fails");
        } else {
            TEST_FAIL("should fail to close invalid fd");
        }
    }

    // ========================================================================
    // Test 11: Read from invalid fd
    // ========================================================================
    printf("\nTest 11: Read from invalid fd\n");
    {
        char buf[64];
        size_t actual = 0;
        hive_status_t status = hive_file_read(-1, buf, sizeof(buf), &actual);
        if (HIVE_FAILED(status)) {
            TEST_PASS("read from invalid fd fails");
        } else {
            TEST_FAIL("should fail to read from invalid fd");
        }
    }

    // ========================================================================
    // Test 12: Write to invalid fd
    // ========================================================================
    printf("\nTest 12: Write to invalid fd\n");
    {
        const char *data = "test";
        size_t actual = 0;
        hive_status_t status = hive_file_write(-1, data, strlen(data), &actual);
        if (HIVE_FAILED(status)) {
            TEST_PASS("write to invalid fd fails");
        } else {
            TEST_FAIL("should fail to write to invalid fd");
        }
    }

    // ========================================================================
    // Test 13: pread beyond EOF
    // ========================================================================
    printf("\nTest 13: pread beyond EOF\n");
    {
        // Create a small test file
        hive_status_t status = hive_file_open(
            TEST_FILE, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644, &fd);
        if (HIVE_SUCCEEDED(status)) {
            const char *data = "short";
            size_t written = 0;
            hive_file_write(fd, data, 5, &written);
            hive_file_close(fd);

            // Open for reading
            status = hive_file_open(TEST_FILE, HIVE_O_RDONLY, 0, &fd);
            if (HIVE_SUCCEEDED(status)) {
                char buf[64] = {0};
                size_t actual = 0;
                // Read at offset 1000 (way beyond the 5 byte file)
                status = hive_file_pread(fd, buf, sizeof(buf), 1000, &actual);
                if (HIVE_SUCCEEDED(status) && actual == 0) {
                    TEST_PASS("pread beyond EOF returns 0 bytes");
                } else if (HIVE_FAILED(status)) {
                    TEST_PASS("pread beyond EOF returns error");
                } else {
                    printf("    Read %zu bytes at offset 1000\n", actual);
                    TEST_FAIL("pread beyond EOF should return 0 or error");
                }
                hive_file_close(fd);
            }
        }
    }

    // ========================================================================
    // Test 14: Double close
    // ========================================================================
    printf("\nTest 14: Double close\n");
    {
        hive_status_t status = hive_file_open(TEST_FILE, HIVE_O_RDONLY, 0, &fd);
        if (HIVE_SUCCEEDED(status)) {
            int saved_fd = fd;

            // First close should succeed
            status = hive_file_close(fd);
            if (HIVE_FAILED(status)) {
                TEST_FAIL("first close failed");
            } else {
                // Second close should fail
                status = hive_file_close(saved_fd);
                if (HIVE_FAILED(status)) {
                    TEST_PASS("double close fails");
                } else {
                    TEST_FAIL("double close should fail");
                }
            }
        }
    }

    // ========================================================================
    // Test 15: Zero-length read/write
    // ========================================================================
    printf("\nTest 15: Zero-length read/write\n");
    {
        hive_status_t status = hive_file_open(TEST_FILE, HIVE_O_RDWR, 0, &fd);
        if (HIVE_SUCCEEDED(status)) {
            size_t actual = 0;

            // Zero-length write
            status = hive_file_write(fd, "x", 0, &actual);
            if (HIVE_SUCCEEDED(status) && actual == 0) {
                TEST_PASS("zero-length write succeeds");
            } else if (HIVE_SUCCEEDED(status)) {
                printf("    Zero-length write returned %zu bytes\n", actual);
                TEST_FAIL("zero-length write should return 0 bytes");
            } else {
                TEST_FAIL("zero-length write failed");
            }

            // Zero-length read
            char buf[1];
            status = hive_file_read(fd, buf, 0, &actual);
            if (HIVE_SUCCEEDED(status) && actual == 0) {
                TEST_PASS("zero-length read succeeds");
            } else if (HIVE_SUCCEEDED(status)) {
                printf("    Zero-length read returned %zu bytes\n", actual);
                TEST_FAIL("zero-length read should return 0 bytes");
            } else {
                TEST_FAIL("zero-length read failed");
            }

            hive_file_close(fd);
        }
    }

    // Cleanup test file
    unlink(TEST_FILE);

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n",
           tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return;
}

int main(void) {
    printf("=== File I/O (hive_file) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id_t runner;
    if (HIVE_FAILED(hive_spawn(run_file_tests, NULL, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    return tests_failed > 0 ? 1 : 0;
}
