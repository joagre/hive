#include "hive_runtime.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <time.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

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

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static void run_timer_tests(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // ========================================================================
    // Test 1: One-shot timer (hive_timer_after)
    // ========================================================================
    printf("\nTest 1: One-shot timer (hive_timer_after)\n");
    {
        timer_id_t timer;
        hive_status_t status = hive_timer_after(100000, &timer); // 100ms
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_timer_after failed");
        } else if (timer == HIVE_TIMER_ID_INVALID) {
            TEST_FAIL("got invalid timer ID");
        } else {
            uint64_t start = time_ms();
            hive_message_t msg;
            status = hive_ipc_recv(&msg, -1); // Wait indefinitely
            uint64_t elapsed = time_ms() - start;

            if (HIVE_FAILED(status)) {
                TEST_FAIL("did not receive timer message");
            } else if (!hive_msg_is_timer(&msg)) {
                TEST_FAIL("message is not a timer tick");
            } else if (elapsed >= 80 && elapsed <= 200) {
                printf("    Timer fired after %lu ms (expected ~100ms)\n",
                       (unsigned long)elapsed);
                TEST_PASS("one-shot timer fires at correct time");
            } else {
                printf("    Timer fired after %lu ms (expected ~100ms)\n",
                       (unsigned long)elapsed);
                TEST_FAIL("timer fired at wrong time");
            }
        }
    }

    // ========================================================================
    // Test 2: Timer cancellation
    // ========================================================================
    printf("\nTest 2: Timer cancellation\n");
    {
        timer_id_t timer;
        hive_timer_after(100000, &timer); // 100ms

        hive_status_t status = hive_timer_cancel(timer);
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_timer_cancel failed");
        } else {
            hive_message_t msg;
            status = hive_ipc_recv(&msg, 200); // 200ms timeout

            if (status.code == HIVE_ERR_TIMEOUT) {
                TEST_PASS("cancelled timer does not fire");
            } else if (HIVE_FAILED(status)) {
                TEST_PASS("cancelled timer does not fire (no message)");
            } else if (hive_msg_is_timer(&msg)) {
                TEST_FAIL("received timer tick after cancellation");
            } else {
                TEST_PASS("cancelled timer does not fire");
            }
        }
    }

    // ========================================================================
    // Test 3: Timer sender is the owning actor
    // ========================================================================
    printf("\nTest 3: Timer sender is the owning actor\n");
    {
        timer_id_t timer;
        hive_timer_after(50000, &timer);

        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, -1);

        if (HIVE_FAILED(status)) {
            TEST_FAIL("did not receive timer message");
        } else if (msg.sender == hive_self()) {
            TEST_PASS("timer message sender is the owning actor");
        } else {
            printf("    Sender: %u, expected: %u (self)\n", msg.sender,
                   hive_self());
            TEST_FAIL("wrong sender ID");
        }
    }

    // ========================================================================
    // Test 4: hive_msg_is_timer correctly identifies timer messages
    // ========================================================================
    printf("\nTest 4: hive_msg_is_timer identifies timer messages\n");
    {
        timer_id_t timer;
        hive_timer_after(50000, &timer);

        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, -1);

        if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
            TEST_PASS("timer message detected by hive_msg_is_timer");
        } else {
            TEST_FAIL("timer message not detected");
        }

        // Now test that regular messages are NOT detected as timer ticks
        actor_id_t self = hive_self();
        const char *data = "not a timer";
        hive_ipc_notify(self, HIVE_TAG_NONE, data, 12);

        status = hive_ipc_recv(&msg, 100);
        if (HIVE_SUCCEEDED(status) && !hive_msg_is_timer(&msg)) {
            TEST_PASS("regular message NOT detected as timer tick");
        } else {
            TEST_FAIL("could not distinguish regular message");
        }
    }

    // ========================================================================
    // Test 5: Cancel invalid timer
    // ========================================================================
    printf("\nTest 5: Cancel invalid timer\n");
    {
        hive_status_t status = hive_timer_cancel(HIVE_TIMER_ID_INVALID);
        if (HIVE_FAILED(status)) {
            TEST_PASS("cancel HIVE_TIMER_ID_INVALID fails");
        } else {
            TEST_FAIL("cancel HIVE_TIMER_ID_INVALID should fail");
        }

        status = hive_timer_cancel(9999);
        if (HIVE_FAILED(status)) {
            TEST_PASS("cancel non-existent timer fails");
        } else {
            TEST_FAIL("cancel non-existent timer should fail");
        }
    }

    // ========================================================================
    // Test 6: Short delay timer
    // ========================================================================
    printf("\nTest 6: Short delay timer\n");
    {
        timer_id_t timer;
        uint64_t start = time_ms();

        hive_timer_after(10000, &timer); // 10ms

        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, -1);

        uint64_t elapsed = time_ms() - start;

        if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
            printf("    Short timer fired after %lu ms\n",
                   (unsigned long)elapsed);
            TEST_PASS("short delay timer works");
        } else {
            TEST_FAIL("short delay timer did not fire");
        }
    }

    // ========================================================================
    // Test 7: Periodic timer (hive_timer_every)
    // NOTE: This test may fail due to implementation limitations with
    //       periodic timers not firing after the first one-shot timer completes.
    // ========================================================================
    printf("\nTest 7: Periodic timer (hive_timer_every)\n");
    {
        timer_id_t timer;
        hive_status_t status = hive_timer_every(50000, &timer); // 50ms interval
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_timer_every failed to create timer");
        } else {
            int tick_count = 0;
            uint64_t start = time_ms();

            // Try to receive 5 ticks
            for (int i = 0; i < 5; i++) {
                hive_message_t msg;
                status = hive_ipc_recv(&msg, 200); // 200ms timeout per tick
                if (HIVE_FAILED(status)) {
                    printf("    Tick %d: recv failed (timeout or error)\n",
                           i + 1);
                    break;
                }
                if (hive_msg_is_timer(&msg)) {
                    tick_count++;
                }
            }

            uint64_t elapsed = time_ms() - start;
            hive_timer_cancel(timer);

            if (tick_count >= 5) {
                printf("    Received %d ticks in %lu ms\n", tick_count,
                       (unsigned long)elapsed);
                TEST_PASS("periodic timer fires multiple times");
            } else {
                printf("    Only received %d/5 ticks in %lu ms\n", tick_count,
                       (unsigned long)elapsed);
                TEST_FAIL("periodic timer did not fire enough times");
            }
        }
    }

    // ========================================================================
    // Test 8: Multiple simultaneous timers
    // NOTE: This test may fail due to implementation limitations with
    //       multiple timers not all firing correctly.
    // ========================================================================
    printf("\nTest 8: Multiple simultaneous timers\n");
    {
        timer_id_t timer1, timer2, timer3;

        hive_status_t s1 = hive_timer_after(50000, &timer1);  // 50ms
        hive_status_t s2 = hive_timer_after(100000, &timer2); // 100ms
        hive_status_t s3 = hive_timer_after(150000, &timer3); // 150ms

        if (HIVE_FAILED(s1) || HIVE_FAILED(s2) || HIVE_FAILED(s3)) {
            TEST_FAIL("failed to create multiple timers");
        } else {
            int received = 0;
            uint64_t start = time_ms();

            for (int i = 0; i < 3; i++) {
                hive_message_t msg;
                hive_status_t status =
                    hive_ipc_recv(&msg, 300); // 300ms timeout
                if (HIVE_FAILED(status)) {
                    printf("    Timer %d: recv failed\n", i + 1);
                    continue;
                }
                if (hive_msg_is_timer(&msg)) {
                    uint64_t elapsed = time_ms() - start;
                    printf("    Timer tick %d received at %lu ms\n",
                           received + 1, (unsigned long)elapsed);
                    received++;
                }
            }

            if (received == 3) {
                TEST_PASS("all 3 timers fired");
            } else {
                printf("    Only received %d/3 timer ticks\n", received);
                TEST_FAIL("not all timers fired");
            }
        }
    }

    // ========================================================================
    // Test 9: Cancel periodic timer
    // ========================================================================
    printf("\nTest 9: Cancel periodic timer\n");
    {
        timer_id_t timer;
        hive_status_t status = hive_timer_every(30000, &timer); // 30ms interval
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_timer_every failed");
        } else {
            // Try to receive some ticks
            int ticks = 0;
            for (int i = 0; i < 3; i++) {
                hive_message_t msg;
                status = hive_ipc_recv(&msg, 100); // 100ms timeout
                if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
                    ticks++;
                }
            }

            // Cancel the timer
            status = hive_timer_cancel(timer);
            if (HIVE_FAILED(status)) {
                TEST_FAIL("hive_timer_cancel failed");
            } else {
                // Wait and ensure no more ticks arrive
                hive_message_t msg;
                status = hive_ipc_recv(&msg, 100); // 100ms

                if (status.code == HIVE_ERR_TIMEOUT) {
                    printf(
                        "    Received %d ticks before cancel, then stopped\n",
                        ticks);
                    TEST_PASS("periodic timer stops after cancel");
                } else if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
                    TEST_FAIL("received tick after cancel");
                } else {
                    TEST_PASS("periodic timer stops after cancel");
                }
            }
        }
    }

    // ========================================================================
    // Test 10: Timer pool exhaustion (HIVE_TIMER_ENTRY_POOL_SIZE=64)
    // ========================================================================
    printf("\nTest 10: Timer pool exhaustion (HIVE_TIMER_ENTRY_POOL_SIZE=%d)\n",
           HIVE_TIMER_ENTRY_POOL_SIZE);
    {
        timer_id_t timers[HIVE_TIMER_ENTRY_POOL_SIZE + 10];
        int created = 0;

        // Create timers until pool exhaustion
        for (int i = 0; i < HIVE_TIMER_ENTRY_POOL_SIZE + 10; i++) {
            hive_status_t status = hive_timer_after(
                10000000, &timers[i]); // 10 second delay (won't fire)
            if (HIVE_FAILED(status)) {
                printf("    Timer creation failed after %d timers (pool "
                       "exhausted)\n",
                       created);
                break;
            }
            created++;
        }

        if (created < HIVE_TIMER_ENTRY_POOL_SIZE + 10) {
            TEST_PASS("timer pool exhaustion detected");
        } else {
            printf("    Created all %d timers without exhaustion\n", created);
            TEST_FAIL("expected timer pool to exhaust");
        }

        // Cancel all created timers
        for (int i = 0; i < created; i++) {
            hive_timer_cancel(timers[i]);
        }
    }

    // ========================================================================
    // Test 11: Zero delay timer (fires immediately)
    // ========================================================================
    printf("\nTest 11: Zero delay timer\n");
    {
        timer_id_t timer;
        uint64_t start = time_ms();

        hive_status_t status =
            hive_timer_after(0, &timer); // 0 delay - should fire immediately
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_timer_after(0) failed");
        } else {
            hive_message_t msg;
            status = hive_ipc_recv(&msg, 100); // 100ms timeout
            uint64_t elapsed = time_ms() - start;

            if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
                printf("    Zero delay timer fired after %lu ms\n",
                       (unsigned long)elapsed);
                TEST_PASS("zero delay timer fires immediately");
            } else {
                TEST_FAIL("zero delay timer did not fire");
            }
        }
    }

    // ========================================================================
    // Test 12: Zero-interval periodic timer (should be handled safely)
    // ========================================================================
    printf("\nTest 12: Zero-interval periodic timer\n");
    {
        timer_id_t timer;

        // A zero-interval periodic timer could fire very fast
        // It should either be rejected or treated as minimum interval
        hive_status_t status = hive_timer_every(0, &timer);

        if (HIVE_FAILED(status)) {
            TEST_PASS("hive_timer_every(0) is rejected");
        } else {
            // If accepted, it should fire but not overwhelm the system
            // Receive a few ticks and cancel immediately
            int ticks = 0;
            for (int i = 0; i < 5; i++) {
                hive_message_t msg;
                status = hive_ipc_recv(&msg, 10); // 10ms timeout
                if (HIVE_SUCCEEDED(status) && hive_msg_is_timer(&msg)) {
                    ticks++;
                }
            }

            hive_timer_cancel(timer);

            if (ticks > 0) {
                printf("    Zero-interval timer fired %d times in 50ms\n",
                       ticks);
                TEST_PASS("hive_timer_every(0) handled safely");
            } else {
                TEST_FAIL("zero-interval timer created but never fired");
            }
        }
    }

    // ========================================================================
    // Test 13: hive_sleep basic functionality
    // ========================================================================
    printf("\nTest 13: hive_sleep basic functionality\n");
    {
        uint64_t start = time_ms();
        hive_status_t status = hive_sleep(100000); // 100ms
        uint64_t elapsed = time_ms() - start;

        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_sleep failed");
        } else if (elapsed >= 80 && elapsed <= 200) {
            printf("    hive_sleep(100ms) took %lu ms\n",
                   (unsigned long)elapsed);
            TEST_PASS("hive_sleep sleeps for correct duration");
        } else {
            printf("    hive_sleep(100ms) took %lu ms (expected ~100ms)\n",
                   (unsigned long)elapsed);
            TEST_FAIL("hive_sleep duration incorrect");
        }
    }

    // ========================================================================
    // Test 14: hive_get_time returns monotonically increasing values
    // ========================================================================
    printf("\nTest 14: hive_get_time monotonicity\n");
    {
        uint64_t t1 = hive_get_time();
        // Small busy loop to let some time pass
        for (volatile int i = 0; i < 10000; i++) {
        }
        uint64_t t2 = hive_get_time();
        // Another small delay
        for (volatile int i = 0; i < 10000; i++) {
        }
        uint64_t t3 = hive_get_time();

        if (t2 >= t1 && t3 >= t2) {
            printf("    t1=%lu, t2=%lu, t3=%lu (monotonic)\n",
                   (unsigned long)t1, (unsigned long)t2, (unsigned long)t3);
            TEST_PASS("hive_get_time is monotonically increasing");
        } else {
            printf("    t1=%lu, t2=%lu, t3=%lu (NOT monotonic!)\n",
                   (unsigned long)t1, (unsigned long)t2, (unsigned long)t3);
            TEST_FAIL("hive_get_time is not monotonic");
        }
    }

    // ========================================================================
    // Test 15: hive_get_time measures elapsed time correctly
    // ========================================================================
    printf("\nTest 15: hive_get_time measures elapsed time\n");
    {
        uint64_t start = hive_get_time();
        hive_sleep(100000); // Sleep 100ms
        uint64_t end = hive_get_time();
        uint64_t elapsed_us = end - start;

        // Should be approximately 100ms (100000us), allow 80-200ms range
        if (elapsed_us >= 80000 && elapsed_us <= 200000) {
            printf("    Elapsed: %lu us (expected ~100000us)\n",
                   (unsigned long)elapsed_us);
            TEST_PASS("hive_get_time measures elapsed time correctly");
        } else {
            printf("    Elapsed: %lu us (expected ~100000us)\n",
                   (unsigned long)elapsed_us);
            TEST_FAIL("hive_get_time elapsed time incorrect");
        }
    }

    // ========================================================================
    // Test 16: hive_get_time returns microseconds (sanity check)
    // ========================================================================
    printf("\nTest 16: hive_get_time returns reasonable values\n");
    {
        uint64_t t = hive_get_time();
        // Should be non-zero and less than ~100 years in microseconds
        // 100 years ~ 3.15e15 microseconds
        if (t > 0 && t < 3150000000000000ULL) {
            printf("    Current time: %lu us\n", (unsigned long)t);
            TEST_PASS("hive_get_time returns reasonable microsecond value");
        } else {
            printf("    Current time: %lu us (suspicious)\n", (unsigned long)t);
            TEST_FAIL("hive_get_time returned suspicious value");
        }
    }

    // ========================================================================
    // Test 17: hive_sleep preserves messages (selective receive)
    // ========================================================================
    printf("\nTest 17: hive_sleep preserves messages\n");
    {
        // Notify a message to self before sleeping
        actor_id_t self = hive_self();
        int test_data = 12345;
        hive_status_t status =
            hive_ipc_notify(self, HIVE_TAG_NONE, &test_data, sizeof(test_data));
        if (HIVE_FAILED(status)) {
            TEST_FAIL("failed to notify message to self");
        } else {
            // Sleep - the message should NOT be consumed
            status = hive_sleep(50000); // 50ms
            if (HIVE_FAILED(status)) {
                TEST_FAIL("hive_sleep failed");
            } else {
                // Now receive - the message should still be there
                hive_message_t msg;
                status = hive_ipc_recv(&msg, 0); // Non-blocking

                if (HIVE_FAILED(status)) {
                    TEST_FAIL("message lost during sleep");
                } else if (hive_msg_is_timer(&msg)) {
                    TEST_FAIL("received timer message instead of our message");
                } else {
                    int *received = (int *)msg.data;
                    if (*received == test_data) {
                        TEST_PASS("message preserved during sleep");
                    } else {
                        printf("    Expected %d, got %d\n", test_data,
                               *received);
                        TEST_FAIL("message data corrupted");
                    }
                }
            }
        }
    }

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n",
           tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return;
}

int main(void) {
    printf("=== Timer (hive_timer) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    actor_id_t runner;
    if (HIVE_FAILED(hive_spawn(run_timer_tests, NULL, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    return tests_failed > 0 ? 1 : 0;
}
