#include "acrt_runtime.h"
#include "acrt_timer.h"
#include "acrt_ipc.h"
#include "acrt_static_config.h"
#include <stdio.h>
#include <time.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); tests_failed++; } while(0)

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static void run_timer_tests(void *arg) {
    (void)arg;

    // ========================================================================
    // Test 1: One-shot timer (acrt_timer_after)
    // ========================================================================
    printf("\nTest 1: One-shot timer (acrt_timer_after)\n");
    {
        timer_id timer;
        acrt_status status = acrt_timer_after(100000, &timer);  // 100ms
        if (ACRT_FAILED(status)) {
            TEST_FAIL("acrt_timer_after failed");
        } else if (timer == TIMER_ID_INVALID) {
            TEST_FAIL("got invalid timer ID");
        } else {
            uint64_t start = time_ms();
            acrt_message msg;
            status = acrt_ipc_recv(&msg, -1);  // Wait indefinitely
            uint64_t elapsed = time_ms() - start;

            if (ACRT_FAILED(status)) {
                TEST_FAIL("did not receive timer message");
            } else if (!acrt_msg_is_timer(&msg)) {
                TEST_FAIL("message is not a timer tick");
            } else if (elapsed >= 80 && elapsed <= 200) {
                printf("    Timer fired after %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
                TEST_PASS("one-shot timer fires at correct time");
            } else {
                printf("    Timer fired after %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
                TEST_FAIL("timer fired at wrong time");
            }
        }
    }

    // ========================================================================
    // Test 2: Timer cancellation
    // ========================================================================
    printf("\nTest 2: Timer cancellation\n");
    {
        timer_id timer;
        acrt_timer_after(100000, &timer);  // 100ms

        acrt_status status = acrt_timer_cancel(timer);
        if (ACRT_FAILED(status)) {
            TEST_FAIL("acrt_timer_cancel failed");
        } else {
            acrt_message msg;
            status = acrt_ipc_recv(&msg, 200);  // 200ms timeout

            if (status.code == ACRT_ERR_TIMEOUT) {
                TEST_PASS("cancelled timer does not fire");
            } else if (ACRT_FAILED(status)) {
                TEST_PASS("cancelled timer does not fire (no message)");
            } else if (acrt_msg_is_timer(&msg)) {
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
        timer_id timer;
        acrt_timer_after(50000, &timer);

        acrt_message msg;
        acrt_status status = acrt_ipc_recv(&msg, -1);

        if (ACRT_FAILED(status)) {
            TEST_FAIL("did not receive timer message");
        } else if (msg.sender == acrt_self()) {
            TEST_PASS("timer message sender is the owning actor");
        } else {
            printf("    Sender: %u, expected: %u (self)\n", msg.sender, acrt_self());
            TEST_FAIL("wrong sender ID");
        }
    }

    // ========================================================================
    // Test 4: acrt_msg_is_timer correctly identifies timer messages
    // ========================================================================
    printf("\nTest 4: acrt_msg_is_timer identifies timer messages\n");
    {
        timer_id timer;
        acrt_timer_after(50000, &timer);

        acrt_message msg;
        acrt_status status = acrt_ipc_recv(&msg, -1);

        if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
            TEST_PASS("timer message detected by acrt_msg_is_timer");
        } else {
            TEST_FAIL("timer message not detected");
        }

        // Now test that regular messages are NOT detected as timer ticks
        actor_id self = acrt_self();
        const char *data = "not a timer";
        acrt_ipc_notify(self, data, 12);

        status = acrt_ipc_recv(&msg, 100);
        if (!ACRT_FAILED(status) && !acrt_msg_is_timer(&msg)) {
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
        acrt_status status = acrt_timer_cancel(TIMER_ID_INVALID);
        if (ACRT_FAILED(status)) {
            TEST_PASS("cancel TIMER_ID_INVALID fails");
        } else {
            TEST_FAIL("cancel TIMER_ID_INVALID should fail");
        }

        status = acrt_timer_cancel(9999);
        if (ACRT_FAILED(status)) {
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
        timer_id timer;
        uint64_t start = time_ms();

        acrt_timer_after(10000, &timer);  // 10ms

        acrt_message msg;
        acrt_status status = acrt_ipc_recv(&msg, -1);

        uint64_t elapsed = time_ms() - start;

        if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
            printf("    Short timer fired after %lu ms\n", (unsigned long)elapsed);
            TEST_PASS("short delay timer works");
        } else {
            TEST_FAIL("short delay timer did not fire");
        }
    }

    // ========================================================================
    // Test 7: Periodic timer (acrt_timer_every)
    // NOTE: This test may fail due to implementation limitations with
    //       periodic timers not firing after the first one-shot timer completes.
    // ========================================================================
    printf("\nTest 7: Periodic timer (acrt_timer_every)\n");
    {
        timer_id timer;
        acrt_status status = acrt_timer_every(50000, &timer);  // 50ms interval
        if (ACRT_FAILED(status)) {
            TEST_FAIL("acrt_timer_every failed to create timer");
        } else {
            int tick_count = 0;
            uint64_t start = time_ms();

            // Try to receive 5 ticks
            for (int i = 0; i < 5; i++) {
                acrt_message msg;
                status = acrt_ipc_recv(&msg, 200);  // 200ms timeout per tick
                if (ACRT_FAILED(status)) {
                    printf("    Tick %d: recv failed (timeout or error)\n", i + 1);
                    break;
                }
                if (acrt_msg_is_timer(&msg)) {
                    tick_count++;
                }
            }

            uint64_t elapsed = time_ms() - start;
            acrt_timer_cancel(timer);

            if (tick_count >= 5) {
                printf("    Received %d ticks in %lu ms\n", tick_count, (unsigned long)elapsed);
                TEST_PASS("periodic timer fires multiple times");
            } else {
                printf("    Only received %d/5 ticks in %lu ms\n", tick_count, (unsigned long)elapsed);
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
        timer_id timer1, timer2, timer3;

        acrt_status s1 = acrt_timer_after(50000, &timer1);   // 50ms
        acrt_status s2 = acrt_timer_after(100000, &timer2);  // 100ms
        acrt_status s3 = acrt_timer_after(150000, &timer3);  // 150ms

        if (ACRT_FAILED(s1) || ACRT_FAILED(s2) || ACRT_FAILED(s3)) {
            TEST_FAIL("failed to create multiple timers");
        } else {
            int received = 0;
            uint64_t start = time_ms();

            for (int i = 0; i < 3; i++) {
                acrt_message msg;
                acrt_status status = acrt_ipc_recv(&msg, 300);  // 300ms timeout
                if (ACRT_FAILED(status)) {
                    printf("    Timer %d: recv failed\n", i + 1);
                    continue;
                }
                if (acrt_msg_is_timer(&msg)) {
                    uint64_t elapsed = time_ms() - start;
                    printf("    Timer tick %d received at %lu ms\n", received + 1, (unsigned long)elapsed);
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
        timer_id timer;
        acrt_status status = acrt_timer_every(30000, &timer);  // 30ms interval
        if (ACRT_FAILED(status)) {
            TEST_FAIL("acrt_timer_every failed");
        } else {
            // Try to receive some ticks
            int ticks = 0;
            for (int i = 0; i < 3; i++) {
                acrt_message msg;
                status = acrt_ipc_recv(&msg, 100);  // 100ms timeout
                if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
                    ticks++;
                }
            }

            // Cancel the timer
            status = acrt_timer_cancel(timer);
            if (ACRT_FAILED(status)) {
                TEST_FAIL("acrt_timer_cancel failed");
            } else {
                // Wait and ensure no more ticks arrive
                acrt_message msg;
                status = acrt_ipc_recv(&msg, 100);  // 100ms

                if (status.code == ACRT_ERR_TIMEOUT) {
                    printf("    Received %d ticks before cancel, then stopped\n", ticks);
                    TEST_PASS("periodic timer stops after cancel");
                } else if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
                    TEST_FAIL("received tick after cancel");
                } else {
                    TEST_PASS("periodic timer stops after cancel");
                }
            }
        }
    }

    // ========================================================================
    // Test 10: Timer pool exhaustion (ACRT_TIMER_ENTRY_POOL_SIZE=64)
    // ========================================================================
    printf("\nTest 10: Timer pool exhaustion (ACRT_TIMER_ENTRY_POOL_SIZE=%d)\n",
           ACRT_TIMER_ENTRY_POOL_SIZE);
    {
        timer_id timers[ACRT_TIMER_ENTRY_POOL_SIZE + 10];
        int created = 0;

        // Create timers until pool exhaustion
        for (int i = 0; i < ACRT_TIMER_ENTRY_POOL_SIZE + 10; i++) {
            acrt_status status = acrt_timer_after(10000000, &timers[i]);  // 10 second delay (won't fire)
            if (ACRT_FAILED(status)) {
                printf("    Timer creation failed after %d timers (pool exhausted)\n", created);
                break;
            }
            created++;
        }

        if (created < ACRT_TIMER_ENTRY_POOL_SIZE + 10) {
            TEST_PASS("timer pool exhaustion detected");
        } else {
            printf("    Created all %d timers without exhaustion\n", created);
            TEST_FAIL("expected timer pool to exhaust");
        }

        // Cancel all created timers
        for (int i = 0; i < created; i++) {
            acrt_timer_cancel(timers[i]);
        }
    }

    // ========================================================================
    // Test 11: Zero delay timer (fires immediately)
    // ========================================================================
    printf("\nTest 11: Zero delay timer\n");
    {
        timer_id timer;
        uint64_t start = time_ms();

        acrt_status status = acrt_timer_after(0, &timer);  // 0 delay - should fire immediately
        if (ACRT_FAILED(status)) {
            TEST_FAIL("acrt_timer_after(0) failed");
        } else {
            acrt_message msg;
            status = acrt_ipc_recv(&msg, 100);  // 100ms timeout
            uint64_t elapsed = time_ms() - start;

            if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
                printf("    Zero delay timer fired after %lu ms\n", (unsigned long)elapsed);
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
        timer_id timer;

        // A zero-interval periodic timer could fire very fast
        // It should either be rejected or treated as minimum interval
        acrt_status status = acrt_timer_every(0, &timer);

        if (ACRT_FAILED(status)) {
            TEST_PASS("acrt_timer_every(0) is rejected");
        } else {
            // If accepted, it should fire but not overwhelm the system
            // Receive a few ticks and cancel immediately
            int ticks = 0;
            for (int i = 0; i < 5; i++) {
                acrt_message msg;
                status = acrt_ipc_recv(&msg, 10);  // 10ms timeout
                if (!ACRT_FAILED(status) && acrt_msg_is_timer(&msg)) {
                    ticks++;
                }
            }

            acrt_timer_cancel(timer);

            if (ticks > 0) {
                printf("    Zero-interval timer fired %d times in 50ms\n", ticks);
                TEST_PASS("acrt_timer_every(0) handled safely");
            } else {
                TEST_FAIL("zero-interval timer created but never fired");
            }
        }
    }

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    acrt_exit();
}

int main(void) {
    printf("=== Timer (acrt_timer) Test Suite ===\n");

    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner;
    if (ACRT_FAILED(acrt_spawn_ex(run_timer_tests, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        acrt_cleanup();
        return 1;
    }

    acrt_run();
    acrt_cleanup();

    return tests_failed > 0 ? 1 : 0;
}
