#include "rt_runtime.h"
#include "rt_timer.h"
#include "rt_ipc.h"
#include "rt_static_config.h"
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
    // Test 1: One-shot timer (rt_timer_after)
    // ========================================================================
    printf("\nTest 1: One-shot timer (rt_timer_after)\n");
    {
        timer_id timer;
        rt_status status = rt_timer_after(100000, &timer);  // 100ms
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_timer_after failed");
        } else if (timer == TIMER_ID_INVALID) {
            TEST_FAIL("got invalid timer ID");
        } else {
            uint64_t start = time_ms();
            rt_message msg;
            status = rt_ipc_recv(&msg, -1);  // Wait indefinitely
            uint64_t elapsed = time_ms() - start;

            if (RT_FAILED(status)) {
                TEST_FAIL("did not receive timer message");
            } else if (!rt_timer_is_tick(&msg)) {
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
        rt_timer_after(100000, &timer);  // 100ms

        rt_status status = rt_timer_cancel(timer);
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_timer_cancel failed");
        } else {
            rt_message msg;
            status = rt_ipc_recv(&msg, 200);  // 200ms timeout

            if (status.code == RT_ERR_TIMEOUT) {
                TEST_PASS("cancelled timer does not fire");
            } else if (RT_FAILED(status)) {
                TEST_PASS("cancelled timer does not fire (no message)");
            } else if (rt_timer_is_tick(&msg)) {
                TEST_FAIL("received timer tick after cancellation");
            } else {
                TEST_PASS("cancelled timer does not fire");
            }
        }
    }

    // ========================================================================
    // Test 3: Timer sender ID is RT_SENDER_TIMER
    // ========================================================================
    printf("\nTest 3: Timer sender ID is RT_SENDER_TIMER\n");
    {
        timer_id timer;
        rt_timer_after(50000, &timer);

        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, -1);

        if (RT_FAILED(status)) {
            TEST_FAIL("did not receive timer message");
        } else if (msg.sender == RT_SENDER_TIMER) {
            TEST_PASS("timer message has RT_SENDER_TIMER sender");
        } else {
            printf("    Sender: %u, expected: %u\n", msg.sender, RT_SENDER_TIMER);
            TEST_FAIL("wrong sender ID");
        }
    }

    // ========================================================================
    // Test 4: rt_timer_is_tick correctly identifies timer messages
    // ========================================================================
    printf("\nTest 4: rt_timer_is_tick identifies timer messages\n");
    {
        timer_id timer;
        rt_timer_after(50000, &timer);

        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, -1);

        if (!RT_FAILED(status) && rt_timer_is_tick(&msg)) {
            TEST_PASS("timer message detected by rt_timer_is_tick");
        } else {
            TEST_FAIL("timer message not detected");
        }

        // Now test that regular messages are NOT detected as timer ticks
        actor_id self = rt_self();
        const char *data = "not a timer";
        rt_ipc_send(self, data, 12, IPC_ASYNC);

        status = rt_ipc_recv(&msg, 100);
        if (!RT_FAILED(status) && !rt_timer_is_tick(&msg)) {
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
        rt_status status = rt_timer_cancel(TIMER_ID_INVALID);
        if (RT_FAILED(status)) {
            TEST_PASS("cancel TIMER_ID_INVALID fails");
        } else {
            TEST_FAIL("cancel TIMER_ID_INVALID should fail");
        }

        status = rt_timer_cancel(9999);
        if (RT_FAILED(status)) {
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

        rt_timer_after(10000, &timer);  // 10ms

        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, -1);

        uint64_t elapsed = time_ms() - start;

        if (!RT_FAILED(status) && rt_timer_is_tick(&msg)) {
            printf("    Short timer fired after %lu ms\n", (unsigned long)elapsed);
            TEST_PASS("short delay timer works");
        } else {
            TEST_FAIL("short delay timer did not fire");
        }
    }

    // ========================================================================
    // Test 7: Periodic timer (rt_timer_every)
    // NOTE: This test may fail due to implementation limitations with
    //       periodic timers not firing after the first one-shot timer completes.
    // ========================================================================
    printf("\nTest 7: Periodic timer (rt_timer_every)\n");
    {
        timer_id timer;
        rt_status status = rt_timer_every(50000, &timer);  // 50ms interval
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_timer_every failed to create timer");
        } else {
            int tick_count = 0;
            uint64_t start = time_ms();

            // Try to receive 5 ticks
            for (int i = 0; i < 5; i++) {
                rt_message msg;
                status = rt_ipc_recv(&msg, 200);  // 200ms timeout per tick
                if (RT_FAILED(status)) {
                    printf("    Tick %d: recv failed (timeout or error)\n", i + 1);
                    break;
                }
                if (rt_timer_is_tick(&msg)) {
                    tick_count++;
                }
            }

            uint64_t elapsed = time_ms() - start;
            rt_timer_cancel(timer);

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

        rt_status s1 = rt_timer_after(50000, &timer1);   // 50ms
        rt_status s2 = rt_timer_after(100000, &timer2);  // 100ms
        rt_status s3 = rt_timer_after(150000, &timer3);  // 150ms

        if (RT_FAILED(s1) || RT_FAILED(s2) || RT_FAILED(s3)) {
            TEST_FAIL("failed to create multiple timers");
        } else {
            int received = 0;
            uint64_t start = time_ms();

            for (int i = 0; i < 3; i++) {
                rt_message msg;
                rt_status status = rt_ipc_recv(&msg, 300);  // 300ms timeout
                if (RT_FAILED(status)) {
                    printf("    Timer %d: recv failed\n", i + 1);
                    continue;
                }
                if (rt_timer_is_tick(&msg)) {
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
        rt_status status = rt_timer_every(30000, &timer);  // 30ms interval
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_timer_every failed");
        } else {
            // Try to receive some ticks
            int ticks = 0;
            for (int i = 0; i < 3; i++) {
                rt_message msg;
                status = rt_ipc_recv(&msg, 100);  // 100ms timeout
                if (!RT_FAILED(status) && rt_timer_is_tick(&msg)) {
                    ticks++;
                }
            }

            // Cancel the timer
            status = rt_timer_cancel(timer);
            if (RT_FAILED(status)) {
                TEST_FAIL("rt_timer_cancel failed");
            } else {
                // Wait and ensure no more ticks arrive
                rt_message msg;
                status = rt_ipc_recv(&msg, 100);  // 100ms

                if (status.code == RT_ERR_TIMEOUT) {
                    printf("    Received %d ticks before cancel, then stopped\n", ticks);
                    TEST_PASS("periodic timer stops after cancel");
                } else if (!RT_FAILED(status) && rt_timer_is_tick(&msg)) {
                    TEST_FAIL("received tick after cancel");
                } else {
                    TEST_PASS("periodic timer stops after cancel");
                }
            }
        }
    }

    // ========================================================================
    // Test 10: Timer pool exhaustion (RT_TIMER_ENTRY_POOL_SIZE=64)
    // ========================================================================
    printf("\nTest 10: Timer pool exhaustion (RT_TIMER_ENTRY_POOL_SIZE=%d)\n",
           RT_TIMER_ENTRY_POOL_SIZE);
    {
        timer_id timers[RT_TIMER_ENTRY_POOL_SIZE + 10];
        int created = 0;

        // Create timers until pool exhaustion
        for (int i = 0; i < RT_TIMER_ENTRY_POOL_SIZE + 10; i++) {
            rt_status status = rt_timer_after(10000000, &timers[i]);  // 10 second delay (won't fire)
            if (RT_FAILED(status)) {
                printf("    Timer creation failed after %d timers (pool exhausted)\n", created);
                break;
            }
            created++;
        }

        if (created < RT_TIMER_ENTRY_POOL_SIZE + 10) {
            TEST_PASS("timer pool exhaustion detected");
        } else {
            printf("    Created all %d timers without exhaustion\n", created);
            TEST_FAIL("expected timer pool to exhaust");
        }

        // Cancel all created timers
        for (int i = 0; i < created; i++) {
            rt_timer_cancel(timers[i]);
        }
    }

    // ========================================================================
    // Test 11: Zero delay timer (fires immediately)
    // ========================================================================
    printf("\nTest 11: Zero delay timer\n");
    {
        timer_id timer;
        uint64_t start = time_ms();

        rt_status status = rt_timer_after(0, &timer);  // 0 delay - should fire immediately
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_timer_after(0) failed");
        } else {
            rt_message msg;
            status = rt_ipc_recv(&msg, 100);  // 100ms timeout
            uint64_t elapsed = time_ms() - start;

            if (!RT_FAILED(status) && rt_timer_is_tick(&msg)) {
                printf("    Zero delay timer fired after %lu ms\n", (unsigned long)elapsed);
                TEST_PASS("zero delay timer fires immediately");
            } else {
                TEST_FAIL("zero delay timer did not fire");
            }
        }
    }

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    rt_exit();
}

int main(void) {
    printf("=== Timer (rt_timer) Test Suite ===\n");

    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner = rt_spawn_ex(run_timer_tests, NULL, &cfg);
    if (runner == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn test runner\n");
        rt_cleanup();
        return 1;
    }

    rt_run();
    rt_cleanup();

    return tests_failed > 0 ? 1 : 0;
}
