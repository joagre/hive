// HAL Event System Test Suite
//
// Tests the HAL event signaling API for interrupt-driven actor wakeup:
// - Event lifecycle (create/destroy)
// - Signal/is_set/clear operations
// - hive_event_wait() with timeout
// - hive_select() with HAL event sources

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_select.h"
#include "hal/hive_hal_event.h"
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

static void run_hal_event_tests(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // ========================================================================
    // Test 1: Event creation
    // ========================================================================
    printf("\nTest 1: Event creation\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event != HIVE_HAL_EVENT_INVALID) {
            printf("    Created event ID: %u\n", (unsigned)event);
            TEST_PASS("event creation succeeds");
            hive_hal_event_destroy(event);
        } else {
            TEST_FAIL("event creation failed");
        }
    }

    // ========================================================================
    // Test 2: Event is initially not set
    // ========================================================================
    printf("\nTest 2: Event is initially not set\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else if (!hive_hal_event_is_set(event)) {
            TEST_PASS("new event is not set");
            hive_hal_event_destroy(event);
        } else {
            TEST_FAIL("new event should not be set");
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 3: Signal sets the event
    // ========================================================================
    printf("\nTest 3: Signal sets the event\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            hive_hal_event_signal(event);
            if (hive_hal_event_is_set(event)) {
                TEST_PASS("signal sets the event");
            } else {
                TEST_FAIL("event should be set after signal");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 4: Clear clears the event
    // ========================================================================
    printf("\nTest 4: Clear clears the event\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            hive_hal_event_signal(event);
            hive_hal_event_clear(event);
            if (!hive_hal_event_is_set(event)) {
                TEST_PASS("clear clears the event");
            } else {
                TEST_FAIL("event should not be set after clear");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 5: Multiple events are independent
    // ========================================================================
    printf("\nTest 5: Multiple events are independent\n");
    {
        hive_hal_event_id_t event1 = hive_hal_event_create();
        hive_hal_event_id_t event2 = hive_hal_event_create();

        if (event1 == HIVE_HAL_EVENT_INVALID ||
            event2 == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create events");
        } else {
            hive_hal_event_signal(event1);

            if (hive_hal_event_is_set(event1) &&
                !hive_hal_event_is_set(event2)) {
                TEST_PASS("signaling one event does not affect another");
            } else {
                TEST_FAIL("events should be independent");
            }
        }

        if (event1 != HIVE_HAL_EVENT_INVALID) {
            hive_hal_event_destroy(event1);
        }
        if (event2 != HIVE_HAL_EVENT_INVALID) {
            hive_hal_event_destroy(event2);
        }
    }

    // ========================================================================
    // Test 6: Event ID reuse after destroy
    // ========================================================================
    printf("\nTest 6: Event ID reuse after destroy\n");
    {
        hive_hal_event_id_t event1 = hive_hal_event_create();
        if (event1 == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create first event");
        } else {
            hive_hal_event_destroy(event1);

            hive_hal_event_id_t event2 = hive_hal_event_create();
            if (event2 != HIVE_HAL_EVENT_INVALID) {
                printf("    First ID: %u, reused ID: %u\n", (unsigned)event1,
                       (unsigned)event2);
                TEST_PASS("destroyed event ID can be reused");
                hive_hal_event_destroy(event2);
            } else {
                TEST_FAIL("could not create event after destroy");
            }
        }
    }

    // ========================================================================
    // Test 7: hive_event_wait() with pre-signaled event
    // ========================================================================
    printf("\nTest 7: hive_event_wait() with pre-signaled event\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            // Signal before waiting
            hive_hal_event_signal(event);

            uint64_t start = time_ms();
            hive_status_t status = hive_event_wait(event, 1000);
            uint64_t elapsed = time_ms() - start;

            if (HIVE_SUCCEEDED(status)) {
                printf("    Wait returned in %lu ms (should be near 0)\n",
                       (unsigned long)elapsed);
                if (elapsed < 100) {
                    TEST_PASS("pre-signaled event returns immediately");
                } else {
                    TEST_FAIL("pre-signaled event should not wait");
                }
            } else {
                TEST_FAIL("hive_event_wait failed");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 8: hive_event_wait() timeout
    // ========================================================================
    printf("\nTest 8: hive_event_wait() timeout\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            uint64_t start = time_ms();
            hive_status_t status = hive_event_wait(event, 100); // 100ms timeout
            uint64_t elapsed = time_ms() - start;

            if (status.code == HIVE_ERR_TIMEOUT) {
                printf("    Timeout after %lu ms (expected ~100ms)\n",
                       (unsigned long)elapsed);
                if (elapsed >= 80 && elapsed <= 200) {
                    TEST_PASS("hive_event_wait times out correctly");
                } else {
                    TEST_FAIL("timeout duration incorrect");
                }
            } else if (status.code == HIVE_ERR_WOULDBLOCK) {
                // Also acceptable - event not set
                TEST_PASS("hive_event_wait returns wouldblock");
            } else {
                TEST_FAIL("expected timeout or wouldblock");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 9: hive_event_wait() non-blocking (timeout=0)
    // ========================================================================
    printf("\nTest 9: hive_event_wait() non-blocking (timeout=0)\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            uint64_t start = time_ms();
            hive_status_t status = hive_event_wait(event, 0);
            uint64_t elapsed = time_ms() - start;

            if (status.code == HIVE_ERR_WOULDBLOCK ||
                status.code == HIVE_ERR_TIMEOUT) {
                printf("    Non-blocking check took %lu ms\n",
                       (unsigned long)elapsed);
                if (elapsed < 50) {
                    TEST_PASS("non-blocking returns immediately");
                } else {
                    TEST_FAIL("non-blocking should be instant");
                }
            } else {
                TEST_FAIL("expected wouldblock or timeout");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 10: hive_select() with HAL event source
    // ========================================================================
    printf("\nTest 10: hive_select() with HAL event source\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            // Signal the event before select
            hive_hal_event_signal(event);

            hive_select_source_t sources[] = {
                {HIVE_SEL_HAL_EVENT, .event = event},
            };
            hive_select_result_t result;

            hive_status_t status = hive_select(sources, 1, &result, 100);

            if (HIVE_SUCCEEDED(status)) {
                if (result.type == HIVE_SEL_HAL_EVENT && result.index == 0) {
                    TEST_PASS("hive_select returns HAL event");
                } else {
                    printf("    result.type=%d, result.index=%zu\n",
                           result.type, result.index);
                    TEST_FAIL("wrong result type or index");
                }
            } else {
                printf("    status.code=%d\n", status.code);
                TEST_FAIL("hive_select failed");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 11: hive_select() with HAL event and IPC timer
    // ========================================================================
    printf("\nTest 11: hive_select() with HAL event and IPC timer\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        hive_timer_id_t timer;
        hive_timer_after(50000, &timer); // 50ms timer

        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            // Don't signal the event - timer should fire first
            hive_select_source_t sources[] = {
                {HIVE_SEL_HAL_EVENT, .event = event},
                {HIVE_SEL_IPC,
                 .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, HIVE_ID_ANY, timer}},
            };
            hive_select_result_t result;

            uint64_t start = time_ms();
            hive_status_t status = hive_select(sources, 2, &result, 200);
            uint64_t elapsed = time_ms() - start;

            if (HIVE_SUCCEEDED(status)) {
                if (result.type == HIVE_SEL_IPC && result.index == 1) {
                    printf("    Timer fired after %lu ms\n",
                           (unsigned long)elapsed);
                    TEST_PASS("timer fires when HAL event not signaled");
                } else {
                    printf("    result.type=%d, result.index=%zu\n",
                           result.type, result.index);
                    TEST_FAIL("expected timer to fire");
                }
            } else {
                printf("    status.code=%d\n", status.code);
                TEST_FAIL("hive_select failed");
            }
            hive_hal_event_destroy(event);
        }
    }

    // ========================================================================
    // Test 12: hive_select() priority - first source wins
    // ========================================================================
    printf("\nTest 12: hive_select() priority - first source wins\n");
    {
        hive_hal_event_id_t event1 = hive_hal_event_create();
        hive_hal_event_id_t event2 = hive_hal_event_create();

        if (event1 == HIVE_HAL_EVENT_INVALID ||
            event2 == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create events");
        } else {
            // Signal both events
            hive_hal_event_signal(event1);
            hive_hal_event_signal(event2);

            hive_select_source_t sources[] = {
                {HIVE_SEL_HAL_EVENT, .event = event1},
                {HIVE_SEL_HAL_EVENT, .event = event2},
            };
            hive_select_result_t result;

            hive_status_t status = hive_select(sources, 2, &result, 100);

            if (HIVE_SUCCEEDED(status)) {
                if (result.index == 0) {
                    TEST_PASS("first source wins when multiple ready");
                } else {
                    printf("    result.index=%zu (expected 0)\n", result.index);
                    TEST_FAIL("first source should win");
                }
            } else {
                TEST_FAIL("hive_select failed");
            }
        }

        if (event1 != HIVE_HAL_EVENT_INVALID) {
            hive_hal_event_destroy(event1);
        }
        if (event2 != HIVE_HAL_EVENT_INVALID) {
            hive_hal_event_destroy(event2);
        }
    }

    // ========================================================================
    // Test 13: Event pool exhaustion
    // ========================================================================
    printf("\nTest 13: Event pool exhaustion (HIVE_HAL_EVENT_MAX=%d)\n",
           HIVE_HAL_EVENT_MAX);
    {
        hive_hal_event_id_t events[HIVE_HAL_EVENT_MAX + 1];
        int created = 0;

        // Create events until pool exhaustion
        for (int i = 0; i <= HIVE_HAL_EVENT_MAX; i++) {
            events[i] = hive_hal_event_create();
            if (events[i] == HIVE_HAL_EVENT_INVALID) {
                printf("    Event creation failed after %d events\n", created);
                break;
            }
            created++;
        }

        if (created <= HIVE_HAL_EVENT_MAX) {
            TEST_PASS("event pool exhaustion detected");
        } else {
            TEST_FAIL("expected pool to exhaust");
        }

        // Cleanup
        for (int i = 0; i < created; i++) {
            hive_hal_event_destroy(events[i]);
        }
    }

    // ========================================================================
    // Test 14: Destroy clears signaled state
    // ========================================================================
    printf("\nTest 14: Destroy clears signaled state\n");
    {
        hive_hal_event_id_t event1 = hive_hal_event_create();
        if (event1 == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            hive_hal_event_signal(event1);
            hive_hal_event_destroy(event1);

            // Create new event - should reuse the ID
            hive_hal_event_id_t event2 = hive_hal_event_create();
            if (event2 == HIVE_HAL_EVENT_INVALID) {
                TEST_FAIL("could not create second event");
            } else {
                // New event should not be set
                if (!hive_hal_event_is_set(event2)) {
                    TEST_PASS("reused event is not set");
                } else {
                    TEST_FAIL("reused event should not inherit signal state");
                }
                hive_hal_event_destroy(event2);
            }
        }
    }

    // ========================================================================
    // Test 15: hive_event_wait clears event on return
    // ========================================================================
    printf("\nTest 15: hive_event_wait clears event on return\n");
    {
        hive_hal_event_id_t event = hive_hal_event_create();
        if (event == HIVE_HAL_EVENT_INVALID) {
            TEST_FAIL("could not create event");
        } else {
            hive_hal_event_signal(event);
            hive_event_wait(event, 100);

            // Event should be cleared after wait returns
            if (!hive_hal_event_is_set(event)) {
                TEST_PASS("event cleared after wait");
            } else {
                TEST_FAIL("event should be cleared after wait");
            }
            hive_hal_event_destroy(event);
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
    printf("=== HAL Event System Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    hive_actor_id_t runner;
    if (HIVE_FAILED(
            hive_spawn(run_hal_event_tests, NULL, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    return tests_failed > 0 ? 1 : 0;
}
