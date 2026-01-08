#include "rt_runtime.h"
#include "rt_link.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <string.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  ✓ PASS: %s\n", name); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  ✗ FAIL: %s\n", name); tests_failed++; } while(0)

// ============================================================================
// Test 1: Basic monitor - get notification when target exits normally
// ============================================================================

static void target_normal_exit(void *arg) {
    (void)arg;
    // Exit normally immediately
    rt_exit();
}

static void test1_monitor_actor(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic monitor (normal exit)\n");

    // Spawn target
    actor_id target = rt_spawn(target_normal_exit, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    // Monitor it
    uint32_t ref;
    rt_status status = rt_monitor(target, &ref);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_monitor");
        rt_exit();
    }

    // Wait for exit notification
    rt_message msg;
    status = rt_ipc_recv(&msg, 1000);  // 1 second timeout
    if (RT_FAILED(status)) {
        TEST_FAIL("receive exit notification (timeout)");
        rt_exit();
    }

    if (!rt_is_exit_msg(&msg)) {
        TEST_FAIL("message is not exit notification");
        rt_exit();
    }

    rt_exit_msg exit_info;
    rt_decode_exit(&msg, &exit_info);

    if (exit_info.actor != target) {
        TEST_FAIL("exit notification from wrong actor");
        rt_exit();
    }

    if (exit_info.reason != RT_EXIT_NORMAL) {
        TEST_FAIL("exit reason should be NORMAL");
        rt_exit();
    }

    TEST_PASS("monitor receives normal exit notification");
    rt_exit();
}

// ============================================================================
// Test 2: (Crash testing is covered in stack_overflow_test.c)
// ============================================================================

// ============================================================================
// Test 3: Multiple monitors - one actor monitors multiple targets
// ============================================================================

static void target_delayed_exit(void *arg) {
    int delay_ms = *(int *)arg;

    timer_id timer;
    rt_timer_after(delay_ms * 1000, &timer);  // Convert ms to us

    rt_message msg;
    rt_ipc_recv(&msg, -1);  // Wait for timer

    rt_exit();
}

static void test3_multi_monitor_actor(void *arg) {
    (void)arg;
    printf("\nTest 3: Multiple monitors\n");

    // Spawn 3 targets with different delays
    int delays[3] = {50, 100, 150};
    actor_id targets[3];
    uint32_t refs[3];

    for (int i = 0; i < 3; i++) {
        targets[i] = rt_spawn(target_delayed_exit, &delays[i]);
        if (targets[i] == ACTOR_ID_INVALID) {
            TEST_FAIL("spawn target");
            rt_exit();
        }

        rt_status status = rt_monitor(targets[i], &refs[i]);
        if (RT_FAILED(status)) {
            TEST_FAIL("rt_monitor");
            rt_exit();
        }
    }

    // Receive all 3 exit notifications
    int received = 0;
    bool seen[3] = {false, false, false};

    while (received < 3) {
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 2000);  // 2 second timeout
        if (RT_FAILED(status)) {
            printf("  Only received %d/3 notifications\n", received);
            TEST_FAIL("receive all exit notifications");
            rt_exit();
        }

        if (!rt_is_exit_msg(&msg)) {
            continue;  // Skip non-exit messages
        }

        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);

        for (int i = 0; i < 3; i++) {
            if (exit_info.actor == targets[i] && !seen[i]) {
                seen[i] = true;
                received++;
                break;
            }
        }
    }

    TEST_PASS("received all 3 exit notifications");
    rt_exit();
}

// ============================================================================
// Test 4: Demonitor - cancel monitoring before target dies
// ============================================================================

static void target_slow_exit(void *arg) {
    (void)arg;

    timer_id timer;
    rt_timer_after(500000, &timer);  // 500ms delay

    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

static void test4_monitor_cancel_actor(void *arg) {
    (void)arg;
    printf("\nTest 4: Demonitor\n");

    // Spawn target
    actor_id target = rt_spawn(target_slow_exit, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    // Monitor it
    uint32_t ref;
    rt_status status = rt_monitor(target, &ref);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_monitor");
        rt_exit();
    }

    // Immediately monitor_cancel
    status = rt_monitor_cancel(ref);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_monitor_cancel");
        rt_exit();
    }

    // Wait a bit - should NOT receive exit notification
    rt_message msg;
    status = rt_ipc_recv(&msg, 700);  // 700ms timeout (target exits at 500ms)

    if (status.code == RT_ERR_TIMEOUT) {
        TEST_PASS("monitor_cancel prevents exit notification");
    } else if (RT_FAILED(status)) {
        TEST_PASS("monitor_cancel prevents exit notification (no message)");
    } else {
        // Got a message - check if it's an exit notification
        if (rt_is_exit_msg(&msg)) {
            TEST_FAIL("received exit notification after monitor_cancel");
        } else {
            TEST_PASS("monitor_cancel prevents exit notification");
        }
    }

    rt_exit();
}

// ============================================================================
// Test 5: Monitor is unidirectional - target doesn't get notified when monitor dies
// ============================================================================

static bool g_target_received_exit = false;

static void target_waits_for_exit(void *arg) {
    (void)arg;

    // Wait for any message
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 500);  // 500ms timeout

    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        g_target_received_exit = true;
    }

    rt_exit();
}

static void monitor_dies_early(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Monitor the target
    uint32_t ref;
    rt_monitor(target, &ref);

    // Die immediately (monitor dies, target should NOT be notified)
    rt_exit();
}

static void test5_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 5: Monitor is unidirectional (target not notified when monitor dies)\n");

    // Spawn target first
    actor_id target = rt_spawn(target_waits_for_exit, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    // Spawn monitor that will monitor target then die
    actor_id monitor = rt_spawn(monitor_dies_early, &target);
    if (monitor == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn monitor");
        rt_exit();
    }

    // Wait for both to finish
    timer_id timer;
    rt_timer_after(700000, &timer);  // 700ms
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (!g_target_received_exit) {
        TEST_PASS("target NOT notified when monitor dies (unidirectional)");
    } else {
        TEST_FAIL("target received exit notification (should be unidirectional)");
    }

    rt_exit();
}

// ============================================================================
// Test 6: Monitor invalid/dead actor
// ============================================================================

static void test6_monitor_invalid(void *arg) {
    (void)arg;
    printf("\nTest 6: Monitor invalid actor\n");

    uint32_t ref;

    // Try to monitor invalid actor ID
    rt_status status = rt_monitor(ACTOR_ID_INVALID, &ref);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_monitor rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("rt_monitor should reject ACTOR_ID_INVALID");
    }

    // Try to monitor non-existent actor (high ID that doesn't exist)
    status = rt_monitor(9999, &ref);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_monitor rejects non-existent actor");
    } else {
        TEST_FAIL("rt_monitor should reject non-existent actor");
    }

    rt_exit();
}

// ============================================================================
// Test 7: Demonitor invalid/non-existent ref
// ============================================================================

static void test7_monitor_cancel_invalid(void *arg) {
    (void)arg;
    printf("\nTest 7: Demonitor invalid ref\n");

    // Try to monitor_cancel with invalid ref (0 or very high number)
    rt_status status = rt_monitor_cancel(0);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_monitor_cancel rejects ref 0");
    } else {
        TEST_PASS("rt_monitor_cancel ref 0 is no-op");
    }

    status = rt_monitor_cancel(99999);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_monitor_cancel rejects non-existent ref");
    } else {
        TEST_PASS("rt_monitor_cancel non-existent ref is no-op");
    }

    rt_exit();
}

// ============================================================================
// Test 8: Double monitor_cancel (same ref twice)
// ============================================================================

static void double_monitor_cancel_target(void *arg) {
    (void)arg;
    timer_id timer;
    rt_timer_after(500000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);
    rt_exit();
}

static void test8_double_monitor_cancel(void *arg) {
    (void)arg;
    printf("\nTest 8: Double monitor_cancel (same ref twice)\n");

    actor_id target = rt_spawn(double_monitor_cancel_target, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    uint32_t ref;
    rt_status status = rt_monitor(target, &ref);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_monitor");
        rt_exit();
    }

    // First monitor_cancel should succeed
    status = rt_monitor_cancel(ref);
    if (RT_FAILED(status)) {
        TEST_FAIL("first monitor_cancel failed");
        rt_exit();
    }
    TEST_PASS("first monitor_cancel succeeds");

    // Second monitor_cancel should fail or be no-op
    status = rt_monitor_cancel(ref);
    if (RT_FAILED(status)) {
        TEST_PASS("second monitor_cancel fails (already monitor_canceled)");
    } else {
        TEST_PASS("second monitor_cancel is no-op");
    }

    // Wait for target to exit
    timer_id timer;
    rt_timer_after(600000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

// ============================================================================
// Test 9: Monitor pool exhaustion (RT_MONITOR_ENTRY_POOL_SIZE=128)
// ============================================================================

static void monitor_pool_target(void *arg) {
    (void)arg;
    rt_message msg;
    rt_ipc_recv(&msg, 5000);
    rt_exit();
}

static void test9_monitor_pool_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 9: Monitor pool exhaustion (RT_MONITOR_ENTRY_POOL_SIZE=%d)\n",
           RT_MONITOR_ENTRY_POOL_SIZE);

    actor_id targets[RT_MONITOR_ENTRY_POOL_SIZE + 10];
    uint32_t refs[RT_MONITOR_ENTRY_POOL_SIZE + 10];
    int spawned = 0;
    int monitored = 0;

    // Spawn actors and monitor them until pool exhaustion
    for (int i = 0; i < RT_MONITOR_ENTRY_POOL_SIZE + 10; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = 8 * 1024;

        actor_id target = rt_spawn_ex(monitor_pool_target, NULL, &cfg);
        if (target == ACTOR_ID_INVALID) {
            break;
        }
        targets[spawned++] = target;

        rt_status status = rt_monitor(target, &refs[monitored]);
        if (RT_FAILED(status)) {
            printf("    Monitor failed after %d monitors (pool exhausted)\n", monitored);
            break;
        }
        monitored++;
    }

    if (monitored < RT_MONITOR_ENTRY_POOL_SIZE + 10) {
        TEST_PASS("monitor pool exhaustion detected");
    } else {
        printf("    Monitored all %d actors without exhaustion\n", monitored);
        TEST_FAIL("expected monitor pool to exhaust");
    }

    // Signal all actors to exit
    for (int i = 0; i < spawned; i++) {
        int done = 1;
        rt_ipc_notify(targets[i], &done, sizeof(done));
    }

    // Wait for cleanup
    timer_id timer;
    rt_timer_after(200000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    // Drain exit notifications
    while (rt_ipc_pending()) {
        rt_ipc_recv(&msg, 0);
    }

    rt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_monitor_actor,
    test3_multi_monitor_actor,
    test4_monitor_cancel_actor,
    test5_coordinator,
    test6_monitor_invalid,
    test7_monitor_cancel_invalid,
    test8_double_monitor_cancel,
    test9_monitor_pool_exhaustion,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = 64 * 1024;

        actor_id test = rt_spawn_ex(test_funcs[i], NULL, &cfg);
        if (test == ACTOR_ID_INVALID) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        // Link to test actor so we know when it finishes
        rt_link(test);

        // Wait for test to finish
        rt_message msg;
        rt_ipc_recv(&msg, 5000);  // 5 second timeout per test
    }

    rt_exit();
}

int main(void) {
    printf("=== Monitor (rt_monitor) Test Suite ===\n");

    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner = rt_spawn_ex(run_all_tests, NULL, &cfg);
    if (runner == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn test runner\n");
        rt_cleanup();
        return 1;
    }

    rt_run();
    rt_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
