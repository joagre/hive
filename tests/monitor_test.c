#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <string.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

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
    hive_exit();
}

static void test1_monitor_actor(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic monitor (normal exit)\n");

    // Spawn target
    actor_id target;
    if (HIVE_FAILED(hive_spawn(target_normal_exit, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    // Monitor it
    uint32_t ref;
    hive_status status = hive_monitor(target, &ref);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_monitor");
        hive_exit();
    }

    // Wait for exit notification
    hive_message msg;
    status = hive_ipc_recv(&msg, 1000);  // 1 second timeout
    if (HIVE_FAILED(status)) {
        TEST_FAIL("receive exit notification (timeout)");
        hive_exit();
    }

    if (!hive_is_exit_msg(&msg)) {
        TEST_FAIL("message is not exit notification");
        hive_exit();
    }

    hive_exit_msg exit_info;
    hive_decode_exit(&msg, &exit_info);

    if (exit_info.actor != target) {
        TEST_FAIL("exit notification from wrong actor");
        hive_exit();
    }

    if (exit_info.reason != HIVE_EXIT_NORMAL) {
        TEST_FAIL("exit reason should be NORMAL");
        hive_exit();
    }

    TEST_PASS("monitor receives normal exit notification");
    hive_exit();
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
    hive_timer_after(delay_ms * 1000, &timer);  // Convert ms to us

    hive_message msg;
    hive_ipc_recv(&msg, -1);  // Wait for timer

    hive_exit();
}

static void test3_multi_monitor_actor(void *arg) {
    (void)arg;
    printf("\nTest 3: Multiple monitors\n");

    // Spawn 3 targets with different delays
    int delays[3] = {50, 100, 150};
    actor_id targets[3];
    uint32_t refs[3];

    for (int i = 0; i < 3; i++) {
        if (HIVE_FAILED(hive_spawn(target_delayed_exit, &delays[i], &targets[i]))) {
            TEST_FAIL("spawn target");
            hive_exit();
        }

        hive_status status = hive_monitor(targets[i], &refs[i]);
        if (HIVE_FAILED(status)) {
            TEST_FAIL("hive_monitor");
            hive_exit();
        }
    }

    // Receive all 3 exit notifications
    int received = 0;
    bool seen[3] = {false, false, false};

    while (received < 3) {
        hive_message msg;
        hive_status status = hive_ipc_recv(&msg, 2000);  // 2 second timeout
        if (HIVE_FAILED(status)) {
            printf("  Only received %d/3 notifications\n", received);
            TEST_FAIL("receive all exit notifications");
            hive_exit();
        }

        if (!hive_is_exit_msg(&msg)) {
            continue;  // Skip non-exit messages
        }

        hive_exit_msg exit_info;
        hive_decode_exit(&msg, &exit_info);

        for (int i = 0; i < 3; i++) {
            if (exit_info.actor == targets[i] && !seen[i]) {
                seen[i] = true;
                received++;
                break;
            }
        }
    }

    TEST_PASS("received all 3 exit notifications");
    hive_exit();
}

// ============================================================================
// Test 4: Demonitor - cancel monitoring before target dies
// ============================================================================

static void target_slow_exit(void *arg) {
    (void)arg;

    timer_id timer;
    hive_timer_after(500000, &timer);  // 500ms delay

    hive_message msg;
    hive_ipc_recv(&msg, -1);

    hive_exit();
}

static void test4_monitor_cancel_actor(void *arg) {
    (void)arg;
    printf("\nTest 4: Demonitor\n");

    // Spawn target
    actor_id target;
    if (HIVE_FAILED(hive_spawn(target_slow_exit, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    // Monitor it
    uint32_t ref;
    hive_status status = hive_monitor(target, &ref);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_monitor");
        hive_exit();
    }

    // Immediately monitor_cancel
    status = hive_monitor_cancel(ref);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_monitor_cancel");
        hive_exit();
    }

    // Wait a bit - should NOT receive exit notification
    hive_message msg;
    status = hive_ipc_recv(&msg, 700);  // 700ms timeout (target exits at 500ms)

    if (status.code == HIVE_ERR_TIMEOUT) {
        TEST_PASS("monitor_cancel prevents exit notification");
    } else if (HIVE_FAILED(status)) {
        TEST_PASS("monitor_cancel prevents exit notification (no message)");
    } else {
        // Got a message - check if it's an exit notification
        if (hive_is_exit_msg(&msg)) {
            TEST_FAIL("received exit notification after monitor_cancel");
        } else {
            TEST_PASS("monitor_cancel prevents exit notification");
        }
    }

    hive_exit();
}

// ============================================================================
// Test 5: Monitor is unidirectional - target doesn't get notified when monitor dies
// ============================================================================

static bool g_target_received_exit = false;

static void target_waits_for_exit(void *arg) {
    (void)arg;

    // Wait for any message
    hive_message msg;
    hive_status status = hive_ipc_recv(&msg, 500);  // 500ms timeout

    if (HIVE_SUCCEEDED(status) && hive_is_exit_msg(&msg)) {
        g_target_received_exit = true;
    }

    hive_exit();
}

static void monitor_dies_early(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Monitor the target
    uint32_t ref;
    hive_monitor(target, &ref);

    // Die immediately (monitor dies, target should NOT be notified)
    hive_exit();
}

static void test5_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 5: Monitor is unidirectional (target not notified when monitor dies)\n");

    // Spawn target first
    actor_id target;
    if (HIVE_FAILED(hive_spawn(target_waits_for_exit, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    // Spawn monitor that will monitor target then die
    actor_id monitor;
    if (HIVE_FAILED(hive_spawn(monitor_dies_early, &target, &monitor))) {
        TEST_FAIL("spawn monitor");
        hive_exit();
    }

    // Wait for both to finish
    timer_id timer;
    hive_timer_after(700000, &timer);  // 700ms
    hive_message msg;
    hive_ipc_recv(&msg, -1);

    if (!g_target_received_exit) {
        TEST_PASS("target NOT notified when monitor dies (unidirectional)");
    } else {
        TEST_FAIL("target received exit notification (should be unidirectional)");
    }

    hive_exit();
}

// ============================================================================
// Test 6: Monitor invalid/dead actor
// ============================================================================

static void test6_monitor_invalid(void *arg) {
    (void)arg;
    printf("\nTest 6: Monitor invalid actor\n");

    uint32_t ref;

    // Try to monitor invalid actor ID
    hive_status status = hive_monitor(ACTOR_ID_INVALID, &ref);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_monitor rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("hive_monitor should reject ACTOR_ID_INVALID");
    }

    // Try to monitor non-existent actor (high ID that doesn't exist)
    status = hive_monitor(9999, &ref);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_monitor rejects non-existent actor");
    } else {
        TEST_FAIL("hive_monitor should reject non-existent actor");
    }

    hive_exit();
}

// ============================================================================
// Test 7: Demonitor invalid/non-existent ref
// ============================================================================

static void test7_monitor_cancel_invalid(void *arg) {
    (void)arg;
    printf("\nTest 7: Demonitor invalid ref\n");

    // Try to monitor_cancel with invalid ref (0 or very high number)
    hive_status status = hive_monitor_cancel(0);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_monitor_cancel rejects ref 0");
    } else {
        TEST_PASS("hive_monitor_cancel ref 0 is no-op");
    }

    status = hive_monitor_cancel(99999);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_monitor_cancel rejects non-existent ref");
    } else {
        TEST_PASS("hive_monitor_cancel non-existent ref is no-op");
    }

    hive_exit();
}

// ============================================================================
// Test 8: Double monitor_cancel (same ref twice)
// ============================================================================

static void double_monitor_cancel_target(void *arg) {
    (void)arg;
    timer_id timer;
    hive_timer_after(500000, &timer);
    hive_message msg;
    hive_ipc_recv(&msg, -1);
    hive_exit();
}

static void test8_double_monitor_cancel(void *arg) {
    (void)arg;
    printf("\nTest 8: Double monitor_cancel (same ref twice)\n");

    actor_id target;
    if (HIVE_FAILED(hive_spawn(double_monitor_cancel_target, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    uint32_t ref;
    hive_status status = hive_monitor(target, &ref);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_monitor");
        hive_exit();
    }

    // First monitor_cancel should succeed
    status = hive_monitor_cancel(ref);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("first monitor_cancel failed");
        hive_exit();
    }
    TEST_PASS("first monitor_cancel succeeds");

    // Second monitor_cancel should fail or be no-op
    status = hive_monitor_cancel(ref);
    if (HIVE_FAILED(status)) {
        TEST_PASS("second monitor_cancel fails (already monitor_canceled)");
    } else {
        TEST_PASS("second monitor_cancel is no-op");
    }

    // Wait for target to exit
    timer_id timer;
    hive_timer_after(600000, &timer);
    hive_message msg;
    hive_ipc_recv(&msg, -1);

    hive_exit();
}

// ============================================================================
// Test 9: Monitor pool exhaustion (HIVE_MONITOR_ENTRY_POOL_SIZE=128)
// ============================================================================

static void monitor_pool_target(void *arg) {
    (void)arg;
    hive_message msg;
    hive_ipc_recv(&msg, 5000);
    hive_exit();
}

static void test9_monitor_pool_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 9: Monitor pool exhaustion (HIVE_MONITOR_ENTRY_POOL_SIZE=%d)\n",
           HIVE_MONITOR_ENTRY_POOL_SIZE);

    actor_id targets[HIVE_MONITOR_ENTRY_POOL_SIZE + 10];
    uint32_t refs[HIVE_MONITOR_ENTRY_POOL_SIZE + 10];
    int spawned = 0;
    int monitored = 0;

    // Spawn actors and monitor them until pool exhaustion
    for (int i = 0; i < HIVE_MONITOR_ENTRY_POOL_SIZE + 10; i++) {
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = TEST_STACK_SIZE(8 * 1024);

        actor_id target;
        if (HIVE_FAILED(hive_spawn_ex(monitor_pool_target, NULL, &cfg, &target))) {
            break;
        }
        targets[spawned++] = target;

        hive_status status = hive_monitor(target, &refs[monitored]);
        if (HIVE_FAILED(status)) {
            printf("    Monitor failed after %d monitors (pool exhausted)\n", monitored);
            break;
        }
        monitored++;
    }

    if (monitored < HIVE_MONITOR_ENTRY_POOL_SIZE + 10) {
        TEST_PASS("monitor pool exhaustion detected");
    } else {
        printf("    Monitored all %d actors without exhaustion\n", monitored);
        TEST_FAIL("expected monitor pool to exhaust");
    }

    // Signal all actors to exit
    for (int i = 0; i < spawned; i++) {
        int done = 1;
        hive_ipc_notify(targets[i], 0, &done, sizeof(done));
    }

    // Wait for cleanup
    timer_id timer;
    hive_timer_after(200000, &timer);
    hive_message msg;
    hive_ipc_recv(&msg, -1);

    // Drain exit notifications
    while (hive_ipc_pending()) {
        hive_ipc_recv(&msg, 0);
    }

    hive_exit();
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
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

        actor_id test;
        if (HIVE_FAILED(hive_spawn_ex(test_funcs[i], NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        // Link to test actor so we know when it finishes
        hive_link(test);

        // Wait for test to finish
        hive_message msg;
        hive_ipc_recv(&msg, 5000);  // 5 second timeout per test
    }

    hive_exit();
}

int main(void) {
    printf("=== Monitor (hive_monitor) Test Suite ===\n");

    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    actor_id runner;
    if (HIVE_FAILED(hive_spawn_ex(run_all_tests, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        hive_cleanup();
        return 1;
    }

    hive_run();
    hive_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
