#include "hive_runtime.h"
#include "hive_supervisor.h"
#include "hive_ipc.h"
#include "hive_link.h"
#include "hive_timer.h"
#include <stdio.h>
#include <string.h>

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

// =============================================================================
// Test Utilities
// =============================================================================

// Shared state for test coordination
static volatile int g_child_started[4] = {0};
static volatile int g_child_exited[4] = {0};
static volatile int g_shutdown_called = 0;

static void reset_test_state(void) {
    for (int i = 0; i < 4; i++) {
        g_child_started[i] = 0;
        g_child_exited[i] = 0;
    }
    g_shutdown_called = 0;
}

// Wait for a condition with timeout
static void wait_ms(int ms) {
    timer_id t;
    hive_timer_after((uint64_t)ms * 1000, &t);
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, t, &msg, -1);
}

// =============================================================================
// Test Child Actors
// =============================================================================

// Simple child that runs until killed
static void stable_child(void *arg) {
    int id = arg ? *(int *)arg : 0;
    g_child_started[id]++;

    // Wait indefinitely (will be killed by supervisor)
    hive_message msg;
    hive_ipc_recv(&msg, -1);

    g_child_exited[id]++;
    hive_exit();
}

// Child that crashes immediately
static void crashing_child(void *arg) {
    int id = arg ? *(int *)arg : 0;
    g_child_started[id]++;
    g_child_exited[id]++;
    // Return without calling hive_exit() = crash
}

// Child that exits normally
static void exiting_child(void *arg) {
    int id = arg ? *(int *)arg : 0;
    g_child_started[id]++;
    g_child_exited[id]++;
    hive_exit();
}

// Child that crashes after a short delay
static void delayed_crash_child(void *arg) {
    int id = arg ? *(int *)arg : 0;
    g_child_started[id]++;

    wait_ms(50);

    g_child_exited[id]++;
    // Crash
}

// Shutdown callback
static void test_shutdown_callback(void *ctx) {
    (void)ctx;
    g_shutdown_called = 1;
}

// =============================================================================
// Test 1: Basic lifecycle (start/stop supervisor)
// =============================================================================

static void test1_basic_lifecycle(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic supervisor lifecycle\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec children[2] = {
        {.id = "child0",
         .fn = stable_child,
         .arg = &child_ids[0],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.id = "child1",
         .fn = stable_child,
         .arg = &child_ids[1],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.children = children;
    cfg.num_children = 2;
    cfg.on_shutdown = test_shutdown_callback;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Monitor supervisor
    uint32_t mon_ref;
    hive_monitor(supervisor, &mon_ref);

    // Wait for children to start
    wait_ms(100);

    if (g_child_started[0] == 1 && g_child_started[1] == 1) {
        TEST_PASS("children started");
    } else {
        printf("    child0=%d child1=%d\n", g_child_started[0],
               g_child_started[1]);
        TEST_FAIL("children not started correctly");
    }

    // Stop supervisor
    status = hive_supervisor_stop(supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_stop");
        hive_exit();
    }

    // Wait for supervisor exit
    hive_message msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 1000);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("supervisor did not exit");
        hive_exit();
    }

    if (g_shutdown_called) {
        TEST_PASS("shutdown callback called");
    } else {
        TEST_FAIL("shutdown callback not called");
    }

    TEST_PASS("basic lifecycle works");
    hive_exit();
}

// =============================================================================
// Test 2: one_for_one - crash one child, only that child restarts
// =============================================================================

static void test2_one_for_one(void *arg) {
    (void)arg;
    printf("\nTest 2: one_for_one strategy\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec children[2] = {
        {.id = "crasher",
         .fn = delayed_crash_child,
         .arg = &child_ids[0],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.id = "stable",
         .fn = stable_child,
         .arg = &child_ids[1],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_ONE_FOR_ONE;
    cfg.max_restarts = 5;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 2;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Wait for crasher to crash and restart a couple times
    wait_ms(200);

    // Child 0 should have started multiple times (crashed and restarted)
    // Child 1 should have started exactly once
    if (g_child_started[0] >= 2 && g_child_started[1] == 1) {
        TEST_PASS("one_for_one: only crashed child restarted");
    } else {
        printf("    child0 starts=%d, child1 starts=%d\n", g_child_started[0],
               g_child_started[1]);
        TEST_FAIL("one_for_one: wrong restart behavior");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    hive_exit();
}

// =============================================================================
// Test 3: one_for_all - crash one child, all restart
// =============================================================================

static void test3_one_for_all(void *arg) {
    (void)arg;
    printf("\nTest 3: one_for_all strategy\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec children[2] = {
        {.id = "crasher",
         .fn = delayed_crash_child,
         .arg = &child_ids[0],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.id = "stable",
         .fn = stable_child,
         .arg = &child_ids[1],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_ONE_FOR_ALL;
    cfg.max_restarts = 2;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 2;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Wait for crasher to crash and all to restart
    wait_ms(150);

    // Both children should have been restarted together
    if (g_child_started[0] >= 2 && g_child_started[1] >= 2) {
        TEST_PASS("one_for_all: all children restarted");
    } else {
        printf("    child0 starts=%d, child1 starts=%d\n", g_child_started[0],
               g_child_started[1]);
        TEST_FAIL("one_for_all: not all children restarted");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    hive_exit();
}

// =============================================================================
// Test 4: rest_for_one - crash child N, children N+ restart
// =============================================================================

static void test4_rest_for_one(void *arg) {
    (void)arg;
    printf("\nTest 4: rest_for_one strategy\n");
    reset_test_state();

    static int child_ids[3] = {0, 1, 2};
    hive_child_spec children[3] = {
        {.id = "stable0",
         .fn = stable_child,
         .arg = &child_ids[0],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.id = "crasher",
         .fn = delayed_crash_child,
         .arg = &child_ids[1],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.id = "stable2",
         .fn = stable_child,
         .arg = &child_ids[2],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[2].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_REST_FOR_ONE;
    cfg.max_restarts = 2;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 3;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Wait for crasher to crash
    wait_ms(150);

    // Child 0 (before crasher) should start once
    // Child 1 (crasher) should restart multiple times
    // Child 2 (after crasher) should restart when child 1 crashes
    if (g_child_started[0] == 1 && g_child_started[1] >= 2 &&
        g_child_started[2] >= 2) {
        TEST_PASS("rest_for_one: correct restart behavior");
    } else {
        printf("    child0=%d, child1=%d, child2=%d\n", g_child_started[0],
               g_child_started[1], g_child_started[2]);
        TEST_FAIL("rest_for_one: wrong restart behavior");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    hive_exit();
}

// =============================================================================
// Test 5: Restart intensity exceeded
// =============================================================================

static void test5_restart_intensity(void *arg) {
    (void)arg;
    printf("\nTest 5: Restart intensity exceeded\n");
    reset_test_state();

    static int child_id = 0;
    hive_child_spec children[1] = {
        {.id = "rapid_crasher",
         .fn = crashing_child,
         .arg = &child_id,
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.max_restarts = 3;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 1;
    cfg.on_shutdown = test_shutdown_callback;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Monitor supervisor
    uint32_t mon_ref;
    hive_monitor(supervisor, &mon_ref);

    // Wait for supervisor to give up and shut down
    hive_message msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 2000);

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("supervisor shut down after intensity exceeded");
    } else {
        TEST_FAIL("supervisor did not shut down");
    }

    if (g_shutdown_called) {
        TEST_PASS("shutdown callback called on intensity exceeded");
    } else {
        TEST_FAIL("shutdown callback not called");
    }

    hive_exit();
}

// =============================================================================
// Test 6: Restart types (permanent/transient/temporary)
// =============================================================================

static void test6_restart_types(void *arg) {
    (void)arg;
    printf("\nTest 6: Restart types\n");
    reset_test_state();

    static int child_ids[3] = {0, 1, 2};
    hive_child_spec children[3] = {
        // Permanent: should restart on normal exit
        {.id = "permanent",
         .fn = exiting_child,
         .arg = &child_ids[0],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        // Transient: should NOT restart on normal exit
        {.id = "transient",
         .fn = exiting_child,
         .arg = &child_ids[1],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_TRANSIENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        // Temporary: should never restart
        {.id = "temporary",
         .fn = crashing_child,
         .arg = &child_ids[2],
         .arg_size = sizeof(int),
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[2].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.max_restarts = 10;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 3;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        hive_exit();
    }

    // Wait for children to run their course
    wait_ms(200);

    // Permanent: should have restarted multiple times (normal exit triggers restart)
    // Transient: should have started once (normal exit, no restart)
    // Temporary: should have started once (never restarts)
    if (g_child_started[0] >= 2) {
        TEST_PASS("permanent child restarts on normal exit");
    } else {
        printf("    permanent starts=%d\n", g_child_started[0]);
        TEST_FAIL("permanent child should restart");
    }

    if (g_child_started[1] == 1) {
        TEST_PASS("transient child not restarted on normal exit");
    } else {
        printf("    transient starts=%d\n", g_child_started[1]);
        TEST_FAIL("transient child should not restart on normal exit");
    }

    if (g_child_started[2] == 1) {
        TEST_PASS("temporary child never restarted");
    } else {
        printf("    temporary starts=%d\n", g_child_started[2]);
        TEST_FAIL("temporary child should never restart");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    hive_exit();
}

// =============================================================================
// Test 7: Empty children
// =============================================================================

static void test7_empty_children(void *arg) {
    (void)arg;
    printf("\nTest 7: Empty children list\n");
    reset_test_state();

    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.children = NULL;
    cfg.num_children = 0;
    cfg.on_shutdown = test_shutdown_callback;

    actor_config sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start with empty children");
        hive_exit();
    }

    TEST_PASS("supervisor starts with empty children");

    // Monitor supervisor
    uint32_t mon_ref;
    hive_monitor(supervisor, &mon_ref);

    // Stop it
    hive_supervisor_stop(supervisor);

    // Wait for exit
    hive_message msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 1000);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("empty supervisor stops cleanly");
    } else {
        TEST_FAIL("empty supervisor did not stop");
    }

    hive_exit();
}

// =============================================================================
// Test 8: Invalid configurations
// =============================================================================

static void test8_invalid_config(void *arg) {
    (void)arg;
    printf("\nTest 8: Invalid configurations\n");

    actor_id supervisor;

    // NULL config
    hive_status status = hive_supervisor_start(NULL, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects NULL config");
    } else {
        TEST_FAIL("should reject NULL config");
    }

    // NULL out_supervisor
    hive_supervisor_config cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    status = hive_supervisor_start(&cfg, NULL, NULL);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects NULL out_supervisor");
    } else {
        TEST_FAIL("should reject NULL out_supervisor");
    }

    // Too many children
    cfg.num_children = HIVE_MAX_SUPERVISOR_CHILDREN + 1;
    status = hive_supervisor_start(&cfg, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects too many children");
    } else {
        TEST_FAIL("should reject too many children");
    }

    // NULL children array with non-zero count
    cfg.num_children = 1;
    cfg.children = NULL;
    status = hive_supervisor_start(&cfg, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects NULL children with non-zero count");
    } else {
        TEST_FAIL("should reject NULL children");
    }

    // NULL child function
    hive_child_spec bad_child = {
        .id = "bad",
        .fn = NULL,
        .restart = HIVE_CHILD_PERMANENT,
        .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
    };
    cfg.children = &bad_child;
    cfg.num_children = 1;
    status = hive_supervisor_start(&cfg, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects NULL child function");
    } else {
        TEST_FAIL("should reject NULL child function");
    }

    hive_exit();
}

// =============================================================================
// Test 9: Utility functions
// =============================================================================

static void test9_utility_functions(void *arg) {
    (void)arg;
    printf("\nTest 9: Utility functions\n");

    // Test strategy string conversion
    if (strcmp(hive_restart_strategy_str(HIVE_STRATEGY_ONE_FOR_ONE),
               "one_for_one") == 0) {
        TEST_PASS("restart_strategy_str one_for_one");
    } else {
        TEST_FAIL("restart_strategy_str one_for_one");
    }

    if (strcmp(hive_restart_strategy_str(HIVE_STRATEGY_ONE_FOR_ALL),
               "one_for_all") == 0) {
        TEST_PASS("restart_strategy_str one_for_all");
    } else {
        TEST_FAIL("restart_strategy_str one_for_all");
    }

    if (strcmp(hive_restart_strategy_str(HIVE_STRATEGY_REST_FOR_ONE),
               "rest_for_one") == 0) {
        TEST_PASS("restart_strategy_str rest_for_one");
    } else {
        TEST_FAIL("restart_strategy_str rest_for_one");
    }

    // Test child restart string conversion
    if (strcmp(hive_child_restart_str(HIVE_CHILD_PERMANENT), "permanent") ==
        0) {
        TEST_PASS("child_restart_str permanent");
    } else {
        TEST_FAIL("child_restart_str permanent");
    }

    if (strcmp(hive_child_restart_str(HIVE_CHILD_TRANSIENT), "transient") ==
        0) {
        TEST_PASS("child_restart_str transient");
    } else {
        TEST_FAIL("child_restart_str transient");
    }

    if (strcmp(hive_child_restart_str(HIVE_CHILD_TEMPORARY), "temporary") ==
        0) {
        TEST_PASS("child_restart_str temporary");
    } else {
        TEST_FAIL("child_restart_str temporary");
    }

    hive_exit();
}

// =============================================================================
// Test Runner
// =============================================================================

static void (*test_funcs[])(void *) = {
    test1_basic_lifecycle, test2_one_for_one,       test3_one_for_all,
    test4_rest_for_one,    test5_restart_intensity, test6_restart_types,
    test7_empty_children,  test8_invalid_config,    test9_utility_functions,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

        actor_id test;
        if (HIVE_FAILED(hive_spawn_ex(test_funcs[i], NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        hive_link(test);

        hive_message msg;
        hive_ipc_recv(&msg, 10000);
    }

    hive_exit();
}

int main(void) {
    printf("=== Supervisor (hive_supervisor) Test Suite ===\n");

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
    printf("\n%s\n",
           tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
