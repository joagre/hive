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
static volatile int s_child_started[4] = {0};
static volatile int s_child_exited[4] = {0};
static volatile int s_shutdown_called = 0;

static void reset_test_state(void) {
    for (int i = 0; i < 4; i++) {
        s_child_started[i] = 0;
        s_child_exited[i] = 0;
    }
    s_shutdown_called = 0;
}

// Wait for a condition with timeout
static void wait_ms(int ms) {
    timer_id_t t;
    hive_timer_after((uint64_t)ms * 1000, &t);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, t, &msg, -1);
}

// =============================================================================
// Test Child Actors
// =============================================================================

// Simple child that runs until killed
static void stable_child(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = args ? *(int *)args : 0;
    s_child_started[id]++;

    // Wait indefinitely (will be killed by supervisor)
    hive_message_t msg;
    hive_ipc_recv(&msg, -1);

    s_child_exited[id]++;
    return;
}

// Child that crashes immediately
static void crashing_child(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = args ? *(int *)args : 0;
    s_child_started[id]++;
    s_child_exited[id]++;
    hive_exit(HIVE_EXIT_REASON_CRASH);
}

// Child that exits normally
static void exiting_child(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = args ? *(int *)args : 0;
    s_child_started[id]++;
    s_child_exited[id]++;
    return;
}

// Child that crashes after a short delay
static void delayed_crash_child(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = args ? *(int *)args : 0;
    s_child_started[id]++;

    wait_ms(50);

    s_child_exited[id]++;
    hive_exit(HIVE_EXIT_REASON_CRASH);
}

// Shutdown callback
static void test_shutdown_callback(void *ctx) {
    (void)ctx;
    s_shutdown_called = 1;
}

// =============================================================================
// Test 1: Basic lifecycle (start/stop supervisor)
// =============================================================================

static void test1_basic_lifecycle(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Basic supervisor lifecycle\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec_t children[2] = {
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[0],
         .init_args_size = sizeof(int),
         .name = "child0",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[1],
         .init_args_size = sizeof(int),
         .name = "child1",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.children = children;
    cfg.num_children = 2;
    cfg.on_shutdown = test_shutdown_callback;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Monitor supervisor
    uint32_t monitor_id;
    hive_monitor(supervisor, &monitor_id);

    // Wait for children to start
    wait_ms(100);

    if (s_child_started[0] == 1 && s_child_started[1] == 1) {
        TEST_PASS("children started");
    } else {
        printf("    child0=%d child1=%d\n", s_child_started[0],
               s_child_started[1]);
        TEST_FAIL("children not started correctly");
    }

    // Stop supervisor
    status = hive_supervisor_stop(supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_stop");
        return;
    }

    // Wait for supervisor exit
    hive_message_t msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 1000);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("supervisor did not exit");
        return;
    }

    if (s_shutdown_called) {
        TEST_PASS("shutdown callback called");
    } else {
        TEST_FAIL("shutdown callback not called");
    }

    TEST_PASS("basic lifecycle works");
    return;
}

// =============================================================================
// Test 2: one_for_one - crash one child, only that child restarts
// =============================================================================

static void test2_one_for_one(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: one_for_one strategy\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec_t children[2] = {
        {.start = delayed_crash_child,
         .init = NULL,
         .init_args = &child_ids[0],
         .init_args_size = sizeof(int),
         .name = "crasher",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[1],
         .init_args_size = sizeof(int),
         .name = "stable",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_ONE_FOR_ONE;
    cfg.max_restarts = 5;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 2;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Wait for crasher to crash and restart a couple times
    wait_ms(200);

    // Child 0 should have started multiple times (crashed and restarted)
    // Child 1 should have started exactly once
    if (s_child_started[0] >= 2 && s_child_started[1] == 1) {
        TEST_PASS("one_for_one: only crashed child restarted");
    } else {
        printf("    child0 starts=%d, child1 starts=%d\n", s_child_started[0],
               s_child_started[1]);
        TEST_FAIL("one_for_one: wrong restart behavior");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    return;
}

// =============================================================================
// Test 3: one_for_all - crash one child, all restart
// =============================================================================

static void test3_one_for_all(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: one_for_all strategy\n");
    reset_test_state();

    static int child_ids[2] = {0, 1};
    hive_child_spec_t children[2] = {
        {.start = delayed_crash_child,
         .init = NULL,
         .init_args = &child_ids[0],
         .init_args_size = sizeof(int),
         .name = "crasher",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[1],
         .init_args_size = sizeof(int),
         .name = "stable",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_ONE_FOR_ALL;
    cfg.max_restarts = 2;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 2;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Wait for crasher to crash and all to restart
    wait_ms(150);

    // Both children should have been restarted together
    if (s_child_started[0] >= 2 && s_child_started[1] >= 2) {
        TEST_PASS("one_for_all: all children restarted");
    } else {
        printf("    child0 starts=%d, child1 starts=%d\n", s_child_started[0],
               s_child_started[1]);
        TEST_FAIL("one_for_all: not all children restarted");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    return;
}

// =============================================================================
// Test 4: rest_for_one - crash child N, children N+ restart
// =============================================================================

static void test4_rest_for_one(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: rest_for_one strategy\n");
    reset_test_state();

    static int child_ids[3] = {0, 1, 2};
    hive_child_spec_t children[3] = {
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[0],
         .init_args_size = sizeof(int),
         .name = "stable0",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = delayed_crash_child,
         .init = NULL,
         .init_args = &child_ids[1],
         .init_args_size = sizeof(int),
         .name = "crasher",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = stable_child,
         .init = NULL,
         .init_args = &child_ids[2],
         .init_args_size = sizeof(int),
         .name = "stable2",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[2].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.strategy = HIVE_STRATEGY_REST_FOR_ONE;
    cfg.max_restarts = 2;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 3;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Wait for crasher to crash
    wait_ms(150);

    // Child 0 (before crasher) should start once
    // Child 1 (crasher) should restart multiple times
    // Child 2 (after crasher) should restart when child 1 crashes
    if (s_child_started[0] == 1 && s_child_started[1] >= 2 &&
        s_child_started[2] >= 2) {
        TEST_PASS("rest_for_one: correct restart behavior");
    } else {
        printf("    child0=%d, child1=%d, child2=%d\n", s_child_started[0],
               s_child_started[1], s_child_started[2]);
        TEST_FAIL("rest_for_one: wrong restart behavior");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    return;
}

// =============================================================================
// Test 5: Restart intensity exceeded
// =============================================================================

static void test5_restart_intensity(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Restart intensity exceeded\n");
    reset_test_state();

    static int child_id = 0;
    hive_child_spec_t children[1] = {
        {.start = crashing_child,
         .init = NULL,
         .init_args = &child_id,
         .init_args_size = sizeof(int),
         .name = "rapid_crasher",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.max_restarts = 3;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 1;
    cfg.on_shutdown = test_shutdown_callback;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Monitor supervisor
    uint32_t monitor_id;
    hive_monitor(supervisor, &monitor_id);

    // Wait for supervisor to give up and shut down
    hive_message_t msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 2000);

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("supervisor shut down after intensity exceeded");
    } else {
        TEST_FAIL("supervisor did not shut down");
    }

    if (s_shutdown_called) {
        TEST_PASS("shutdown callback called on intensity exceeded");
    } else {
        TEST_FAIL("shutdown callback not called");
    }

    return;
}

// =============================================================================
// Test 6: Restart types (permanent/transient/temporary)
// =============================================================================

static void test6_restart_types(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Restart types\n");
    reset_test_state();

    static int child_ids[3] = {0, 1, 2};
    hive_child_spec_t children[3] = {
        // Permanent: should restart on normal exit
        {.start = exiting_child,
         .init = NULL,
         .init_args = &child_ids[0],
         .init_args_size = sizeof(int),
         .name = "permanent",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        // Transient: should NOT restart on normal exit
        {.start = exiting_child,
         .init = NULL,
         .init_args = &child_ids[1],
         .init_args_size = sizeof(int),
         .name = "transient",
         .auto_register = false,
         .restart = HIVE_CHILD_TRANSIENT,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        // Temporary: should never restart
        {.start = crashing_child,
         .init = NULL,
         .init_args = &child_ids[2],
         .init_args_size = sizeof(int),
         .name = "temporary",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };
    children[0].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[1].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    children[2].actor_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.max_restarts = 10;
    cfg.restart_period_ms = 5000;
    cfg.children = children;
    cfg.num_children = 3;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start");
        return;
    }

    // Wait for children to run their course
    wait_ms(200);

    // Permanent: should have restarted multiple times (normal exit triggers restart)
    // Transient: should have started once (normal exit, no restart)
    // Temporary: should have started once (never restarts)
    if (s_child_started[0] >= 2) {
        TEST_PASS("permanent child restarts on normal exit");
    } else {
        printf("    permanent starts=%d\n", s_child_started[0]);
        TEST_FAIL("permanent child should restart");
    }

    if (s_child_started[1] == 1) {
        TEST_PASS("transient child not restarted on normal exit");
    } else {
        printf("    transient starts=%d\n", s_child_started[1]);
        TEST_FAIL("transient child should not restart on normal exit");
    }

    if (s_child_started[2] == 1) {
        TEST_PASS("temporary child never restarted");
    } else {
        printf("    temporary starts=%d\n", s_child_started[2]);
        TEST_FAIL("temporary child should never restart");
    }

    hive_supervisor_stop(supervisor);
    wait_ms(100);

    return;
}

// =============================================================================
// Test 7: Empty children
// =============================================================================

static void test7_empty_children(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Empty children list\n");
    reset_test_state();

    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
    cfg.children = NULL;
    cfg.num_children = 0;
    cfg.on_shutdown = test_shutdown_callback;

    hive_actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    actor_id_t supervisor;
    hive_status_t status = hive_supervisor_start(&cfg, &sup_cfg, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_supervisor_start with empty children");
        return;
    }

    TEST_PASS("supervisor starts with empty children");

    // Monitor supervisor
    uint32_t monitor_id;
    hive_monitor(supervisor, &monitor_id);

    // Stop it
    hive_supervisor_stop(supervisor);

    // Wait for exit
    hive_message_t msg;
    status = hive_ipc_recv_match(supervisor, HIVE_MSG_EXIT, HIVE_TAG_ANY, &msg,
                                 1000);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("empty supervisor stops cleanly");
    } else {
        TEST_FAIL("empty supervisor did not stop");
    }

    return;
}

// =============================================================================
// Test 8: Invalid configurations
// =============================================================================

static void test8_invalid_config(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Invalid configurations\n");

    actor_id_t supervisor;

    // NULL config
    hive_status_t status = hive_supervisor_start(NULL, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        TEST_PASS("rejects NULL config");
    } else {
        TEST_FAIL("should reject NULL config");
    }

    // NULL out_supervisor
    hive_supervisor_config_t cfg = HIVE_SUPERVISOR_CONFIG_DEFAULT;
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
    hive_child_spec_t bad_child = {
        .start = NULL,
        .init = NULL,
        .init_args = NULL,
        .init_args_size = 0,
        .name = "bad",
        .auto_register = false,
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

    return;
}

// =============================================================================
// Test 9: Utility functions
// =============================================================================

static void test9_utility_functions(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
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

    return;
}

// =============================================================================
// Test Runner
// =============================================================================

static hive_actor_fn_t test_funcs[] = {
    test1_basic_lifecycle, test2_one_for_one,       test3_one_for_all,
    test4_rest_for_one,    test5_restart_intensity, test6_restart_types,
    test7_empty_children,  test8_invalid_config,    test9_utility_functions,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

        actor_id_t test;
        if (HIVE_FAILED(hive_spawn(test_funcs[i], NULL, NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        hive_link(test);

        hive_message_t msg;
        hive_ipc_recv(&msg, 10000);
    }

    return;
}

int main(void) {
    printf("=== Supervisor (hive_supervisor) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    actor_id_t runner;
    if (HIVE_FAILED(hive_spawn(run_all_tests, NULL, NULL, &cfg, &runner))) {
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
