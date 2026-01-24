// Tests for spawn init function and auto-register features

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_link.h"
#include <stdio.h>
#include <string.h>

#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name)               \
    do {                              \
        printf("  PASS: %s\n", name); \
        fflush(stdout);               \
        tests_passed++;               \
    } while (0)
#define TEST_FAIL(name)               \
    do {                              \
        printf("  FAIL: %s\n", name); \
        fflush(stdout);               \
        tests_failed++;               \
    } while (0)

// ============================================================================
// Test 1: Init function transforms arguments
// ============================================================================

typedef struct {
    int input_value;
} init_input;

typedef struct {
    int transformed_value;
} init_output;

static init_output s_init_output;
static int s_received_value = 0;

static void *transform_init(void *init_args) {
    init_input *in = (init_input *)init_args;
    s_init_output.transformed_value = in->input_value * 2;
    return &s_init_output;
}

static void init_receiver_actor(void *args, const hive_spawn_info *siblings,
                                size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    init_output *out = (init_output *)args;
    s_received_value = out->transformed_value;
    hive_exit();
}

static void test1_init_transforms(void *args, const hive_spawn_info *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Init function transforms arguments\n");

    init_input input = {.input_value = 21};
    s_received_value = 0;

    actor_id id;
    hive_status s =
        hive_spawn(init_receiver_actor, transform_init, &input, NULL, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn with init failed");
        hive_exit();
    }

    hive_link(id);
    hive_message msg;
    hive_ipc_recv(&msg, 1000);

    if (s_received_value == 42) {
        TEST_PASS("init function transformed 21 to 42");
    } else {
        printf("    Expected 42, got %d\n", s_received_value);
        TEST_FAIL("init function did not transform correctly");
    }

    hive_exit();
}

// ============================================================================
// Test 2: Init returns NULL (valid case)
// ============================================================================

static bool s_null_args_received = false;

static void *null_init(void *init_args) {
    (void)init_args;
    return NULL;
}

static void null_args_actor(void *args, const hive_spawn_info *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    s_null_args_received = (args == NULL);
    hive_exit();
}

static void test2_init_returns_null(void *args, const hive_spawn_info *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Init returning NULL is valid\n");

    s_null_args_received = false;
    int dummy = 123;

    actor_id id;
    hive_status s = hive_spawn(null_args_actor, null_init, &dummy, NULL, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn with null-returning init failed");
        hive_exit();
    }

    hive_link(id);
    hive_message msg;
    hive_ipc_recv(&msg, 1000);

    if (s_null_args_received) {
        TEST_PASS("actor received NULL args from init");
    } else {
        TEST_FAIL("actor did not receive NULL args");
    }

    hive_exit();
}

// ============================================================================
// Test 3: Auto-register with name
// ============================================================================

static void registered_actor(void *args, const hive_spawn_info *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for parent to check registration
    hive_message msg;
    hive_ipc_recv(&msg, 1000);
    hive_exit();
}

static void test3_auto_register(void *args, const hive_spawn_info *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Auto-register with name\n");

    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "test_registered";
    cfg.auto_register = true;

    actor_id id;
    hive_status s = hive_spawn(registered_actor, NULL, NULL, &cfg, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn with auto_register failed");
        hive_exit();
    }

    // Check that we can find it by name
    actor_id found;
    s = hive_whereis("test_registered", &found);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("hive_whereis failed to find registered actor");
        hive_ipc_notify(id, HIVE_TAG_NONE, NULL, 0);
        hive_exit();
    }

    if (found == id) {
        TEST_PASS("auto_register works - actor found by name");
    } else {
        TEST_FAIL("found actor ID doesn't match spawned ID");
    }

    hive_ipc_notify(id, HIVE_TAG_NONE, NULL, 0);
    hive_sleep(100000);
    hive_exit();
}

// ============================================================================
// Test 4: Auto-register fails if name taken
// ============================================================================

static void placeholder_actor(void *args, const hive_spawn_info *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_message msg;
    hive_ipc_recv(&msg, 2000);
    hive_exit();
}

static void test4_register_conflict(void *args, const hive_spawn_info *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Auto-register fails if name taken\n");

    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "conflict_test";
    cfg.auto_register = true;

    // Spawn first actor with name
    actor_id id1;
    hive_status s = hive_spawn(placeholder_actor, NULL, NULL, &cfg, &id1);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("first spawn failed");
        hive_exit();
    }

    // Try to spawn second actor with same name
    actor_id id2;
    s = hive_spawn(placeholder_actor, NULL, NULL, &cfg, &id2);
    if (s.code == HIVE_ERR_EXISTS) {
        TEST_PASS("second spawn correctly failed with HIVE_ERR_EXISTS");
    } else if (HIVE_SUCCEEDED(s)) {
        TEST_FAIL("second spawn should have failed but succeeded");
        hive_kill(id2);
    } else {
        printf("    Got error code %d instead of HIVE_ERR_EXISTS\n", s.code);
        TEST_FAIL("second spawn failed with wrong error");
    }

    hive_ipc_notify(id1, HIVE_TAG_NONE, NULL, 0);
    hive_sleep(100000);
    hive_exit();
}

// ============================================================================
// Test 5: No init, direct args passthrough
// ============================================================================

static int s_direct_value = 0;

static void direct_args_actor(void *args, const hive_spawn_info *siblings,
                              size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int *val = (int *)args;
    s_direct_value = *val;
    hive_exit();
}

static void test5_no_init(void *args, const hive_spawn_info *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: No init - direct args passthrough\n");

    static int value = 99;
    s_direct_value = 0;

    actor_id id;
    hive_status s = hive_spawn(direct_args_actor, NULL, &value, NULL, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn without init failed");
        hive_exit();
    }

    hive_link(id);
    hive_message msg;
    hive_ipc_recv(&msg, 1000);

    if (s_direct_value == 99) {
        TEST_PASS("args passed directly without init");
    } else {
        printf("    Expected 99, got %d\n", s_direct_value);
        TEST_FAIL("args not passed correctly");
    }

    hive_exit();
}

// ============================================================================
// Main test runner
// ============================================================================

typedef void (*test_func_t)(void *, const hive_spawn_info *, size_t);

static test_func_t test_funcs[] = {
    test1_init_transforms,   test2_init_returns_null, test3_auto_register,
    test4_register_conflict, test5_no_init,
};

static size_t current_test = 0;

static void run_next_test(void *args, const hive_spawn_info *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    if (current_test < sizeof(test_funcs) / sizeof(test_funcs[0])) {
        actor_id id;
        hive_spawn(test_funcs[current_test], NULL, NULL, NULL, &id);
        hive_link(id);
        current_test++;

        hive_message msg;
        hive_ipc_recv(&msg, 5000);

        // Run next test
        hive_spawn(run_next_test, NULL, NULL, NULL, &id);
    }

    hive_exit();
}

int main(void) {
    printf("=== Spawn Init Tests ===\n");

    hive_init();

    actor_id runner;
    hive_spawn(run_next_test, NULL, NULL, NULL, &runner);

    hive_run();
    hive_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);

    if (tests_failed == 0) {
        printf("\nAll tests passed!\n");
    }

    return tests_failed > 0 ? 1 : 0;
}
