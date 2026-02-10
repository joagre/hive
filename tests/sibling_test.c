// Tests for sibling info array passed to actors

#include "hive_runtime.h"
#include "hive_supervisor.h"
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
// Test 1: Standalone spawn gets sibling_count = 1
// ============================================================================

static size_t s_standalone_sibling_count = 0;
static bool s_standalone_self_in_siblings = false;

static void standalone_actor(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    s_standalone_sibling_count = sibling_count;

    hive_actor_id_t self = hive_self();
    for (size_t i = 0; i < sibling_count; i++) {
        if (siblings[i].id == self) {
            s_standalone_self_in_siblings = true;
            break;
        }
    }

    return;
}

static void test1_standalone_siblings(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Standalone spawn sibling array\n");

    s_standalone_sibling_count = 0;
    s_standalone_self_in_siblings = false;

    hive_actor_id_t id;
    hive_status_t s = hive_spawn(standalone_actor, NULL, NULL, NULL, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (s_standalone_sibling_count == 1) {
        TEST_PASS("standalone actor gets sibling_count = 1");
    } else {
        printf("    Expected 1, got %zu\n", s_standalone_sibling_count);
        TEST_FAIL("wrong sibling count for standalone actor");
    }

    if (s_standalone_self_in_siblings) {
        TEST_PASS("standalone actor finds itself in siblings");
    } else {
        TEST_FAIL("standalone actor not in its own sibling array");
    }

    return;
}

// ============================================================================
// Test 2: Supervised children see all siblings
// ============================================================================

#define NUM_CHILDREN 3

static size_t s_child_sibling_counts[NUM_CHILDREN] = {0};
static bool s_child_saw_all_siblings[NUM_CHILDREN] = {false};
static hive_actor_id_t s_child_ids[NUM_CHILDREN] = {0};

static void child_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    int index = *(int *)args;
    s_child_sibling_counts[index] = sibling_count;

    // Check if we can see all siblings
    int found = 0;
    for (size_t i = 0; i < sibling_count; i++) {
        if (siblings[i].name != NULL) {
            if (strcmp(siblings[i].name, "child0") == 0 ||
                strcmp(siblings[i].name, "child1") == 0 ||
                strcmp(siblings[i].name, "child2") == 0) {
                found++;
            }
        }
    }
    s_child_saw_all_siblings[index] = (found == NUM_CHILDREN);
    s_child_ids[index] = hive_self();

    // Wait for shutdown
    hive_message_t msg;
    hive_ipc_recv(&msg, 5000);
    return;
}

static void *child_init(void *init_args) {
    // Just pass through the index
    return init_args;
}

static void test2_supervisor_siblings(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Supervised children see all siblings\n");

    // Reset state
    for (int i = 0; i < NUM_CHILDREN; i++) {
        s_child_sibling_counts[i] = 0;
        s_child_saw_all_siblings[i] = false;
        s_child_ids[i] = HIVE_ACTOR_ID_INVALID;
    }

    static int indices[NUM_CHILDREN] = {0, 1, 2};

    hive_child_spec_t children[] = {
        {.start = child_actor,
         .init = child_init,
         .init_args = &indices[0],
         .init_args_size = sizeof(int),
         .name = "child0",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = child_actor,
         .init = child_init,
         .init_args = &indices[1],
         .init_args_size = sizeof(int),
         .name = "child1",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = child_actor,
         .init = child_init,
         .init_args = &indices[2],
         .init_args_size = sizeof(int),
         .name = "child2",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };

    hive_supervisor_config_t cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ONE,
        .max_restarts = 0,
        .restart_period_ms = 1000,
        .children = children,
        .num_children = NUM_CHILDREN,
        .on_shutdown = NULL,
        .shutdown_ctx = NULL,
    };

    hive_actor_id_t sup_id;
    hive_status_t s = hive_supervisor_start(&cfg, NULL, &sup_id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("supervisor start failed");
        return;
    }

    // Give children time to start and record sibling info
    hive_sleep(200000);

    // Check results
    bool all_got_correct_count = true;
    bool all_saw_siblings = true;

    for (int i = 0; i < NUM_CHILDREN; i++) {
        if (s_child_sibling_counts[i] != NUM_CHILDREN) {
            all_got_correct_count = false;
            printf("    child%d got sibling_count=%zu, expected %d\n", i,
                   s_child_sibling_counts[i], NUM_CHILDREN);
        }
        if (!s_child_saw_all_siblings[i]) {
            all_saw_siblings = false;
            printf("    child%d did not see all siblings\n", i);
        }
    }

    if (all_got_correct_count) {
        TEST_PASS("all children got sibling_count = 3");
    } else {
        TEST_FAIL("children got wrong sibling counts");
    }

    if (all_saw_siblings) {
        TEST_PASS("all children saw all siblings by name");
    } else {
        TEST_FAIL("some children didn't see all siblings");
    }

    hive_supervisor_stop(sup_id);
    hive_sleep(100000);

    return;
}

// ============================================================================
// Test 3: hive_find_sibling helper function
// ============================================================================

static hive_actor_id_t s_found_sibling_id = HIVE_ACTOR_ID_INVALID;

static void finder_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;

    s_found_sibling_id = hive_find_sibling(siblings, sibling_count, "target");

    hive_message_t msg;
    hive_ipc_recv(&msg, 5000);
    return;
}

static void target_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_message_t msg;
    hive_ipc_recv(&msg, 5000);
    return;
}

static void test3_find_sibling(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: hive_find_sibling helper function\n");

    s_found_sibling_id = HIVE_ACTOR_ID_INVALID;

    hive_child_spec_t children[] = {
        {.start = finder_actor,
         .init = NULL,
         .init_args = NULL,
         .init_args_size = 0,
         .name = "finder",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
        {.start = target_actor,
         .init = NULL,
         .init_args = NULL,
         .init_args_size = 0,
         .name = "target",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT},
    };

    hive_supervisor_config_t cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ONE,
        .max_restarts = 0,
        .restart_period_ms = 1000,
        .children = children,
        .num_children = 2,
        .on_shutdown = NULL,
        .shutdown_ctx = NULL,
    };

    hive_actor_id_t sup_id;
    hive_status_t s = hive_supervisor_start(&cfg, NULL, &sup_id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("supervisor start failed");
        return;
    }

    hive_sleep(200000);

    if (s_found_sibling_id != HIVE_ACTOR_ID_INVALID) {
        TEST_PASS("hive_find_sibling found target by name");
    } else {
        TEST_FAIL("hive_find_sibling did not find target");
    }

    hive_supervisor_stop(sup_id);
    hive_sleep(100000);

    return;
}

// ============================================================================
// Test 4: hive_find_sibling returns HIVE_ACTOR_ID_INVALID for unknown name
// ============================================================================

static bool s_not_found_returned_null = false;

static void not_finder_actor(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;

    hive_actor_id_t found =
        hive_find_sibling(siblings, sibling_count, "nonexistent");
    s_not_found_returned_null = (found == HIVE_ACTOR_ID_INVALID);

    return;
}

static void test4_find_sibling_not_found(void *args,
                                         const hive_spawn_info_t *siblings,
                                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: hive_find_sibling returns INVALID for unknown name\n");

    s_not_found_returned_null = false;

    hive_actor_id_t id;
    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "searcher";

    hive_status_t s = hive_spawn(not_finder_actor, NULL, NULL, &cfg, &id);
    if (HIVE_FAILED(s)) {
        TEST_FAIL("spawn failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (s_not_found_returned_null) {
        TEST_PASS("hive_find_sibling returns INVALID for unknown name");
    } else {
        TEST_FAIL("hive_find_sibling did not return INVALID");
    }

    return;
}

// ============================================================================
// Main test runner
// ============================================================================

typedef void (*test_func_t)(void *, const hive_spawn_info_t *, size_t);

static test_func_t test_funcs[] = {
    test1_standalone_siblings,
    test2_supervisor_siblings,
    test3_find_sibling,
    test4_find_sibling_not_found,
};

static size_t current_test = 0;

static void run_next_test(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    if (current_test < sizeof(test_funcs) / sizeof(test_funcs[0])) {
        hive_actor_id_t id;
        hive_spawn(test_funcs[current_test], NULL, NULL, NULL, &id);
        hive_link(id);
        current_test++;

        hive_message_t msg;
        hive_ipc_recv(&msg, 10000);

        // Run next test
        hive_spawn(run_next_test, NULL, NULL, NULL, &id);
    }

    return;
}

int main(void) {
    printf("=== Sibling Info Tests ===\n");

    hive_init();

    hive_actor_id_t runner;
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
