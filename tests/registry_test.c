#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_link.h"
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

// ============================================================================
// Test 1: Basic register and whereis
// ============================================================================

static hive_actor_id_t g_test1_expected_id = HIVE_ACTOR_ID_INVALID;
static bool g_test1_lookup_success = false;

static void test1_register_actor(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_status_t status = hive_register("test_actor");
    if (HIVE_FAILED(status)) {
        return;
    }

    g_test1_expected_id = hive_self();

    // Wait for lookup to complete
    hive_timer_id_t timer;
    hive_timer_after(100000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    return;
}

static void test1_lookup_actor(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Give time for registration
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_actor_id_t found;
    hive_status_t status = hive_whereis("test_actor", &found);
    if (HIVE_SUCCEEDED(status) && found == g_test1_expected_id) {
        g_test1_lookup_success = true;
    }

    return;
}

static void test1_basic_register_whereis(void *args,
                                         const hive_spawn_info_t *siblings,
                                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Basic register and whereis\n");

    g_test1_expected_id = HIVE_ACTOR_ID_INVALID;
    g_test1_lookup_success = false;

    hive_actor_id_t reg_actor, lookup_actor;
    hive_spawn(test1_register_actor, NULL, NULL, NULL, &reg_actor);
    hive_spawn(test1_lookup_actor, NULL, NULL, NULL, &lookup_actor);

    // Wait for completion
    hive_timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test1_lookup_success) {
        TEST_PASS("hive_whereis returns correct actor ID");
    } else {
        TEST_FAIL("hive_whereis did not return expected actor ID");
    }

    return;
}

// ============================================================================
// Test 2: Duplicate name registration fails
// ============================================================================

static bool g_test2_first_registered = false;
static bool g_test2_second_failed = false;

static void test2_first_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_status_t status = hive_register("shared_name");
    if (HIVE_SUCCEEDED(status)) {
        g_test2_first_registered = true;
    }

    // Wait
    hive_timer_id_t timer;
    hive_timer_after(100000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    return;
}

static void test2_second_actor(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Give time for first actor
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_status_t status = hive_register("shared_name");
    if (HIVE_FAILED(status)) {
        g_test2_second_failed = true;
    }

    return;
}

static void test2_duplicate_name(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Duplicate name registration fails\n");

    g_test2_first_registered = false;
    g_test2_second_failed = false;

    hive_actor_id_t first, second;
    hive_spawn(test2_first_actor, NULL, NULL, NULL, &first);
    hive_spawn(test2_second_actor, NULL, NULL, NULL, &second);

    hive_timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test2_first_registered && g_test2_second_failed) {
        TEST_PASS("duplicate name registration rejected");
    } else {
        TEST_FAIL("duplicate name should be rejected");
    }

    return;
}

// ============================================================================
// Test 3: Auto-cleanup on actor exit
// ============================================================================

static bool g_test3_found_before = false;
static bool g_test3_not_found_after = false;

static void test3_registering_actor(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_register("auto_cleanup_name");

    // Wait before exiting to give checker time to verify
    hive_timer_id_t timer;
    hive_timer_after(100000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Exit - name should be auto-cleaned
    return;
}

static void test3_checker_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for registration
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Check name exists
    hive_actor_id_t found;
    if (HIVE_SUCCEEDED(hive_whereis("auto_cleanup_name", &found))) {
        g_test3_found_before = true;
    }

    // Wait for actor to exit (it exits at 100ms from its start)
    hive_timer_after(150000, &timer);
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Check name is gone
    if (HIVE_FAILED(hive_whereis("auto_cleanup_name", &found))) {
        g_test3_not_found_after = true;
    }

    return;
}

static void test3_auto_cleanup(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Auto-cleanup on actor exit\n");

    g_test3_found_before = false;
    g_test3_not_found_after = false;

    hive_actor_id_t reg, checker;
    hive_spawn(test3_registering_actor, NULL, NULL, NULL, &reg);
    hive_spawn(test3_checker_actor, NULL, NULL, NULL, &checker);

    hive_timer_id_t timer;
    hive_timer_after(400000, &timer); // Increased to account for longer checks
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test3_found_before && g_test3_not_found_after) {
        TEST_PASS("name auto-cleaned on actor exit");
    } else {
        printf("    found_before=%d, not_found_after=%d\n",
               g_test3_found_before, g_test3_not_found_after);
        TEST_FAIL("auto-cleanup did not work");
    }

    return;
}

// ============================================================================
// Test 4: Unregister removes name
// ============================================================================

static bool g_test4_found_before = false;
static bool g_test4_not_found_after = false;

static void test4_unregister_actor(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_register("will_unregister");

    // Yield to let checker find it
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_unregister("will_unregister");

    // Wait for checker to verify
    hive_timer_after(100000, &timer);
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    return;
}

static void test4_checker_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for registration
    hive_timer_id_t timer;
    hive_timer_after(25000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_actor_id_t found;
    if (HIVE_SUCCEEDED(hive_whereis("will_unregister", &found))) {
        g_test4_found_before = true;
    }

    // Wait for unregister
    hive_timer_after(75000, &timer);
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (HIVE_FAILED(hive_whereis("will_unregister", &found))) {
        g_test4_not_found_after = true;
    }

    return;
}

static void test4_unregister(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Unregister removes name\n");

    g_test4_found_before = false;
    g_test4_not_found_after = false;

    hive_actor_id_t unreg, checker;
    hive_spawn(test4_unregister_actor, NULL, NULL, NULL, &unreg);
    hive_spawn(test4_checker_actor, NULL, NULL, NULL, &checker);

    hive_timer_id_t timer;
    hive_timer_after(250000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test4_found_before && g_test4_not_found_after) {
        TEST_PASS("hive_unregister removes name");
    } else {
        TEST_FAIL("unregister did not work");
    }

    return;
}

// ============================================================================
// Test 5: Whereis non-existent name fails
// ============================================================================

static void test5_whereis_nonexistent(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Whereis non-existent name fails\n");

    hive_actor_id_t found;
    hive_status_t status = hive_whereis("nonexistent_name", &found);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_whereis returns error for non-existent name");
    } else {
        TEST_FAIL("hive_whereis should fail for non-existent name");
    }

    return;
}

// ============================================================================
// Test 6: NULL arguments rejected
// ============================================================================

static void test6_null_args(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: NULL arguments rejected\n");

    hive_status_t status = hive_register(NULL);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_register rejects NULL name");
    } else {
        TEST_FAIL("hive_register should reject NULL");
    }

    hive_actor_id_t found;
    status = hive_whereis(NULL, &found);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_whereis rejects NULL name");
    } else {
        TEST_FAIL("hive_whereis should reject NULL name");
    }

    status = hive_whereis("test", NULL);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_whereis rejects NULL output");
    } else {
        TEST_FAIL("hive_whereis should reject NULL output");
    }

    status = hive_unregister(NULL);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_unregister rejects NULL name");
    } else {
        TEST_FAIL("hive_unregister should reject NULL");
    }

    return;
}

// ============================================================================
// Test 7: Cannot unregister another actor's name
// ============================================================================

static bool g_test7_unregister_failed = false;

static void test7_owner_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_register("owned_name");

    // Wait for thief attempt
    hive_timer_id_t timer;
    hive_timer_after(150000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    return;
}

static void test7_thief_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for registration
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Try to unregister name we don't own
    hive_status_t status = hive_unregister("owned_name");
    if (HIVE_FAILED(status)) {
        g_test7_unregister_failed = true;
    }

    return;
}

static void test7_cannot_unregister_others(void *args,
                                           const hive_spawn_info_t *siblings,
                                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Cannot unregister another actor's name\n");

    g_test7_unregister_failed = false;

    hive_actor_id_t owner, thief;
    hive_spawn(test7_owner_actor, NULL, NULL, NULL, &owner);
    hive_spawn(test7_thief_actor, NULL, NULL, NULL, &thief);

    hive_timer_id_t timer;
    hive_timer_after(250000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test7_unregister_failed) {
        TEST_PASS("cannot unregister name owned by another actor");
    } else {
        TEST_FAIL("should not be able to unregister another's name");
    }

    return;
}

// ============================================================================
// Test 8: Multiple names per actor
// ============================================================================

static bool g_test8_all_found = false;

static void test8_multi_name_actor(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    hive_register("name_one");
    hive_register("name_two");
    hive_register("name_three");

    // Wait for checker
    hive_timer_id_t timer;
    hive_timer_after(100000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    return;
}

static void test8_checker_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for registrations
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_actor_id_t a1, a2, a3;
    bool found1 = HIVE_SUCCEEDED(hive_whereis("name_one", &a1));
    bool found2 = HIVE_SUCCEEDED(hive_whereis("name_two", &a2));
    bool found3 = HIVE_SUCCEEDED(hive_whereis("name_three", &a3));

    if (found1 && found2 && found3 && a1 == a2 && a2 == a3) {
        g_test8_all_found = true;
    }

    return;
}

static void test8_multiple_names(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Multiple names per actor\n");

    g_test8_all_found = false;

    hive_actor_id_t multi, checker;
    hive_spawn(test8_multi_name_actor, NULL, NULL, NULL, &multi);
    hive_spawn(test8_checker_actor, NULL, NULL, NULL, &checker);

    hive_timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_test8_all_found) {
        TEST_PASS("actor can register multiple names");
    } else {
        TEST_FAIL("multiple names not working");
    }

    return;
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *, const hive_spawn_info_t *, size_t) = {
    test1_basic_register_whereis,
    test2_duplicate_name,
    test3_auto_cleanup,
    test4_unregister,
    test5_whereis_nonexistent,
    test6_null_args,
    test7_cannot_unregister_others,
    test8_multiple_names,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

        hive_actor_id_t test;
        if (HIVE_FAILED(hive_spawn(test_funcs[i], NULL, NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        hive_link(test);

        hive_message_t msg;
        hive_ipc_recv(&msg, 5000);
    }

    return;
}

int main(void) {
    printf("=== Name Registry (hive_register/whereis) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    hive_actor_id_t runner;
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
