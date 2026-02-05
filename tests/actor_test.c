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
        fflush(stdout);               \
        tests_passed++;               \
    } while (0)
#define TEST_FAIL(name)               \
    do {                              \
        printf("  FAIL: %s\n", name); \
        fflush(stdout);               \
        tests_failed++;               \
    } while (0)
#define TEST_KNOWN_BUG(name)               \
    do {                                   \
        printf("  KNOWN BUG: %s\n", name); \
        fflush(stdout);                    \
    } while (0)

// ============================================================================
// Test 1: Basic spawn with default config
// ============================================================================

static bool g_basic_spawn_ran = false;

static void basic_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    g_basic_spawn_ran = true;
    return;
}

static void test1_basic_spawn(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Basic spawn with default config\n");

    g_basic_spawn_ran = false;

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(basic_actor, NULL, NULL, NULL, &id))) {
        TEST_FAIL("hive_spawn returned error");
        return;
    }

    hive_link(id);

    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (g_basic_spawn_ran) {
        TEST_PASS("basic spawn works");
    } else {
        TEST_FAIL("spawned actor did not run");
    }

    return;
}

// ============================================================================
// Test 2: hive_self returns correct ID
// ============================================================================

static hive_actor_id_t g_self_id_from_actor = HIVE_ACTOR_ID_INVALID;

static void self_reporter_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    g_self_id_from_actor = hive_self();
    return;
}

static void test2_rt_self(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: hive_self returns correct ID\n");

    g_self_id_from_actor = HIVE_ACTOR_ID_INVALID;

    hive_actor_id_t spawned_id;
    hive_spawn(self_reporter_actor, NULL, NULL, NULL, &spawned_id);
    hive_link(spawned_id);

    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (g_self_id_from_actor == spawned_id) {
        TEST_PASS("hive_self returns correct actor ID");
    } else {
        printf("    Expected: %u, Got: %u\n", spawned_id, g_self_id_from_actor);
        TEST_FAIL("hive_self returned wrong ID");
    }

    return;
}

// ============================================================================
// Test 3: Argument passing
// ============================================================================

static int g_received_arg = 0;

static void arg_receiver_actor(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int *value = (int *)args;
    g_received_arg = *value;
    return;
}

static void test3_argument_passing(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Argument passing\n");

    static int test_value = 12345;
    g_received_arg = 0;

    hive_actor_id_t id;
    hive_spawn(arg_receiver_actor, NULL, &test_value, NULL, &id);
    hive_link(id);

    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (g_received_arg == 12345) {
        TEST_PASS("argument passed correctly to actor");
    } else {
        printf("    Expected: 12345, Got: %d\n", g_received_arg);
        TEST_FAIL("argument not passed correctly");
    }

    return;
}

// ============================================================================
// Test 4: hive_yield allows other actors to run
// ============================================================================

static int g_yield_counter = 0;
static bool g_yielder_done = false;
static bool g_counter_done = false;

static void counter_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    for (int i = 0; i < 5; i++) {
        g_yield_counter++;
        hive_yield();
    }
    g_counter_done = true;
    return;
}

static void yielder_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Yield multiple times to let counter actor run
    for (int i = 0; i < 10; i++) {
        hive_yield();
    }
    g_yielder_done = true;
    return;
}

static void test4_yield(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: hive_yield allows other actors to run\n");

    g_yield_counter = 0;
    g_yielder_done = false;
    g_counter_done = false;

    hive_actor_id_t counter;
    hive_spawn(counter_actor, NULL, NULL, NULL, &counter);
    hive_actor_id_t yielder;
    hive_spawn(yielder_actor, NULL, NULL, NULL, &yielder);

    hive_link(counter);
    hive_link(yielder);

    // Wait for both to complete
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);
    hive_ipc_recv(&msg, 1000);

    if (g_yield_counter == 5 && g_counter_done && g_yielder_done) {
        TEST_PASS("hive_yield allows cooperative multitasking");
    } else {
        printf("    counter=%d, counter_done=%d, yielder_done=%d\n",
               g_yield_counter, g_counter_done, g_yielder_done);
        TEST_FAIL("hive_yield did not work correctly");
    }

    return;
}

// ============================================================================
// Test 5: hive_actor_alive
// ============================================================================

static void short_lived_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    return;
}

static void test5_actor_alive(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: hive_actor_alive\n");

    hive_actor_id_t id;
    hive_spawn(short_lived_actor, NULL, NULL, NULL, &id);
    hive_link(id);

    // Actor should be alive right after spawn (before it gets chance to run)
    bool alive_before = hive_actor_alive(id);

    // Wait for actor to exit
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    // Actor should be dead after exit
    bool alive_after = hive_actor_alive(id);

    if (alive_before && !alive_after) {
        TEST_PASS("hive_actor_alive returns correct status");
    } else {
        printf("    alive_before=%d, alive_after=%d\n", alive_before,
               alive_after);
        TEST_FAIL("hive_actor_alive returned wrong status");
    }

    // Invalid actor should return false
    if (!hive_actor_alive(HIVE_ACTOR_ID_INVALID)) {
        TEST_PASS("hive_actor_alive returns false for HIVE_ACTOR_ID_INVALID");
    } else {
        TEST_FAIL(
            "hive_actor_alive should return false for HIVE_ACTOR_ID_INVALID");
    }

    if (!hive_actor_alive(9999)) {
        TEST_PASS("hive_actor_alive returns false for non-existent actor");
    } else {
        TEST_FAIL(
            "hive_actor_alive should return false for non-existent actor");
    }

    return;
}

// ============================================================================
// Test 6: Spawn with custom priority
// ============================================================================

static hive_priority_level_t s_captured_priority = HIVE_PRIORITY_NORMAL;

static void priority_reporter_actor(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Can't directly access priority, but we can verify the actor runs
    s_captured_priority =
        HIVE_PRIORITY_HIGH; // Indicate we ran with expected priority
    return;
}

static void test6_custom_priority(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Spawn with custom priority\n");

    s_captured_priority = HIVE_PRIORITY_NORMAL;

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.priority = HIVE_PRIORITY_HIGH;

    hive_actor_id_t id;
    if (HIVE_FAILED(
            hive_spawn(priority_reporter_actor, NULL, NULL, &cfg, &id))) {
        TEST_FAIL("hive_spawn with custom priority failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with custom priority works");

    return;
}

// ============================================================================
// Test 7: Spawn with custom stack size
// ============================================================================

static bool g_large_stack_ok = false;

/* Buffer size for stack test - reduced for QEMU's limited stack */
#ifdef QEMU_TEST_STACK_SIZE
#define LARGE_STACK_BUFFER_SIZE 1024
#else
#define LARGE_STACK_BUFFER_SIZE 32768
#endif

static void large_stack_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Allocate a large array on stack
    char buffer[LARGE_STACK_BUFFER_SIZE];
    memset(buffer, 'A', sizeof(buffer));
    if (buffer[LARGE_STACK_BUFFER_SIZE - 1] == 'A') {
        g_large_stack_ok = true;
    }
    return;
}

static void test7_custom_stack_size(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Spawn with custom stack size\n");

    g_large_stack_ok = false;

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(large_stack_actor, NULL, NULL, &cfg, &id))) {
        TEST_FAIL("hive_spawn with custom stack size failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (g_large_stack_ok) {
        TEST_PASS("custom stack size allows larger stack usage");
    } else {
        TEST_FAIL("large stack actor did not complete");
    }

    return;
}

// ============================================================================
// Test 8: Spawn with malloc_stack = true
// ============================================================================

static bool g_malloc_stack_ran = false;

static void malloc_stack_actor(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    g_malloc_stack_ran = true;
    return;
}

static void test8_malloc_stack(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Spawn with malloc_stack = true\n");

    g_malloc_stack_ran = false;

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = TEST_STACK_SIZE(32 * 1024);

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(malloc_stack_actor, NULL, NULL, &cfg, &id))) {
        TEST_FAIL("hive_spawn with malloc_stack failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    if (g_malloc_stack_ran) {
        TEST_PASS("malloc_stack=true works");
    } else {
        TEST_FAIL("malloc stack actor did not run");
    }

    return;
}

// ============================================================================
// Test 9: Spawn with name
// ============================================================================

static void named_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    return;
}

static void test9_named_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 9: Spawn with name\n");

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "test_actor_name";

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(named_actor, NULL, NULL, &cfg, &id))) {
        TEST_FAIL("hive_spawn with name failed");
        return;
    }

    hive_link(id);
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with name works");

    return;
}

// ============================================================================
// Test 10: Spawn with NULL function should fail
// ============================================================================

static void test10_spawn_null_fn(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 10: Spawn with NULL function\n");

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(NULL, NULL, NULL, NULL, &id))) {
        TEST_PASS("hive_spawn rejects NULL function");
    } else {
        TEST_FAIL("hive_spawn should reject NULL function");
    }

    return;
}

// ============================================================================
// Test 11: Multiple spawns
// ============================================================================

static int g_multi_spawn_count = 0;

static void counting_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    g_multi_spawn_count++;
    return;
}

/* Number of actors to spawn in test 11 - reduced for QEMU's limited actor table */
#ifdef QEMU_TEST_STACK_SIZE
#define MULTI_SPAWN_COUNT 4
#else
#define MULTI_SPAWN_COUNT 10
#endif

static void test11_multiple_spawns(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 11: Multiple spawns\n");

    g_multi_spawn_count = 0;

    hive_actor_id_t ids[MULTI_SPAWN_COUNT];
    for (int i = 0; i < MULTI_SPAWN_COUNT; i++) {
        if (HIVE_FAILED(
                hive_spawn(counting_actor, NULL, NULL, NULL, &ids[i]))) {
            printf("    Failed to spawn actor %d\n", i);
            TEST_FAIL("multiple spawns failed");
            return;
        }
        hive_link(ids[i]);
    }

    // Wait for all to complete
    for (int i = 0; i < MULTI_SPAWN_COUNT; i++) {
        hive_message_t msg;
        hive_ipc_recv(&msg, 1000);
    }

    if (g_multi_spawn_count == MULTI_SPAWN_COUNT) {
        TEST_PASS("spawned and ran multiple actors");
    } else {
        printf("    Only %d/%d actors ran\n", g_multi_spawn_count,
               MULTI_SPAWN_COUNT);
        TEST_FAIL("not all actors ran");
    }

    return;
}

// ============================================================================
// Test 12: Actor returns without calling hive_exit (normal termination)
// ============================================================================

static void returning_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Return without calling hive_exit()
    // Per Erlang semantics: process with no more code terminates normally
}

static void test12_actor_return(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 12: Actor returns without hive_exit (normal termination)\n");
    fflush(stdout);

    hive_actor_id_t returner;
    if (HIVE_FAILED(hive_spawn(returning_actor, NULL, NULL, NULL, &returner))) {
        TEST_FAIL("failed to spawn returning actor");
        return;
    }

    hive_link(returner);

    // Wait for exit notification
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 1000);
    if (HIVE_FAILED(status)) {
        printf("    recv failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("did not receive exit notification");
        return;
    }

    if (!hive_msg_is_exit(&msg)) {
        TEST_FAIL("received non-exit message");
        return;
    }

    hive_exit_msg_t exit_msg;
    status = hive_decode_exit(&msg, &exit_msg);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to decode exit message");
        return;
    }

    if (exit_msg.reason == HIVE_EXIT_REASON_NORMAL) {
        TEST_PASS("actor return terminates with HIVE_EXIT_REASON_NORMAL");
    } else {
        printf("    exit reason: %u (expected HIVE_EXIT_REASON_NORMAL=%u)\n",
               (unsigned)exit_msg.reason, (unsigned)HIVE_EXIT_REASON_NORMAL);
        TEST_FAIL("wrong exit reason");
    }

    return;
}

// ============================================================================
// Test 13: Actor table exhaustion (HIVE_MAX_ACTORS)
// ============================================================================

static void wait_for_signal_actor(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Wait for signal from parent to exit
    hive_message_t msg;
    hive_ipc_recv(&msg, 5000); // Timeout after 5s in case parent dies
    return;
}

static void test13_actor_table_exhaustion(void *args,
                                          const hive_spawn_info_t *siblings,
                                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 13: Actor table exhaustion (HIVE_MAX_ACTORS=%d)\n",
           HIVE_MAX_ACTORS);
    fflush(stdout);

    // Use malloc stacks with small size to avoid arena exhaustion
    // This tests the actual actor table limit, not stack arena
    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = TEST_STACK_SIZE(8 * 1024);

    // We're already using slots for: test runner + this test actor
    // Try to spawn until we hit the limit
    hive_actor_id_t ids[HIVE_MAX_ACTORS];
    int spawned = 0;

    for (int i = 0; i < HIVE_MAX_ACTORS; i++) {
        hive_actor_id_t id;
        if (HIVE_FAILED(
                hive_spawn(wait_for_signal_actor, NULL, NULL, &cfg, &id))) {
            // Exhaustion reached
            break;
        }
        ids[spawned++] = id;
    }

    printf("    Spawned %d actors before exhaustion\n", spawned);
    fflush(stdout);

    // We should have hit actor table exhaustion
    // With HIVE_MAX_ACTORS=64, we should spawn at least 60 (some slots used by test infrastructure)
    if (spawned >= HIVE_MAX_ACTORS - 4) {
        TEST_PASS("actor table exhaustion detected");
    } else {
        printf("    Expected to spawn at least %d actors\n",
               HIVE_MAX_ACTORS - 4);
        TEST_FAIL("spawned fewer actors than expected");
    }

    // Signal all waiting actors to exit
    int dummy = 1;
    for (int i = 0; i < spawned; i++) {
        hive_ipc_notify(ids[i], HIVE_ID_NONE, &dummy, sizeof(dummy));
    }

    // Yield a few times to let them exit
    for (int i = 0; i < spawned + 5; i++) {
        hive_yield();
    }

    return;
}

// ============================================================================
// Test runner
// ============================================================================

static hive_actor_fn_t test_funcs[] = {
    test1_basic_spawn,
    test2_rt_self,
    test3_argument_passing,
    test4_yield,
    test5_actor_alive,
    test6_custom_priority,
    test7_custom_stack_size,
    test8_malloc_stack,
    test9_named_actor,
    test10_spawn_null_fn,
    test11_multiple_spawns,
    test12_actor_return,
    test13_actor_table_exhaustion,
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
    printf("=== Actor (hive_spawn/hive_exit/hive_yield) Test Suite ===\n");
    fflush(stdout);

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
