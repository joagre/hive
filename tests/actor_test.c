#include "acrt_runtime.h"
#include "acrt_ipc.h"
#include "acrt_timer.h"
#include "acrt_link.h"
#include "acrt_static_config.h"
#include <stdio.h>
#include <string.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); fflush(stdout); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); fflush(stdout); tests_failed++; } while(0)
#define TEST_KNOWN_BUG(name) do { printf("  KNOWN BUG: %s\n", name); fflush(stdout); } while(0)

// ============================================================================
// Test 1: Basic spawn with default config
// ============================================================================

static bool g_basic_spawn_ran = false;

static void basic_actor(void *arg) {
    (void)arg;
    g_basic_spawn_ran = true;
    acrt_exit();
}

static void test1_basic_spawn(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic spawn with default config\n");

    g_basic_spawn_ran = false;

    actor_id id;
    if (ACRT_FAILED(acrt_spawn(basic_actor, NULL, &id))) {
        TEST_FAIL("acrt_spawn returned error");
        acrt_exit();
    }

    acrt_link(id);

    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    if (g_basic_spawn_ran) {
        TEST_PASS("basic spawn works");
    } else {
        TEST_FAIL("spawned actor did not run");
    }

    acrt_exit();
}

// ============================================================================
// Test 2: acrt_self returns correct ID
// ============================================================================

static actor_id g_self_id_from_actor = ACTOR_ID_INVALID;

static void self_reporter_actor(void *arg) {
    (void)arg;
    g_self_id_from_actor = acrt_self();
    acrt_exit();
}

static void test2_rt_self(void *arg) {
    (void)arg;
    printf("\nTest 2: acrt_self returns correct ID\n");

    g_self_id_from_actor = ACTOR_ID_INVALID;

    actor_id spawned_id;
    acrt_spawn(self_reporter_actor, NULL, &spawned_id);
    acrt_link(spawned_id);

    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    if (g_self_id_from_actor == spawned_id) {
        TEST_PASS("acrt_self returns correct actor ID");
    } else {
        printf("    Expected: %u, Got: %u\n", spawned_id, g_self_id_from_actor);
        TEST_FAIL("acrt_self returned wrong ID");
    }

    acrt_exit();
}

// ============================================================================
// Test 3: Argument passing
// ============================================================================

static int g_received_arg = 0;

static void arg_receiver_actor(void *arg) {
    int *value = (int *)arg;
    g_received_arg = *value;
    acrt_exit();
}

static void test3_argument_passing(void *arg) {
    (void)arg;
    printf("\nTest 3: Argument passing\n");

    static int test_value = 12345;
    g_received_arg = 0;

    actor_id id;
    acrt_spawn(arg_receiver_actor, &test_value, &id);
    acrt_link(id);

    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    if (g_received_arg == 12345) {
        TEST_PASS("argument passed correctly to actor");
    } else {
        printf("    Expected: 12345, Got: %d\n", g_received_arg);
        TEST_FAIL("argument not passed correctly");
    }

    acrt_exit();
}

// ============================================================================
// Test 4: acrt_yield allows other actors to run
// ============================================================================

static int g_yield_counter = 0;
static bool g_yielder_done = false;
static bool g_counter_done = false;

static void counter_actor(void *arg) {
    (void)arg;
    for (int i = 0; i < 5; i++) {
        g_yield_counter++;
        acrt_yield();
    }
    g_counter_done = true;
    acrt_exit();
}

static void yielder_actor(void *arg) {
    (void)arg;
    // Yield multiple times to let counter actor run
    for (int i = 0; i < 10; i++) {
        acrt_yield();
    }
    g_yielder_done = true;
    acrt_exit();
}

static void test4_yield(void *arg) {
    (void)arg;
    printf("\nTest 4: acrt_yield allows other actors to run\n");

    g_yield_counter = 0;
    g_yielder_done = false;
    g_counter_done = false;

    actor_id counter;
    acrt_spawn(counter_actor, NULL, &counter);
    actor_id yielder;
    acrt_spawn(yielder_actor, NULL, &yielder);

    acrt_link(counter);
    acrt_link(yielder);

    // Wait for both to complete
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);
    acrt_ipc_recv(&msg, 1000);

    if (g_yield_counter == 5 && g_counter_done && g_yielder_done) {
        TEST_PASS("acrt_yield allows cooperative multitasking");
    } else {
        printf("    counter=%d, counter_done=%d, yielder_done=%d\n",
               g_yield_counter, g_counter_done, g_yielder_done);
        TEST_FAIL("acrt_yield did not work correctly");
    }

    acrt_exit();
}

// ============================================================================
// Test 5: acrt_actor_alive
// ============================================================================

static void short_lived_actor(void *arg) {
    (void)arg;
    acrt_exit();
}

static void test5_actor_alive(void *arg) {
    (void)arg;
    printf("\nTest 5: acrt_actor_alive\n");

    actor_id id;
    acrt_spawn(short_lived_actor, NULL, &id);
    acrt_link(id);

    // Actor should be alive right after spawn (before it gets chance to run)
    bool alive_before = acrt_actor_alive(id);

    // Wait for actor to exit
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    // Actor should be dead after exit
    bool alive_after = acrt_actor_alive(id);

    if (alive_before && !alive_after) {
        TEST_PASS("acrt_actor_alive returns correct status");
    } else {
        printf("    alive_before=%d, alive_after=%d\n", alive_before, alive_after);
        TEST_FAIL("acrt_actor_alive returned wrong status");
    }

    // Invalid actor should return false
    if (!acrt_actor_alive(ACTOR_ID_INVALID)) {
        TEST_PASS("acrt_actor_alive returns false for ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("acrt_actor_alive should return false for ACTOR_ID_INVALID");
    }

    if (!acrt_actor_alive(9999)) {
        TEST_PASS("acrt_actor_alive returns false for non-existent actor");
    } else {
        TEST_FAIL("acrt_actor_alive should return false for non-existent actor");
    }

    acrt_exit();
}

// ============================================================================
// Test 6: Spawn with custom priority
// ============================================================================

static acrt_priority_level g_captured_priority = ACRT_PRIORITY_NORMAL;

static void priority_reporter_actor(void *arg) {
    (void)arg;
    // Can't directly access priority, but we can verify the actor runs
    g_captured_priority = ACRT_PRIORITY_HIGH;  // Indicate we ran with expected priority
    acrt_exit();
}

static void test6_custom_priority(void *arg) {
    (void)arg;
    printf("\nTest 6: Spawn with custom priority\n");

    g_captured_priority = ACRT_PRIORITY_NORMAL;

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.priority = ACRT_PRIORITY_HIGH;

    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(priority_reporter_actor, NULL, &cfg, &id))) {
        TEST_FAIL("acrt_spawn_ex with custom priority failed");
        acrt_exit();
    }

    acrt_link(id);
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with custom priority works");

    acrt_exit();
}

// ============================================================================
// Test 7: Spawn with custom stack size
// ============================================================================

static bool g_large_stack_ok = false;

static void large_stack_actor(void *arg) {
    (void)arg;
    // Allocate a large array on stack
    char buffer[32768];  // 32KB
    memset(buffer, 'A', sizeof(buffer));
    if (buffer[32767] == 'A') {
        g_large_stack_ok = true;
    }
    acrt_exit();
}

static void test7_custom_stack_size(void *arg) {
    (void)arg;
    printf("\nTest 7: Spawn with custom stack size\n");

    g_large_stack_ok = false;

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 64 * 1024;  // 64KB

    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(large_stack_actor, NULL, &cfg, &id))) {
        TEST_FAIL("acrt_spawn_ex with custom stack size failed");
        acrt_exit();
    }

    acrt_link(id);
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    if (g_large_stack_ok) {
        TEST_PASS("custom stack size allows larger stack usage");
    } else {
        TEST_FAIL("large stack actor did not complete");
    }

    acrt_exit();
}

// ============================================================================
// Test 8: Spawn with malloc_stack = true
// ============================================================================

static bool g_malloc_stack_ran = false;

static void malloc_stack_actor(void *arg) {
    (void)arg;
    g_malloc_stack_ran = true;
    acrt_exit();
}

static void test8_malloc_stack(void *arg) {
    (void)arg;
    printf("\nTest 8: Spawn with malloc_stack = true\n");

    g_malloc_stack_ran = false;

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = 32 * 1024;

    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(malloc_stack_actor, NULL, &cfg, &id))) {
        TEST_FAIL("acrt_spawn_ex with malloc_stack failed");
        acrt_exit();
    }

    acrt_link(id);
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    if (g_malloc_stack_ran) {
        TEST_PASS("malloc_stack=true works");
    } else {
        TEST_FAIL("malloc stack actor did not run");
    }

    acrt_exit();
}

// ============================================================================
// Test 9: Spawn with name
// ============================================================================

static void named_actor(void *arg) {
    (void)arg;
    acrt_exit();
}

static void test9_named_actor(void *arg) {
    (void)arg;
    printf("\nTest 9: Spawn with name\n");

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.name = "test_actor_name";

    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(named_actor, NULL, &cfg, &id))) {
        TEST_FAIL("acrt_spawn_ex with name failed");
        acrt_exit();
    }

    acrt_link(id);
    acrt_message msg;
    acrt_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with name works");

    acrt_exit();
}

// ============================================================================
// Test 10: Spawn with NULL function should fail
// ============================================================================

static void test10_spawn_null_fn(void *arg) {
    (void)arg;
    printf("\nTest 10: Spawn with NULL function\n");

    actor_id id;
    if (ACRT_FAILED(acrt_spawn(NULL, NULL, &id))) {
        TEST_PASS("acrt_spawn rejects NULL function");
    } else {
        TEST_FAIL("acrt_spawn should reject NULL function");
    }

    acrt_exit();
}

// ============================================================================
// Test 11: Multiple spawns
// ============================================================================

static int g_multi_spawn_count = 0;

static void counting_actor(void *arg) {
    (void)arg;
    g_multi_spawn_count++;
    acrt_exit();
}

static void test11_multiple_spawns(void *arg) {
    (void)arg;
    printf("\nTest 11: Multiple spawns\n");

    g_multi_spawn_count = 0;

    actor_id ids[10];
    for (int i = 0; i < 10; i++) {
        if (ACRT_FAILED(acrt_spawn(counting_actor, NULL, &ids[i]))) {
            printf("    Failed to spawn actor %d\n", i);
            TEST_FAIL("multiple spawns failed");
            acrt_exit();
        }
        acrt_link(ids[i]);
    }

    // Wait for all to complete
    for (int i = 0; i < 10; i++) {
        acrt_message msg;
        acrt_ipc_recv(&msg, 1000);
    }

    if (g_multi_spawn_count == 10) {
        TEST_PASS("spawned and ran 10 actors");
    } else {
        printf("    Only %d/10 actors ran\n", g_multi_spawn_count);
        TEST_FAIL("not all actors ran");
    }

    acrt_exit();
}

// ============================================================================
// Test 12: Actor returns without calling acrt_exit (crash detection)
// ============================================================================

static void crashing_actor(void *arg) {
    (void)arg;
    // Deliberately return without calling acrt_exit()
    // This should be detected as ACRT_EXIT_CRASH
}

static void test12_actor_crash(void *arg) {
    (void)arg;
    printf("\nTest 12: Actor returns without acrt_exit (crash detection)\n");
    fflush(stdout);

    actor_id crasher;
    if (ACRT_FAILED(acrt_spawn(crashing_actor, NULL, &crasher))) {
        TEST_FAIL("failed to spawn crashing actor");
        acrt_exit();
    }

    acrt_link(crasher);

    // Wait for exit notification
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 1000);
    if (ACRT_FAILED(status)) {
        printf("    recv failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("did not receive exit notification");
        acrt_exit();
    }

    if (!acrt_is_exit_msg(&msg)) {
        TEST_FAIL("received non-exit message");
        acrt_exit();
    }

    acrt_exit_msg exit_msg;
    status = acrt_decode_exit(&msg, &exit_msg);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("failed to decode exit message");
        acrt_exit();
    }

    if (exit_msg.reason == ACRT_EXIT_CRASH) {
        TEST_PASS("crash detected with ACRT_EXIT_CRASH");
    } else {
        printf("    exit reason: %d (expected ACRT_EXIT_CRASH=%d)\n", exit_msg.reason, ACRT_EXIT_CRASH);
        TEST_FAIL("wrong exit reason");
    }

    acrt_exit();
}

// ============================================================================
// Test 13: Actor table exhaustion (ACRT_MAX_ACTORS)
// ============================================================================

static void wait_for_signal_actor(void *arg) {
    (void)arg;
    // Wait for signal from parent to exit
    acrt_message msg;
    acrt_ipc_recv(&msg, 5000);  // Timeout after 5s in case parent dies
    acrt_exit();
}

static void test13_actor_table_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 13: Actor table exhaustion (ACRT_MAX_ACTORS=%d)\n", ACRT_MAX_ACTORS);
    fflush(stdout);

    // Use malloc stacks with small size to avoid arena exhaustion
    // This tests the actual actor table limit, not stack arena
    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = 8 * 1024;  // 8KB stacks

    // We're already using slots for: test runner + this test actor
    // Try to spawn until we hit the limit
    actor_id ids[ACRT_MAX_ACTORS];
    int spawned = 0;

    for (int i = 0; i < ACRT_MAX_ACTORS; i++) {
        actor_id id;
        if (ACRT_FAILED(acrt_spawn_ex(wait_for_signal_actor, NULL, &cfg, &id))) {
            // Exhaustion reached
            break;
        }
        ids[spawned++] = id;
    }

    printf("    Spawned %d actors before exhaustion\n", spawned);
    fflush(stdout);

    // We should have hit actor table exhaustion
    // With ACRT_MAX_ACTORS=64, we should spawn at least 60 (some slots used by test infrastructure)
    if (spawned >= ACRT_MAX_ACTORS - 4) {
        TEST_PASS("actor table exhaustion detected");
    } else {
        printf("    Expected to spawn at least %d actors\n", ACRT_MAX_ACTORS - 4);
        TEST_FAIL("spawned fewer actors than expected");
    }

    // Signal all waiting actors to exit
    int dummy = 1;
    for (int i = 0; i < spawned; i++) {
        acrt_ipc_notify(ids[i], &dummy, sizeof(dummy));
    }

    // Yield a few times to let them exit
    for (int i = 0; i < spawned + 5; i++) {
        acrt_yield();
    }

    acrt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
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
    test12_actor_crash,
    test13_actor_table_exhaustion,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = 64 * 1024;

        actor_id test;
        if (ACRT_FAILED(acrt_spawn_ex(test_funcs[i], NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        acrt_link(test);

        acrt_message msg;
        acrt_ipc_recv(&msg, 5000);
    }

    acrt_exit();
}

int main(void) {
    printf("=== Actor (acrt_spawn/acrt_exit/acrt_yield) Test Suite ===\n");
    fflush(stdout);

    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner;
    if (ACRT_FAILED(acrt_spawn_ex(run_all_tests, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        acrt_cleanup();
        return 1;
    }

    acrt_run();
    acrt_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
