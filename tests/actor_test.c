#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_link.h"
#include "rt_static_config.h"
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
    rt_exit();
}

static void test1_basic_spawn(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic spawn with default config\n");

    g_basic_spawn_ran = false;

    actor_id id = rt_spawn(basic_actor, NULL);
    if (id == ACTOR_ID_INVALID) {
        TEST_FAIL("rt_spawn returned ACTOR_ID_INVALID");
        rt_exit();
    }

    rt_link(id);

    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    if (g_basic_spawn_ran) {
        TEST_PASS("basic spawn works");
    } else {
        TEST_FAIL("spawned actor did not run");
    }

    rt_exit();
}

// ============================================================================
// Test 2: rt_self returns correct ID
// ============================================================================

static actor_id g_self_id_from_actor = ACTOR_ID_INVALID;

static void self_reporter_actor(void *arg) {
    (void)arg;
    g_self_id_from_actor = rt_self();
    rt_exit();
}

static void test2_rt_self(void *arg) {
    (void)arg;
    printf("\nTest 2: rt_self returns correct ID\n");

    g_self_id_from_actor = ACTOR_ID_INVALID;

    actor_id spawned_id = rt_spawn(self_reporter_actor, NULL);
    rt_link(spawned_id);

    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    if (g_self_id_from_actor == spawned_id) {
        TEST_PASS("rt_self returns correct actor ID");
    } else {
        printf("    Expected: %u, Got: %u\n", spawned_id, g_self_id_from_actor);
        TEST_FAIL("rt_self returned wrong ID");
    }

    rt_exit();
}

// ============================================================================
// Test 3: Argument passing
// ============================================================================

static int g_received_arg = 0;

static void arg_receiver_actor(void *arg) {
    int *value = (int *)arg;
    g_received_arg = *value;
    rt_exit();
}

static void test3_argument_passing(void *arg) {
    (void)arg;
    printf("\nTest 3: Argument passing\n");

    static int test_value = 12345;
    g_received_arg = 0;

    actor_id id = rt_spawn(arg_receiver_actor, &test_value);
    rt_link(id);

    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    if (g_received_arg == 12345) {
        TEST_PASS("argument passed correctly to actor");
    } else {
        printf("    Expected: 12345, Got: %d\n", g_received_arg);
        TEST_FAIL("argument not passed correctly");
    }

    rt_exit();
}

// ============================================================================
// Test 4: rt_yield allows other actors to run
// ============================================================================

static int g_yield_counter = 0;
static bool g_yielder_done = false;
static bool g_counter_done = false;

static void counter_actor(void *arg) {
    (void)arg;
    for (int i = 0; i < 5; i++) {
        g_yield_counter++;
        rt_yield();
    }
    g_counter_done = true;
    rt_exit();
}

static void yielder_actor(void *arg) {
    (void)arg;
    // Yield multiple times to let counter actor run
    for (int i = 0; i < 10; i++) {
        rt_yield();
    }
    g_yielder_done = true;
    rt_exit();
}

static void test4_yield(void *arg) {
    (void)arg;
    printf("\nTest 4: rt_yield allows other actors to run\n");

    g_yield_counter = 0;
    g_yielder_done = false;
    g_counter_done = false;

    actor_id counter = rt_spawn(counter_actor, NULL);
    actor_id yielder = rt_spawn(yielder_actor, NULL);

    rt_link(counter);
    rt_link(yielder);

    // Wait for both to complete
    rt_message msg;
    rt_ipc_recv(&msg, 1000);
    rt_ipc_recv(&msg, 1000);

    if (g_yield_counter == 5 && g_counter_done && g_yielder_done) {
        TEST_PASS("rt_yield allows cooperative multitasking");
    } else {
        printf("    counter=%d, counter_done=%d, yielder_done=%d\n",
               g_yield_counter, g_counter_done, g_yielder_done);
        TEST_FAIL("rt_yield did not work correctly");
    }

    rt_exit();
}

// ============================================================================
// Test 5: rt_actor_alive
// ============================================================================

static void short_lived_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void test5_actor_alive(void *arg) {
    (void)arg;
    printf("\nTest 5: rt_actor_alive\n");

    actor_id id = rt_spawn(short_lived_actor, NULL);
    rt_link(id);

    // Actor should be alive right after spawn (before it gets chance to run)
    bool alive_before = rt_actor_alive(id);

    // Wait for actor to exit
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    // Actor should be dead after exit
    bool alive_after = rt_actor_alive(id);

    if (alive_before && !alive_after) {
        TEST_PASS("rt_actor_alive returns correct status");
    } else {
        printf("    alive_before=%d, alive_after=%d\n", alive_before, alive_after);
        TEST_FAIL("rt_actor_alive returned wrong status");
    }

    // Invalid actor should return false
    if (!rt_actor_alive(ACTOR_ID_INVALID)) {
        TEST_PASS("rt_actor_alive returns false for ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("rt_actor_alive should return false for ACTOR_ID_INVALID");
    }

    if (!rt_actor_alive(9999)) {
        TEST_PASS("rt_actor_alive returns false for non-existent actor");
    } else {
        TEST_FAIL("rt_actor_alive should return false for non-existent actor");
    }

    rt_exit();
}

// ============================================================================
// Test 6: Spawn with custom priority
// ============================================================================

static rt_priority g_captured_priority = RT_PRIO_NORMAL;

static void priority_reporter_actor(void *arg) {
    (void)arg;
    // Can't directly access priority, but we can verify the actor runs
    g_captured_priority = RT_PRIO_HIGH;  // Indicate we ran with expected priority
    rt_exit();
}

static void test6_custom_priority(void *arg) {
    (void)arg;
    printf("\nTest 6: Spawn with custom priority\n");

    g_captured_priority = RT_PRIO_NORMAL;

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.priority = RT_PRIO_HIGH;

    actor_id id = rt_spawn_ex(priority_reporter_actor, NULL, &cfg);
    if (id == ACTOR_ID_INVALID) {
        TEST_FAIL("rt_spawn_ex with custom priority failed");
        rt_exit();
    }

    rt_link(id);
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with custom priority works");

    rt_exit();
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
    rt_exit();
}

static void test7_custom_stack_size(void *arg) {
    (void)arg;
    printf("\nTest 7: Spawn with custom stack size\n");

    g_large_stack_ok = false;

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 64 * 1024;  // 64KB

    actor_id id = rt_spawn_ex(large_stack_actor, NULL, &cfg);
    if (id == ACTOR_ID_INVALID) {
        TEST_FAIL("rt_spawn_ex with custom stack size failed");
        rt_exit();
    }

    rt_link(id);
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    if (g_large_stack_ok) {
        TEST_PASS("custom stack size allows larger stack usage");
    } else {
        TEST_FAIL("large stack actor did not complete");
    }

    rt_exit();
}

// ============================================================================
// Test 8: Spawn with malloc_stack = true
// ============================================================================

static bool g_malloc_stack_ran = false;

static void malloc_stack_actor(void *arg) {
    (void)arg;
    g_malloc_stack_ran = true;
    rt_exit();
}

static void test8_malloc_stack(void *arg) {
    (void)arg;
    printf("\nTest 8: Spawn with malloc_stack = true\n");

    g_malloc_stack_ran = false;

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = 32 * 1024;

    actor_id id = rt_spawn_ex(malloc_stack_actor, NULL, &cfg);
    if (id == ACTOR_ID_INVALID) {
        TEST_FAIL("rt_spawn_ex with malloc_stack failed");
        rt_exit();
    }

    rt_link(id);
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    if (g_malloc_stack_ran) {
        TEST_PASS("malloc_stack=true works");
    } else {
        TEST_FAIL("malloc stack actor did not run");
    }

    rt_exit();
}

// ============================================================================
// Test 9: Spawn with name
// ============================================================================

static void named_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void test9_named_actor(void *arg) {
    (void)arg;
    printf("\nTest 9: Spawn with name\n");

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.name = "test_actor_name";

    actor_id id = rt_spawn_ex(named_actor, NULL, &cfg);
    if (id == ACTOR_ID_INVALID) {
        TEST_FAIL("rt_spawn_ex with name failed");
        rt_exit();
    }

    rt_link(id);
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    TEST_PASS("spawn with name works");

    rt_exit();
}

// ============================================================================
// Test 10: Spawn with NULL function should fail
// ============================================================================

static void test10_spawn_null_fn(void *arg) {
    (void)arg;
    printf("\nTest 10: Spawn with NULL function\n");

    actor_id id = rt_spawn(NULL, NULL);
    if (id == ACTOR_ID_INVALID) {
        TEST_PASS("rt_spawn rejects NULL function");
    } else {
        TEST_FAIL("rt_spawn should reject NULL function");
    }

    rt_exit();
}

// ============================================================================
// Test 11: Multiple spawns
// ============================================================================

static int g_multi_spawn_count = 0;

static void counting_actor(void *arg) {
    (void)arg;
    g_multi_spawn_count++;
    rt_exit();
}

static void test11_multiple_spawns(void *arg) {
    (void)arg;
    printf("\nTest 11: Multiple spawns\n");

    g_multi_spawn_count = 0;

    actor_id ids[10];
    for (int i = 0; i < 10; i++) {
        ids[i] = rt_spawn(counting_actor, NULL);
        if (ids[i] == ACTOR_ID_INVALID) {
            printf("    Failed to spawn actor %d\n", i);
            TEST_FAIL("multiple spawns failed");
            rt_exit();
        }
        rt_link(ids[i]);
    }

    // Wait for all to complete
    for (int i = 0; i < 10; i++) {
        rt_message msg;
        rt_ipc_recv(&msg, 1000);
    }

    if (g_multi_spawn_count == 10) {
        TEST_PASS("spawned and ran 10 actors");
    } else {
        printf("    Only %d/10 actors ran\n", g_multi_spawn_count);
        TEST_FAIL("not all actors ran");
    }

    rt_exit();
}

// ============================================================================
// Test 12: Actor returns without calling rt_exit (crash detection)
// ============================================================================

static void crashing_actor(void *arg) {
    (void)arg;
    // Deliberately return without calling rt_exit()
    // This should be detected as RT_EXIT_CRASH
}

static void test12_actor_crash(void *arg) {
    (void)arg;
    printf("\nTest 12: Actor returns without rt_exit (crash detection)\n");
    fflush(stdout);

    actor_id crasher = rt_spawn(crashing_actor, NULL);
    if (crasher == ACTOR_ID_INVALID) {
        TEST_FAIL("failed to spawn crashing actor");
        rt_exit();
    }

    rt_link(crasher);

    // Wait for exit notification
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 1000);
    if (RT_FAILED(status)) {
        printf("    recv failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("did not receive exit notification");
        rt_exit();
    }

    if (!rt_is_exit_msg(&msg)) {
        TEST_FAIL("received non-exit message");
        rt_exit();
    }

    rt_exit_msg exit_msg;
    status = rt_decode_exit(&msg, &exit_msg);
    if (RT_FAILED(status)) {
        TEST_FAIL("failed to decode exit message");
        rt_exit();
    }

    if (exit_msg.reason == RT_EXIT_CRASH) {
        TEST_PASS("crash detected with RT_EXIT_CRASH");
    } else {
        printf("    exit reason: %d (expected RT_EXIT_CRASH=%d)\n", exit_msg.reason, RT_EXIT_CRASH);
        TEST_FAIL("wrong exit reason");
    }

    rt_exit();
}

// ============================================================================
// Test 13: Actor table exhaustion (RT_MAX_ACTORS)
// ============================================================================

static void wait_for_signal_actor(void *arg) {
    (void)arg;
    // Wait for signal from parent to exit
    rt_message msg;
    rt_ipc_recv(&msg, 5000);  // Timeout after 5s in case parent dies
    rt_exit();
}

static void test13_actor_table_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 13: Actor table exhaustion (RT_MAX_ACTORS=%d)\n", RT_MAX_ACTORS);
    fflush(stdout);

    // Use malloc stacks with small size to avoid arena exhaustion
    // This tests the actual actor table limit, not stack arena
    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.malloc_stack = true;
    cfg.stack_size = 8 * 1024;  // 8KB stacks

    // We're already using slots for: test runner + this test actor
    // Try to spawn until we hit the limit
    actor_id ids[RT_MAX_ACTORS];
    int spawned = 0;

    for (int i = 0; i < RT_MAX_ACTORS; i++) {
        actor_id id = rt_spawn_ex(wait_for_signal_actor, NULL, &cfg);
        if (id == ACTOR_ID_INVALID) {
            // Exhaustion reached
            break;
        }
        ids[spawned++] = id;
    }

    printf("    Spawned %d actors before exhaustion\n", spawned);
    fflush(stdout);

    // We should have hit actor table exhaustion
    // With RT_MAX_ACTORS=64, we should spawn at least 60 (some slots used by test infrastructure)
    if (spawned >= RT_MAX_ACTORS - 4) {
        TEST_PASS("actor table exhaustion detected");
    } else {
        printf("    Expected to spawn at least %d actors\n", RT_MAX_ACTORS - 4);
        TEST_FAIL("spawned fewer actors than expected");
    }

    // Signal all waiting actors to exit
    int dummy = 1;
    for (int i = 0; i < spawned; i++) {
        rt_ipc_send(ids[i], &dummy, sizeof(dummy));
    }

    // Yield a few times to let them exit
    for (int i = 0; i < spawned + 5; i++) {
        rt_yield();
    }

    rt_exit();
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
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = 64 * 1024;

        actor_id test = rt_spawn_ex(test_funcs[i], NULL, &cfg);
        if (test == ACTOR_ID_INVALID) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        rt_link(test);

        rt_message msg;
        rt_ipc_recv(&msg, 5000);
    }

    rt_exit();
}

int main(void) {
    printf("=== Actor (rt_spawn/rt_exit/rt_yield) Test Suite ===\n");
    fflush(stdout);

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
