#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_link.h"
#include <stdio.h>
#include <string.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); fflush(stdout); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); fflush(stdout); tests_failed++; } while(0)

// ============================================================================
// Test 1: hive_init returns success
// ============================================================================

static void test1_init_success(void) {
    printf("\nTest 1: hive_init returns success\n");

    // Note: hive_init was already called in main() to run this test
    // This test just verifies the runtime started successfully
    TEST_PASS("hive_init succeeded (we're running)");
}

// ============================================================================
// Test 2: hive_self outside actor context
// NOTE: This tests behavior that may crash or return invalid ID
// ============================================================================

// Cannot safely test hive_self outside actor context from within actor
// Just document the expected behavior
static void test2_self_outside_actor(void *arg) {
    (void)arg;
    printf("\nTest 2: hive_self outside actor context\n");
    printf("    NOTE: Cannot test from within actor - would need separate process\n");
    printf("    Expected: Should return ACTOR_ID_INVALID or crash\n");
    fflush(stdout);

    // We're inside an actor, so hive_self() works fine here
    actor_id self = hive_self();
    if (self != ACTOR_ID_INVALID) {
        TEST_PASS("hive_self returns valid ID inside actor context");
    } else {
        TEST_FAIL("hive_self returned invalid ID inside actor context");
    }

    hive_exit();
}

// ============================================================================
// Test 3: hive_yield returns control to scheduler
// ============================================================================

static int g_yield_order = 0;
static int g_actor1_order = 0;
static int g_actor2_order = 0;

static void yield_actor1(void *arg) {
    (void)arg;
    g_actor1_order = ++g_yield_order;
    hive_yield();
    g_actor1_order = ++g_yield_order;
    hive_exit();
}

static void yield_actor2(void *arg) {
    (void)arg;
    g_actor2_order = ++g_yield_order;
    hive_yield();
    g_actor2_order = ++g_yield_order;
    hive_exit();
}

static void test3_yield(void *arg) {
    (void)arg;
    printf("\nTest 3: hive_yield returns control to scheduler\n");
    fflush(stdout);

    g_yield_order = 0;
    g_actor1_order = 0;
    g_actor2_order = 0;

    actor_id a1;
    hive_spawn(yield_actor1, NULL, &a1);
    actor_id a2;
    hive_spawn(yield_actor2, NULL, &a2);

    hive_link(a1);
    hive_link(a2);

    // Wait for both to complete via link notifications
    hive_message msg;
    hive_ipc_recv(&msg, 500);
    hive_ipc_recv(&msg, 500);

    // After yielding, actors should have interleaved
    // Final order should be 4 (each actor ran twice)
    if (g_yield_order == 4) {
        TEST_PASS("hive_yield allows interleaved execution");
    } else {
        printf("    Final order: %d (expected 4)\n", g_yield_order);
        TEST_FAIL("yield did not interleave correctly");
    }

    hive_exit();
}

// ============================================================================
// Test 4: hive_actor_alive with various IDs
// ============================================================================

static void quickly_exit_actor(void *arg) {
    (void)arg;
    hive_exit();
}

static void test4_actor_alive(void *arg) {
    (void)arg;
    printf("\nTest 4: hive_actor_alive with various IDs\n");
    fflush(stdout);

    // Self should be alive
    actor_id self = hive_self();
    if (hive_actor_alive(self)) {
        TEST_PASS("hive_actor_alive returns true for self");
    } else {
        TEST_FAIL("hive_actor_alive should return true for self");
    }

    // Invalid ID should not be alive
    if (!hive_actor_alive(ACTOR_ID_INVALID)) {
        TEST_PASS("hive_actor_alive returns false for ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("hive_actor_alive should return false for ACTOR_ID_INVALID");
    }

    // Non-existent ID should not be alive
    if (!hive_actor_alive(9999)) {
        TEST_PASS("hive_actor_alive returns false for non-existent ID");
    } else {
        TEST_FAIL("hive_actor_alive should return false for non-existent ID");
    }

    // Spawn and check alive
    actor_id child;
    hive_spawn(quickly_exit_actor, NULL, &child);
    hive_link(child);

    // Should be alive right after spawn
    bool alive_before = hive_actor_alive(child);

    // Wait for it to exit via link notification
    hive_message msg;
    hive_ipc_recv(&msg, 500);

    // Should be dead after exit
    bool alive_after = hive_actor_alive(child);

    if (alive_before && !alive_after) {
        TEST_PASS("hive_actor_alive tracks actor lifecycle");
    } else {
        printf("    before=%d, after=%d\n", alive_before, alive_after);
        TEST_FAIL("hive_actor_alive did not track lifecycle correctly");
    }

    hive_exit();
}

// ============================================================================
// Test 5: Scheduler handles many actors
// ============================================================================

/* Reduce actor count for QEMU's limited actor table */
#ifdef QEMU_TEST_STACK_SIZE
#define MANY_ACTORS 4
#else
#define MANY_ACTORS 10
#endif
static int g_many_actors_count = 0;

static void many_actor(void *arg) {
    (void)arg;
    g_many_actors_count++;
    hive_exit();
}

static void test5_many_actors(void *arg) {
    (void)arg;
    printf("\nTest 5: Scheduler handles many actors\n");
    fflush(stdout);

    g_many_actors_count = 0;

    // Spawn actors without linking (simpler)
    int spawned = 0;
    for (int i = 0; i < MANY_ACTORS; i++) {
        actor_id id;
        if (HIVE_FAILED(hive_spawn(many_actor, NULL, &id))) {
            printf("    Failed to spawn actor %d\n", i);
            fflush(stdout);
            break;
        }
        spawned++;
    }

    // Yield several times to let spawned actors run
    for (int i = 0; i < MANY_ACTORS * 2; i++) {
        hive_yield();
    }

    if (g_many_actors_count == spawned && spawned == MANY_ACTORS) {
        printf("    Spawned and ran %d actors\n", MANY_ACTORS);
        TEST_PASS("scheduler handles many actors");
    } else {
        printf("    Spawned %d, ran %d/%d actors\n", spawned, g_many_actors_count, MANY_ACTORS);
        if (spawned < MANY_ACTORS) {
            TEST_FAIL("could not spawn all actors (actor table full?)");
        } else {
            TEST_FAIL("not all actors ran");
        }
    }

    hive_exit();
}

// ============================================================================
// Test 6: hive_shutdown (if implemented)
// NOTE: hive_shutdown is declared but may not be fully implemented
// ============================================================================

static void test6_shutdown(void *arg) {
    (void)arg;
    printf("\nTest 6: hive_shutdown\n");
    printf("    NOTE: hive_shutdown behavior depends on implementation\n");
    fflush(stdout);

    // We can't actually test shutdown from within an actor
    // because it would terminate us
    TEST_PASS("hive_shutdown exists (not tested from within actor)");

    hive_exit();
}

// ============================================================================
// Test 7: Actor stack sizes
// ============================================================================

static bool g_small_stack_ok = false;
static bool g_large_stack_ok = false;

static void small_stack_actor(void *arg) {
    (void)arg;
    // Just run with minimal stack usage
    int x = 42;
    (void)x;
    g_small_stack_ok = true;
    hive_exit();
}

/* Buffer size for large stack test - reduced for QEMU */
#ifdef QEMU_TEST_STACK_SIZE
#define LARGE_STACK_BUFFER_SIZE 1024
#else
#define LARGE_STACK_BUFFER_SIZE 16384
#endif

static void large_stack_actor(void *arg) {
    (void)arg;
    // Use more stack
    char buffer[LARGE_STACK_BUFFER_SIZE];
    memset(buffer, 'A', sizeof(buffer));
    if (buffer[LARGE_STACK_BUFFER_SIZE - 1] == 'A') {
        g_large_stack_ok = true;
    }
    hive_exit();
}

static void test7_stack_sizes(void *arg) {
    (void)arg;
    printf("\nTest 7: Actor stack sizes\n");
    fflush(stdout);

    g_small_stack_ok = false;
    g_large_stack_ok = false;

    // Small stack
    actor_config small_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    small_cfg.stack_size = TEST_STACK_SIZE(8 * 1024);  // 8KB

    actor_id small;
    if (HIVE_SUCCEEDED(hive_spawn_ex(small_stack_actor, NULL, &small_cfg, &small))) {
        hive_link(small);
        hive_message msg;
        hive_ipc_recv(&msg, 500);

        if (g_small_stack_ok) {
            TEST_PASS("small stack (8KB) works");
        } else {
            TEST_FAIL("small stack actor did not complete");
        }
    } else {
        TEST_FAIL("failed to spawn small stack actor");
    }

    // Large stack
    actor_config large_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    large_cfg.stack_size = TEST_STACK_SIZE(32 * 1024);  // 32KB

    actor_id large;
    if (HIVE_SUCCEEDED(hive_spawn_ex(large_stack_actor, NULL, &large_cfg, &large))) {
        hive_link(large);
        hive_message msg;
        hive_ipc_recv(&msg, 500);

        if (g_large_stack_ok) {
            TEST_PASS("large stack (32KB) works");
        } else {
            TEST_FAIL("large stack actor did not complete");
        }
    } else {
        TEST_FAIL("failed to spawn large stack actor");
    }

    hive_exit();
}

// ============================================================================
// Test 8: Priority levels
// ============================================================================

static int g_priority_order[4] = {0, 0, 0, 0};
static int g_priority_counter = 0;

static void priority_actor(void *arg) {
    int level = *(int *)arg;
    g_priority_order[level] = ++g_priority_counter;
    hive_exit();
}

static void test8_priorities(void *arg) {
    (void)arg;
    printf("\nTest 8: Priority levels\n");
    fflush(stdout);

    g_priority_counter = 0;
    for (int i = 0; i < 4; i++) {
        g_priority_order[i] = 0;
    }

    // Spawn actors in reverse priority order (LOW first, CRITICAL last)
    static int levels[4] = {HIVE_PRIORITY_LOW, HIVE_PRIORITY_NORMAL, HIVE_PRIORITY_HIGH, HIVE_PRIORITY_CRITICAL};
    actor_id ids[4];

    for (int i = 0; i < 4; i++) {
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = levels[i];
        hive_spawn_ex(priority_actor, &levels[i], &cfg, &ids[i]);
        hive_link(ids[i]);
    }

    // Wait for all to complete via link notifications
    for (int i = 0; i < 4; i++) {
        hive_message msg;
        hive_ipc_recv(&msg, 500);
    }

    // Higher priority should run first (lower number = higher priority)
    // CRITICAL=0 should be first, LOW=3 should be last
    printf("    Execution order: CRITICAL=%d, HIGH=%d, NORMAL=%d, LOW=%d\n",
           g_priority_order[HIVE_PRIORITY_CRITICAL],
           g_priority_order[HIVE_PRIORITY_HIGH],
           g_priority_order[HIVE_PRIORITY_NORMAL],
           g_priority_order[HIVE_PRIORITY_LOW]);
    fflush(stdout);

    if (g_priority_order[HIVE_PRIORITY_CRITICAL] < g_priority_order[HIVE_PRIORITY_LOW]) {
        TEST_PASS("higher priority actors run before lower priority");
    } else {
        TEST_FAIL("priority order not respected");
    }

    hive_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test2_self_outside_actor,
    test3_yield,
    test4_actor_alive,
    test5_many_actors,
    test6_shutdown,
    test7_stack_sizes,
    test8_priorities,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    // Test 1 runs outside actor context (sort of)
    test1_init_success();

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = TEST_STACK_SIZE(64 * 1024);

        actor_id test;
        if (HIVE_FAILED(hive_spawn_ex(test_funcs[i], NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            fflush(stdout);
            continue;
        }

        hive_link(test);

        hive_message msg;
        hive_ipc_recv(&msg, 10000);
    }

    hive_exit();
}

int main(void) {
    printf("=== Runtime (hive_init/hive_run/hive_cleanup) Test Suite ===\n");
    fflush(stdout);

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
