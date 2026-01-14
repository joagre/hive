#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
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
// Test 1: Higher priority actors run before lower priority ones
// ============================================================================

// Shared execution order tracking
#define MAX_EXEC_ORDER 16
static int g_exec_order[MAX_EXEC_ORDER];
static int g_exec_count = 0;

static void priority_actor(void *arg) {
    int id = *(int *)arg;

    // Record execution order
    if (g_exec_count < MAX_EXEC_ORDER) {
        g_exec_order[g_exec_count++] = id;
    }

    hive_exit();
}

static void test1_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 1: Higher priority runs first\n");

    g_exec_count = 0;

    // Actor IDs encode priority: 0=CRITICAL, 1=HIGH, 2=NORMAL, 3=LOW
    static int ids[4] = {3, 2, 1, 0};  // Spawn in reverse order (LOW first)

    // Spawn LOW priority first
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.priority = HIVE_PRIORITY_LOW;
    actor_id id;
    hive_spawn_ex(priority_actor, &ids[0], &cfg, &id);

    // Spawn NORMAL priority
    cfg.priority = HIVE_PRIORITY_NORMAL;
    hive_spawn_ex(priority_actor, &ids[1], &cfg, &id);

    // Spawn HIGH priority
    cfg.priority = HIVE_PRIORITY_HIGH;
    hive_spawn_ex(priority_actor, &ids[2], &cfg, &id);

    // Spawn CRITICAL priority
    cfg.priority = HIVE_PRIORITY_CRITICAL;
    hive_spawn_ex(priority_actor, &ids[3], &cfg, &id);

    // Yield to let them all run
    // Since we're NORMAL priority, CRITICAL and HIGH should run before us
    hive_yield();

    // Give time for all to complete
    timer_id timer;
    hive_timer_after(50000, &timer);  // 50ms
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Check execution order: should be CRITICAL(0), HIGH(1), NORMAL(2), LOW(3)
    // But coordinator is also NORMAL, so order depends on round-robin
    // The key check: CRITICAL and HIGH must come before NORMAL and LOW

    bool critical_before_normal = false;
    bool high_before_low = false;
    int critical_pos = -1, high_pos = -1, normal_pos = -1, low_pos = -1;

    for (int i = 0; i < g_exec_count; i++) {
        if (g_exec_order[i] == 0) critical_pos = i;
        if (g_exec_order[i] == 1) high_pos = i;
        if (g_exec_order[i] == 2) normal_pos = i;
        if (g_exec_order[i] == 3) low_pos = i;
    }

    printf("  Execution order: ");
    for (int i = 0; i < g_exec_count; i++) {
        const char *names[] = {"CRITICAL", "HIGH", "NORMAL", "LOW"};
        printf("%s ", names[g_exec_order[i]]);
    }
    printf("\n");

    if (critical_pos >= 0 && normal_pos >= 0) {
        critical_before_normal = (critical_pos < normal_pos);
    }
    if (high_pos >= 0 && low_pos >= 0) {
        high_before_low = (high_pos < low_pos);
    }

    if (critical_before_normal && high_before_low) {
        TEST_PASS("higher priority actors run before lower priority");
    } else {
        TEST_FAIL("priority ordering violated");
        printf("    critical_pos=%d, high_pos=%d, normal_pos=%d, low_pos=%d\n",
               critical_pos, high_pos, normal_pos, low_pos);
    }

    hive_exit();
}

// ============================================================================
// Test 2: Round-robin within same priority level
// ============================================================================

static int g_rr_order[8];
static int g_rr_count = 0;

static void rr_actor(void *arg) {
    int id = *(int *)arg;

    // Record first execution
    if (g_rr_count < 8) {
        g_rr_order[g_rr_count++] = id;
    }

    // Yield and run again to test round-robin
    hive_yield();

    if (g_rr_count < 8) {
        g_rr_order[g_rr_count++] = id;
    }

    hive_exit();
}

static void test2_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 2: Round-robin within same priority\n");

    g_rr_count = 0;

    // Spawn 3 actors at NORMAL priority
    static int ids[3] = {1, 2, 3};
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.priority = HIVE_PRIORITY_NORMAL;

    for (int i = 0; i < 3; i++) {
        actor_id id;
        hive_spawn_ex(rr_actor, &ids[i], &cfg, &id);
    }

    // Wait for them to complete
    timer_id timer;
    hive_timer_after(100000, &timer);  // 100ms
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    printf("  Execution sequence: ");
    for (int i = 0; i < g_rr_count; i++) {
        printf("%d ", g_rr_order[i]);
    }
    printf("\n");

    // Check that actors alternate (round-robin behavior)
    // First pass: 1, 2, 3 (or some permutation)
    // Second pass: same permutation again
    // The key is that one actor doesn't monopolize

    bool has_interleaving = false;
    for (int i = 1; i < g_rr_count; i++) {
        if (g_rr_order[i] != g_rr_order[i-1]) {
            has_interleaving = true;
            break;
        }
    }

    if (has_interleaving && g_rr_count >= 6) {
        TEST_PASS("round-robin scheduling within priority level");
    } else if (g_rr_count >= 6) {
        TEST_FAIL("no interleaving detected");
    } else {
        TEST_FAIL("not enough executions recorded");
    }

    hive_exit();
}

// ============================================================================
// Test 3: High priority actor runs immediately after becoming ready
// ============================================================================

static bool g_high_ran_first = false;
static bool g_low_finished = false;

static void high_prio_late_spawn(void *arg) {
    (void)arg;
    // This high-priority actor was spawned by the low-priority one
    // It should run before the low-priority actor continues
    if (!g_low_finished) {
        g_high_ran_first = true;
    }
    hive_exit();
}

static void low_prio_spawner(void *arg) {
    (void)arg;

    // Spawn a high-priority actor
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.priority = HIVE_PRIORITY_HIGH;
    actor_id id;
    hive_spawn_ex(high_prio_late_spawn, NULL, &cfg, &id);

    // Yield - high priority should run now
    hive_yield();

    g_low_finished = true;
    hive_exit();
}

static void test3_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 3: High priority preempts after yield\n");

    g_high_ran_first = false;
    g_low_finished = false;

    // Spawn a LOW priority actor that will spawn a HIGH priority actor
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.priority = HIVE_PRIORITY_LOW;
    actor_id id;
    hive_spawn_ex(low_prio_spawner, NULL, &cfg, &id);

    // Wait for completion
    timer_id timer;
    hive_timer_after(100000, &timer);
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_high_ran_first) {
        TEST_PASS("high priority actor runs before low priority continues");
    } else {
        TEST_FAIL("high priority actor did not preempt");
    }

    hive_exit();
}

// ============================================================================
// Test 4: All priority levels eventually run (no starvation)
// ============================================================================

static bool g_prio_ran[4] = {false, false, false, false};

static void starvation_actor(void *arg) {
    int prio = *(int *)arg;
    g_prio_ran[prio] = true;
    hive_exit();
}

static void test4_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 4: No starvation (all priorities run)\n");

    memset(g_prio_ran, 0, sizeof(g_prio_ran));

    // Spawn one actor at each priority level
    static int prios[4] = {0, 1, 2, 3};

    for (int i = 0; i < 4; i++) {
        actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = (hive_priority_level)i;
        actor_id id;
        hive_spawn_ex(starvation_actor, &prios[i], &cfg, &id);
    }

    // Wait for all to complete
    timer_id timer;
    hive_timer_after(100000, &timer);
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    bool all_ran = g_prio_ran[0] && g_prio_ran[1] && g_prio_ran[2] && g_prio_ran[3];

    printf("  CRITICAL ran: %s\n", g_prio_ran[0] ? "yes" : "no");
    printf("  HIGH ran: %s\n", g_prio_ran[1] ? "yes" : "no");
    printf("  NORMAL ran: %s\n", g_prio_ran[2] ? "yes" : "no");
    printf("  LOW ran: %s\n", g_prio_ran[3] ? "yes" : "no");

    if (all_ran) {
        TEST_PASS("all priority levels eventually execute");
    } else {
        TEST_FAIL("some priority levels starved");
    }

    hive_exit();
}

// ============================================================================
// Test 5: Default priority is NORMAL
// ============================================================================

static hive_priority_level g_default_prio = HIVE_PRIORITY_COUNT;

static void check_default_prio(void *arg) {
    (void)arg;
    // We need to check the actor's priority - but we don't have direct access
    // We'll verify by checking HIVE_ACTOR_CONFIG_DEFAULT
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    g_default_prio = cfg.priority;
    hive_exit();
}

static void test5_coordinator(void *arg) {
    (void)arg;
    printf("\nTest 5: Default priority is NORMAL\n");

    // Check HIVE_ACTOR_CONFIG_DEFAULT directly
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;

    if (cfg.priority == HIVE_PRIORITY_NORMAL) {
        TEST_PASS("HIVE_ACTOR_CONFIG_DEFAULT has NORMAL priority");
    } else {
        TEST_FAIL("default priority is not NORMAL");
        printf("    default priority = %d (expected %d)\n", cfg.priority, HIVE_PRIORITY_NORMAL);
    }

    // Also spawn an actor with default config to verify
    actor_id checker;
    hive_spawn(check_default_prio, NULL, &checker);

    timer_id timer;
    hive_timer_after(50000, &timer);
    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_coordinator,
    test2_coordinator,
    test3_coordinator,
    test4_coordinator,
    test5_coordinator,
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
        hive_ipc_recv(&msg, 5000);
    }

    hive_exit();
}

int main(void) {
    printf("=== Priority Scheduling Test Suite ===\n");

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
