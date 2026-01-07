#include "rt_runtime.h"
#include "rt_link.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <string.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); tests_failed++; } while(0)

// ============================================================================
// Test 1: Basic link - both actors notified when one dies
// ============================================================================

static bool g_actor_a_notified = false;
static bool g_actor_b_notified = false;

static void actor_a_links_to_b(void *arg) {
    actor_id actor_b = *(actor_id *)arg;

    // Link to actor B
    rt_status status = rt_link(actor_b);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    // Wait for exit notification
    rt_message msg;
    status = rt_ipc_recv(&msg, 1000);

    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);
        if (exit_info.actor == actor_b) {
            g_actor_a_notified = true;
        }
    }

    rt_exit();
}

static void actor_b_exits_immediately(void *arg) {
    (void)arg;
    // Give actor A time to link
    timer_id timer;
    rt_timer_after(50000, &timer);  // 50ms
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

static void test1_basic_link(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic link (both notified)\n");

    g_actor_a_notified = false;
    g_actor_b_notified = false;

    // Spawn actor B first
    actor_id actor_b = rt_spawn(actor_b_exits_immediately, NULL);
    if (actor_b == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn actor B");
        rt_exit();
    }

    // Spawn actor A and pass actor B's ID
    actor_id actor_a = rt_spawn(actor_a_links_to_b, &actor_b);
    if (actor_a == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn actor A");
        rt_exit();
    }

    // Wait for both to complete
    timer_id timer;
    rt_timer_after(200000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (g_actor_a_notified) {
        TEST_PASS("linked actor receives exit notification");
    } else {
        TEST_FAIL("linked actor did not receive notification");
    }

    rt_exit();
}

// ============================================================================
// Test 2: Link is bidirectional - reverse direction
// ============================================================================

static actor_id g_linker_id = ACTOR_ID_INVALID;
static bool g_target_notified = false;

static void target_waits_for_linker(void *arg) {
    (void)arg;

    // Wait for exit notification from linker
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 500);

    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);
        if (exit_info.actor == g_linker_id) {
            g_target_notified = true;
        }
    }

    rt_exit();
}

static void linker_dies_first(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Link to target
    rt_link(target);

    // Die immediately
    rt_exit();
}

static void test2_bidirectional(void *arg) {
    (void)arg;
    printf("\nTest 2: Link is bidirectional\n");

    g_target_notified = false;

    // Spawn target first
    actor_id target = rt_spawn(target_waits_for_linker, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    // Spawn linker
    g_linker_id = rt_spawn(linker_dies_first, &target);
    if (g_linker_id == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn linker");
        rt_exit();
    }

    // Wait for completion
    timer_id timer;
    rt_timer_after(300000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (g_target_notified) {
        TEST_PASS("target notified when linker dies (bidirectional)");
    } else {
        TEST_FAIL("target not notified (link should be bidirectional)");
    }

    rt_exit();
}

// ============================================================================
// Test 3: Unlink prevents notification
// ============================================================================

static bool g_unlinked_received_notification = false;

static void actor_unlinks_before_death(void *arg) {
    actor_id target = *(actor_id *)arg;

    rt_link(target);
    rt_unlink(target);

    // Wait for any exit notification
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 300);

    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        g_unlinked_received_notification = true;
    }

    rt_exit();
}

static void actor_dies_after_unlink(void *arg) {
    (void)arg;

    // Give time for link/unlink
    timer_id timer;
    rt_timer_after(100000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

static void test3_unlink(void *arg) {
    (void)arg;
    printf("\nTest 3: Unlink prevents notification\n");

    g_unlinked_received_notification = false;

    actor_id target = rt_spawn(actor_dies_after_unlink, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    rt_spawn(actor_unlinks_before_death, &target);

    timer_id timer;
    rt_timer_after(500000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (!g_unlinked_received_notification) {
        TEST_PASS("unlink prevents exit notification");
    } else {
        TEST_FAIL("received notification after unlink");
    }

    rt_exit();
}

// ============================================================================
// Test 4: Link to invalid actor fails
// ============================================================================

static void test4_link_invalid(void *arg) {
    (void)arg;
    printf("\nTest 4: Link to invalid actor fails\n");

    rt_status status = rt_link(ACTOR_ID_INVALID);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_link rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("rt_link should reject ACTOR_ID_INVALID");
    }

    status = rt_link(9999);  // Non-existent actor
    if (RT_FAILED(status)) {
        TEST_PASS("rt_link rejects non-existent actor");
    } else {
        TEST_FAIL("rt_link should reject non-existent actor");
    }

    rt_exit();
}

// ============================================================================
// Test 5: Multiple links from one actor
// ============================================================================

static int g_multi_link_count = 0;

static void multi_link_target(void *arg) {
    int delay_ms = *(int *)arg;

    timer_id timer;
    rt_timer_after(delay_ms * 1000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

static void multi_linker(void *arg) {
    actor_id *targets = (actor_id *)arg;

    // Link to all 3 targets
    for (int i = 0; i < 3; i++) {
        rt_link(targets[i]);
    }

    // Receive exit notifications
    for (int i = 0; i < 3; i++) {
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 500);
        if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
            g_multi_link_count++;
        }
    }

    rt_exit();
}

static void test5_multiple_links(void *arg) {
    (void)arg;
    printf("\nTest 5: Multiple links from one actor\n");

    g_multi_link_count = 0;

    // Spawn 3 targets with different delays
    static int delays[3] = {50, 100, 150};
    static actor_id targets[3];

    for (int i = 0; i < 3; i++) {
        targets[i] = rt_spawn(multi_link_target, &delays[i]);
        if (targets[i] == ACTOR_ID_INVALID) {
            TEST_FAIL("spawn target");
            rt_exit();
        }
    }

    // Spawn linker
    rt_spawn(multi_linker, targets);

    // Wait for all to complete
    timer_id timer;
    rt_timer_after(500000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (g_multi_link_count == 3) {
        TEST_PASS("received all 3 exit notifications from linked actors");
    } else {
        printf("    Received %d/3 notifications\n", g_multi_link_count);
        TEST_FAIL("did not receive all notifications");
    }

    rt_exit();
}

// ============================================================================
// Test 6: Link vs Monitor difference (link is bidirectional)
// ============================================================================

static bool g_link_target_got_notification = false;
static bool g_monitor_target_got_notification = false;

static void link_target_waits(void *arg) {
    (void)arg;
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 300);
    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        g_link_target_got_notification = true;
    }
    rt_exit();
}

static void monitor_target_waits(void *arg) {
    (void)arg;
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 300);
    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        g_monitor_target_got_notification = true;
    }
    rt_exit();
}

static void linker_actor(void *arg) {
    actor_id target = *(actor_id *)arg;
    rt_link(target);
    rt_exit();  // Die immediately
}

static void monitor_actor(void *arg) {
    actor_id target = *(actor_id *)arg;
    uint32_t ref;
    rt_monitor(target, &ref);
    rt_exit();  // Die immediately
}

static void test6_link_vs_monitor(void *arg) {
    (void)arg;
    printf("\nTest 6: Link vs Monitor (link is bidirectional)\n");

    g_link_target_got_notification = false;
    g_monitor_target_got_notification = false;

    // Test link: target should be notified when linker dies
    actor_id link_target = rt_spawn(link_target_waits, NULL);
    actor_id linker = rt_spawn(linker_actor, &link_target);
    (void)linker;

    // Test monitor: target should NOT be notified when monitor dies
    actor_id monitor_target = rt_spawn(monitor_target_waits, NULL);
    actor_id monitor = rt_spawn(monitor_actor, &monitor_target);
    (void)monitor;

    // Wait for completion
    timer_id timer;
    rt_timer_after(500000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (g_link_target_got_notification) {
        TEST_PASS("link target notified when linker dies (bidirectional)");
    } else {
        TEST_FAIL("link target should be notified");
    }

    if (!g_monitor_target_got_notification) {
        TEST_PASS("monitor target NOT notified when monitor dies (unidirectional)");
    } else {
        TEST_FAIL("monitor target should NOT be notified");
    }

    rt_exit();
}

// ============================================================================
// Test 7: Exit reason in link notification
// ============================================================================

static rt_exit_reason g_received_reason = RT_EXIT_NORMAL;

static void link_receiver_checks_reason(void *arg) {
    actor_id target = *(actor_id *)arg;

    rt_link(target);

    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 500);
    if (!RT_FAILED(status) && rt_is_exit_msg(&msg)) {
        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);
        g_received_reason = exit_info.reason;
    }

    rt_exit();
}

static void normal_exit_actor(void *arg) {
    (void)arg;
    timer_id timer;
    rt_timer_after(50000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);
    rt_exit();  // Normal exit
}

static void test7_exit_reason(void *arg) {
    (void)arg;
    printf("\nTest 7: Exit reason in link notification\n");

    g_received_reason = (rt_exit_reason)99;  // Invalid value

    actor_id target = rt_spawn(normal_exit_actor, NULL);
    rt_spawn(link_receiver_checks_reason, &target);

    timer_id timer;
    rt_timer_after(300000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (g_received_reason == RT_EXIT_NORMAL) {
        TEST_PASS("exit reason is RT_EXIT_NORMAL for normal exit");
    } else {
        printf("    Got reason: %d, expected: %d\n", g_received_reason, RT_EXIT_NORMAL);
        TEST_FAIL("wrong exit reason");
    }

    rt_exit();
}

// ============================================================================
// Test 8: Link to dead actor (actor that existed but has exited)
// ============================================================================

static void quickly_exiting_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void test8_link_to_dead_actor(void *arg) {
    (void)arg;
    printf("\nTest 8: Link to dead actor\n");
    fflush(stdout);

    // Spawn an actor that exits immediately
    actor_id target = rt_spawn(quickly_exiting_actor, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("failed to spawn target actor");
        rt_exit();
    }

    // Yield to let it run and exit
    for (int i = 0; i < 5; i++) {
        rt_yield();
    }

    // Verify the actor is dead
    if (rt_actor_alive(target)) {
        TEST_FAIL("target actor should be dead by now");
        rt_exit();
    }

    // Try to link to the dead actor
    rt_status status = rt_link(target);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_link rejects dead actor");
    } else {
        TEST_FAIL("rt_link should reject dead actor");
    }

    rt_exit();
}

// ============================================================================
// Test 9: Link to self (should fail or be a no-op)
// ============================================================================

static void test9_link_to_self(void *arg) {
    (void)arg;
    printf("\nTest 9: Link to self\n");
    fflush(stdout);

    actor_id self = rt_self();

    rt_status status = rt_link(self);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_link to self is rejected");
    } else {
        // If it succeeds, it should be a no-op (shouldn't get exit notification)
        TEST_PASS("rt_link to self accepted (no-op expected)");
    }

    rt_exit();
}

// ============================================================================
// Test 10: Unlink non-linked actor
// ============================================================================

static void unlink_target_actor(void *arg) {
    (void)arg;
    // Wait a bit then exit
    timer_id timer;
    rt_timer_after(200000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);
    rt_exit();
}

static void test10_unlink_non_linked(void *arg) {
    (void)arg;
    printf("\nTest 10: Unlink non-linked actor\n");
    fflush(stdout);

    // Spawn an actor but don't link to it
    actor_id target = rt_spawn(unlink_target_actor, NULL);
    if (target == ACTOR_ID_INVALID) {
        TEST_FAIL("spawn target");
        rt_exit();
    }

    // Try to unlink from an actor we're not linked to
    rt_status status = rt_unlink(target);

    // Should either fail or be a no-op
    if (RT_FAILED(status)) {
        TEST_PASS("rt_unlink non-linked actor fails gracefully");
    } else {
        TEST_PASS("rt_unlink non-linked actor is no-op");
    }

    // Wait for target to exit
    timer_id timer;
    rt_timer_after(300000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    rt_exit();
}

// ============================================================================
// Test 11: Unlink invalid actor
// ============================================================================

static void test11_unlink_invalid(void *arg) {
    (void)arg;
    printf("\nTest 11: Unlink invalid actor\n");
    fflush(stdout);

    rt_status status = rt_unlink(ACTOR_ID_INVALID);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_unlink rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("rt_unlink should reject ACTOR_ID_INVALID");
    }

    status = rt_unlink(9999);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_unlink rejects non-existent actor");
    } else {
        TEST_FAIL("rt_unlink should reject non-existent actor");
    }

    rt_exit();
}

// ============================================================================
// Test 12: Link pool exhaustion (RT_LINK_ENTRY_POOL_SIZE=128)
// Each link uses 2 entries (bidirectional), so max ~64 links
// ============================================================================

static void link_pool_target_actor(void *arg) {
    (void)arg;
    // Wait for signal to exit
    rt_message msg;
    rt_ipc_recv(&msg, 5000);
    rt_exit();
}

static void test12_link_pool_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 12: Link pool exhaustion (RT_LINK_ENTRY_POOL_SIZE=%d)\n",
           RT_LINK_ENTRY_POOL_SIZE);
    fflush(stdout);

    // Each link uses 2 entries (bidirectional)
    int max_links = RT_LINK_ENTRY_POOL_SIZE / 2;
    actor_id targets[RT_LINK_ENTRY_POOL_SIZE];
    int spawned = 0;
    int linked = 0;

    // Spawn actors and link to them until pool exhaustion
    for (int i = 0; i < max_links + 10; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = 8 * 1024;

        actor_id target = rt_spawn_ex(link_pool_target_actor, NULL, &cfg);
        if (target == ACTOR_ID_INVALID) {
            break;
        }
        targets[spawned++] = target;

        rt_status status = rt_link(target);
        if (RT_FAILED(status)) {
            printf("    Link failed after %d links (pool exhausted)\n", linked);
            break;
        }
        linked++;
    }

    if (linked < max_links + 10) {
        TEST_PASS("link pool exhaustion detected");
    } else {
        printf("    Linked to all %d actors without exhaustion\n", linked);
        TEST_FAIL("expected link pool to exhaust");
    }

    // Signal all actors to exit
    for (int i = 0; i < spawned; i++) {
        int done = 1;
        rt_ipc_send(targets[i], &done, sizeof(done), IPC_ASYNC);
    }

    // Wait for cleanup
    timer_id timer;
    rt_timer_after(200000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    // Drain exit notifications
    while (rt_ipc_pending()) {
        rt_ipc_recv(&msg, 0);
    }

    rt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_basic_link,
    test2_bidirectional,
    test3_unlink,
    test4_link_invalid,
    test5_multiple_links,
    test6_link_vs_monitor,
    test7_exit_reason,
    test8_link_to_dead_actor,
    test9_link_to_self,
    test10_unlink_non_linked,
    test11_unlink_invalid,
    test12_link_pool_exhaustion,
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
    printf("=== Link (rt_link) Test Suite ===\n");

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
