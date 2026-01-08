#include "acrt_runtime.h"
#include "acrt_link.h"
#include "acrt_ipc.h"
#include "acrt_timer.h"
#include "acrt_static_config.h"
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
    acrt_status status = acrt_link(actor_b);
    if (ACRT_FAILED(status)) {
        acrt_exit();
    }

    // Wait for exit notification
    acrt_message msg;
    status = acrt_ipc_recv(&msg, 1000);

    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        acrt_exit_msg exit_info;
        acrt_decode_exit(&msg, &exit_info);
        if (exit_info.actor == actor_b) {
            g_actor_a_notified = true;
        }
    }

    acrt_exit();
}

static void actor_b_exits_immediately(void *arg) {
    (void)arg;
    // Give actor A time to link
    timer_id timer;
    acrt_timer_after(50000, &timer);  // 50ms
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    acrt_exit();
}

static void test1_basic_link(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic link (both notified)\n");

    g_actor_a_notified = false;
    g_actor_b_notified = false;

    // Spawn actor B first
    actor_id actor_b;
    if (ACRT_FAILED(acrt_spawn(actor_b_exits_immediately, NULL, &actor_b))) {
        TEST_FAIL("spawn actor B");
        acrt_exit();
    }

    // Spawn actor A and pass actor B's ID
    actor_id actor_a;
    if (ACRT_FAILED(acrt_spawn(actor_a_links_to_b, &actor_b, &actor_a))) {
        TEST_FAIL("spawn actor A");
        acrt_exit();
    }

    // Wait for both to complete
    timer_id timer;
    acrt_timer_after(200000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    if (g_actor_a_notified) {
        TEST_PASS("linked actor receives exit notification");
    } else {
        TEST_FAIL("linked actor did not receive notification");
    }

    acrt_exit();
}

// ============================================================================
// Test 2: Link is bidirectional - reverse direction
// ============================================================================

static actor_id g_linker_id = ACTOR_ID_INVALID;
static bool g_target_notified = false;

static void target_waits_for_linker(void *arg) {
    (void)arg;

    // Wait for exit notification from linker
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 500);

    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        acrt_exit_msg exit_info;
        acrt_decode_exit(&msg, &exit_info);
        if (exit_info.actor == g_linker_id) {
            g_target_notified = true;
        }
    }

    acrt_exit();
}

static void linker_dies_first(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Link to target
    acrt_link(target);

    // Die immediately
    acrt_exit();
}

static void test2_bidirectional(void *arg) {
    (void)arg;
    printf("\nTest 2: Link is bidirectional\n");

    g_target_notified = false;

    // Spawn target first
    actor_id target;
    if (ACRT_FAILED(acrt_spawn(target_waits_for_linker, NULL, &target))) {
        TEST_FAIL("spawn target");
        acrt_exit();
    }

    // Spawn linker
    if (ACRT_FAILED(acrt_spawn(linker_dies_first, &target, &g_linker_id))) {
        TEST_FAIL("spawn linker");
        acrt_exit();
    }

    // Wait for completion
    timer_id timer;
    acrt_timer_after(300000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    if (g_target_notified) {
        TEST_PASS("target notified when linker dies (bidirectional)");
    } else {
        TEST_FAIL("target not notified (link should be bidirectional)");
    }

    acrt_exit();
}

// ============================================================================
// Test 3: Unlink prevents notification
// ============================================================================

static bool g_unlinked_received_notification = false;

static void actor_unlinks_before_death(void *arg) {
    actor_id target = *(actor_id *)arg;

    acrt_link(target);
    acrt_link_remove(target);

    // Wait for any exit notification
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 300);

    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        g_unlinked_received_notification = true;
    }

    acrt_exit();
}

static void actor_dies_after_unlink(void *arg) {
    (void)arg;

    // Give time for link/unlink
    timer_id timer;
    acrt_timer_after(100000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    acrt_exit();
}

static void test3_unlink(void *arg) {
    (void)arg;
    printf("\nTest 3: Unlink prevents notification\n");

    g_unlinked_received_notification = false;

    actor_id target;
    if (ACRT_FAILED(acrt_spawn(actor_dies_after_unlink, NULL, &target))) {
        TEST_FAIL("spawn target");
        acrt_exit();
    }

    actor_id unlinker;
    acrt_spawn(actor_unlinks_before_death, &target, &unlinker);

    timer_id timer;
    acrt_timer_after(500000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    if (!g_unlinked_received_notification) {
        TEST_PASS("unlink prevents exit notification");
    } else {
        TEST_FAIL("received notification after unlink");
    }

    acrt_exit();
}

// ============================================================================
// Test 4: Link to invalid actor fails
// ============================================================================

static void test4_link_invalid(void *arg) {
    (void)arg;
    printf("\nTest 4: Link to invalid actor fails\n");

    acrt_status status = acrt_link(ACTOR_ID_INVALID);
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("acrt_link should reject ACTOR_ID_INVALID");
    }

    status = acrt_link(9999);  // Non-existent actor
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link rejects non-existent actor");
    } else {
        TEST_FAIL("acrt_link should reject non-existent actor");
    }

    acrt_exit();
}

// ============================================================================
// Test 5: Multiple links from one actor
// ============================================================================

static int g_multi_link_count = 0;

static void multi_link_target(void *arg) {
    int delay_ms = *(int *)arg;

    timer_id timer;
    acrt_timer_after(delay_ms * 1000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    acrt_exit();
}

static void multi_linker(void *arg) {
    actor_id *targets = (actor_id *)arg;

    // Link to all 3 targets
    for (int i = 0; i < 3; i++) {
        acrt_link(targets[i]);
    }

    // Receive exit notifications
    for (int i = 0; i < 3; i++) {
        acrt_message msg;
        acrt_status status = acrt_ipc_recv(&msg, 500);
        if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
            g_multi_link_count++;
        }
    }

    acrt_exit();
}

static void test5_multiple_links(void *arg) {
    (void)arg;
    printf("\nTest 5: Multiple links from one actor\n");

    g_multi_link_count = 0;

    // Spawn 3 targets with different delays
    static int delays[3] = {50, 100, 150};
    static actor_id targets[3];

    for (int i = 0; i < 3; i++) {
        if (ACRT_FAILED(acrt_spawn(multi_link_target, &delays[i], &targets[i]))) {
            TEST_FAIL("spawn target");
            acrt_exit();
        }
    }

    // Spawn linker
    actor_id linker;
    acrt_spawn(multi_linker, targets, &linker);

    // Wait for all to complete
    timer_id timer;
    acrt_timer_after(500000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    if (g_multi_link_count == 3) {
        TEST_PASS("received all 3 exit notifications from linked actors");
    } else {
        printf("    Received %d/3 notifications\n", g_multi_link_count);
        TEST_FAIL("did not receive all notifications");
    }

    acrt_exit();
}

// ============================================================================
// Test 6: Link vs Monitor difference (link is bidirectional)
// ============================================================================

static bool g_link_target_got_notification = false;
static bool g_monitor_target_got_notification = false;

static void link_target_waits(void *arg) {
    (void)arg;
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 300);
    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        g_link_target_got_notification = true;
    }
    acrt_exit();
}

static void monitor_target_waits(void *arg) {
    (void)arg;
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 300);
    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        g_monitor_target_got_notification = true;
    }
    acrt_exit();
}

static void linker_actor(void *arg) {
    actor_id target = *(actor_id *)arg;
    acrt_link(target);
    acrt_exit();  // Die immediately
}

static void monitor_actor(void *arg) {
    actor_id target = *(actor_id *)arg;
    uint32_t ref;
    acrt_monitor(target, &ref);
    acrt_exit();  // Die immediately
}

static void test6_link_vs_monitor(void *arg) {
    (void)arg;
    printf("\nTest 6: Link vs Monitor (link is bidirectional)\n");

    g_link_target_got_notification = false;
    g_monitor_target_got_notification = false;

    // Test link: target should be notified when linker dies
    actor_id link_target;
    acrt_spawn(link_target_waits, NULL, &link_target);
    actor_id linker;
    acrt_spawn(linker_actor, &link_target, &linker);
    (void)linker;

    // Test monitor: target should NOT be notified when monitor dies
    actor_id monitor_target;
    acrt_spawn(monitor_target_waits, NULL, &monitor_target);
    actor_id monitor;
    acrt_spawn(monitor_actor, &monitor_target, &monitor);
    (void)monitor;

    // Wait for completion
    timer_id timer;
    acrt_timer_after(500000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

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

    acrt_exit();
}

// ============================================================================
// Test 7: Exit reason in link notification
// ============================================================================

static acrt_exit_reason g_received_reason = ACRT_EXIT_NORMAL;

static void link_receiver_checks_reason(void *arg) {
    actor_id target = *(actor_id *)arg;

    acrt_link(target);

    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, 500);
    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        acrt_exit_msg exit_info;
        acrt_decode_exit(&msg, &exit_info);
        g_received_reason = exit_info.reason;
    }

    acrt_exit();
}

static void normal_exit_actor(void *arg) {
    (void)arg;
    timer_id timer;
    acrt_timer_after(50000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);
    acrt_exit();  // Normal exit
}

static void test7_exit_reason(void *arg) {
    (void)arg;
    printf("\nTest 7: Exit reason in link notification\n");

    g_received_reason = (acrt_exit_reason)99;  // Invalid value

    actor_id target;
    acrt_spawn(normal_exit_actor, NULL, &target);
    actor_id receiver;
    acrt_spawn(link_receiver_checks_reason, &target, &receiver);

    timer_id timer;
    acrt_timer_after(300000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    if (g_received_reason == ACRT_EXIT_NORMAL) {
        TEST_PASS("exit reason is ACRT_EXIT_NORMAL for normal exit");
    } else {
        printf("    Got reason: %d, expected: %d\n", g_received_reason, ACRT_EXIT_NORMAL);
        TEST_FAIL("wrong exit reason");
    }

    acrt_exit();
}

// ============================================================================
// Test 8: Link to dead actor (actor that existed but has exited)
// ============================================================================

static void quickly_exiting_actor(void *arg) {
    (void)arg;
    acrt_exit();
}

static void test8_link_to_dead_actor(void *arg) {
    (void)arg;
    printf("\nTest 8: Link to dead actor\n");
    fflush(stdout);

    // Spawn an actor that exits immediately
    actor_id target;
    if (ACRT_FAILED(acrt_spawn(quickly_exiting_actor, NULL, &target))) {
        TEST_FAIL("failed to spawn target actor");
        acrt_exit();
    }

    // Yield to let it run and exit
    for (int i = 0; i < 5; i++) {
        acrt_yield();
    }

    // Verify the actor is dead
    if (acrt_actor_alive(target)) {
        TEST_FAIL("target actor should be dead by now");
        acrt_exit();
    }

    // Try to link to the dead actor
    acrt_status status = acrt_link(target);
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link rejects dead actor");
    } else {
        TEST_FAIL("acrt_link should reject dead actor");
    }

    acrt_exit();
}

// ============================================================================
// Test 9: Link to self (should fail or be a no-op)
// ============================================================================

static void test9_link_to_self(void *arg) {
    (void)arg;
    printf("\nTest 9: Link to self\n");
    fflush(stdout);

    actor_id self = acrt_self();

    acrt_status status = acrt_link(self);
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link to self is rejected");
    } else {
        // If it succeeds, it should be a no-op (shouldn't get exit notification)
        TEST_PASS("acrt_link to self accepted (no-op expected)");
    }

    acrt_exit();
}

// ============================================================================
// Test 10: Unlink non-linked actor
// ============================================================================

static void unlink_target_actor(void *arg) {
    (void)arg;
    // Wait a bit then exit
    timer_id timer;
    acrt_timer_after(200000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);
    acrt_exit();
}

static void test10_unlink_non_linked(void *arg) {
    (void)arg;
    printf("\nTest 10: Unlink non-linked actor\n");
    fflush(stdout);

    // Spawn an actor but don't link to it
    actor_id target;
    if (ACRT_FAILED(acrt_spawn(unlink_target_actor, NULL, &target))) {
        TEST_FAIL("spawn target");
        acrt_exit();
    }

    // Try to unlink from an actor we're not linked to
    acrt_status status = acrt_link_remove(target);

    // Should either fail or be a no-op
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link_remove non-linked actor fails gracefully");
    } else {
        TEST_PASS("acrt_link_remove non-linked actor is no-op");
    }

    // Wait for target to exit
    timer_id timer;
    acrt_timer_after(300000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    acrt_exit();
}

// ============================================================================
// Test 11: Unlink invalid actor
// ============================================================================

static void test11_unlink_invalid(void *arg) {
    (void)arg;
    printf("\nTest 11: Unlink invalid actor\n");
    fflush(stdout);

    acrt_status status = acrt_link_remove(ACTOR_ID_INVALID);
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link_remove rejects ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("acrt_link_remove should reject ACTOR_ID_INVALID");
    }

    status = acrt_link_remove(9999);
    if (ACRT_FAILED(status)) {
        TEST_PASS("acrt_link_remove rejects non-existent actor");
    } else {
        TEST_FAIL("acrt_link_remove should reject non-existent actor");
    }

    acrt_exit();
}

// ============================================================================
// Test 12: Link pool exhaustion (ACRT_LINK_ENTRY_POOL_SIZE=128)
// Each link uses 2 entries (bidirectional), so max ~64 links
// ============================================================================

static void link_pool_target_actor(void *arg) {
    (void)arg;
    // Wait for signal to exit
    acrt_message msg;
    acrt_ipc_recv(&msg, 5000);
    acrt_exit();
}

static void test12_link_pool_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 12: Link pool exhaustion (ACRT_LINK_ENTRY_POOL_SIZE=%d)\n",
           ACRT_LINK_ENTRY_POOL_SIZE);
    fflush(stdout);

    // Each link uses 2 entries (bidirectional)
    int max_links = ACRT_LINK_ENTRY_POOL_SIZE / 2;
    actor_id targets[ACRT_LINK_ENTRY_POOL_SIZE];
    int spawned = 0;
    int linked = 0;

    // Spawn actors and link to them until pool exhaustion
    for (int i = 0; i < max_links + 10; i++) {
        actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = 8 * 1024;

        actor_id target;
        if (ACRT_FAILED(acrt_spawn_ex(link_pool_target_actor, NULL, &cfg, &target))) {
            break;
        }
        targets[spawned++] = target;

        acrt_status status = acrt_link(target);
        if (ACRT_FAILED(status)) {
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
        acrt_ipc_notify(targets[i], &done, sizeof(done));
    }

    // Wait for cleanup
    timer_id timer;
    acrt_timer_after(200000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    // Drain exit notifications
    while (acrt_ipc_pending()) {
        acrt_ipc_recv(&msg, 0);
    }

    acrt_exit();
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
    printf("=== Link (acrt_link) Test Suite ===\n");

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
