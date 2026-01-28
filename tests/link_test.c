#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
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
// Test 1: Basic link - both actors notified when one dies
// ============================================================================

static bool g_actor_a_notified = false;
static bool g_actor_b_notified = false;

static void actor_a_links_to_b(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t actor_b = *(actor_id_t *)args;

    // Link to actor B
    hive_status_t status = hive_link(actor_b);
    if (HIVE_FAILED(status)) {
        hive_exit();
    }

    // Wait for exit notification
    hive_message_t msg;
    status = hive_ipc_recv(&msg, 1000);

    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        hive_exit_msg_t exit_info;
        hive_decode_exit(&msg, &exit_info);
        if (exit_info.actor == actor_b) {
            g_actor_a_notified = true;
        }
    }

    hive_exit();
}

static void actor_b_exits_immediately(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Give actor A time to link
    timer_id_t timer;
    hive_timer_after(50000, &timer); // 50ms
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_exit();
}

static void test1_basic_link(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Basic link (both notified)\n");

    g_actor_a_notified = false;
    g_actor_b_notified = false;

    // Spawn actor B first
    actor_id_t actor_b;
    if (HIVE_FAILED(hive_spawn(actor_b_exits_immediately, NULL, NULL, NULL,
                               &actor_b))) {
        TEST_FAIL("spawn actor B");
        hive_exit();
    }

    // Spawn actor A and pass actor B's ID
    actor_id_t actor_a;
    if (HIVE_FAILED(
            hive_spawn(actor_a_links_to_b, NULL, &actor_b, NULL, &actor_a))) {
        TEST_FAIL("spawn actor A");
        hive_exit();
    }

    // Wait for both to complete
    timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_actor_a_notified) {
        TEST_PASS("linked actor receives exit notification");
    } else {
        TEST_FAIL("linked actor did not receive notification");
    }

    hive_exit();
}

// ============================================================================
// Test 2: Link is bidirectional - reverse direction
// ============================================================================

static actor_id_t g_linker_id = HIVE_ACTOR_ID_INVALID;
static bool g_target_notified = false;

static void target_waits_for_linker(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for exit notification from linker
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 500);

    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        hive_exit_msg_t exit_info;
        hive_decode_exit(&msg, &exit_info);
        if (exit_info.actor == g_linker_id) {
            g_target_notified = true;
        }
    }

    hive_exit();
}

static void linker_dies_first(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t target = *(actor_id_t *)args;

    // Link to target
    hive_link(target);

    // Die immediately
    hive_exit();
}

static void test2_bidirectional(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Link is bidirectional\n");

    g_target_notified = false;

    // Spawn target first
    actor_id_t target;
    if (HIVE_FAILED(
            hive_spawn(target_waits_for_linker, NULL, NULL, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    // Spawn linker
    if (HIVE_FAILED(
            hive_spawn(linker_dies_first, NULL, &target, NULL, &g_linker_id))) {
        TEST_FAIL("spawn linker");
        hive_exit();
    }

    // Wait for completion
    timer_id_t timer;
    hive_timer_after(300000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_target_notified) {
        TEST_PASS("target notified when linker dies (bidirectional)");
    } else {
        TEST_FAIL("target not notified (link should be bidirectional)");
    }

    hive_exit();
}

// ============================================================================
// Test 3: Unlink prevents notification
// ============================================================================

static bool g_unlinked_received_notification = false;

static void actor_unlinks_before_death(void *args,
                                       const hive_spawn_info_t *siblings,
                                       size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t target = *(actor_id_t *)args;

    hive_link(target);
    hive_unlink(target);

    // Wait for any exit notification
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 300);

    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        g_unlinked_received_notification = true;
    }

    hive_exit();
}

static void actor_dies_after_unlink(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Give time for link/unlink
    timer_id_t timer;
    hive_timer_after(100000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_exit();
}

static void test3_unlink(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Unlink prevents notification\n");

    g_unlinked_received_notification = false;

    actor_id_t target;
    if (HIVE_FAILED(
            hive_spawn(actor_dies_after_unlink, NULL, NULL, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    actor_id_t unlinker;
    hive_spawn(actor_unlinks_before_death, NULL, &target, NULL, &unlinker);

    timer_id_t timer;
    hive_timer_after(500000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (!g_unlinked_received_notification) {
        TEST_PASS("unlink prevents exit notification");
    } else {
        TEST_FAIL("received notification after unlink");
    }

    hive_exit();
}

// ============================================================================
// Test 4: Link to invalid actor fails
// ============================================================================

static void test4_link_invalid(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Link to invalid actor fails\n");

    hive_status_t status = hive_link(HIVE_ACTOR_ID_INVALID);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_link rejects HIVE_ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("hive_link should reject HIVE_ACTOR_ID_INVALID");
    }

    status = hive_link(9999); // Non-existent actor
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_link rejects non-existent actor");
    } else {
        TEST_FAIL("hive_link should reject non-existent actor");
    }

    hive_exit();
}

// ============================================================================
// Test 5: Multiple links from one actor
// ============================================================================

static int g_multi_link_count = 0;

static void multi_link_target(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int delay_ms = *(int *)args;

    timer_id_t timer;
    hive_timer_after(delay_ms * 1000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_exit();
}

static void multi_linker(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t *targets = (actor_id_t *)args;

    // Link to all 3 targets
    for (int i = 0; i < 3; i++) {
        hive_link(targets[i]);
    }

    // Receive exit notifications
    for (int i = 0; i < 3; i++) {
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 500);
        if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
            g_multi_link_count++;
        }
    }

    hive_exit();
}

static void test5_multiple_links(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Multiple links from one actor\n");

    g_multi_link_count = 0;

    // Spawn 3 targets with different delays
    static int delays[3] = {50, 100, 150};
    static actor_id_t targets[3];

    for (int i = 0; i < 3; i++) {
        if (HIVE_FAILED(hive_spawn(multi_link_target, NULL, &delays[i], NULL,
                                   &targets[i]))) {
            TEST_FAIL("spawn target");
            hive_exit();
        }
    }

    // Spawn linker
    actor_id_t linker;
    hive_spawn(multi_linker, NULL, targets, NULL, &linker);

    // Wait for all to complete
    timer_id_t timer;
    hive_timer_after(500000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_multi_link_count == 3) {
        TEST_PASS("received all 3 exit notifications from linked actors");
    } else {
        printf("    Received %d/3 notifications\n", g_multi_link_count);
        TEST_FAIL("did not receive all notifications");
    }

    hive_exit();
}

// ============================================================================
// Test 6: Link vs Monitor difference (link is bidirectional)
// ============================================================================

static bool g_link_target_got_notification = false;
static bool g_monitor_target_got_notification = false;

static void link_target_waits(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 300);
    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        g_link_target_got_notification = true;
    }
    hive_exit();
}

static void monitor_target_waits(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 300);
    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        g_monitor_target_got_notification = true;
    }
    hive_exit();
}

static void linker_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t target = *(actor_id_t *)args;
    hive_link(target);
    hive_exit(); // Die immediately
}

static void monitor_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t target = *(actor_id_t *)args;
    uint32_t ref;
    hive_monitor(target, &ref);
    hive_exit(); // Die immediately
}

static void test6_link_vs_monitor(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Link vs Monitor (link is bidirectional)\n");

    g_link_target_got_notification = false;
    g_monitor_target_got_notification = false;

    // Test link: target should be notified when linker dies
    actor_id_t link_target;
    hive_spawn(link_target_waits, NULL, NULL, NULL, &link_target);
    actor_id_t linker;
    hive_spawn(linker_actor, NULL, &link_target, NULL, &linker);
    (void)linker;

    // Test monitor: target should NOT be notified when monitor dies
    actor_id_t monitor_target;
    hive_spawn(monitor_target_waits, NULL, NULL, NULL, &monitor_target);
    actor_id_t monitor;
    hive_spawn(monitor_actor, NULL, &monitor_target, NULL, &monitor);
    (void)monitor;

    // Wait for completion
    timer_id_t timer;
    hive_timer_after(500000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (g_link_target_got_notification) {
        TEST_PASS("link target notified when linker dies (bidirectional)");
    } else {
        TEST_FAIL("link target should be notified");
    }

    if (!g_monitor_target_got_notification) {
        TEST_PASS(
            "monitor target NOT notified when monitor dies (unidirectional)");
    } else {
        TEST_FAIL("monitor target should NOT be notified");
    }

    hive_exit();
}

// ============================================================================
// Test 7: Exit reason in link notification
// ============================================================================

static hive_exit_reason_t s_received_reason = HIVE_EXIT_NORMAL;

static void link_receiver_checks_reason(void *args,
                                        const hive_spawn_info_t *siblings,
                                        size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t target = *(actor_id_t *)args;

    hive_link(target);

    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 500);
    if (HIVE_SUCCEEDED(status) && hive_msg_is_exit(&msg)) {
        hive_exit_msg_t exit_info;
        hive_decode_exit(&msg, &exit_info);
        s_received_reason = exit_info.reason;
    }

    hive_exit();
}

static void normal_exit_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
    hive_exit(); // Normal exit
}

static void test7_exit_reason(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Exit reason in link notification\n");

    s_received_reason = (hive_exit_reason_t)99; // Invalid value

    actor_id_t target;
    hive_spawn(normal_exit_actor, NULL, NULL, NULL, &target);
    actor_id_t receiver;
    hive_spawn(link_receiver_checks_reason, NULL, &target, NULL, &receiver);

    timer_id_t timer;
    hive_timer_after(300000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    if (s_received_reason == HIVE_EXIT_NORMAL) {
        TEST_PASS("exit reason is HIVE_EXIT_NORMAL for normal exit");
    } else {
        printf("    Got reason: %d, expected: %d\n", s_received_reason,
               HIVE_EXIT_NORMAL);
        TEST_FAIL("wrong exit reason");
    }

    hive_exit();
}

// ============================================================================
// Test 8: Link to dead actor (actor that existed but has exited)
// ============================================================================

static void quickly_exiting_actor(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_exit();
}

static void test8_link_to_dead_actor(void *args,
                                     const hive_spawn_info_t *siblings,
                                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Link to dead actor\n");
    fflush(stdout);

    // Spawn an actor that exits immediately
    actor_id_t target;
    if (HIVE_FAILED(
            hive_spawn(quickly_exiting_actor, NULL, NULL, NULL, &target))) {
        TEST_FAIL("failed to spawn target actor");
        hive_exit();
    }

    // Yield to let it run and exit
    for (int i = 0; i < 5; i++) {
        hive_yield();
    }

    // Verify the actor is dead
    if (hive_actor_alive(target)) {
        TEST_FAIL("target actor should be dead by now");
        hive_exit();
    }

    // Try to link to the dead actor
    hive_status_t status = hive_link(target);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_link rejects dead actor");
    } else {
        TEST_FAIL("hive_link should reject dead actor");
    }

    hive_exit();
}

// ============================================================================
// Test 9: Link to self (should fail or be a no-op)
// ============================================================================

static void test9_link_to_self(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 9: Link to self\n");
    fflush(stdout);

    actor_id_t self = hive_self();

    hive_status_t status = hive_link(self);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_link to self is rejected");
    } else {
        // If it succeeds, it should be a no-op (shouldn't get exit notification)
        TEST_PASS("hive_link to self accepted (no-op expected)");
    }

    hive_exit();
}

// ============================================================================
// Test 10: Unlink non-linked actor
// ============================================================================

static void unlink_target_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Wait a bit then exit
    timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
    hive_exit();
}

static void test10_unlink_non_linked(void *args,
                                     const hive_spawn_info_t *siblings,
                                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 10: Unlink non-linked actor\n");
    fflush(stdout);

    // Spawn an actor but don't link to it
    actor_id_t target;
    if (HIVE_FAILED(
            hive_spawn(unlink_target_actor, NULL, NULL, NULL, &target))) {
        TEST_FAIL("spawn target");
        hive_exit();
    }

    // Try to unlink from an actor we're not linked to
    hive_status_t status = hive_unlink(target);

    // Should either fail or be a no-op
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_unlink non-linked actor fails gracefully");
    } else {
        TEST_PASS("hive_unlink non-linked actor is no-op");
    }

    // Wait for target to exit
    timer_id_t timer;
    hive_timer_after(300000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    hive_exit();
}

// ============================================================================
// Test 11: Unlink invalid actor
// ============================================================================

static void test11_unlink_invalid(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 11: Unlink invalid actor\n");
    fflush(stdout);

    hive_status_t status = hive_unlink(HIVE_ACTOR_ID_INVALID);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_unlink rejects HIVE_ACTOR_ID_INVALID");
    } else {
        TEST_FAIL("hive_unlink should reject HIVE_ACTOR_ID_INVALID");
    }

    status = hive_unlink(9999);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_unlink rejects non-existent actor");
    } else {
        TEST_FAIL("hive_unlink should reject non-existent actor");
    }

    hive_exit();
}

// ============================================================================
// Test 12: Link pool exhaustion (HIVE_LINK_ENTRY_POOL_SIZE=128)
// Each link uses 2 entries (bidirectional), so max ~64 links
// ============================================================================

static void link_pool_target_actor(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Wait for signal to exit
    hive_message_t msg;
    hive_ipc_recv(&msg, 5000);
    hive_exit();
}

static void test12_link_pool_exhaustion(void *args,
                                        const hive_spawn_info_t *siblings,
                                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 12: Link pool exhaustion (HIVE_LINK_ENTRY_POOL_SIZE=%d)\n",
           HIVE_LINK_ENTRY_POOL_SIZE);
    fflush(stdout);

    // Each link uses 2 entries (bidirectional)
    int max_links = HIVE_LINK_ENTRY_POOL_SIZE / 2;
    actor_id_t targets[HIVE_LINK_ENTRY_POOL_SIZE];
    int spawned = 0;
    int linked = 0;

    // Spawn actors and link to them until pool exhaustion
    for (int i = 0; i < max_links + 10; i++) {
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = TEST_STACK_SIZE(8 * 1024);

        actor_id_t target;
        if (HIVE_FAILED(hive_spawn(link_pool_target_actor, NULL, NULL, &cfg,
                                   &target))) {
            break;
        }
        targets[spawned++] = target;

        hive_status_t status = hive_link(target);
        if (HIVE_FAILED(status)) {
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
        hive_ipc_notify(targets[i], HIVE_TAG_NONE, &done, sizeof(done));
    }

    // Wait for cleanup
    timer_id_t timer;
    hive_timer_after(200000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Drain exit notifications
    while (hive_ipc_pending()) {
        hive_ipc_recv(&msg, 0);
    }

    hive_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *, const hive_spawn_info_t *, size_t) = {
    test1_basic_link,      test2_bidirectional,
    test3_unlink,          test4_link_invalid,
    test5_multiple_links,  test6_link_vs_monitor,
    test7_exit_reason,     test8_link_to_dead_actor,
    test9_link_to_self,    test10_unlink_non_linked,
    test11_unlink_invalid, test12_link_pool_exhaustion,
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

        actor_id_t test;
        if (HIVE_FAILED(hive_spawn(test_funcs[i], NULL, NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        hive_link(test);

        hive_message_t msg;
        hive_ipc_recv(&msg, 5000);
    }

    hive_exit();
}

int main(void) {
    printf("=== Link (hive_link) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(128 * 1024);

    actor_id_t runner;
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
