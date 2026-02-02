#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

static bool test_passed = false;
static int messages_received = 0;

// =============================================================================
// Test 1: pool_block=false (default) returns HIVE_ERR_NOMEM
// =============================================================================

static void receiver_no_block(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Just block forever - don't process messages
    hive_message_t msg;
    hive_ipc_recv(&msg, -1);
    return;
}

static void sender_no_block(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t receiver = *(actor_id_t *)args;

    // Verify default is non-blocking
    if (hive_pool_get_block()) {
        printf("    FAIL: default pool_block should be false\n");
        return;
    }

    // Notify messages until pool is exhausted
    int sent = 0;
    int data = 0;

    while (true) {
        hive_status_t s =
            hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));
        if (HIVE_FAILED(s)) {
            if (s.code == HIVE_ERR_NOMEM) {
                printf(
                    "    Pool exhausted after %d messages (HIVE_ERR_NOMEM)\n",
                    sent);
                test_passed = true;
            } else {
                printf("    FAIL: unexpected error %d\n", s.code);
            }
            break;
        }
        sent++;
        data++;

        // Safety limit
        if (sent > HIVE_MAILBOX_ENTRY_POOL_SIZE + 100) {
            printf("    FAIL: sent too many without exhausting pool\n");
            break;
        }
    }

    return;
}

// =============================================================================
// Test 2: pool_block=true blocks until pool available
// =============================================================================

static actor_id_t blocking_sender_id;

static void consumer_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Consume messages as they arrive to free pool space for notifier
    // Use timeout=0 for non-blocking receive, yield between attempts
    printf("    Consumer: starting to consume messages...\n");

    int empty_cycles = 0;
    while (empty_cycles < 10) { // Exit after 10 consecutive empty receives
        hive_message_t msg;
        hive_status_t s = hive_ipc_recv(&msg, 0); // Non-blocking
        if (HIVE_SUCCEEDED(s)) {
            messages_received++;
            empty_cycles = 0; // Reset counter on successful receive
        } else {
            empty_cycles++;
        }
        hive_yield(); // Let notifier run
    }

    printf("    Consumer: consumed %d messages\n", messages_received);
    return;
}

static void sender_blocking(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t receiver = *(actor_id_t *)args;

    // Verify pool_block is set from config
    if (!hive_pool_get_block()) {
        printf("    FAIL: pool_block should be true (from config)\n");
        return;
    }

    printf("    Blocking sender: notifying messages until pool exhausted...\n");

    // Notify messages - should block when pool exhausted
    int sent = 0;
    int data = 0;

    // Notify many messages - will eventually block when pool exhausted
    // Consumer will free space, allowing more notifies
    for (int i = 0; i < HIVE_MESSAGE_DATA_POOL_SIZE + 20; i++) {
        hive_status_t s =
            hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));
        if (HIVE_FAILED(s)) {
            // Should not get NOMEM with blocking enabled!
            printf("    FAIL: got error %d with pool_block=true\n", s.code);
            return;
        }
        sent++;
        data++;
    }

    printf("    Blocking sender: notified %d messages (blocking worked!)\n",
           sent);
    test_passed = true;
    return;
}

// =============================================================================
// Test 3: hive_pool_set_block runtime API
// =============================================================================

static void runtime_api_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Test initial value (should be false from default config)
    if (hive_pool_get_block() != false) {
        printf("    FAIL: initial pool_block should be false\n");
        return;
    }

    // Test HIVE_POOL_BLOCK
    hive_pool_set_block(HIVE_POOL_BLOCK);
    if (hive_pool_get_block() != true) {
        printf("    FAIL: pool_block should be true after HIVE_POOL_BLOCK\n");
        return;
    }

    // Test HIVE_POOL_NO_BLOCK
    hive_pool_set_block(HIVE_POOL_NO_BLOCK);
    if (hive_pool_get_block() != false) {
        printf(
            "    FAIL: pool_block should be false after HIVE_POOL_NO_BLOCK\n");
        return;
    }

    // Test HIVE_POOL_DEFAULT (should restore spawn default = false)
    hive_pool_set_block(HIVE_POOL_BLOCK);
    hive_pool_set_block(HIVE_POOL_DEFAULT);
    if (hive_pool_get_block() != false) {
        printf(
            "    FAIL: pool_block should be false after HIVE_POOL_DEFAULT\n");
        return;
    }

    printf("    Runtime API test passed\n");
    test_passed = true;
    return;
}

// =============================================================================
// Test 4: Reserved entries allow timers under pool exhaustion
// =============================================================================

static void timer_under_exhaustion_sender(void *args,
                                          const hive_spawn_info_t *siblings,
                                          size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t receiver = *(actor_id_t *)args;

    // Fill pool until exhausted (leave only reserved entries)
    int sent = 0;
    int data = 0;
    while (true) {
        hive_status_t s =
            hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));
        if (HIVE_FAILED(s)) {
            break; // Pool exhausted for user messages
        }
        sent++;
        data++;
        if (sent > HIVE_MAILBOX_ENTRY_POOL_SIZE + 100) {
            printf("    FAIL: sent too many without exhausting pool\n");
            return;
        }
    }

    printf("    Pool exhausted after %d user messages\n", sent);
    printf("    Reserved entries: %d\n", HIVE_RESERVED_SYSTEM_ENTRIES);

    // Now try to use a timer - should work because timers use reserved entries
    timer_id_t timer;
    hive_status_t s = hive_timer_after(10000, &timer); // 10ms
    if (HIVE_FAILED(s)) {
        printf("    FAIL: timer creation failed: %s\n", HIVE_ERR_STR(s));
        return;
    }

    printf("    Timer created successfully under pool exhaustion\n");

    // Wait for timer with timeout
    hive_message_t msg;
    s = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, 1000);
    if (HIVE_SUCCEEDED(s)) {
        printf("    Timer fired successfully!\n");
        test_passed = true;
    } else {
        printf("    FAIL: timer receive failed: %s\n", HIVE_ERR_STR(s));
    }

    return;
}

static void timer_test_receiver(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Just block - don't process messages (let pool stay full)
    hive_message_t msg;
    hive_ipc_recv(&msg, -1);
    return;
}

// =============================================================================
// Main
// =============================================================================

int main(void) {
    printf("=== Pool Block Tests ===\n\n");
    int passed = 0;
    int failed = 0;

    // Test 1: Default non-blocking behavior
    printf("Test 1: pool_block=false returns HIVE_ERR_NOMEM\n");
    test_passed = false;
    hive_init();
    {
        actor_id_t receiver;
        hive_spawn(receiver_no_block, NULL, NULL, NULL, &receiver);

        actor_id_t sender;
        hive_spawn(sender_no_block, NULL, &receiver, NULL, &sender);

        hive_run();
    }
    hive_cleanup();

    if (test_passed) {
        printf("  PASS\n\n");
        passed++;
    } else {
        printf("  FAIL\n\n");
        failed++;
    }

    // Test 2: Blocking behavior
    printf("Test 2: pool_block=true blocks until pool available\n");
    test_passed = false;
    messages_received = 0;
    hive_init();
    {
        // Spawn consumer first
        actor_id_t consumer;
        hive_spawn(consumer_actor, NULL, NULL, NULL, &consumer);

        // Spawn blocking sender with pool_block=true
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.pool_block = true;

        actor_id_t sender;
        hive_spawn(sender_blocking, NULL, &consumer, &cfg, &sender);
        blocking_sender_id = sender;

        hive_run();
    }
    hive_cleanup();

    if (test_passed) {
        printf("  PASS\n\n");
        passed++;
    } else {
        printf("  FAIL\n\n");
        failed++;
    }

    // Test 3: Runtime API
    printf("Test 3: hive_pool_set_block/get_block runtime API\n");
    test_passed = false;
    hive_init();
    {
        actor_id_t actor;
        hive_spawn(runtime_api_actor, NULL, NULL, NULL, &actor);
        hive_run();
    }
    hive_cleanup();

    if (test_passed) {
        printf("  PASS\n\n");
        passed++;
    } else {
        printf("  FAIL\n\n");
        failed++;
    }

    // Test 4: Reserved entries for timers under pool exhaustion
    printf("Test 4: Reserved entries allow timers under pool exhaustion\n");
    test_passed = false;
    hive_init();
    {
        actor_id_t receiver;
        hive_spawn(timer_test_receiver, NULL, NULL, NULL, &receiver);

        actor_id_t sender;
        hive_spawn(timer_under_exhaustion_sender, NULL, &receiver, NULL,
                   &sender);

        hive_run();
    }
    hive_cleanup();

    if (test_passed) {
        printf("  PASS\n\n");
        passed++;
    } else {
        printf("  FAIL\n\n");
        failed++;
    }

    // Summary
    printf("=== Results ===\n");
    printf("Passed: %d\n", passed);
    printf("Failed: %d\n", failed);

    return failed > 0 ? 1 : 0;
}
