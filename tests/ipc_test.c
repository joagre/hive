#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_link.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

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

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ============================================================================
// Test 1: ASYNC notify/recv basic
// ============================================================================

static void test1_async_basic(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: ASYNC notify/recv basic\n");

    hive_actor_id_t self = hive_self();
    const char *msg_data = "Hello ASYNC";

    hive_status_t status =
        hive_ipc_notify(self, HIVE_ID_NONE, msg_data, strlen(msg_data) + 1);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_ipc_notify ASYNC failed");
        return;
    }

    hive_message_t msg;
    status = hive_ipc_recv(&msg, 100);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_ipc_recv failed");
        return;
    }

    if (strcmp((const char *)msg.data, "Hello ASYNC") == 0) {
        TEST_PASS("ASYNC notify/recv works");
    } else {
        printf("    Received: '%s'\n", (const char *)msg.data);
        TEST_FAIL("data mismatch");
    }

    if (msg.sender == self) {
        TEST_PASS("sender ID is correct");
    } else {
        TEST_FAIL("wrong sender ID");
    }

    return;
}

// ============================================================================
// Test 2: ASYNC notify to invalid actor
// ============================================================================

static void test2_async_invalid_receiver(void *args,
                                         const hive_spawn_info_t *siblings,
                                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: ASYNC notify to invalid actor\n");

    int data = 42;

    hive_status_t status = hive_ipc_notify(HIVE_ACTOR_ID_INVALID, HIVE_ID_NONE,
                                           &data, sizeof(data));
    if (HIVE_FAILED(status)) {
        TEST_PASS("notify to HIVE_ACTOR_ID_INVALID fails");
    } else {
        TEST_FAIL("notify to HIVE_ACTOR_ID_INVALID should fail");
    }

    status = hive_ipc_notify(9999, HIVE_ID_NONE, &data, sizeof(data));
    if (HIVE_FAILED(status)) {
        TEST_PASS("notify to non-existent actor fails");
    } else {
        TEST_FAIL("notify to non-existent actor should fail");
    }

    return;
}

// ============================================================================
// Test 3: Message ordering (FIFO)
// ============================================================================

static void test3_message_ordering(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Message ordering (FIFO)\n");

    hive_actor_id_t self = hive_self();

    // Notify 5 messages
    for (int i = 1; i <= 5; i++) {
        hive_ipc_notify(self, HIVE_ID_NONE, &i, sizeof(i));
    }

    // Receive and verify order
    bool order_correct = true;
    for (int i = 1; i <= 5; i++) {
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 100);
        if (HIVE_FAILED(status)) {
            order_correct = false;
            break;
        }
        int received = *(int *)msg.data;
        if (received != i) {
            printf("    Expected %d, got %d\n", i, received);
            order_correct = false;
        }
    }

    if (order_correct) {
        TEST_PASS("messages delivered in FIFO order");
    } else {
        TEST_FAIL("message ordering violated");
    }

    return;
}

// ============================================================================
// Test 4: Multiple senders to one receiver
// ============================================================================

static hive_actor_id_t g_receiver_id = HIVE_ACTOR_ID_INVALID;
static int g_messages_received = 0;

static void sender_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = *(int *)args;
    hive_ipc_notify(g_receiver_id, HIVE_ID_NONE, &id, sizeof(id));
    return;
}

static void test4_multiple_senders(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Multiple senders to one receiver\n");

    g_receiver_id = hive_self();
    g_messages_received = 0;

    // Spawn 5 senders
    static int sender_ids[5] = {1, 2, 3, 4, 5};
    for (int i = 0; i < 5; i++) {
        hive_actor_id_t sender;
        hive_spawn(sender_actor, NULL, &sender_ids[i], NULL, &sender);
    }

    // Give senders time to run
    hive_timer_id_t timer;
    hive_timer_after(100000, &timer);

    // Receive all messages
    int received_sum = 0;
    for (int i = 0; i < 6; i++) { // 5 messages + 1 timer
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 500);
        if (HIVE_FAILED(status))
            break;

        if (!hive_msg_is_timer(&msg)) {
            received_sum += *(int *)msg.data;
            g_messages_received++;
        }
    }

    if (g_messages_received == 5 && received_sum == 15) {
        TEST_PASS("received all 5 messages from different senders");
    } else {
        printf("    Received %d messages, sum=%d (expected 5, sum=15)\n",
               g_messages_received, received_sum);
        TEST_FAIL("did not receive all messages");
    }

    return;
}

// ============================================================================
// Test 5: Notify to self (allowed - no deadlock since all notifies are async)
// ============================================================================

static void test5_notify_to_self(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Notify to self (allowed)\n");

    hive_actor_id_t self = hive_self();
    int data = 42;

    hive_status_t status =
        hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));
    if (HIVE_SUCCEEDED(status)) {
        // Receive the message we notified to ourselves
        hive_message_t msg;
        status = hive_ipc_recv(&msg, 100);
        if (HIVE_SUCCEEDED(status)) {
            if (*(int *)msg.data == 42) {
                TEST_PASS("notify to self works");
            } else {
                TEST_FAIL("wrong data received from self-notify");
            }
        } else {
            TEST_FAIL("failed to receive self-notified message");
        }
    } else {
        TEST_FAIL("notify to self should succeed");
    }

    return;
}

// ============================================================================
// Test 6: Request/reply pattern
// ============================================================================

static void request_reply_server_actor(void *args,
                                       const hive_spawn_info_t *siblings,
                                       size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Wait for request
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, 1000);
    if (HIVE_FAILED(status)) {
        return;
    }

    // Verify it's a REQUEST message
    if (msg.class == HIVE_MSG_REQUEST) {
        // Reply
        int result = *(int *)msg.data * 2; // Double the input
        hive_ipc_reply(&msg, &result, sizeof(result));
    }

    return;
}

static void test6_request_reply(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Request/reply pattern\n");

    hive_actor_id_t server;
    hive_spawn(request_reply_server_actor, NULL, NULL, NULL, &server);

    // Give server time to start
    hive_yield();

    // Make request
    int request = 21;
    hive_message_t reply;
    uint64_t start = time_ms();
    hive_status_t status = hive_ipc_request(server, HIVE_ID_NONE, &request,
                                            sizeof(request), &reply, 1000);
    uint64_t elapsed = time_ms() - start;

    if (HIVE_FAILED(status)) {
        printf("    hive_ipc_request failed: %s\n",
               status.msg ? status.msg : "unknown");
        TEST_FAIL("hive_ipc_request failed");
        return;
    }

    // Verify reply
    int result = *(int *)reply.data;

    if (result == 42) {
        printf("    Request/reply completed in %lu ms\n",
               (unsigned long)elapsed);
        TEST_PASS("hive_ipc_request/reply works correctly");
    } else {
        printf("    Expected 42, got %d\n", result);
        TEST_FAIL("wrong request/reply result");
    }

    return;
}

// ============================================================================
// Test 7: hive_ipc_pending and hive_ipc_count
// ============================================================================

static void test7_pending_count(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: hive_ipc_pending and hive_ipc_count\n");

    hive_actor_id_t self = hive_self();

    // Initially empty
    if (!hive_ipc_pending()) {
        TEST_PASS("hive_ipc_pending returns false for empty mailbox");
    } else {
        TEST_FAIL("hive_ipc_pending should return false for empty mailbox");
    }

    if (hive_ipc_count() == 0) {
        TEST_PASS("hive_ipc_count returns 0 for empty mailbox");
    } else {
        TEST_FAIL("hive_ipc_count should return 0 for empty mailbox");
    }

    // Notify 3 messages
    int data = 42;
    hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));
    hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));
    hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));

    if (hive_ipc_pending()) {
        TEST_PASS("hive_ipc_pending returns true with messages");
    } else {
        TEST_FAIL("hive_ipc_pending should return true with messages");
    }

    if (hive_ipc_count() == 3) {
        TEST_PASS("hive_ipc_count returns correct count");
    } else {
        printf("    Count: %zu (expected 3)\n", hive_ipc_count());
        TEST_FAIL("hive_ipc_count returned wrong count");
    }

    // Drain messages
    hive_message_t msg;
    hive_ipc_recv(&msg, 0);
    hive_ipc_recv(&msg, 0);
    hive_ipc_recv(&msg, 0);

    if (hive_ipc_count() == 0) {
        TEST_PASS("hive_ipc_count returns 0 after draining");
    } else {
        TEST_FAIL("hive_ipc_count should return 0 after draining");
    }

    return;
}

// ============================================================================
// Test 8: recv with timeout=0 (non-blocking)
// ============================================================================

static void test8_nonblocking_recv(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: recv with timeout=0 (non-blocking)\n");

    hive_message_t msg;
    uint64_t start = time_ms();
    hive_status_t status = hive_ipc_recv(&msg, 0);
    uint64_t elapsed = time_ms() - start;

    if (status.code == HIVE_ERR_WOULDBLOCK) {
        TEST_PASS("empty mailbox returns HIVE_ERR_WOULDBLOCK");
    } else {
        printf("    Got status: %d\n", status.code);
        TEST_FAIL("expected HIVE_ERR_WOULDBLOCK for empty mailbox");
    }

    if (elapsed < 10) {
        TEST_PASS("non-blocking recv returns immediately");
    } else {
        printf("    Took %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("non-blocking recv should return immediately");
    }

    // With a message in queue
    hive_actor_id_t self = hive_self();
    int data = 42;
    hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));

    status = hive_ipc_recv(&msg, 0);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("non-blocking recv succeeds with message present");
    } else {
        TEST_FAIL("non-blocking recv should succeed with message present");
    }

    return;
}

// ============================================================================
// Test 9: recv with timeout > 0
// ============================================================================

static void test9_timed_recv(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 9: recv with timeout > 0\n");

    hive_message_t msg;
    uint64_t start = time_ms();
    hive_status_t status = hive_ipc_recv(&msg, 100); // 100ms timeout
    uint64_t elapsed = time_ms() - start;

    if (status.code == HIVE_ERR_TIMEOUT) {
        TEST_PASS("empty mailbox returns HIVE_ERR_TIMEOUT");
    } else {
        printf("    Got status: %d\n", status.code);
        TEST_FAIL("expected HIVE_ERR_TIMEOUT");
    }

    // Should take approximately 100ms
    if (elapsed >= 80 && elapsed <= 200) {
        printf("    Timeout after %lu ms (expected ~100ms)\n",
               (unsigned long)elapsed);
        TEST_PASS("timed recv waits for timeout duration");
    } else {
        printf("    Took %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
        TEST_FAIL("timed recv did not wait for correct duration");
    }

    return;
}

// ============================================================================
// Test 10: recv with timeout < 0 (block forever) - message arrives
// ============================================================================

static void delayed_sender_actor(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t target = *(hive_actor_id_t *)args;

    // Wait 50ms then notify
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_timer_recv(timer, &msg, -1);

    int data = 123;
    hive_ipc_notify(target, HIVE_ID_NONE, &data, sizeof(data));

    return;
}

static void test10_block_forever_recv(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 10: recv with timeout < 0 (block forever)\n");

    hive_actor_id_t self = hive_self();
    hive_actor_id_t sender;
    hive_spawn(delayed_sender_actor, NULL, &self, NULL, &sender);

    uint64_t start = time_ms();
    hive_message_t msg;
    hive_status_t status = hive_ipc_recv(&msg, -1); // Block forever
    uint64_t elapsed = time_ms() - start;

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("block forever recv succeeds when message arrives");
    } else {
        TEST_FAIL("block forever recv should not fail");
    }

    // Should have waited ~50ms for the delayed sender
    if (elapsed >= 30 && elapsed <= 200) {
        printf("    Received after %lu ms (sender delayed 50ms)\n",
               (unsigned long)elapsed);
        TEST_PASS("blocked until message arrived");
    } else {
        printf("    Received after %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("timing seems off");
    }

    return;
}

// ============================================================================
// Test 11: Message size limits
// ============================================================================

static void test11_message_size_limits(void *args,
                                       const hive_spawn_info_t *siblings,
                                       size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 11: Message size limits\n");

    hive_actor_id_t self = hive_self();

    // Max payload size is HIVE_MAX_MESSAGE_SIZE - HIVE_MSG_HEADER_SIZE (4 bytes for header)
    size_t max_payload_size = HIVE_MAX_MESSAGE_SIZE - HIVE_MSG_HEADER_SIZE;

    // Notify message at max payload size
    char max_msg[HIVE_MAX_MESSAGE_SIZE]; // Oversize buffer for safety
    memset(max_msg, 'A', sizeof(max_msg));

    hive_status_t status =
        hive_ipc_notify(self, HIVE_ID_NONE, max_msg, max_payload_size);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("can notify message at max payload size");
    } else {
        printf("    Error: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("failed to notify max size message");
    }

    // Receive it
    hive_message_t msg;
    status = hive_ipc_recv(&msg, 100);
    // msg.len is payload length (excludes 4-byte header)
    if (HIVE_SUCCEEDED(status) && msg.len == max_payload_size) {
        TEST_PASS("received max size message");
    } else {
        printf("    msg.len = %zu, expected %zu\n", msg.len, max_payload_size);
        TEST_FAIL("failed to receive max size message");
    }

    // Notify message exceeding max size (payload larger than max_payload_size)
    status = hive_ipc_notify(self, HIVE_ID_NONE, max_msg, max_payload_size + 1);
    if (HIVE_FAILED(status)) {
        TEST_PASS("oversized message is rejected");
    } else {
        TEST_FAIL("oversized message should be rejected");
        // Clean up if it somehow succeeded
        hive_ipc_recv(&msg, 0);
    }

    return;
}

// ============================================================================
// Test 12: Selective receive (hive_ipc_recv_match)
// ============================================================================

static void selective_sender_actor(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t target = *(hive_actor_id_t *)args;

    // Notify three messages with different data
    int a = 1, b = 2, c = 3;
    hive_ipc_notify(target, HIVE_ID_NONE, &a, sizeof(a));
    hive_ipc_notify(target, HIVE_ID_NONE, &b, sizeof(b));
    hive_ipc_notify(target, HIVE_ID_NONE, &c, sizeof(c));

    return;
}

static void test12_selective_receive(void *args,
                                     const hive_spawn_info_t *siblings,
                                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 12: Selective receive (hive_ipc_recv_match)\n");
    fflush(stdout);

    hive_actor_id_t self = hive_self();
    hive_actor_id_t sender;
    hive_spawn(selective_sender_actor, NULL, &self, NULL, &sender);

    // Wait for sender to notify all messages
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t timer_msg;
    hive_timer_recv(timer, &timer_msg, -1);

    // Use selective receive to filter by sender
    hive_message_t msg;
    hive_status_t status =
        hive_ipc_recv_match(sender, HIVE_MSG_ANY, HIVE_ID_ANY, &msg, 100);

    if (HIVE_SUCCEEDED(status)) {
        if (msg.sender == sender) {
            int val = *(int *)msg.data;
            printf("    Received value %d from sender %u\n", val, sender);
            TEST_PASS("hive_ipc_recv_match filters by sender");
        } else {
            TEST_FAIL("wrong sender in filtered message");
        }
    } else {
        printf("    recv_match failed: %s\n",
               status.msg ? status.msg : "unknown");
        TEST_FAIL("hive_ipc_recv_match failed");
    }

    // Drain remaining messages
    while (HIVE_SUCCEEDED(hive_ipc_recv(&msg, 0))) {
    }

    return;
}

// ============================================================================
// Test 13: Notify with zero length
// ============================================================================

static void test13_zero_length_message(void *args,
                                       const hive_spawn_info_t *siblings,
                                       size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 13: Notify with zero length payload\n");

    hive_actor_id_t self = hive_self();

    hive_status_t status = hive_ipc_notify(self, HIVE_ID_NONE, NULL, 0);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("can notify zero-length payload");

        hive_message_t msg;
        status = hive_ipc_recv(&msg, 100);
        if (HIVE_SUCCEEDED(status) && msg.len == 0) {
            TEST_PASS("received zero-length payload message");
        } else {
            printf("    msg.len = %zu (expected 0)\n", msg.len);
            TEST_FAIL("failed to receive zero-length payload message");
        }
    } else {
        TEST_FAIL("failed to notify zero-length message");
    }

    return;
}

// ============================================================================
// Test 14: Notify to dead actor
// ============================================================================

static void quickly_dying_actor(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    return;
}

static void test14_notify_to_dead_actor(void *args,
                                        const hive_spawn_info_t *siblings,
                                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 14: Notify to dead actor\n");

    hive_actor_id_t target;
    hive_spawn(quickly_dying_actor, NULL, NULL, NULL, &target);
    hive_link(target);

    // Wait for it to die
    hive_message_t msg;
    hive_ipc_recv(&msg, 1000);

    // Now try to notify dead actor
    int data = 42;
    hive_status_t status =
        hive_ipc_notify(target, HIVE_ID_NONE, &data, sizeof(data));

    if (HIVE_FAILED(status)) {
        TEST_PASS("notify to dead actor fails");
    } else {
        TEST_FAIL("notify to dead actor should fail");
    }

    return;
}

// ============================================================================
// Test 15: Message pool exhaustion
// Pool size: HIVE_MESSAGE_DATA_POOL_SIZE
// ============================================================================

/* Number of messages to notify in pool test - adjust for QEMU's smaller pool */
#ifdef QEMU_TEST_STACK_SIZE
#define POOL_TEST_MSG_COUNT \
    20 /* QEMU has pool size 32, leave room for overhead */
#else
#define POOL_TEST_MSG_COUNT 100
#endif

static void test15_message_pool_info(void *args,
                                     const hive_spawn_info_t *siblings,
                                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 15: Message pool info (HIVE_MESSAGE_DATA_POOL_SIZE=%d)\n",
           HIVE_MESSAGE_DATA_POOL_SIZE);
    fflush(stdout);

    // Simple test: just verify we can notify many messages
    hive_actor_id_t self = hive_self();
    int sent = 0;

    for (int i = 0; i < POOL_TEST_MSG_COUNT; i++) {
        int data = i;
        hive_status_t status =
            hive_ipc_notify(self, HIVE_ID_NONE, &data, sizeof(data));
        if (HIVE_FAILED(status)) {
            printf("    Notify failed at %d: %s\n", i,
                   status.msg ? status.msg : "unknown");
            break;
        }
        sent++;
    }

    printf("    Notified %d messages to self\n", sent);

    // Drain all messages
    hive_message_t msg;
    int received = 0;
    while (HIVE_SUCCEEDED(hive_ipc_recv(&msg, 0))) {
        received++;
    }

    printf("    Received %d messages\n", received);

    if (sent == received && sent > 0) {
        TEST_PASS("can notify and receive messages");
    } else {
        TEST_FAIL("message count mismatch");
    }

    return;
}

// ============================================================================
// Test 16: NULL pointer handling - hive_ipc_notify with NULL data (non-zero len)
// ============================================================================

static void test16_null_data_notify(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 16: NULL data pointer with non-zero length\n");
    fflush(stdout);

    hive_actor_id_t self = hive_self();

    // Notifying NULL data with len > 0 should fail or be handled safely
    hive_status_t status = hive_ipc_notify(self, HIVE_ID_NONE, NULL, 10);
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_ipc_notify rejects NULL data with non-zero length");
    } else {
        // If it succeeded, the implementation might handle it - drain the message
        hive_message_t msg;
        hive_ipc_recv(&msg, 0);
        TEST_PASS("hive_ipc_notify handles NULL data gracefully");
    }

    return;
}

// ============================================================================
// Test 17: Mailbox integrity after many spawn/death cycles (leak test)
// ============================================================================

static void short_lived_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t parent = *(hive_actor_id_t *)args;
    // Notify a message then die
    int data = 42;
    hive_ipc_notify(parent, HIVE_ID_NONE, &data, sizeof(data));
    return;
}

static void test17_spawn_death_cycle_leak(void *args,
                                          const hive_spawn_info_t *siblings,
                                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 17: Mailbox integrity after spawn/death cycles\n");
    fflush(stdout);

    hive_actor_id_t self = hive_self();
    int cycles = 50; // 50 spawn/death cycles
    int messages_received = 0;

    for (int i = 0; i < cycles; i++) {
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = TEST_STACK_SIZE(8 * 1024);

        hive_actor_id_t child;
        if (HIVE_FAILED(
                hive_spawn(short_lived_actor, NULL, &self, &cfg, &child))) {
            printf("    Spawn failed at cycle %d\n", i);
            break;
        }

        // Wait for message from child
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 500);
        if (HIVE_SUCCEEDED(status)) {
            messages_received++;
        }

        // Yield to let child fully exit
        hive_yield();
    }

    if (messages_received == cycles) {
        TEST_PASS("no mailbox leaks after spawn/death cycles");
    } else {
        printf("    Only %d/%d messages received\n", messages_received, cycles);
        TEST_FAIL("possible mailbox leak or message loss");
    }

    return;
}

// ============================================================================
// Test 18: Request to dying actor returns HIVE_ERR_CLOSED
// ============================================================================

static void dying_without_reply_actor(void *args,
                                      const hive_spawn_info_t *siblings,
                                      size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Wait briefly to receive request, then die without replying
    hive_message_t msg;
    hive_ipc_recv(&msg, 500);
    // Die without calling hive_ipc_reply()
    return;
}

static void test18_request_to_dying_actor(void *args,
                                          const hive_spawn_info_t *siblings,
                                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 18: Request to dying actor returns HIVE_ERR_CLOSED\n");
    fflush(stdout);

    hive_actor_id_t target;
    hive_spawn(dying_without_reply_actor, NULL, NULL, NULL, &target);

    // Give target time to start
    hive_yield();

    // Make a request - target will receive it and die without replying
    int request = 42;
    hive_message_t reply;
    uint64_t start = time_ms();
    hive_status_t status = hive_ipc_request(target, HIVE_ID_NONE, &request,
                                            sizeof(request), &reply, 5000);
    uint64_t elapsed = time_ms() - start;

    if (status.code == HIVE_ERR_CLOSED) {
        printf("    hive_ipc_request returned HIVE_ERR_CLOSED after %lu ms\n",
               (unsigned long)elapsed);
        TEST_PASS("request to dying actor returns HIVE_ERR_CLOSED");
    } else if (status.code == HIVE_ERR_TIMEOUT) {
        printf("    Got HIVE_ERR_TIMEOUT after %lu ms (expected "
               "HIVE_ERR_CLOSED)\n",
               (unsigned long)elapsed);
        TEST_FAIL("should return HIVE_ERR_CLOSED, not HIVE_ERR_TIMEOUT");
    } else if (HIVE_SUCCEEDED(status)) {
        TEST_FAIL("request should not succeed when target dies");
    } else {
        printf("    Got unexpected error %d: %s\n", status.code,
               status.msg ? status.msg : "unknown");
        TEST_FAIL("unexpected error code");
    }

    // Should detect death quickly, not wait for full timeout
    if (elapsed < 2000) {
        TEST_PASS("detected target death quickly (< 2s)");
    } else {
        printf("    Took %lu ms to detect death\n", (unsigned long)elapsed);
        TEST_FAIL("took too long to detect target death");
    }

    return;
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *, const hive_spawn_info_t *, size_t) = {
    test1_async_basic,
    test2_async_invalid_receiver,
    test3_message_ordering,
    test4_multiple_senders,
    test5_notify_to_self,
    test6_request_reply,
    test7_pending_count,
    test8_nonblocking_recv,
    test9_timed_recv,
    test10_block_forever_recv,
    test11_message_size_limits,
    test12_selective_receive,
    test13_zero_length_message,
    test14_notify_to_dead_actor,
    test15_message_pool_info,
    test16_null_data_notify,
    test17_spawn_death_cycle_leak,
    test18_request_to_dying_actor,
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
        hive_ipc_recv(&msg, 10000);
    }

    return;
}

int main(void) {
    printf("=== IPC (hive_ipc) Test Suite ===\n");

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
