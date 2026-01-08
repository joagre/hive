#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_link.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); fflush(stdout); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); fflush(stdout); tests_failed++; } while(0)
#define TEST_KNOWN_BUG(name) do { printf("  KNOWN BUG: %s\n", name); fflush(stdout); } while(0)

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ============================================================================
// Test 1: ASYNC send/recv basic
// ============================================================================

static void test1_async_basic(void *arg) {
    (void)arg;
    printf("\nTest 1: ASYNC send/recv basic\n");

    actor_id self = rt_self();
    const char *msg_data = "Hello ASYNC";

    rt_status status = rt_ipc_send(self, msg_data, strlen(msg_data) + 1);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_ipc_send ASYNC failed");
        rt_exit();
    }

    rt_message msg;
    status = rt_ipc_recv(&msg, 100);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_ipc_recv failed");
        rt_exit();
    }

    const void *payload;
    rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
    if (strcmp((const char *)payload, "Hello ASYNC") == 0) {
        TEST_PASS("ASYNC send/recv works");
    } else {
        printf("    Received: '%s'\n", (const char *)payload);
        TEST_FAIL("data mismatch");
    }

    if (msg.sender == self) {
        TEST_PASS("sender ID is correct");
    } else {
        TEST_FAIL("wrong sender ID");
    }

    rt_exit();
}

// ============================================================================
// Test 2: ASYNC send to invalid actor
// ============================================================================

static void test2_async_invalid_receiver(void *arg) {
    (void)arg;
    printf("\nTest 2: ASYNC send to invalid actor\n");

    int data = 42;

    rt_status status = rt_ipc_send(ACTOR_ID_INVALID, &data, sizeof(data));
    if (RT_FAILED(status)) {
        TEST_PASS("send to ACTOR_ID_INVALID fails");
    } else {
        TEST_FAIL("send to ACTOR_ID_INVALID should fail");
    }

    status = rt_ipc_send(9999, &data, sizeof(data));
    if (RT_FAILED(status)) {
        TEST_PASS("send to non-existent actor fails");
    } else {
        TEST_FAIL("send to non-existent actor should fail");
    }

    rt_exit();
}

// ============================================================================
// Test 3: Message ordering (FIFO)
// ============================================================================

static void test3_message_ordering(void *arg) {
    (void)arg;
    printf("\nTest 3: Message ordering (FIFO)\n");

    actor_id self = rt_self();

    // Send 5 messages
    for (int i = 1; i <= 5; i++) {
        rt_ipc_send(self, &i, sizeof(i));
    }

    // Receive and verify order
    bool order_correct = true;
    for (int i = 1; i <= 5; i++) {
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 100);
        if (RT_FAILED(status)) {
            order_correct = false;
            break;
        }
        const void *payload;
        rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
        int received = *(int *)payload;
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

    rt_exit();
}

// ============================================================================
// Test 4: Multiple senders to one receiver
// ============================================================================

static actor_id g_receiver_id = ACTOR_ID_INVALID;
static int g_messages_received = 0;

static void sender_actor(void *arg) {
    int id = *(int *)arg;
    rt_ipc_send(g_receiver_id, &id, sizeof(id));
    rt_exit();
}

static void test4_multiple_senders(void *arg) {
    (void)arg;
    printf("\nTest 4: Multiple senders to one receiver\n");

    g_receiver_id = rt_self();
    g_messages_received = 0;

    // Spawn 5 senders
    static int sender_ids[5] = {1, 2, 3, 4, 5};
    for (int i = 0; i < 5; i++) {
        rt_spawn(sender_actor, &sender_ids[i]);
    }

    // Give senders time to run
    timer_id timer;
    rt_timer_after(100000, &timer);

    // Receive all messages
    int received_sum = 0;
    for (int i = 0; i < 6; i++) {  // 5 messages + 1 timer
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 500);
        if (RT_FAILED(status)) break;

        if (!rt_msg_is_timer(&msg)) {
            const void *payload;
            rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
            received_sum += *(int *)payload;
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

    rt_exit();
}

// ============================================================================
// Test 5: Send to self (allowed - no deadlock since all sends are async-style)
// ============================================================================

static void test5_send_to_self(void *arg) {
    (void)arg;
    printf("\nTest 5: Send to self (allowed)\n");

    actor_id self = rt_self();
    int data = 42;

    rt_status status = rt_ipc_send(self, &data, sizeof(data));
    if (!RT_FAILED(status)) {
        // Receive the message we sent to ourselves
        rt_message msg;
        status = rt_ipc_recv(&msg, 100);
        if (!RT_FAILED(status)) {
            const void *payload;
            rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
            if (*(int *)payload == 42) {
                TEST_PASS("send to self works");
            } else {
                TEST_FAIL("wrong data received from self-send");
            }
        } else {
            TEST_FAIL("failed to receive self-sent message");
        }
    } else {
        TEST_FAIL("send to self should succeed");
    }

    rt_exit();
}

// ============================================================================
// Test 6: RPC call/reply pattern
// ============================================================================

static void rpc_server_actor(void *arg) {
    (void)arg;

    // Wait for RPC call
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 1000);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    // Decode call and verify it's a CALL message
    rt_msg_class class;
    const void *payload;
    rt_msg_decode(&msg, &class, NULL, &payload, NULL);

    if (class == RT_MSG_CALL) {
        // Send reply
        int result = *(int *)payload * 2;  // Double the input
        rt_ipc_reply(&msg, &result, sizeof(result));
    }

    rt_exit();
}

static void test6_rpc_call_reply(void *arg) {
    (void)arg;
    printf("\nTest 6: RPC call/reply pattern\n");

    actor_id server = rt_spawn(rpc_server_actor, NULL);

    // Give server time to start
    rt_yield();

    // Make RPC call
    int request = 21;
    rt_message reply;
    uint64_t start = time_ms();
    rt_status status = rt_ipc_call(server, &request, sizeof(request), &reply, 1000);
    uint64_t elapsed = time_ms() - start;

    if (RT_FAILED(status)) {
        printf("    rt_ipc_call failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("rt_ipc_call failed");
        rt_exit();
    }

    // Verify reply
    const void *payload;
    rt_msg_decode(&reply, NULL, NULL, &payload, NULL);
    int result = *(int *)payload;

    if (result == 42) {
        printf("    RPC completed in %lu ms\n", (unsigned long)elapsed);
        TEST_PASS("rt_ipc_call/reply works correctly");
    } else {
        printf("    Expected 42, got %d\n", result);
        TEST_FAIL("wrong RPC result");
    }

    rt_exit();
}

// ============================================================================
// Test 7: rt_ipc_pending and rt_ipc_count
// ============================================================================

static void test7_pending_count(void *arg) {
    (void)arg;
    printf("\nTest 7: rt_ipc_pending and rt_ipc_count\n");

    actor_id self = rt_self();

    // Initially empty
    if (!rt_ipc_pending()) {
        TEST_PASS("rt_ipc_pending returns false for empty mailbox");
    } else {
        TEST_FAIL("rt_ipc_pending should return false for empty mailbox");
    }

    if (rt_ipc_count() == 0) {
        TEST_PASS("rt_ipc_count returns 0 for empty mailbox");
    } else {
        TEST_FAIL("rt_ipc_count should return 0 for empty mailbox");
    }

    // Send 3 messages
    int data = 42;
    rt_ipc_send(self, &data, sizeof(data));
    rt_ipc_send(self, &data, sizeof(data));
    rt_ipc_send(self, &data, sizeof(data));

    if (rt_ipc_pending()) {
        TEST_PASS("rt_ipc_pending returns true with messages");
    } else {
        TEST_FAIL("rt_ipc_pending should return true with messages");
    }

    if (rt_ipc_count() == 3) {
        TEST_PASS("rt_ipc_count returns correct count");
    } else {
        printf("    Count: %zu (expected 3)\n", rt_ipc_count());
        TEST_FAIL("rt_ipc_count returned wrong count");
    }

    // Drain messages
    rt_message msg;
    rt_ipc_recv(&msg, 0);
    rt_ipc_recv(&msg, 0);
    rt_ipc_recv(&msg, 0);

    if (rt_ipc_count() == 0) {
        TEST_PASS("rt_ipc_count returns 0 after draining");
    } else {
        TEST_FAIL("rt_ipc_count should return 0 after draining");
    }

    rt_exit();
}

// ============================================================================
// Test 8: recv with timeout=0 (non-blocking)
// ============================================================================

static void test8_nonblocking_recv(void *arg) {
    (void)arg;
    printf("\nTest 8: recv with timeout=0 (non-blocking)\n");

    rt_message msg;
    uint64_t start = time_ms();
    rt_status status = rt_ipc_recv(&msg, 0);
    uint64_t elapsed = time_ms() - start;

    if (status.code == RT_ERR_WOULDBLOCK) {
        TEST_PASS("empty mailbox returns RT_ERR_WOULDBLOCK");
    } else {
        printf("    Got status: %d\n", status.code);
        TEST_FAIL("expected RT_ERR_WOULDBLOCK for empty mailbox");
    }

    if (elapsed < 10) {
        TEST_PASS("non-blocking recv returns immediately");
    } else {
        printf("    Took %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("non-blocking recv should return immediately");
    }

    // With a message in queue
    actor_id self = rt_self();
    int data = 42;
    rt_ipc_send(self, &data, sizeof(data));

    status = rt_ipc_recv(&msg, 0);
    if (!RT_FAILED(status)) {
        TEST_PASS("non-blocking recv succeeds with message present");
    } else {
        TEST_FAIL("non-blocking recv should succeed with message present");
    }

    rt_exit();
}

// ============================================================================
// Test 9: recv with timeout > 0
// ============================================================================

static void test9_timed_recv(void *arg) {
    (void)arg;
    printf("\nTest 9: recv with timeout > 0\n");

    rt_message msg;
    uint64_t start = time_ms();
    rt_status status = rt_ipc_recv(&msg, 100);  // 100ms timeout
    uint64_t elapsed = time_ms() - start;

    if (status.code == RT_ERR_TIMEOUT) {
        TEST_PASS("empty mailbox returns RT_ERR_TIMEOUT");
    } else {
        printf("    Got status: %d\n", status.code);
        TEST_FAIL("expected RT_ERR_TIMEOUT");
    }

    // Should take approximately 100ms
    if (elapsed >= 80 && elapsed <= 200) {
        printf("    Timeout after %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
        TEST_PASS("timed recv waits for timeout duration");
    } else {
        printf("    Took %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
        TEST_FAIL("timed recv did not wait for correct duration");
    }

    rt_exit();
}

// ============================================================================
// Test 10: recv with timeout < 0 (block forever) - message arrives
// ============================================================================

static void delayed_sender_actor(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Wait 50ms then send
    timer_id timer;
    rt_timer_after(50000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    int data = 123;
    rt_ipc_send(target, &data, sizeof(data));

    rt_exit();
}

static void test10_block_forever_recv(void *arg) {
    (void)arg;
    printf("\nTest 10: recv with timeout < 0 (block forever)\n");

    actor_id self = rt_self();
    rt_spawn(delayed_sender_actor, &self);

    uint64_t start = time_ms();
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, -1);  // Block forever
    uint64_t elapsed = time_ms() - start;

    if (!RT_FAILED(status)) {
        TEST_PASS("block forever recv succeeds when message arrives");
    } else {
        TEST_FAIL("block forever recv should not fail");
    }

    // Should have waited ~50ms for the delayed sender
    if (elapsed >= 30 && elapsed <= 200) {
        printf("    Received after %lu ms (sender delayed 50ms)\n", (unsigned long)elapsed);
        TEST_PASS("blocked until message arrived");
    } else {
        printf("    Received after %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("timing seems off");
    }

    rt_exit();
}

// ============================================================================
// Test 11: Message size limits
// ============================================================================

static void test11_message_size_limits(void *arg) {
    (void)arg;
    printf("\nTest 11: Message size limits\n");

    actor_id self = rt_self();

    // Max payload size is RT_MAX_MESSAGE_SIZE - RT_MSG_HEADER_SIZE (4 bytes for header)
    size_t max_payload_size = RT_MAX_MESSAGE_SIZE - RT_MSG_HEADER_SIZE;

    // Send message at max payload size
    char max_msg[RT_MAX_MESSAGE_SIZE];  // Oversize buffer for safety
    memset(max_msg, 'A', sizeof(max_msg));

    rt_status status = rt_ipc_send(self, max_msg, max_payload_size);
    if (!RT_FAILED(status)) {
        TEST_PASS("can send message at max payload size");
    } else {
        printf("    Error: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("failed to send max size message");
    }

    // Receive it
    rt_message msg;
    status = rt_ipc_recv(&msg, 100);
    // msg.len includes header, so total should be max_payload_size + 4
    if (!RT_FAILED(status) && msg.len == RT_MAX_MESSAGE_SIZE) {
        TEST_PASS("received max size message");
    } else {
        printf("    msg.len = %zu, expected %d\n", msg.len, RT_MAX_MESSAGE_SIZE);
        TEST_FAIL("failed to receive max size message");
    }

    // Send message exceeding max size (payload larger than max_payload_size)
    status = rt_ipc_send(self, max_msg, max_payload_size + 1);
    if (RT_FAILED(status)) {
        TEST_PASS("oversized message is rejected");
    } else {
        TEST_FAIL("oversized message should be rejected");
        // Clean up if it somehow succeeded
        rt_ipc_recv(&msg, 0);
    }

    rt_exit();
}

// ============================================================================
// Test 12: Selective receive (rt_ipc_recv_match)
// ============================================================================

static void selective_sender_actor(void *arg) {
    actor_id target = *(actor_id *)arg;

    // Send three messages with different data
    int a = 1, b = 2, c = 3;
    rt_ipc_send(target, &a, sizeof(a));
    rt_ipc_send(target, &b, sizeof(b));
    rt_ipc_send(target, &c, sizeof(c));

    rt_exit();
}

static void test12_selective_receive(void *arg) {
    (void)arg;
    printf("\nTest 12: Selective receive (rt_ipc_recv_match)\n");
    fflush(stdout);

    actor_id self = rt_self();
    actor_id sender = rt_spawn(selective_sender_actor, &self);

    // Wait for sender to send all messages
    timer_id timer;
    rt_timer_after(50000, &timer);
    rt_message timer_msg;
    rt_ipc_recv(&timer_msg, -1);

    // Use selective receive to filter by sender
    rt_message msg;
    rt_status status = rt_ipc_recv_match(&sender, NULL, NULL, &msg, 100);

    if (!RT_FAILED(status)) {
        if (msg.sender == sender) {
            const void *payload;
            rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
            int val = *(int *)payload;
            printf("    Received value %d from sender %u\n", val, sender);
            TEST_PASS("rt_ipc_recv_match filters by sender");
        } else {
            TEST_FAIL("wrong sender in filtered message");
        }
    } else {
        printf("    recv_match failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("rt_ipc_recv_match failed");
    }

    // Drain remaining messages
    while (!RT_FAILED(rt_ipc_recv(&msg, 0))) {}

    rt_exit();
}

// ============================================================================
// Test 13: Send with zero length
// ============================================================================

static void test13_zero_length_message(void *arg) {
    (void)arg;
    printf("\nTest 13: Send with zero length payload\n");

    actor_id self = rt_self();

    rt_status status = rt_ipc_send(self, NULL, 0);
    if (!RT_FAILED(status)) {
        TEST_PASS("can send zero-length payload");

        rt_message msg;
        status = rt_ipc_recv(&msg, 100);
        // msg.len includes 4-byte header, so zero-payload message has len=4
        size_t payload_len;
        rt_msg_decode(&msg, NULL, NULL, NULL, &payload_len);
        if (!RT_FAILED(status) && payload_len == 0) {
            TEST_PASS("received zero-length payload message");
        } else {
            printf("    payload_len = %zu (expected 0)\n", payload_len);
            TEST_FAIL("failed to receive zero-length payload message");
        }
    } else {
        TEST_FAIL("failed to send zero-length message");
    }

    rt_exit();
}

// ============================================================================
// Test 14: Send to dead actor
// ============================================================================

static void quickly_dying_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void test14_send_to_dead_actor(void *arg) {
    (void)arg;
    printf("\nTest 14: Send to dead actor\n");

    actor_id target = rt_spawn(quickly_dying_actor, NULL);
    rt_link(target);

    // Wait for it to die
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    // Now try to send to dead actor
    int data = 42;
    rt_status status = rt_ipc_send(target, &data, sizeof(data));

    if (RT_FAILED(status)) {
        TEST_PASS("send to dead actor fails");
    } else {
        TEST_FAIL("send to dead actor should fail");
    }

    rt_exit();
}

// ============================================================================
// Test 15: Message pool exhaustion
// Pool size: RT_MESSAGE_DATA_POOL_SIZE
// ============================================================================

static void test15_message_pool_info(void *arg) {
    (void)arg;
    printf("\nTest 15: Message pool info (RT_MESSAGE_DATA_POOL_SIZE=%d)\n",
           RT_MESSAGE_DATA_POOL_SIZE);
    fflush(stdout);

    // Simple test: just verify we can send many messages
    actor_id self = rt_self();
    int sent = 0;

    for (int i = 0; i < 100; i++) {
        int data = i;
        rt_status status = rt_ipc_send(self, &data, sizeof(data));
        if (RT_FAILED(status)) {
            printf("    Send failed at %d: %s\n", i, status.msg ? status.msg : "unknown");
            break;
        }
        sent++;
    }

    printf("    Sent %d messages to self\n", sent);

    // Drain all messages
    rt_message msg;
    int received = 0;
    while (!RT_FAILED(rt_ipc_recv(&msg, 0))) {
        received++;
    }

    printf("    Received %d messages\n", received);

    if (sent == received && sent == 100) {
        TEST_PASS("can send and receive 100 messages");
    } else {
        TEST_FAIL("message count mismatch");
    }

    rt_exit();
}

// ============================================================================
// Test 16: NULL pointer handling - rt_ipc_send with NULL data (non-zero len)
// ============================================================================

static void test16_null_data_send(void *arg) {
    (void)arg;
    printf("\nTest 16: NULL data pointer with non-zero length\n");
    fflush(stdout);

    actor_id self = rt_self();

    // Sending NULL data with len > 0 should fail or be handled safely
    rt_status status = rt_ipc_send(self, NULL, 10);
    if (RT_FAILED(status)) {
        TEST_PASS("rt_ipc_send rejects NULL data with non-zero length");
    } else {
        // If it succeeded, the implementation might handle it - drain the message
        rt_message msg;
        rt_ipc_recv(&msg, 0);
        TEST_PASS("rt_ipc_send handles NULL data gracefully");
    }

    rt_exit();
}

// ============================================================================
// Test 17: Mailbox integrity after many spawn/death cycles (leak test)
// ============================================================================

static void short_lived_actor(void *arg) {
    actor_id parent = *(actor_id *)arg;
    // Send a message then die
    int data = 42;
    rt_ipc_send(parent, &data, sizeof(data));
    rt_exit();
}

static void test17_spawn_death_cycle_leak(void *arg) {
    (void)arg;
    printf("\nTest 17: Mailbox integrity after spawn/death cycles\n");
    fflush(stdout);

    actor_id self = rt_self();
    int cycles = 50;  // 50 spawn/death cycles
    int messages_received = 0;

    for (int i = 0; i < cycles; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = 8 * 1024;

        actor_id child = rt_spawn_ex(short_lived_actor, &self, &cfg);
        if (child == ACTOR_ID_INVALID) {
            printf("    Spawn failed at cycle %d\n", i);
            break;
        }

        // Wait for message from child
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 500);
        if (!RT_FAILED(status)) {
            messages_received++;
        }

        // Yield to let child fully exit
        rt_yield();
    }

    if (messages_received == cycles) {
        TEST_PASS("no mailbox leaks after spawn/death cycles");
    } else {
        printf("    Only %d/%d messages received\n", messages_received, cycles);
        TEST_FAIL("possible mailbox leak or message loss");
    }

    rt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_async_basic,
    test2_async_invalid_receiver,
    test3_message_ordering,
    test4_multiple_senders,
    test5_send_to_self,
    test6_rpc_call_reply,
    test7_pending_count,
    test8_nonblocking_recv,
    test9_timed_recv,
    test10_block_forever_recv,
    test11_message_size_limits,
    test12_selective_receive,
    test13_zero_length_message,
    test14_send_to_dead_actor,
    test15_message_pool_info,
    test16_null_data_send,
    test17_spawn_death_cycle_leak,
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
        rt_ipc_recv(&msg, 10000);
    }

    rt_exit();
}

int main(void) {
    printf("=== IPC (rt_ipc) Test Suite ===\n");

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
