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

    rt_status status = rt_ipc_send(self, msg_data, strlen(msg_data) + 1, IPC_ASYNC);
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

    if (strcmp((const char *)msg.data, "Hello ASYNC") == 0) {
        TEST_PASS("ASYNC send/recv works");
    } else {
        printf("    Received: '%s'\n", (const char *)msg.data);
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

    rt_status status = rt_ipc_send(ACTOR_ID_INVALID, &data, sizeof(data), IPC_ASYNC);
    if (RT_FAILED(status)) {
        TEST_PASS("send to ACTOR_ID_INVALID fails");
    } else {
        TEST_FAIL("send to ACTOR_ID_INVALID should fail");
    }

    status = rt_ipc_send(9999, &data, sizeof(data), IPC_ASYNC);
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
        rt_ipc_send(self, &i, sizeof(i), IPC_ASYNC);
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

    rt_exit();
}

// ============================================================================
// Test 4: Multiple senders to one receiver
// ============================================================================

static actor_id g_receiver_id = ACTOR_ID_INVALID;
static int g_messages_received = 0;

static void sender_actor(void *arg) {
    int id = *(int *)arg;
    rt_ipc_send(g_receiver_id, &id, sizeof(id), IPC_ASYNC);
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

        if (msg.sender != RT_SENDER_TIMER) {
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

    rt_exit();
}

// ============================================================================
// Test 5: SYNC send to self should fail (deadlock prevention)
// ============================================================================

static void test5_sync_self_deadlock(void *arg) {
    (void)arg;
    printf("\nTest 5: SYNC send to self should fail (deadlock prevention)\n");

    actor_id self = rt_self();
    int data = 42;

    rt_status status = rt_ipc_send(self, &data, sizeof(data), IPC_SYNC);
    if (RT_FAILED(status)) {
        TEST_PASS("SYNC send to self is rejected (deadlock prevention)");
    } else {
        TEST_FAIL("SYNC send to self should be rejected");
    }

    rt_exit();
}

// ============================================================================
// Test 6: SYNC send/recv with release
// ============================================================================

static void sync_receiver_actor(void *arg) {
    actor_id sender = *(actor_id *)arg;

    // Wait for SYNC message
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 1000);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    // Verify data
    if (strcmp((const char *)msg.data, "SYNC_DATA") == 0) {
        // Release the message - this unblocks the sender
        rt_ipc_release(&msg);
    }

    // Notify sender we're done
    int done = 1;
    rt_ipc_send(sender, &done, sizeof(done), IPC_ASYNC);

    rt_exit();
}

static void test6_sync_send_release(void *arg) {
    (void)arg;
    printf("\nTest 6: SYNC send/recv with release\n");

    actor_id self = rt_self();
    actor_id receiver = rt_spawn(sync_receiver_actor, &self);

    // Give receiver time to start
    rt_yield();

    // Send SYNC message
    const char *data = "SYNC_DATA";
    uint64_t start = time_ms();
    rt_status status = rt_ipc_send(receiver, data, strlen(data) + 1, IPC_SYNC);
    uint64_t elapsed = time_ms() - start;

    if (RT_FAILED(status)) {
        TEST_FAIL("SYNC send failed");
        rt_exit();
    }

    // We should have blocked until receiver called rt_ipc_release
    if (elapsed < 1000) {  // Should be fast if release works
        TEST_PASS("SYNC send blocked until release");
    } else {
        printf("    Send took %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("SYNC send did not unblock after release");
    }

    // Wait for receiver to confirm
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

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
    rt_ipc_send(self, &data, sizeof(data), IPC_ASYNC);
    rt_ipc_send(self, &data, sizeof(data), IPC_ASYNC);
    rt_ipc_send(self, &data, sizeof(data), IPC_ASYNC);

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
    rt_ipc_send(self, &data, sizeof(data), IPC_ASYNC);

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
    rt_ipc_send(target, &data, sizeof(data), IPC_ASYNC);

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

    // Send message at max size
    char max_msg[RT_MAX_MESSAGE_SIZE];
    memset(max_msg, 'A', RT_MAX_MESSAGE_SIZE);

    rt_status status = rt_ipc_send(self, max_msg, RT_MAX_MESSAGE_SIZE, IPC_ASYNC);
    if (!RT_FAILED(status)) {
        TEST_PASS("can send message at RT_MAX_MESSAGE_SIZE");
    } else {
        printf("    Error: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("failed to send max size message");
    }

    // Receive it
    rt_message msg;
    status = rt_ipc_recv(&msg, 100);
    if (!RT_FAILED(status) && msg.len == RT_MAX_MESSAGE_SIZE) {
        TEST_PASS("received max size message");
    } else {
        TEST_FAIL("failed to receive max size message");
    }

    // Send message exceeding max size
    char oversized[RT_MAX_MESSAGE_SIZE + 1];
    memset(oversized, 'B', sizeof(oversized));

    status = rt_ipc_send(self, oversized, RT_MAX_MESSAGE_SIZE + 1, IPC_ASYNC);
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
// Test 12: SYNC auto-release on next recv
// Documentation: rt_types.h says "SYNC: valid until rt_ipc_release() (next recv auto-releases)"
// ============================================================================

static void sync_sender_for_auto_release(void *arg) {
    actor_id target = *(actor_id *)arg;

    const char *data = "AUTO_RELEASE_TEST";
    rt_status status = rt_ipc_send(target, data, strlen(data) + 1, IPC_SYNC);
    if (!RT_FAILED(status)) {
        // Send notification that we were unblocked
        int done = 1;
        rt_ipc_send(target, &done, sizeof(done), IPC_ASYNC);
    }

    rt_exit();
}

static void test12_sync_auto_release(void *arg) {
    (void)arg;
    printf("\nTest 12: SYNC auto-release on next recv\n");
    fflush(stdout);

    actor_id self = rt_self();
    actor_id sender = rt_spawn(sync_sender_for_auto_release, &self);
    rt_link(sender);

    // First recv gets the SYNC message
    rt_message msg1;
    rt_status status = rt_ipc_recv(&msg1, 1000);
    if (RT_FAILED(status)) {
        printf("    First recv failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("failed to receive SYNC message");
        rt_exit();
    }

    // Don't call rt_ipc_release - the next recv should auto-release
    // According to rt_types.h: "SYNC: valid until rt_ipc_release() (next recv auto-releases)"
    rt_message msg2;
    status = rt_ipc_recv(&msg2, 1000);

    if (!RT_FAILED(status)) {
        if (rt_is_exit_msg(&msg2)) {
            // Got exit notification instead of the "done" message
            printf("    Received exit notification instead of 'done' message\n");
            TEST_FAIL("sender died before sending confirmation (auto-release bug)");
        } else {
            TEST_PASS("next recv auto-released previous SYNC message");
        }
    } else {
        printf("    Second recv failed: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("auto-release did not unblock SYNC sender");
    }

    rt_exit();
}

// ============================================================================
// Test 13: Send with zero length
// ============================================================================

static void test13_zero_length_message(void *arg) {
    (void)arg;
    printf("\nTest 13: Send with zero length\n");

    actor_id self = rt_self();

    rt_status status = rt_ipc_send(self, NULL, 0, IPC_ASYNC);
    if (!RT_FAILED(status)) {
        TEST_PASS("can send zero-length message");

        rt_message msg;
        status = rt_ipc_recv(&msg, 100);
        if (!RT_FAILED(status) && msg.len == 0) {
            TEST_PASS("received zero-length message");
        } else {
            TEST_FAIL("failed to receive zero-length message");
        }
    } else {
        TEST_FAIL("failed to send zero-length message");
    }

    rt_exit();
}

// ============================================================================
// Test 14: SYNC send to dead actor
// ============================================================================

static void quickly_dying_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void test14_sync_to_dead_actor(void *arg) {
    (void)arg;
    printf("\nTest 14: SYNC send to dead actor\n");

    actor_id target = rt_spawn(quickly_dying_actor, NULL);
    rt_link(target);

    // Wait for it to die
    rt_message msg;
    rt_ipc_recv(&msg, 1000);

    // Now try to send SYNC to dead actor
    int data = 42;
    rt_status status = rt_ipc_send(target, &data, sizeof(data), IPC_SYNC);

    if (RT_FAILED(status)) {
        TEST_PASS("SYNC send to dead actor fails");
    } else {
        TEST_FAIL("SYNC send to dead actor should fail");
    }

    rt_exit();
}

// ============================================================================
// Test 15: Sync buffer pool exhaustion
// Pool size: RT_SYNC_BUFFER_POOL_SIZE = 64
// ============================================================================

static volatile int g_sync_senders_blocked = 0;
static volatile int g_sync_senders_failed = 0;

static void sync_sender_exhaustion(void *arg) {
    actor_id target = *(actor_id *)arg;
    const char *data = "SYNC_EXHAUST";

    rt_status status = rt_ipc_send(target, data, strlen(data) + 1, IPC_SYNC);
    if (RT_FAILED(status)) {
        g_sync_senders_failed++;
    } else {
        g_sync_senders_blocked++;
    }

    rt_exit();
}

static void test15_sync_pool_exhaustion(void *arg) {
    (void)arg;
    printf("\nTest 15: Sync buffer pool exhaustion (RT_SYNC_BUFFER_POOL_SIZE=%d)\n",
           RT_SYNC_BUFFER_POOL_SIZE);
    fflush(stdout);

    g_sync_senders_blocked = 0;
    g_sync_senders_failed = 0;

    actor_id self = rt_self();

    // Spawn more senders than sync buffer pool size
    // Use malloc stacks to avoid arena exhaustion
    int num_senders = RT_SYNC_BUFFER_POOL_SIZE + 10;
    actor_id senders[RT_SYNC_BUFFER_POOL_SIZE + 10];

    for (int i = 0; i < num_senders; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.malloc_stack = true;
        cfg.stack_size = 8 * 1024;  // Small stacks to fit more actors

        senders[i] = rt_spawn_ex(sync_sender_exhaustion, &self, &cfg);
        if (senders[i] == ACTOR_ID_INVALID) {
            num_senders = i;
            break;
        }
    }

    printf("    Spawned %d sender actors\n", num_senders);

    // Yield to let senders attempt to send
    for (int i = 0; i < num_senders; i++) {
        rt_yield();
    }

    // Some should have failed with NOMEM (if we spawned more than pool size)
    if (g_sync_senders_failed > 0) {
        printf("    %d senders got RT_ERR_NOMEM (pool exhausted)\n", g_sync_senders_failed);
        TEST_PASS("sync buffer pool exhaustion returns RT_ERR_NOMEM");
    } else if (num_senders > RT_SYNC_BUFFER_POOL_SIZE) {
        printf("    All %d senders blocked, expected some NOMEM\n", num_senders);
        TEST_FAIL("expected some senders to fail with NOMEM");
    } else {
        // We couldn't spawn enough actors to exhaust the pool (actor table limit)
        printf("    Only %d senders (limited by actor table), pool size is %d\n",
               num_senders, RT_SYNC_BUFFER_POOL_SIZE);
        printf("    %d blocked, %d failed\n", g_sync_senders_blocked, g_sync_senders_failed);
        TEST_PASS("sync pool handles concurrent senders correctly");
    }

    // Release all blocked senders by receiving their messages
    rt_message msg;
    while (rt_ipc_pending()) {
        rt_status status = rt_ipc_recv(&msg, 0);
        if (!RT_FAILED(status)) {
            rt_ipc_release(&msg);
        }
    }

    // Wait for senders to exit
    timer_id timer;
    rt_timer_after(100000, &timer);
    rt_ipc_recv(&msg, -1);

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
    test5_sync_self_deadlock,
    test6_sync_send_release,
    test7_pending_count,
    test8_nonblocking_recv,
    test9_timed_recv,
    test10_block_forever_recv,
    test11_message_size_limits,
    test12_sync_auto_release,
    test13_zero_length_message,
    test14_sync_to_dead_actor,
    test15_sync_pool_exhaustion,
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
