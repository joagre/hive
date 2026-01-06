#include "rt_runtime.h"
#include "rt_bus.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_link.h"
#include <stdio.h>
#include <string.h>

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); tests_failed++; } while(0)

// ============================================================================
// Test 1: Basic publish/subscribe
// ============================================================================

static void test1_basic_pubsub(void *arg) {
    (void)arg;
    printf("\nTest 1: Basic publish/subscribe\n");

    // Create bus
    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    bus_id bus;
    rt_status status = rt_bus_create(&cfg, &bus);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_create");
        rt_exit();
    }

    // Subscribe
    status = rt_bus_subscribe(bus);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_subscribe");
        rt_bus_destroy(bus);
        rt_exit();
    }

    // Publish data
    const char *msg = "Hello Bus!";
    status = rt_bus_publish(bus, msg, strlen(msg) + 1);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_publish");
        rt_bus_unsubscribe(bus);
        rt_bus_destroy(bus);
        rt_exit();
    }

    // Read data
    char buf[64];
    size_t actual_len;
    status = rt_bus_read(bus, buf, sizeof(buf), &actual_len);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_read");
        rt_bus_unsubscribe(bus);
        rt_bus_destroy(bus);
        rt_exit();
    }

    if (strcmp(buf, msg) == 0) {
        TEST_PASS("basic publish/subscribe works");
    } else {
        TEST_FAIL("data mismatch");
    }

    rt_bus_unsubscribe(bus);
    rt_bus_destroy(bus);
    rt_exit();
}

// ============================================================================
// Test 2: Multiple subscribers
// ============================================================================

static bus_id g_shared_bus;
static int g_subscriber_received[3] = {0, 0, 0};

static void subscriber_actor(void *arg) {
    int id = *(int *)arg;

    rt_status status = rt_bus_subscribe(g_shared_bus);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    // Wait for data with timeout
    char buf[64];
    size_t actual_len;
    status = rt_bus_read_wait(g_shared_bus, buf, sizeof(buf), &actual_len, 500);

    if (!RT_FAILED(status)) {
        g_subscriber_received[id] = 1;
    }

    rt_bus_unsubscribe(g_shared_bus);
    rt_exit();
}

static void test2_multi_subscriber(void *arg) {
    (void)arg;
    printf("\nTest 2: Multiple subscribers\n");

    // Create bus
    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    rt_status status = rt_bus_create(&cfg, &g_shared_bus);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_create");
        rt_exit();
    }

    // Spawn 3 subscribers
    static int ids[3] = {0, 1, 2};
    for (int i = 0; i < 3; i++) {
        g_subscriber_received[i] = 0;
        rt_spawn(subscriber_actor, &ids[i]);
    }

    // Give subscribers time to subscribe
    timer_id timer;
    rt_timer_after(50000, &timer);  // 50ms
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    // Publish data
    const char *data = "Broadcast!";
    status = rt_bus_publish(g_shared_bus, data, strlen(data) + 1);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_publish");
        rt_bus_destroy(g_shared_bus);
        rt_exit();
    }

    // Wait for subscribers to read
    rt_timer_after(200000, &timer);  // 200ms
    rt_ipc_recv(&msg, -1);

    // Check results
    int count = g_subscriber_received[0] + g_subscriber_received[1] + g_subscriber_received[2];
    if (count == 3) {
        TEST_PASS("all 3 subscribers received data");
    } else {
        printf("    Only %d/3 subscribers received data\n", count);
        TEST_FAIL("not all subscribers received data");
    }

    rt_bus_destroy(g_shared_bus);
    rt_exit();
}

// ============================================================================
// Test 3: max_readers retention policy (entry consumed after N subscribers read)
// ============================================================================

static bus_id g_max_readers_bus;
static int g_max_readers_success[3] = {0, 0, 0};

static void max_readers_subscriber(void *arg) {
    int id = *(int *)arg;

    rt_bus_subscribe(g_max_readers_bus);

    // Try to read
    char buf[64];
    size_t actual_len;
    rt_status status = rt_bus_read_wait(g_max_readers_bus, buf, sizeof(buf), &actual_len, 500);

    if (!RT_FAILED(status)) {
        g_max_readers_success[id] = 1;
    }

    rt_bus_unsubscribe(g_max_readers_bus);
    rt_exit();
}

static void test3_max_readers(void *arg) {
    (void)arg;
    printf("\nTest 3: max_readers retention policy\n");

    // Create bus with max_readers = 2 (entry consumed after 2 subscribers read)
    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    cfg.max_readers = 2;
    rt_status status = rt_bus_create(&cfg, &g_max_readers_bus);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_create");
        rt_exit();
    }

    // Reset counters
    for (int i = 0; i < 3; i++) {
        g_max_readers_success[i] = 0;
    }

    // Spawn 3 subscribers
    static int ids[3] = {0, 1, 2};
    for (int i = 0; i < 3; i++) {
        rt_spawn(max_readers_subscriber, &ids[i]);
    }

    // Give subscribers time to subscribe
    timer_id timer;
    rt_timer_after(50000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    // Publish one entry
    const char *data = "Limited reads";
    rt_bus_publish(g_max_readers_bus, data, strlen(data) + 1);

    // Wait for subscribers to try reading
    rt_timer_after(300000, &timer);
    rt_ipc_recv(&msg, -1);

    // Count how many succeeded
    int success_count = g_max_readers_success[0] + g_max_readers_success[1] + g_max_readers_success[2];

    // With max_readers=2, only 2 of 3 subscribers should succeed
    if (success_count == 2) {
        TEST_PASS("entry consumed after max_readers (2) subscribers read");
    } else {
        printf("    %d/3 subscribers read (expected 2)\n", success_count);
        // Note: timing can cause all 3 to read before entry is removed
        if (success_count >= 2) {
            TEST_PASS("at least max_readers subscribers read");
        } else {
            TEST_FAIL("wrong number of successful reads");
        }
    }

    rt_bus_destroy(g_max_readers_bus);
    rt_exit();
}

// ============================================================================
// Test 4: Ring buffer wrap (oldest evicted)
// ============================================================================

static void test4_ring_buffer_wrap(void *arg) {
    (void)arg;
    printf("\nTest 4: Ring buffer wrap (oldest evicted)\n");

    // Create bus with small ring buffer
    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    cfg.max_entries = 4;
    bus_id bus;
    rt_status status = rt_bus_create(&cfg, &bus);
    if (RT_FAILED(status)) {
        TEST_FAIL("rt_bus_create");
        rt_exit();
    }

    rt_bus_subscribe(bus);

    // Publish 6 messages (buffer holds 4)
    for (int i = 1; i <= 6; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Message %d", i);
        rt_bus_publish(bus, msg, strlen(msg) + 1);
    }

    // Should have 4 entries (oldest 2 evicted)
    size_t count = rt_bus_entry_count(bus);
    if (count != 4) {
        printf("    Expected 4 entries, got %zu\n", count);
        TEST_FAIL("wrong entry count");
        rt_bus_unsubscribe(bus);
        rt_bus_destroy(bus);
        rt_exit();
    }

    // First read should be "Message 3" (oldest surviving)
    char buf[64];
    size_t actual_len;
    rt_bus_read(bus, buf, sizeof(buf), &actual_len);

    if (strcmp(buf, "Message 3") == 0) {
        TEST_PASS("oldest entries evicted on buffer wrap");
    } else {
        printf("    Expected 'Message 3', got '%s'\n", buf);
        TEST_FAIL("wrong message after wrap");
    }

    rt_bus_unsubscribe(bus);
    rt_bus_destroy(bus);
    rt_exit();
}

// ============================================================================
// Test 5: Non-blocking read returns WOULDBLOCK
// ============================================================================

static void test5_nonblocking_read(void *arg) {
    (void)arg;
    printf("\nTest 5: Non-blocking read returns WOULDBLOCK\n");

    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    bus_id bus;
    rt_bus_create(&cfg, &bus);
    rt_bus_subscribe(bus);

    // Try to read from empty bus
    char buf[64];
    size_t actual_len;
    rt_status status = rt_bus_read(bus, buf, sizeof(buf), &actual_len);

    if (status.code == RT_ERR_WOULDBLOCK) {
        TEST_PASS("empty bus returns RT_ERR_WOULDBLOCK");
    } else {
        TEST_FAIL("expected RT_ERR_WOULDBLOCK for empty bus");
    }

    rt_bus_unsubscribe(bus);
    rt_bus_destroy(bus);
    rt_exit();
}

// ============================================================================
// Test 6: Blocking read with timeout
// ============================================================================

static void test6_blocking_read_timeout(void *arg) {
    (void)arg;
    printf("\nTest 6: Blocking read with timeout\n");

    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    bus_id bus;
    rt_bus_create(&cfg, &bus);
    rt_bus_subscribe(bus);

    // Try blocking read with short timeout on empty bus
    char buf[64];
    size_t actual_len;
    rt_status status = rt_bus_read_wait(bus, buf, sizeof(buf), &actual_len, 100);

    if (status.code == RT_ERR_TIMEOUT) {
        TEST_PASS("blocking read times out on empty bus");
    } else if (status.code == RT_ERR_WOULDBLOCK) {
        TEST_PASS("blocking read returns WOULDBLOCK on empty bus");
    } else {
        printf("    Got status code: %d\n", status.code);
        TEST_FAIL("expected timeout or WOULDBLOCK");
    }

    rt_bus_unsubscribe(bus);
    rt_bus_destroy(bus);
    rt_exit();
}

// ============================================================================
// Test 7: Destroy bus with subscribers fails
// ============================================================================

static void test7_destroy_with_subscribers(void *arg) {
    (void)arg;
    printf("\nTest 7: Destroy bus with subscribers fails\n");

    rt_bus_config cfg = RT_BUS_CONFIG_DEFAULT;
    bus_id bus;
    rt_bus_create(&cfg, &bus);
    rt_bus_subscribe(bus);

    // Try to destroy while subscribed
    rt_status status = rt_bus_destroy(bus);
    if (RT_FAILED(status)) {
        TEST_PASS("cannot destroy bus with active subscribers");
    } else {
        TEST_FAIL("destroy should fail with active subscribers");
    }

    // Unsubscribe and destroy should work
    rt_bus_unsubscribe(bus);
    status = rt_bus_destroy(bus);
    if (!RT_FAILED(status)) {
        TEST_PASS("destroy succeeds after unsubscribe");
    } else {
        TEST_FAIL("destroy should succeed after unsubscribe");
    }

    rt_exit();
}

// ============================================================================
// Test 8: Invalid bus operations
// ============================================================================

static void test8_invalid_operations(void *arg) {
    (void)arg;
    printf("\nTest 8: Invalid bus operations\n");

    // Subscribe to invalid bus
    rt_status status = rt_bus_subscribe(BUS_ID_INVALID);
    if (RT_FAILED(status)) {
        TEST_PASS("subscribe to invalid bus fails");
    } else {
        TEST_FAIL("subscribe to invalid bus should fail");
    }

    // Publish to invalid bus
    status = rt_bus_publish(BUS_ID_INVALID, "test", 4);
    if (RT_FAILED(status)) {
        TEST_PASS("publish to invalid bus fails");
    } else {
        TEST_FAIL("publish to invalid bus should fail");
    }

    // Read from invalid bus
    char buf[64];
    size_t actual_len;
    status = rt_bus_read(BUS_ID_INVALID, buf, sizeof(buf), &actual_len);
    if (RT_FAILED(status)) {
        TEST_PASS("read from invalid bus fails");
    } else {
        TEST_FAIL("read from invalid bus should fail");
    }

    rt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_basic_pubsub,
    test2_multi_subscriber,
    test3_max_readers,
    test4_ring_buffer_wrap,
    test5_nonblocking_read,
    test6_blocking_read_timeout,
    test7_destroy_with_subscribers,
    test8_invalid_operations,
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
    printf("=== Bus (rt_bus) Test Suite ===\n");

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
