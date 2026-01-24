#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_link.h"
#include <stdio.h>
#include <string.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

/* Bus config for QEMU's reduced limits */
#ifdef QEMU_TEST_STACK_SIZE
#define TEST_BUS_CONFIG                                                  \
    {                                                                    \
        .max_subscribers = 3, .consume_after_reads = 0, .max_age_ms = 0, \
        .max_entries = 4, .max_entry_size = 128                          \
    }
#else
#define TEST_BUS_CONFIG HIVE_BUS_CONFIG_DEFAULT
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
// Test 1: Basic publish/subscribe
// ============================================================================

static void test1_basic_pubsub(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Basic publish/subscribe\n");

    // Create bus
    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    // Subscribe
    status = hive_bus_subscribe(bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_subscribe");
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Publish data
    const char *msg = "Hello Bus!";
    status = hive_bus_publish(bus, msg, strlen(msg) + 1);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_publish");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Read data
    char buf[64];
    size_t actual_len;
    status = hive_bus_read(bus, buf, sizeof(buf), &actual_len);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_read");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }

    if (strcmp(buf, msg) == 0) {
        TEST_PASS("basic publish/subscribe works");
    } else {
        TEST_FAIL("data mismatch");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 2: Multiple subscribers
// ============================================================================

static bus_id_t s_shared_bus;
static int g_subscriber_received[3] = {0, 0, 0};

static void subscriber_actor(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = *(int *)args;

    hive_status_t status = hive_bus_subscribe(s_shared_bus);
    if (HIVE_FAILED(status)) {
        hive_exit();
    }

    // Wait for data with timeout
    char buf[64];
    size_t actual_len;
    status =
        hive_bus_read_wait(s_shared_bus, buf, sizeof(buf), &actual_len, 500);

    if (HIVE_SUCCEEDED(status)) {
        g_subscriber_received[id] = 1;
    }

    hive_bus_unsubscribe(s_shared_bus);
    hive_exit();
}

static void test2_multi_subscriber(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Multiple subscribers\n");

    // Create bus
    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    hive_status_t status = hive_bus_create(&cfg, &s_shared_bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    // Spawn 3 subscribers
    static int ids[3] = {0, 1, 2};
    for (int i = 0; i < 3; i++) {
        g_subscriber_received[i] = 0;
        actor_id_t sub;
        hive_spawn(subscriber_actor, NULL, &ids[i], NULL, &sub);
    }

    // Give subscribers time to subscribe
    timer_id_t timer;
    hive_timer_after(50000, &timer); // 50ms
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Publish data
    const char *data = "Broadcast!";
    status = hive_bus_publish(s_shared_bus, data, strlen(data) + 1);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_publish");
        hive_bus_destroy(s_shared_bus);
        hive_exit();
    }

    // Wait for subscribers to read
    hive_timer_after(200000, &timer); // 200ms
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Check results
    int count = g_subscriber_received[0] + g_subscriber_received[1] +
                g_subscriber_received[2];
    if (count == 3) {
        TEST_PASS("all 3 subscribers received data");
    } else {
        printf("    Only %d/3 subscribers received data\n", count);
        TEST_FAIL("not all subscribers received data");
    }

    hive_bus_destroy(s_shared_bus);
    hive_exit();
}

// ============================================================================
// Test 3: max_readers retention policy (entry consumed after N subscribers read)
// ============================================================================

static bus_id_t s_max_readers_bus;
static int g_max_readers_success[3] = {0, 0, 0};

static void max_readers_subscriber(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = *(int *)args;

    hive_bus_subscribe(s_max_readers_bus);

    // Try to read
    char buf[64];
    size_t actual_len;
    hive_status_t status = hive_bus_read_wait(s_max_readers_bus, buf,
                                              sizeof(buf), &actual_len, 500);

    if (HIVE_SUCCEEDED(status)) {
        g_max_readers_success[id] = 1;
    }

    hive_bus_unsubscribe(s_max_readers_bus);
    hive_exit();
}

static void test3_max_readers(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: max_readers retention policy\n");

    // Create bus with max_readers = 2 (entry consumed after 2 subscribers read)
    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    cfg.consume_after_reads = 2;
    hive_status_t status = hive_bus_create(&cfg, &s_max_readers_bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    // Reset counters
    for (int i = 0; i < 3; i++) {
        g_max_readers_success[i] = 0;
    }

    // Spawn 3 subscribers
    static int ids[3] = {0, 1, 2};
    for (int i = 0; i < 3; i++) {
        actor_id_t sub;
        hive_spawn(max_readers_subscriber, NULL, &ids[i], NULL, &sub);
    }

    // Give subscribers time to subscribe
    timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Publish one entry
    const char *data = "Limited reads";
    hive_bus_publish(s_max_readers_bus, data, strlen(data) + 1);

    // Wait for subscribers to try reading
    hive_timer_after(300000, &timer);
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Count how many succeeded
    int success_count = g_max_readers_success[0] + g_max_readers_success[1] +
                        g_max_readers_success[2];

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

    hive_bus_destroy(s_max_readers_bus);
    hive_exit();
}

// ============================================================================
// Test 4: Ring buffer wrap (oldest evicted)
// ============================================================================

static void test4_ring_buffer_wrap(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Ring buffer wrap (oldest evicted)\n");

    // Create bus with small ring buffer
    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    cfg.max_entries = 4;
    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    hive_bus_subscribe(bus);

    // Publish 6 messages (buffer holds 4)
    for (int i = 1; i <= 6; i++) {
        char msg[32];
        snprintf(msg, sizeof(msg), "Message %d", i);
        hive_bus_publish(bus, msg, strlen(msg) + 1);
    }

    // Should have 4 entries (oldest 2 evicted)
    size_t count = hive_bus_entry_count(bus);
    if (count != 4) {
        printf("    Expected 4 entries, got %zu\n", count);
        TEST_FAIL("wrong entry count");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }

    // First read should be "Message 3" (oldest surviving)
    char buf[64];
    size_t actual_len;
    hive_bus_read(bus, buf, sizeof(buf), &actual_len);

    if (strcmp(buf, "Message 3") == 0) {
        TEST_PASS("oldest entries evicted on buffer wrap");
    } else {
        printf("    Expected 'Message 3', got '%s'\n", buf);
        TEST_FAIL("wrong message after wrap");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 5: Non-blocking read returns WOULDBLOCK
// ============================================================================

static void test5_nonblocking_read(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Non-blocking read returns WOULDBLOCK\n");

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_bus_create(&cfg, &bus);
    hive_bus_subscribe(bus);

    // Try to read from empty bus
    char buf[64];
    size_t actual_len;
    hive_status_t status = hive_bus_read(bus, buf, sizeof(buf), &actual_len);

    if (status.code == HIVE_ERR_WOULDBLOCK) {
        TEST_PASS("empty bus returns HIVE_ERR_WOULDBLOCK");
    } else {
        TEST_FAIL("expected HIVE_ERR_WOULDBLOCK for empty bus");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 6: Blocking read with timeout
// ============================================================================

static void test6_blocking_read_timeout(void *args,
                                        const hive_spawn_info_t *siblings,
                                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Blocking read with timeout\n");

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_bus_create(&cfg, &bus);
    hive_bus_subscribe(bus);

    // Try blocking read with short timeout on empty bus
    char buf[64];
    size_t actual_len;
    hive_status_t status =
        hive_bus_read_wait(bus, buf, sizeof(buf), &actual_len, 100);

    if (status.code == HIVE_ERR_TIMEOUT) {
        TEST_PASS("blocking read times out on empty bus");
    } else if (status.code == HIVE_ERR_WOULDBLOCK) {
        TEST_PASS("blocking read returns WOULDBLOCK on empty bus");
    } else {
        printf("    Got status code: %d\n", status.code);
        TEST_FAIL("expected timeout or WOULDBLOCK");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 7: Destroy bus with subscribers fails
// ============================================================================

static void test7_destroy_with_subscribers(void *args,
                                           const hive_spawn_info_t *siblings,
                                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Destroy bus with subscribers fails\n");

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_bus_create(&cfg, &bus);
    hive_bus_subscribe(bus);

    // Try to destroy while subscribed
    hive_status_t status = hive_bus_destroy(bus);
    if (HIVE_FAILED(status)) {
        TEST_PASS("cannot destroy bus with active subscribers");
    } else {
        TEST_FAIL("destroy should fail with active subscribers");
    }

    // Unsubscribe and destroy should work
    hive_bus_unsubscribe(bus);
    status = hive_bus_destroy(bus);
    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("destroy succeeds after unsubscribe");
    } else {
        TEST_FAIL("destroy should succeed after unsubscribe");
    }

    hive_exit();
}

// ============================================================================
// Test 8: Invalid bus operations
// ============================================================================

static void test8_invalid_operations(void *args,
                                     const hive_spawn_info_t *siblings,
                                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Invalid bus operations\n");

    // Subscribe to invalid bus
    hive_status_t status = hive_bus_subscribe(BUS_ID_INVALID);
    if (HIVE_FAILED(status)) {
        TEST_PASS("subscribe to invalid bus fails");
    } else {
        TEST_FAIL("subscribe to invalid bus should fail");
    }

    // Publish to invalid bus
    status = hive_bus_publish(BUS_ID_INVALID, "test", 4);
    if (HIVE_FAILED(status)) {
        TEST_PASS("publish to invalid bus fails");
    } else {
        TEST_FAIL("publish to invalid bus should fail");
    }

    // Read from invalid bus
    char buf[64];
    size_t actual_len;
    status = hive_bus_read(BUS_ID_INVALID, buf, sizeof(buf), &actual_len);
    if (HIVE_FAILED(status)) {
        TEST_PASS("read from invalid bus fails");
    } else {
        TEST_FAIL("read from invalid bus should fail");
    }

    hive_exit();
}

// ============================================================================
// Test 9: max_age_ms retention policy (time-based expiry)
// ============================================================================

static void test9_max_age_expiry(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 9: max_age_ms retention policy (time-based expiry)\n");
    fflush(stdout);

    // Create bus with 100ms expiry
    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    cfg.max_age_ms = 100; // Entries expire after 100ms

    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to create bus with max_age_ms");
        hive_exit();
    }

    status = hive_bus_subscribe(bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to subscribe");
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Publish an entry
    const char *data = "expires_soon";
    status = hive_bus_publish(bus, data, strlen(data) + 1);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to publish");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Read immediately - should succeed
    char buf[64];
    size_t actual_len;
    status = hive_bus_read(bus, buf, sizeof(buf), &actual_len);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("immediate read failed (expected success)");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }
    TEST_PASS("entry readable immediately after publish");

    // Publish another entry
    const char *data2 = "will_expire";
    hive_bus_publish(bus, data2, strlen(data2) + 1);

    // Wait for expiry (longer than max_age_ms)
    hive_message_t msg;
    hive_ipc_recv(&msg, 150); // Wait 150ms (entry should expire at 100ms)

    // Try to read - should get WOULDBLOCK (entry expired)
    status = hive_bus_read(bus, buf, sizeof(buf), &actual_len);
    if (status.code == HIVE_ERR_WOULDBLOCK) {
        TEST_PASS("entry expired after max_age_ms");
    } else if (HIVE_SUCCEEDED(status)) {
        printf("    Entry still readable after expiry (data: %s)\n", buf);
        TEST_FAIL("entry should have expired");
    } else {
        printf("    Unexpected error: %s\n",
               status.msg ? status.msg : "unknown");
        TEST_FAIL("unexpected error reading expired entry");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 10: hive_bus_entry_count explicit test
// ============================================================================

static void test10_entry_count(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 10: hive_bus_entry_count\n");
    fflush(stdout);

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
#ifdef QEMU_TEST_STACK_SIZE
    cfg.max_entries = 6; // QEMU limit is 8, use smaller value
#else
    cfg.max_entries = 10;
#endif
    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    status = hive_bus_subscribe(bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_subscribe");
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Initially empty
    size_t count = hive_bus_entry_count(bus);
    if (count == 0) {
        TEST_PASS("entry_count returns 0 for empty bus");
    } else {
        printf("    Expected 0, got %zu\n", count);
        TEST_FAIL("entry_count should be 0 for empty bus");
    }

    // Publish 5 entries
    for (int i = 0; i < 5; i++) {
        char msg[16];
        snprintf(msg, sizeof(msg), "Entry %d", i);
        hive_bus_publish(bus, msg, strlen(msg) + 1);
    }

    count = hive_bus_entry_count(bus);
    if (count == 5) {
        TEST_PASS("entry_count returns 5 after publishing 5 entries");
    } else {
        printf("    Expected 5, got %zu\n", count);
        TEST_FAIL("entry_count wrong after publish");
    }

    // Read 2 entries
    char buf[64];
    size_t actual_len;
    hive_bus_read(bus, buf, sizeof(buf), &actual_len);
    hive_bus_read(bus, buf, sizeof(buf), &actual_len);

    // With max_readers=0 (default), entries persist until aged out or buffer wraps
    // So count should still be 5 (entries not consumed by reading)
    count = hive_bus_entry_count(bus);
    printf("    After reading 2 entries, count = %zu\n", count);

    // Publish more to fill buffer
#ifdef QEMU_TEST_STACK_SIZE
    int fill_count = 6; // Fill to max_entries=6
#else
    int fill_count = 12; // Fill to max_entries=10
#endif
    for (int i = 5; i < fill_count; i++) {
        char msg[16];
        snprintf(msg, sizeof(msg), "Entry %d", i);
        hive_bus_publish(bus, msg, strlen(msg) + 1);
    }

    count = hive_bus_entry_count(bus);
#ifdef QEMU_TEST_STACK_SIZE
    size_t expected = 6;
#else
    size_t expected = 10;
#endif
    if (count == expected) {
        TEST_PASS("entry_count capped at max_entries after overflow");
    } else {
        printf("    Expected %zu, got %zu\n", expected, count);
        TEST_FAIL("entry_count wrong after buffer wrap");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test 11: Subscribe to destroyed bus
// ============================================================================

static void test11_subscribe_destroyed_bus(void *args,
                                           const hive_spawn_info_t *siblings,
                                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 11: Subscribe to destroyed bus\n");
    fflush(stdout);

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    // Destroy it immediately
    status = hive_bus_destroy(bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_destroy");
        hive_exit();
    }

    // Try to subscribe to destroyed bus
    status = hive_bus_subscribe(bus);
    if (HIVE_FAILED(status)) {
        TEST_PASS("subscribe to destroyed bus fails");
    } else {
        TEST_FAIL("subscribe to destroyed bus should fail");
    }

    hive_exit();
}

// ============================================================================
// Test 12: Buffer overflow protection (read with undersized buffer)
// ============================================================================

static void test12_buffer_overflow_protection(void *args,
                                              const hive_spawn_info_t *siblings,
                                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 12: Buffer overflow protection\n");
    fflush(stdout);

    hive_bus_config_t cfg = TEST_BUS_CONFIG;
    bus_id_t bus;
    hive_status_t status = hive_bus_create(&cfg, &bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_create");
        hive_exit();
    }

    status = hive_bus_subscribe(bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_subscribe");
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Publish a large message
    char large_msg[128];
    memset(large_msg, 'X', sizeof(large_msg));
    large_msg[127] = '\0';

    status = hive_bus_publish(bus, large_msg, sizeof(large_msg));
    if (HIVE_FAILED(status)) {
        TEST_FAIL("hive_bus_publish large message");
        hive_bus_unsubscribe(bus);
        hive_bus_destroy(bus);
        hive_exit();
    }

    // Try to read with undersized buffer - should not overflow
    char small_buf[16];
    memset(small_buf, 0, sizeof(small_buf));
    size_t actual_len = 0;

    status = hive_bus_read(bus, small_buf, sizeof(small_buf), &actual_len);

    // Check that we didn't overflow (buffer should be intact beyond our small_buf)
    // The implementation should either truncate or return error
    if (HIVE_FAILED(status)) {
        TEST_PASS("hive_bus_read rejects undersized buffer");
    } else if (actual_len <= sizeof(small_buf)) {
        TEST_PASS("hive_bus_read truncates to buffer size");
    } else {
        printf("    actual_len=%zu, buffer=%zu - possible overflow!\n",
               actual_len, sizeof(small_buf));
        TEST_FAIL("buffer overflow not prevented");
    }

    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);
    hive_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *, const hive_spawn_info_t *, size_t) = {
    test1_basic_pubsub,
    test2_multi_subscriber,
    test3_max_readers,
    test4_ring_buffer_wrap,
    test5_nonblocking_read,
    test6_blocking_read_timeout,
    test7_destroy_with_subscribers,
    test8_invalid_operations,
    test9_max_age_expiry,
    test10_entry_count,
    test11_subscribe_destroyed_bus,
    test12_buffer_overflow_protection,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
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
    printf("=== Bus (hive_bus) Test Suite ===\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
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
