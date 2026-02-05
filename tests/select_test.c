#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_link.h"
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

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// ============================================================================
// Test 1: Single IPC source (wildcard) - equivalent to hive_ipc_recv()
// ============================================================================

static void test1_ipc_wildcard(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 1: Single IPC source (wildcard)\n");

    hive_actor_id_t self = hive_self();

    // Notify a message to self
    int data = 42;
    hive_ipc_notify(self, 123, &data, sizeof(data));

    // Use hive_select with wildcard IPC filter
    hive_select_source_t source = {.type = HIVE_SEL_IPC, .ipc = {0}};
    hive_select_result_t result;
    hive_status_t status = hive_select(&source, 1, &result, 100);

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("hive_select with wildcard IPC succeeds");
    } else {
        TEST_FAIL("hive_select with wildcard IPC failed");
    }

    if (result.type == HIVE_SEL_IPC && result.index == 0) {
        TEST_PASS("result type and index correct");
    } else {
        TEST_FAIL("result type or index incorrect");
    }

    if (*(int *)result.ipc.data == 42 && result.ipc.id == 123) {
        TEST_PASS("received correct data and id");
    } else {
        TEST_FAIL("data or id mismatch");
    }

    return;
}

// ============================================================================
// Test 2: Single IPC source (filtered) - equivalent to hive_ipc_recv_match()
// ============================================================================

#define TAG_A 100
#define TAG_B 200

static void test2_ipc_filtered(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 2: Single IPC source (filtered)\n");

    hive_actor_id_t self = hive_self();

    // Notify messages with different tags
    int a = 1, b = 2;
    hive_ipc_notify(self, TAG_A, &a, sizeof(a));
    hive_ipc_notify(self, TAG_B, &b, sizeof(b));

    // Select only messages with TAG_B (id field)
    hive_select_source_t source = {
        .type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_B}};
    hive_select_result_t result;
    hive_status_t status = hive_select(&source, 1, &result, 100);

    if (HIVE_SUCCEEDED(status) && result.ipc.id == TAG_B) {
        TEST_PASS("filtered select returns TAG_B message");
    } else {
        TEST_FAIL("expected TAG_B message");
    }

    if (*(int *)result.ipc.data == 2) {
        TEST_PASS("received correct data for TAG_B");
    } else {
        TEST_FAIL("data mismatch");
    }

    // Now get TAG_A which should still be in mailbox
    source.ipc.id = TAG_A;
    status = hive_select(&source, 1, &result, 100);

    if (HIVE_SUCCEEDED(status) && result.ipc.id == TAG_A) {
        TEST_PASS("TAG_A still available after filtered select");
    } else {
        TEST_FAIL("TAG_A should still be in mailbox");
    }

    return;
}

// ============================================================================
// Test 3: Single bus source - equivalent to hive_bus_read() with timeout
// ============================================================================

static hive_bus_id_t s_test_bus = HIVE_BUS_ID_INVALID;

static void test3_publisher(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    // Wait a bit then publish
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, HIVE_ID_ANY, timer,
                        &msg, -1);

    int data = 99;
    hive_bus_publish(s_test_bus, &data, sizeof(data));
    return;
}

static void test3_bus_source(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 3: Single bus source\n");

    // Create and subscribe to bus
    hive_bus_config_t cfg = HIVE_BUS_CONFIG_DEFAULT;
    hive_status_t status = hive_bus_create(&cfg, &s_test_bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to create bus");
        return;
    }

    status = hive_bus_subscribe(s_test_bus);
    if (HIVE_FAILED(status)) {
        TEST_FAIL("failed to subscribe to bus");
        hive_bus_destroy(s_test_bus);
        return;
    }

    // Spawn publisher
    hive_actor_id_t publisher;
    hive_spawn(test3_publisher, NULL, NULL, NULL, &publisher);

    // Wait for bus data using hive_select
    hive_select_source_t source = {.type = HIVE_SEL_BUS, .bus = s_test_bus};
    hive_select_result_t result;
    uint64_t start = time_ms();
    status = hive_select(&source, 1, &result, 500);
    uint64_t elapsed = time_ms() - start;

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("hive_select with bus source succeeds");
    } else {
        printf("    status: %s\n", HIVE_ERR_STR(status));
        TEST_FAIL("hive_select with bus source failed");
    }

    if (result.type == HIVE_SEL_BUS && result.index == 0) {
        TEST_PASS("result type and index correct");
    } else {
        TEST_FAIL("result type or index incorrect");
    }

    if (result.bus.len == sizeof(int) && *(int *)result.bus.data == 99) {
        TEST_PASS("received correct bus data");
    } else {
        TEST_FAIL("bus data mismatch");
    }

    if (elapsed >= 40 && elapsed <= 200) {
        printf("    received after %lu ms (expected ~50ms)\n",
               (unsigned long)elapsed);
        TEST_PASS("timing correct");
    } else {
        printf("    received after %lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("timing incorrect");
    }

    // Cleanup
    hive_bus_unsubscribe(s_test_bus);
    hive_bus_destroy(s_test_bus);
    s_test_bus = HIVE_BUS_ID_INVALID;

    return;
}

// ============================================================================
// Test 4: Multi-source IPC + IPC (first matches)
// ============================================================================

static void test4_ipc_multi_first(void *args, const hive_spawn_info_t *siblings,
                                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 4: Multi-source IPC + IPC (first matches)\n");

    hive_actor_id_t self = hive_self();

    // Notify message matching first filter
    int data = 111;
    hive_ipc_notify(self, TAG_A, &data, sizeof(data));

    // Wait for either TAG_A or TAG_B (filtering by id)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_A}},
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_B}},
    };
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 2, &result, 100);

    if (HIVE_SUCCEEDED(status) && result.index == 0) {
        TEST_PASS("multi-source IPC matched first filter");
    } else {
        printf("    index=%zu\n", result.index);
        TEST_FAIL("expected first filter to match");
    }

    if (result.ipc.id == TAG_A) {
        TEST_PASS("received TAG_A message");
    } else {
        TEST_FAIL("wrong id");
    }

    return;
}

// ============================================================================
// Test 5: Multi-source IPC + IPC (second matches)
// ============================================================================

static void test5_ipc_multi_second(void *args,
                                   const hive_spawn_info_t *siblings,
                                   size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 5: Multi-source IPC + IPC (second matches)\n");

    hive_actor_id_t self = hive_self();

    // Notify message matching second filter
    int data = 222;
    hive_ipc_notify(self, TAG_B, &data, sizeof(data));

    // Wait for either TAG_A or TAG_B (filtering by id)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_A}},
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_B}},
    };
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 2, &result, 100);

    if (HIVE_SUCCEEDED(status) && result.index == 1) {
        TEST_PASS("multi-source IPC matched second filter");
    } else {
        printf("    index=%zu\n", result.index);
        TEST_FAIL("expected second filter to match");
    }

    if (result.ipc.id == TAG_B) {
        TEST_PASS("received TAG_B message");
    } else {
        TEST_FAIL("wrong id");
    }

    return;
}

// ============================================================================
// Test 6: Multi-source bus + bus
// ============================================================================

static hive_bus_id_t s_bus1 = HIVE_BUS_ID_INVALID;
static hive_bus_id_t s_bus2 = HIVE_BUS_ID_INVALID;

static void test6_bus_publisher(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int which_bus = *(int *)args;
    // Wait a bit then publish to specified bus
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, HIVE_ID_ANY, timer,
                        &msg, -1);

    int data = which_bus == 1 ? 111 : 222;
    hive_bus_publish(which_bus == 1 ? s_bus1 : s_bus2, &data, sizeof(data));
    return;
}

static void test6_bus_multi(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 6: Multi-source bus + bus\n");

    // Create two buses
    hive_bus_config_t cfg = HIVE_BUS_CONFIG_DEFAULT;
    hive_bus_create(&cfg, &s_bus1);
    hive_bus_create(&cfg, &s_bus2);
    hive_bus_subscribe(s_bus1);
    hive_bus_subscribe(s_bus2);

    // Spawn publisher for bus 2
    static int which = 2;
    hive_actor_id_t publisher;
    hive_spawn(test6_bus_publisher, NULL, &which, NULL, &publisher);

    // Wait for data from either bus
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_BUS, .bus = s_bus1},
        {.type = HIVE_SEL_BUS, .bus = s_bus2},
    };
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 2, &result, 500);

    if (HIVE_SUCCEEDED(status) && result.index == 1) {
        TEST_PASS("received from second bus");
    } else {
        printf("    status=%d, index=%zu\n", status.code, result.index);
        TEST_FAIL("expected second bus");
    }

    if (*(int *)result.bus.data == 222) {
        TEST_PASS("correct data from bus 2");
    } else {
        TEST_FAIL("wrong data");
    }

    // Cleanup
    hive_bus_unsubscribe(s_bus1);
    hive_bus_unsubscribe(s_bus2);
    hive_bus_destroy(s_bus1);
    hive_bus_destroy(s_bus2);
    s_bus1 = s_bus2 = HIVE_BUS_ID_INVALID;

    return;
}

// ============================================================================
// Test 7: Multi-source IPC + bus (mixed)
// ============================================================================

static void test7_mixed_sender(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t target = *(hive_actor_id_t *)args;
    // Wait a bit then notify IPC message
    hive_timer_id_t timer;
    hive_timer_after(50000, &timer);
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, HIVE_ID_ANY, timer,
                        &msg, -1);

    int data = 777;
    hive_ipc_notify(target, TAG_A, &data, sizeof(data));
    return;
}

static void test7_mixed_sources(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 7: Multi-source IPC + bus (mixed)\n");

    hive_actor_id_t self = hive_self();

    // Create and subscribe to bus
    hive_bus_config_t cfg = HIVE_BUS_CONFIG_DEFAULT;
    hive_bus_id_t bus;
    hive_bus_create(&cfg, &bus);
    hive_bus_subscribe(bus);

    // Spawn sender that will notify IPC message
    hive_actor_id_t sender;
    hive_spawn(test7_mixed_sender, NULL, &self, NULL, &sender);

    // Wait for either bus data or IPC message (filtering by id)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_BUS, .bus = bus},
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_A}},
    };
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 2, &result, 500);

    if (HIVE_SUCCEEDED(status) && result.type == HIVE_SEL_IPC) {
        TEST_PASS("received IPC in mixed select");
    } else {
        printf("    status=%d, type=%d\n", status.code, result.type);
        TEST_FAIL("expected IPC result");
    }

    if (result.index == 1 && result.ipc.id == TAG_A) {
        TEST_PASS("correct index and id for IPC");
    } else {
        printf("    index=%zu, id=%u\n", result.index, result.ipc.id);
        TEST_FAIL("index or id mismatch");
    }

    // Cleanup
    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);

    return;
}

// ============================================================================
// Test 8: Priority ordering - bus wins over IPC when both ready
// ============================================================================

static void test8_priority_order(void *args, const hive_spawn_info_t *siblings,
                                 size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 8: Priority ordering - bus wins over IPC when both ready\n");

    hive_actor_id_t self = hive_self();

    // Create and subscribe to bus
    hive_bus_config_t cfg = HIVE_BUS_CONFIG_DEFAULT;
    hive_bus_id_t bus;
    hive_bus_create(&cfg, &bus);
    hive_bus_subscribe(bus);

    // Notify IPC message first
    int ipc_data = 111;
    hive_ipc_notify(self, TAG_A, &ipc_data, sizeof(ipc_data));

    // Publish bus data second
    int bus_data = 222;
    hive_bus_publish(bus, &bus_data, sizeof(bus_data));

    // Select - first source in array wins (array order priority)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = TAG_A}},
        {.type = HIVE_SEL_BUS, .bus = bus},
    };
    hive_select_result_t result;
    hive_status_t status = hive_select(sources, 2, &result, 100);

    // IPC is at index 0, should win with array order priority
    if (HIVE_SUCCEEDED(status) && result.type == HIVE_SEL_IPC) {
        TEST_PASS("array order priority (IPC first wins)");
    } else {
        printf("    type=%d\n", result.type);
        TEST_FAIL("expected IPC (first in array) to win");
    }

    if (result.index == 0 && *(int *)result.ipc.data == 111) {
        TEST_PASS("correct index and data for IPC");
    } else {
        TEST_FAIL("index or data mismatch");
    }

    // Now bus should be available
    status = hive_select(&sources[1], 1, &result, 100);

    if (HIVE_SUCCEEDED(status) && result.type == HIVE_SEL_BUS) {
        TEST_PASS("bus still available after IPC consumed");
    } else {
        TEST_FAIL("bus should still be available");
    }

    // Cleanup
    hive_bus_unsubscribe(bus);
    hive_bus_destroy(bus);

    return;
}

// ============================================================================
// Test 9: Timeout behavior
// ============================================================================

static void test9_timeout(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 9: Timeout behavior\n");

    // Select on nothing with timeout (filtering by non-existent id)
    hive_select_source_t source = {
        .type = HIVE_SEL_IPC, .ipc = {.class = HIVE_MSG_NOTIFY, .id = 9999}};
    hive_select_result_t result;

    // Non-blocking (timeout=0)
    hive_status_t status = hive_select(&source, 1, &result, 0);
    if (status.code == HIVE_ERR_WOULDBLOCK) {
        TEST_PASS("non-blocking returns WOULDBLOCK");
    } else {
        printf("    status=%d\n", status.code);
        TEST_FAIL("expected WOULDBLOCK");
    }

    // With timeout
    uint64_t start = time_ms();
    status = hive_select(&source, 1, &result, 100);
    uint64_t elapsed = time_ms() - start;

    if (status.code == HIVE_ERR_TIMEOUT) {
        TEST_PASS("timed select returns TIMEOUT");
    } else {
        printf("    status=%d\n", status.code);
        TEST_FAIL("expected TIMEOUT");
    }

    if (elapsed >= 80 && elapsed <= 200) {
        printf("    timed out after %lu ms (expected ~100ms)\n",
               (unsigned long)elapsed);
        TEST_PASS("timeout duration correct");
    } else {
        printf("    elapsed=%lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("timeout duration incorrect");
    }

    return;
}

// ============================================================================
// Test 10: Error cases
// ============================================================================

static void test10_error_cases(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 10: Error cases\n");

    hive_select_result_t result;

    // NULL sources
    hive_status_t status = hive_select(NULL, 1, &result, 100);
    if (status.code == HIVE_ERR_INVALID) {
        TEST_PASS("NULL sources rejected");
    } else {
        TEST_FAIL("expected INVALID for NULL sources");
    }

    // Zero sources
    hive_select_source_t source = {.type = HIVE_SEL_IPC};
    status = hive_select(&source, 0, &result, 100);
    if (status.code == HIVE_ERR_INVALID) {
        TEST_PASS("zero sources rejected");
    } else {
        TEST_FAIL("expected INVALID for zero sources");
    }

    // NULL result
    status = hive_select(&source, 1, NULL, 100);
    if (status.code == HIVE_ERR_INVALID) {
        TEST_PASS("NULL result rejected");
    } else {
        TEST_FAIL("expected INVALID for NULL result");
    }

    // Unsubscribed bus
    hive_bus_id_t invalid_bus = 9999;
    hive_select_source_t bus_source = {.type = HIVE_SEL_BUS,
                                       .bus = invalid_bus};
    status = hive_select(&bus_source, 1, &result, 100);
    if (status.code == HIVE_ERR_INVALID) {
        TEST_PASS("unsubscribed bus rejected");
    } else {
        printf("    status=%d\n", status.code);
        TEST_FAIL("expected INVALID for unsubscribed bus");
    }

    return;
}

// ============================================================================
// Test 11: Immediate return when data ready
// ============================================================================

static void test11_immediate_return(void *args,
                                    const hive_spawn_info_t *siblings,
                                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("\nTest 11: Immediate return when data ready\n");

    hive_actor_id_t self = hive_self();

    // Pre-notify a message
    int data = 42;
    hive_ipc_notify(self, TAG_A, &data, sizeof(data));

    // Select should return immediately
    hive_select_source_t source = {.type = HIVE_SEL_IPC, .ipc = {0}};
    hive_select_result_t result;
    uint64_t start = time_ms();
    hive_status_t status =
        hive_select(&source, 1, &result, -1); // infinite timeout
    uint64_t elapsed = time_ms() - start;

    if (HIVE_SUCCEEDED(status)) {
        TEST_PASS("select with ready data succeeds");
    } else {
        TEST_FAIL("select with ready data failed");
    }

    if (elapsed < 10) {
        TEST_PASS("immediate return when data ready");
    } else {
        printf("    elapsed=%lu ms\n", (unsigned long)elapsed);
        TEST_FAIL("should return immediately");
    }

    return;
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *, const hive_spawn_info_t *, size_t) = {
    test1_ipc_wildcard,    test2_ipc_filtered,      test3_bus_source,
    test4_ipc_multi_first, test5_ipc_multi_second,  test6_bus_multi,
    test7_mixed_sources,   test8_priority_order,    test9_timeout,
    test10_error_cases,    test11_immediate_return,
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
    printf("=== hive_select() Test Suite ===\n");

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
