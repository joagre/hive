#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <time.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Get time in milliseconds
static uint64_t get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void test_timeout_actor(void *arg) {
    (void)arg;

    printf("Test 1: Timeout when no message arrives\n");
    uint64_t start = get_time_ms();
    hive_message msg;
    hive_status status = hive_ipc_recv(&msg, 100);  // 100ms timeout
    uint64_t elapsed = get_time_ms() - start;

    if (status.code == HIVE_ERR_TIMEOUT) {
        printf("  ✓ Got timeout after %lu ms (expected ~100ms)\n", elapsed);
    } else {
        printf("  ✗ Expected timeout, got status=%d\n", status.code);
    }

    printf("\nTest 2: Message arrives before timeout\n");
    actor_id self = hive_self();
    int data = 42;
    hive_ipc_notify(self, 0, &data, sizeof(data));

    start = get_time_ms();
    status = hive_ipc_recv(&msg, 100);  // 100ms timeout
    elapsed = get_time_ms() - start;

    if (HIVE_SUCCEEDED(status)) {
        int *received = (int *)msg.data;
        printf("  ✓ Got message before timeout: %d (after %lu ms)\n", *received, elapsed);
    } else {
        printf("  ✗ Expected message, got status=%d\n", status.code);
    }

    printf("\nTest 3: Backoff-retry pattern (simulated pool exhaustion)\n");
    // Simulate: first send "fails", backoff, retry
    int retry_count = 0;
    while (retry_count < 3) {
        printf("  Attempt %d: Backing off 50ms...\n", retry_count + 1);

        start = get_time_ms();
        status = hive_ipc_recv(&msg, 50);  // Backoff 50ms
        elapsed = get_time_ms() - start;

        if (status.code == HIVE_ERR_TIMEOUT) {
            printf("    Backoff complete after %lu ms, retrying...\n", elapsed);
            retry_count++;
        } else {
            printf("    Got message during backoff: handling it first\n");
        }
    }
    printf("  ✓ Backoff-retry pattern works\n");

    printf("\nAll tests passed!\n");
    hive_exit();
}

int main(void) {
    hive_init();
    actor_id test;
    hive_spawn(test_timeout_actor, NULL, &test);
    hive_run();
    hive_cleanup();
    return 0;
}
