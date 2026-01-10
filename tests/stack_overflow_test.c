
/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif
// NOTE: This test intentionally causes stack overflow, which corrupts memory.
// Valgrind will report errors - this is expected behavior for this test.
// Run with: valgrind --error-exitcode=0 ./build/stack_overflow_test

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_link.h"
#include <stdio.h>
#include <stdlib.h>

/* Buffer size to overflow stack - must exceed stack size */
#ifdef QEMU_TEST_STACK_SIZE
#define OVERFLOW_BUFFER_SIZE 3000  /* QEMU stack is 2KB, this exceeds it */
#else
#define OVERFLOW_BUFFER_SIZE 8200  /* Native stack is 8KB, this exceeds it */
#endif

// Actor that deliberately corrupts stack guard
void overflow_actor(void *arg) {
    (void)arg;

    printf("Overflow actor: Deliberately corrupting stack guard...\n");

    // Allocate buffer that will overflow the stack
    // Stack is 8KB (native) or 2KB (QEMU), usable is less after guards
    // This will overflow and corrupt the guard at the bottom of the stack
    volatile char buffer[OVERFLOW_BUFFER_SIZE];

    // Touch the buffer to force allocation and prevent optimization
    volatile int sum = 0;
    for (int i = 0; i < 100; i++) {
        buffer[i] = (char)i;
        sum += buffer[i];
    }
    // Also touch near the end to ensure full allocation
    buffer[OVERFLOW_BUFFER_SIZE - 1] = (char)sum;

    // Yield to trigger guard check
    printf("Overflow actor: Yielding to allow guard check...\n");
    hive_yield();

    printf("Overflow actor: If you see this, overflow wasn't detected!\n");
    hive_exit();
}

void linked_actor(void *arg) {
    actor_id overflow_id = *(actor_id *)arg;

    printf("Linked actor: Linking to overflow actor...\n");
    hive_link(overflow_id);

    // Wait for exit notification
    printf("Linked actor: Waiting for exit notification...\n");
    hive_message msg;
    hive_status status = hive_ipc_recv(&msg, -1);
    if (HIVE_FAILED(status)) {
        printf("Linked actor: ✗ FAIL - Failed to receive exit notification\n");
        hive_exit();
    }

    if (hive_is_exit_msg(&msg)) {
        hive_exit_msg exit_info;
        hive_decode_exit(&msg, &exit_info);

        printf("Linked actor: ✓ PASS - Received exit notification from actor %u\n", exit_info.actor);
        printf("Linked actor:   Exit reason: %s\n",
               exit_info.reason == HIVE_EXIT_CRASH_STACK ? "HIVE_EXIT_CRASH_STACK (stack overflow)" :
               exit_info.reason == HIVE_EXIT_CRASH ? "HIVE_EXIT_CRASH" :
               exit_info.reason == HIVE_EXIT_NORMAL ? "HIVE_EXIT_NORMAL" : "UNKNOWN");

        if (exit_info.reason == HIVE_EXIT_CRASH_STACK) {
            printf("Linked actor: ✓ PASS - Correct exit reason for stack overflow\n");
        } else {
            printf("Linked actor: ✗ FAIL - Expected HIVE_EXIT_CRASH_STACK, got %d\n", exit_info.reason);
        }
    } else {
        printf("Linked actor: ✗ FAIL - Message is not an exit notification\n");
    }

    hive_exit();
}

void witness_actor(void *arg) {
    (void)arg;

    printf("Witness: Running after overflow actor... runtime still works!\n");
    printf("Witness: ✓ PASS - System continued running despite stack overflow\n");

    hive_exit();
}

int main(void) {
    printf("=== Stack Overflow Detection Test ===\n");
    printf("Tests that stack overflow is detected and system continues running\n");
    printf("Tests that links/monitors ARE notified (as per spec)\n\n");

    hive_init();

    // Spawn actor with small stack (8KB) to make overflow easy
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(8 * 1024);  // 8KB stack
    cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id overflow;
    hive_spawn_ex(overflow_actor, NULL, &cfg, &overflow);
    printf("Main: Spawned overflow actor (ID: %u) with %zu byte stack\n",
           overflow, cfg.stack_size);

    // Spawn linked actor to verify exit notification
    actor_id linked;
    hive_spawn(linked_actor, &overflow, &linked);
    printf("Main: Spawned linked actor\n");

    // Spawn witness actor to prove system still works after overflow
    actor_id witness;
    hive_spawn(witness_actor, NULL, &witness);
    printf("Main: Spawned witness actor\n\n");

    hive_run();
    hive_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected behavior:\n");
    printf("  1. Stack overflow detected (ERROR log)\n");
    printf("  2. Linked actor receives exit notification with HIVE_EXIT_CRASH_STACK\n");
    printf("  3. Witness actor runs successfully\n");
    printf("  4. No segfault\n");
    printf("Result: PASS if all checks passed\n");

    return 0;
}
