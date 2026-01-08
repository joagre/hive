// NOTE: This test intentionally causes stack overflow, which corrupts memory.
// Valgrind will report errors - this is expected behavior for this test.
// Run with: valgrind --error-exitcode=0 ./build/stack_overflow_test

#include "acrt_runtime.h"
#include "acrt_ipc.h"
#include "acrt_link.h"
#include <stdio.h>
#include <stdlib.h>

// Actor that deliberately corrupts stack guard
void overflow_actor(void *arg) {
    (void)arg;

    printf("Overflow actor: Deliberately corrupting stack guard...\n");

    // Allocate buffer that will overflow the stack
    // Stack is 8KB, usable is ~8176 bytes after guards (8 bytes each end)
    // This will overflow and corrupt the guard at the bottom of the stack
    volatile char buffer[8200];

    // Touch the buffer to force allocation and prevent optimization
    volatile int sum = 0;
    for (int i = 0; i < 100; i++) {
        buffer[i] = (char)i;
        sum += buffer[i];
    }
    // Also touch near the end to ensure full allocation
    buffer[8199] = (char)sum;

    // Yield to trigger guard check
    printf("Overflow actor: Yielding to allow guard check...\n");
    acrt_yield();

    printf("Overflow actor: If you see this, overflow wasn't detected!\n");
    acrt_exit();
}

void linked_actor(void *arg) {
    actor_id overflow_id = *(actor_id *)arg;

    printf("Linked actor: Linking to overflow actor...\n");
    acrt_link(overflow_id);

    // Wait for exit notification
    printf("Linked actor: Waiting for exit notification...\n");
    acrt_message msg;
    acrt_status status = acrt_ipc_recv(&msg, -1);
    if (ACRT_FAILED(status)) {
        printf("Linked actor: ✗ FAIL - Failed to receive exit notification\n");
        acrt_exit();
    }

    if (acrt_is_exit_msg(&msg)) {
        acrt_exit_msg exit_info;
        acrt_decode_exit(&msg, &exit_info);

        printf("Linked actor: ✓ PASS - Received exit notification from actor %u\n", exit_info.actor);
        printf("Linked actor:   Exit reason: %s\n",
               exit_info.reason == ACRT_EXIT_CRASH_STACK ? "ACRT_EXIT_CRASH_STACK (stack overflow)" :
               exit_info.reason == ACRT_EXIT_CRASH ? "ACRT_EXIT_CRASH" :
               exit_info.reason == ACRT_EXIT_NORMAL ? "ACRT_EXIT_NORMAL" : "UNKNOWN");

        if (exit_info.reason == ACRT_EXIT_CRASH_STACK) {
            printf("Linked actor: ✓ PASS - Correct exit reason for stack overflow\n");
        } else {
            printf("Linked actor: ✗ FAIL - Expected ACRT_EXIT_CRASH_STACK, got %d\n", exit_info.reason);
        }
    } else {
        printf("Linked actor: ✗ FAIL - Message is not an exit notification\n");
    }

    acrt_exit();
}

void witness_actor(void *arg) {
    (void)arg;

    printf("Witness: Running after overflow actor... runtime still works!\n");
    printf("Witness: ✓ PASS - System continued running despite stack overflow\n");

    acrt_exit();
}

int main(void) {
    printf("=== Stack Overflow Detection Test ===\n");
    printf("Tests that stack overflow is detected and system continues running\n");
    printf("Tests that links/monitors ARE notified (as per spec)\n\n");

    acrt_init();

    // Spawn actor with small stack (8KB) to make overflow easy
    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 8 * 1024;  // 8KB stack
    cfg.priority = ACRT_PRIORITY_NORMAL;

    actor_id overflow;
    acrt_spawn_ex(overflow_actor, NULL, &cfg, &overflow);
    printf("Main: Spawned overflow actor (ID: %u) with %zu byte stack\n",
           overflow, cfg.stack_size);

    // Spawn linked actor to verify exit notification
    actor_id linked;
    acrt_spawn(linked_actor, &overflow, &linked);
    printf("Main: Spawned linked actor\n");

    // Spawn witness actor to prove system still works after overflow
    actor_id witness;
    acrt_spawn(witness_actor, NULL, &witness);
    printf("Main: Spawned witness actor\n\n");

    acrt_run();
    acrt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected behavior:\n");
    printf("  1. Stack overflow detected (ERROR log)\n");
    printf("  2. Linked actor receives exit notification with ACRT_EXIT_CRASH_STACK\n");
    printf("  3. Witness actor runs successfully\n");
    printf("  4. No segfault\n");
    printf("Result: PASS if all checks passed\n");

    return 0;
}
