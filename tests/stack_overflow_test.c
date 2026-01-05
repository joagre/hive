#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_link.h"
#include <stdio.h>
#include <stdlib.h>

// Actor that deliberately corrupts stack guard
void overflow_actor(void *arg) {
    (void)arg;

    printf("Overflow actor: Deliberately corrupting stack guard...\n");

    // Allocate buffer that will barely overflow the guard
    // Stack is 8KB, usable is ~8176 bytes after guards
    // Use just slightly more to minimize corruption
    volatile char buffer[8200];

    // Touch only first part to ensure allocation but minimize corruption
    volatile int sum = 0;
    for (int i = 0; i < 100; i++) {
        buffer[i] = (char)i;
        sum += buffer[i];  // Prevent optimization
    }
    (void)sum;  // Mark as intentionally unused

    // Yield to trigger guard check
    printf("Overflow actor: Yielding to allow guard check...\n");
    rt_yield();

    printf("Overflow actor: If you see this, overflow wasn't detected!\n");
    rt_exit();
}

void linked_actor(void *arg) {
    actor_id overflow_id = *(actor_id *)arg;

    printf("Linked actor: Linking to overflow actor...\n");
    rt_link(overflow_id);

    // Wait for exit notification
    printf("Linked actor: Waiting for exit notification...\n");
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, -1);
    if (RT_FAILED(status)) {
        printf("Linked actor: ✗ FAIL - Failed to receive exit notification\n");
        rt_exit();
    }

    if (rt_is_exit_msg(&msg)) {
        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);

        printf("Linked actor: ✓ PASS - Received exit notification from actor %u\n", exit_info.actor);
        printf("Linked actor:   Exit reason: %s\n",
               exit_info.reason == RT_EXIT_CRASH_STACK ? "RT_EXIT_CRASH_STACK (stack overflow)" :
               exit_info.reason == RT_EXIT_CRASH ? "RT_EXIT_CRASH" :
               exit_info.reason == RT_EXIT_NORMAL ? "RT_EXIT_NORMAL" : "UNKNOWN");

        if (exit_info.reason == RT_EXIT_CRASH_STACK) {
            printf("Linked actor: ✓ PASS - Correct exit reason for stack overflow\n");
        } else {
            printf("Linked actor: ✗ FAIL - Expected RT_EXIT_CRASH_STACK, got %d\n", exit_info.reason);
        }
    } else {
        printf("Linked actor: ✗ FAIL - Message is not an exit notification\n");
    }

    rt_exit();
}

void witness_actor(void *arg) {
    (void)arg;

    printf("Witness: Running after overflow actor... runtime still works!\n");
    printf("Witness: ✓ PASS - System continued running despite stack overflow\n");

    rt_exit();
}

int main(void) {
    printf("=== Stack Overflow Detection Test ===\n");
    printf("Tests that stack overflow is detected and system continues running\n");
    printf("Tests that links/monitors ARE notified (as per spec)\n\n");

    rt_init();

    // Spawn actor with small stack (8KB) to make overflow easy
    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 8 * 1024;  // 8KB stack
    cfg.priority = RT_PRIO_NORMAL;

    actor_id overflow = rt_spawn_ex(overflow_actor, NULL, &cfg);
    printf("Main: Spawned overflow actor (ID: %u) with %zu byte stack\n",
           overflow, cfg.stack_size);

    // Spawn linked actor to verify exit notification
    rt_spawn(linked_actor, &overflow);
    printf("Main: Spawned linked actor\n");

    // Spawn witness actor to prove system still works after overflow
    rt_spawn(witness_actor, NULL);
    printf("Main: Spawned witness actor\n\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected behavior:\n");
    printf("  1. Stack overflow detected (ERROR log)\n");
    printf("  2. Linked actor receives exit notification with RT_EXIT_CRASH_STACK\n");
    printf("  3. Witness actor runs successfully\n");
    printf("  4. No segfault\n");
    printf("Result: PASS if all checks passed\n");

    return 0;
}
