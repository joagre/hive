#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_link.h"
#include <stdio.h>
#include <stdlib.h>

// Actor that deliberately corrupts stack guard
void overflow_actor(void *arg) {
    (void)arg;

    printf("Overflow actor: Deliberately corrupting stack guard...\n");

    // Get a stack variable to find approximate stack location
    volatile char marker = 0;
    void *stack_ptr = (void *)&marker;

    // The stack grows downward on x86-64
    // We need to find and corrupt the lower guard
    // This is a bit hacky but works for testing

    // Estimate: allocate large buffer to get close to guard
    volatile char buffer[7000];  // Gets us close to 8KB limit

    // Touch it to ensure allocation
    for (int i = 0; i < 7000; i++) {
        buffer[i] = (char)i;
    }

    // Yield to trigger guard check
    printf("Overflow actor: Yielding to allow guard check...\n");
    rt_yield();

    printf("Overflow actor: If you see this, overflow wasn't detected!\n");
    rt_exit();
}

void monitor_actor(void *arg) {
    actor_id target = *(actor_id *)arg;

    printf("Monitor: Monitoring actor %u for stack overflow...\n", target);

    // Monitor the target actor
    uint32_t monitor_ref;
    rt_monitor(target, &monitor_ref);

    // Wait for exit notification
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (rt_is_exit_msg(&msg)) {
        rt_exit_msg exit_info;
        rt_decode_exit(&msg, &exit_info);

        printf("Monitor: Received exit notification\n");
        printf("Monitor:   Actor ID: %u\n", exit_info.actor);
        printf("Monitor:   Exit reason: ");

        if (exit_info.reason == RT_EXIT_NORMAL) {
            printf("NORMAL\n");
            printf("Monitor: FAIL - Expected stack overflow, got normal exit\n");
        } else if (exit_info.reason == RT_EXIT_CRASH) {
            printf("CRASH\n");
            printf("Monitor: FAIL - Expected stack overflow, got generic crash\n");
        } else if (exit_info.reason == RT_EXIT_CRASH_STACK) {
            printf("CRASH_STACK\n");
            printf("Monitor: PASS - Stack overflow detected and reported!\n");
        } else if (exit_info.reason == RT_EXIT_KILLED) {
            printf("KILLED\n");
            printf("Monitor: FAIL - Expected stack overflow, got killed\n");
        } else {
            printf("UNKNOWN(%d)\n", exit_info.reason);
            printf("Monitor: FAIL - Unknown exit reason\n");
        }
    } else {
        printf("Monitor: FAIL - Expected exit message, got regular message from %u\n",
               msg.sender);
    }

    rt_exit();
}

int main(void) {
    printf("=== Stack Overflow Detection Test ===\n");
    printf("Tests that stack overflow is detected and reported as RT_EXIT_CRASH_STACK\n\n");

    rt_init();

    // Spawn actor with small stack (8KB) to make overflow easy
    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 8 * 1024;  // 8KB stack
    cfg.priority = RT_PRIO_NORMAL;

    actor_id overflow = rt_spawn_ex(overflow_actor, NULL, &cfg);
    printf("Main: Spawned overflow actor (ID: %u) with %zu byte stack\n",
           overflow, cfg.stack_size);

    actor_id monitor = rt_spawn(monitor_actor, &overflow);
    printf("Main: Spawned monitor actor (ID: %u)\n\n", monitor);

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected: Stack overflow detected with RT_EXIT_CRASH_STACK\n");
    printf("Result: Check Monitor output above\n");

    return 0;
}
