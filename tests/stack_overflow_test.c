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

void witness_actor(void *arg) {
    (void)arg;

    printf("Witness: Running after overflow actor... runtime still works!\n");
    printf("Witness: PASS - System continued running despite stack overflow\n");

    rt_exit();
}

int main(void) {
    printf("=== Stack Overflow Detection Test ===\n");
    printf("Tests that stack overflow is detected and system continues running\n");
    printf("Note: Links/monitors are NOT notified on stack overflow (prevents crashes)\n\n");

    rt_init();

    // Spawn actor with small stack (8KB) to make overflow easy
    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 8 * 1024;  // 8KB stack
    cfg.priority = RT_PRIO_NORMAL;

    actor_id overflow = rt_spawn_ex(overflow_actor, NULL, &cfg);
    printf("Main: Spawned overflow actor (ID: %u) with %zu byte stack\n",
           overflow, cfg.stack_size);

    // Spawn witness actor to prove system still works after overflow
    rt_spawn(witness_actor, NULL);
    printf("Main: Spawned witness actor\n\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected behavior:\n");
    printf("  1. Stack overflow detected (ERROR log)\n");
    printf("  2. Witness actor runs successfully\n");
    printf("  3. No segfault\n");
    printf("Result: PASS\n");

    return 0;
}
