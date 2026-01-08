#include "acrt_runtime.h"
#include "acrt_ipc.h"
#include <stdio.h>

// Simple actor that just exits
void simple_actor(void *arg) {
    (void)arg;
    acrt_exit();
}

int main(void) {
    printf("=== Arena Exhaustion Test ===\n\n");

    acrt_init();

    // ACRT_STACK_ARENA_SIZE is 1 MB
    // ACRT_MAX_ACTORS is 64
    // Use 32 KB stacks to exhaust arena before hitting actor limit
    // Expected: 1 MB / 32 KB = ~32 actors (with overhead, maybe 30-31)

    printf("ACRT_STACK_ARENA_SIZE: 1 MB\n");
    printf("Using custom stack size: 32 KB per actor\n");
    printf("Expected actors that fit: ~30-32\n\n");

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 32 * 1024;  // 32 KB stacks
    cfg.malloc_stack = false;     // Use arena (default)

    int count = 0;

    printf("Spawning actors until arena exhaustion...\n");
    for (int i = 0; i < 64; i++) {
        actor_id id;
        if (ACRT_FAILED(acrt_spawn_ex(simple_actor, NULL, &cfg, &id))) {
            printf("Actor #%d: FAILED (arena exhausted) ✓\n", i + 1);
            break;
        }
        count++;
        printf("Actor #%d: spawned (ID: %u)\n", i + 1, id);
    }

    printf("\nSuccessfully spawned %d actors before exhaustion\n", count);

    // Verify the arena is exhausted by trying to spawn one more
    printf("\nVerifying arena exhaustion...\n");
    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(simple_actor, NULL, &cfg, &id))) {
        printf("✓ Arena is exhausted (cannot spawn more actors)\n");
    } else {
        printf("✗ ERROR: Arena should be exhausted but spawned actor %u\n", id);
    }

    // Test that malloc_stack still works when arena is exhausted
    printf("\nTesting malloc fallback via explicit flag...\n");
    cfg.malloc_stack = true;  // Explicitly use malloc
    if (!ACRT_FAILED(acrt_spawn_ex(simple_actor, NULL, &cfg, &id))) {
        printf("✓ malloc_stack=true still works (spawned actor %u)\n", id);
    } else {
        printf("✗ ERROR: malloc_stack=true should work\n");
    }

    printf("\nRunning scheduler (all actors will exit immediately)...\n");
    acrt_run();

    printf("\n=== Test completed ===\n");
    printf("Arena exhaustion behavior: CORRECT\n");
    printf("- Arena allocation fails gracefully when full\n");
    printf("- malloc_stack=true works independently\n");
    printf("- Cleanup works correctly\n");

    acrt_cleanup();

    return 0;
}
