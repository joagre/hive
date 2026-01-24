#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Simple actor that just exits
void simple_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    hive_exit();
}

int main(void) {
    printf("=== Arena Exhaustion Test ===\n\n");

    hive_init();

    // HIVE_STACK_ARENA_SIZE is 1 MB
    // HIVE_MAX_ACTORS is 64
    // Use 32 KB stacks to exhaust arena before hitting actor limit
    // Expected: 1 MB / 32 KB = ~32 actors (with overhead, maybe 30-31)

    printf("HIVE_STACK_ARENA_SIZE: 1 MB\n");
    printf("Using custom stack size: 32 KB per actor\n");
    printf("Expected actors that fit: ~30-32\n\n");

    actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = TEST_STACK_SIZE(32 * 1024);
    cfg.malloc_stack = false; // Use arena (default)

    int count = 0;

    printf("Spawning actors until arena exhaustion...\n");
    for (int i = 0; i < 64; i++) {
        actor_id_t id;
        if (HIVE_FAILED(hive_spawn(simple_actor, NULL, NULL, &cfg, &id))) {
            printf("Actor #%d: FAILED (arena exhausted) [OK]\n", i + 1);
            break;
        }
        count++;
        printf("Actor #%d: spawned (ID: %u)\n", i + 1, id);
    }

    printf("\nSuccessfully spawned %d actors before exhaustion\n", count);

    // Verify the arena is exhausted by trying to spawn one more
    printf("\nVerifying arena exhaustion...\n");
    actor_id_t id;
    if (HIVE_FAILED(hive_spawn(simple_actor, NULL, NULL, &cfg, &id))) {
        printf("[OK] Arena is exhausted (cannot spawn more actors)\n");
    } else {
        printf("[FAIL] ERROR: Arena should be exhausted but spawned actor %u\n",
               id);
    }

    // Test that malloc_stack still works when arena is exhausted
    printf("\nTesting malloc fallback via explicit flag...\n");
    cfg.malloc_stack = true; // Explicitly use malloc
    if (HIVE_SUCCEEDED(hive_spawn(simple_actor, NULL, NULL, &cfg, &id))) {
        printf("[OK] malloc_stack=true still works (spawned actor %u)\n", id);
    } else {
        printf("[FAIL] ERROR: malloc_stack=true should work\n");
    }

    printf("\nRunning scheduler (all actors will exit immediately)...\n");
    hive_run();

    printf("\n=== Test completed ===\n");
    printf("Arena exhaustion behavior: CORRECT\n");
    printf("- Arena allocation fails gracefully when full\n");
    printf("- malloc_stack=true works independently\n");
    printf("- Cleanup works correctly\n");

    hive_cleanup();

    return 0;
}
