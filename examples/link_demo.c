#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>

// Shared actor IDs
static hive_actor_id_t g_actor_a = HIVE_ACTOR_ID_INVALID;
static hive_actor_id_t g_actor_b = HIVE_ACTOR_ID_INVALID;

// Actor A - links to B, then waits for exit notification
static void actor_a(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Actor A started (ID: %u)\n", hive_self());
    printf("Actor A: Waiting for Actor B to spawn...\n");

    // Wait a bit for B to spawn
    hive_timer_id_t wait_timer;
    hive_timer_after(100000, &wait_timer); // 100ms

    hive_message_t msg;
    hive_timer_recv(wait_timer, &msg, -1);
    printf("Actor A: Timer fired, linking to Actor B...\n");

    // Link to Actor B
    hive_status_t status = hive_link(g_actor_b);
    if (HIVE_FAILED(status)) {
        printf("Actor A: Failed to link to B: %s\n", HIVE_ERR_STR(status));
        return;
    }

    printf("Actor A: Successfully linked to Actor B\n");
    printf("Actor A: Waiting for exit notification from B...\n");

    // Wait for exit notification
    hive_ipc_recv(&msg, -1);

    if (msg.class == HIVE_MSG_EXIT) {
        hive_exit_msg_t *exit_info = (hive_exit_msg_t *)msg.data;

        printf("Actor A: Received exit notification!\n");
        printf("Actor A:   Died actor: %u\n", exit_info->actor);
        printf("Actor A:   Exit reason: %u\n", (unsigned)exit_info->reason);
    } else {
        printf("Actor A: Received unexpected message from %u\n", msg.sender);
    }

    printf("Actor A: Exiting normally\n");
    return;
}

// Actor B - waits a bit, then exits normally
static void actor_b(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Actor B started (ID: %u)\n", hive_self());
    printf("Actor B: Waiting 500ms before exiting...\n");

    // Wait 500ms
    hive_timer_id_t wait_timer;
    hive_timer_after(500000, &wait_timer);

    hive_message_t msg;
    hive_timer_recv(wait_timer, &msg, -1);

    printf("Actor B: Exiting normally\n");
    return;
}

int main(void) {
    printf("=== Actor Runtime Link Demo ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn Actor B first
    hive_actor_config_t actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "actor_b";
    if (HIVE_FAILED(hive_spawn(actor_b, NULL, NULL, &actor_cfg, &g_actor_b))) {
        fprintf(stderr, "Failed to spawn Actor B\n");
        hive_cleanup();
        return 1;
    }

    // Spawn Actor A
    actor_cfg.name = "actor_a";
    if (HIVE_FAILED(hive_spawn(actor_a, NULL, NULL, &actor_cfg, &g_actor_a))) {
        fprintf(stderr, "Failed to spawn Actor A\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned Actor A (ID: %u) and Actor B (ID: %u)\n\n", g_actor_a,
           g_actor_b);

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Demo completed ===\n");

    return 0;
}
