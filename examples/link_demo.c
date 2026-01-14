#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>

// Shared actor IDs
static actor_id g_actor_a = ACTOR_ID_INVALID;
static actor_id g_actor_b = ACTOR_ID_INVALID;

// Actor A - links to B, then waits for exit notification
static void actor_a(void *arg) {
    (void)arg;

    printf("Actor A started (ID: %u)\n", hive_self());
    printf("Actor A: Waiting for Actor B to spawn...\n");

    // Wait a bit for B to spawn
    timer_id wait_timer;
    hive_timer_after(100000, &wait_timer);  // 100ms

    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, wait_timer, &msg, -1);
    printf("Actor A: Timer fired, linking to Actor B...\n");

    // Link to Actor B
    hive_status status = hive_link(g_actor_b);
    if (HIVE_FAILED(status)) {
        printf("Actor A: Failed to link to B: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Actor A: Successfully linked to Actor B\n");
    printf("Actor A: Waiting for exit notification from B...\n");

    // Wait for exit notification
    hive_ipc_recv(&msg, -1);

    if (msg.class == HIVE_MSG_EXIT) {
        hive_exit_msg *exit_info = (hive_exit_msg *)msg.data;

        printf("Actor A: Received exit notification!\n");
        printf("Actor A:   Died actor: %u\n", exit_info->actor);
        printf("Actor A:   Exit reason: %s\n", hive_exit_reason_str(exit_info->reason));
    } else {
        printf("Actor A: Received unexpected message from %u\n", msg.sender);
    }

    printf("Actor A: Exiting normally\n");
    hive_exit();
}

// Actor B - waits a bit, then exits normally
static void actor_b(void *arg) {
    (void)arg;

    printf("Actor B started (ID: %u)\n", hive_self());
    printf("Actor B: Waiting 500ms before exiting...\n");

    // Wait 500ms
    timer_id wait_timer;
    hive_timer_after(500000, &wait_timer);

    hive_message msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, wait_timer, &msg, -1);

    printf("Actor B: Exiting normally\n");
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Link Demo ===\n\n");

    // Initialize runtime
    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n", HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn Actor B first
    actor_config actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "actor_b";
    if (HIVE_FAILED(hive_spawn_ex(actor_b, NULL, &actor_cfg, &g_actor_b))) {
        fprintf(stderr, "Failed to spawn Actor B\n");
        hive_cleanup();
        return 1;
    }

    // Spawn Actor A
    actor_cfg.name = "actor_a";
    if (HIVE_FAILED(hive_spawn_ex(actor_a, NULL, &actor_cfg, &g_actor_a))) {
        fprintf(stderr, "Failed to spawn Actor A\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned Actor A (ID: %u) and Actor B (ID: %u)\n\n", g_actor_a, g_actor_b);

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Demo completed ===\n");

    return 0;
}
