#include "acrt_runtime.h"
#include "acrt_link.h"
#include "acrt_ipc.h"
#include "acrt_timer.h"
#include <stdio.h>

// Shared actor IDs
static actor_id g_actor_a = ACTOR_ID_INVALID;
static actor_id g_actor_b = ACTOR_ID_INVALID;

// Actor A - links to B, then waits for exit notification
static void actor_a(void *arg) {
    (void)arg;

    printf("Actor A started (ID: %u)\n", acrt_self());
    printf("Actor A: Waiting for Actor B to spawn...\n");

    // Wait a bit for B to spawn
    timer_id wait_timer;
    acrt_timer_after(100000, &wait_timer);  // 100ms

    acrt_message msg;
    acrt_ipc_recv(&msg, -1);
    if (acrt_msg_is_timer(&msg)) {
        printf("Actor A: Timer fired, linking to Actor B...\n");
    }

    // Link to Actor B
    acrt_status status = acrt_link(g_actor_b);
    if (ACRT_FAILED(status)) {
        printf("Actor A: Failed to link to B: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("Actor A: Successfully linked to Actor B\n");
    printf("Actor A: Waiting for exit notification from B...\n");

    // Wait for exit notification
    acrt_ipc_recv(&msg, -1);

    if (acrt_is_exit_msg(&msg)) {
        acrt_exit_msg exit_info;
        acrt_decode_exit(&msg, &exit_info);

        printf("Actor A: Received exit notification!\n");
        printf("Actor A:   Died actor: %u\n", exit_info.actor);
        printf("Actor A:   Exit reason: %s\n",
               exit_info.reason == ACRT_EXIT_NORMAL ? "NORMAL" :
               exit_info.reason == ACRT_EXIT_CRASH ? "CRASH" : "KILLED");
    } else {
        printf("Actor A: Received unexpected message from %u\n", msg.sender);
    }

    printf("Actor A: Exiting normally\n");
    acrt_exit();
}

// Actor B - waits a bit, then exits normally
static void actor_b(void *arg) {
    (void)arg;

    printf("Actor B started (ID: %u)\n", acrt_self());
    printf("Actor B: Waiting 500ms before exiting...\n");

    // Wait 500ms
    timer_id wait_timer;
    acrt_timer_after(500000, &wait_timer);

    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    printf("Actor B: Exiting normally\n");
    acrt_exit();
}

int main(void) {
    printf("=== Actor Runtime Link Demo ===\n\n");

    // Initialize runtime
    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Spawn Actor B first
    actor_config actor_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "actor_b";
    if (ACRT_FAILED(acrt_spawn_ex(actor_b, NULL, &actor_cfg, &g_actor_b))) {
        fprintf(stderr, "Failed to spawn Actor B\n");
        acrt_cleanup();
        return 1;
    }

    // Spawn Actor A
    actor_cfg.name = "actor_a";
    if (ACRT_FAILED(acrt_spawn_ex(actor_a, NULL, &actor_cfg, &g_actor_a))) {
        fprintf(stderr, "Failed to spawn Actor A\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned Actor A (ID: %u) and Actor B (ID: %u)\n\n", g_actor_a, g_actor_b);

    // Run scheduler
    acrt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    acrt_cleanup();

    printf("\n=== Demo completed ===\n");

    return 0;
}
