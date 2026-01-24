#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <string.h>

// Message types
typedef struct {
    int count;
} ping_msg;

// Pong actor
static void pong_actor(void *args, const hive_spawn_info *siblings,
                       size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Pong actor started (ID: %u)\n", hive_self());

    actor_id ping_id = ACTOR_ID_INVALID;

    for (int i = 0; i < 5; i++) {
        // Wait for ping
        hive_message msg;
        hive_status status =
            hive_ipc_recv(&msg, -1); // Block until message arrives

        if (HIVE_FAILED(status)) {
            printf("Pong: Failed to receive message: %s\n",
                   HIVE_ERR_STR(status));
            break;
        }

        // Get ping actor ID from first message
        if (ping_id == ACTOR_ID_INVALID) {
            ping_id = msg.sender;
        }

        ping_msg pm_copy = *(ping_msg *)msg.data;
        printf("Pong: Received ping #%d from actor %u\n", pm_copy.count,
               msg.sender);

        // Send pong back
        pm_copy.count++;
        status =
            hive_ipc_notify(ping_id, HIVE_TAG_NONE, &pm_copy, sizeof(ping_msg));

        if (HIVE_FAILED(status)) {
            printf("Pong: Failed to send message: %s\n", HIVE_ERR_STR(status));
            break;
        }

        printf("Pong: Sent pong #%d\n", pm_copy.count);
    }

    printf("Pong actor exiting\n");
    hive_exit();
}

// Ping actor
static void ping_actor(void *args, const hive_spawn_info *siblings,
                       size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id pong_id = (actor_id)(uintptr_t)args;

    printf("Ping actor started (ID: %u)\n", hive_self());

    // Send first ping
    ping_msg pm = {.count = 0};
    hive_status status =
        hive_ipc_notify(pong_id, HIVE_TAG_NONE, &pm, sizeof(ping_msg));

    if (HIVE_FAILED(status)) {
        printf("Ping: Failed to send initial message: %s\n",
               HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Ping: Sent initial ping #%d\n", pm.count);

    for (int i = 0; i < 5; i++) {
        // Wait for pong
        hive_message msg;
        status = hive_ipc_recv(&msg, -1); // Block until message arrives

        if (HIVE_FAILED(status)) {
            printf("Ping: Failed to receive message: %s\n",
                   HIVE_ERR_STR(status));
            break;
        }

        ping_msg recv_pm = *(ping_msg *)msg.data;
        printf("Ping: Received pong #%d from actor %u\n", recv_pm.count,
               msg.sender);

        // Send ping back
        recv_pm.count++;
        status =
            hive_ipc_notify(pong_id, HIVE_TAG_NONE, &recv_pm, sizeof(ping_msg));

        if (HIVE_FAILED(status)) {
            printf("Ping: Failed to send message: %s\n", HIVE_ERR_STR(status));
            break;
        }

        printf("Ping: Sent ping #%d\n", recv_pm.count);
    }

    printf("Ping actor exiting\n");
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Ping-Pong Example ===\n\n");

    // Initialize runtime
    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    printf("Runtime initialized\n");

    // Spawn pong actor first
    actor_config pong_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    pong_cfg.name = "pong";
    pong_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id pong_id;
    if (HIVE_FAILED(hive_spawn(pong_actor, NULL, NULL, &pong_cfg, &pong_id))) {
        fprintf(stderr, "Failed to spawn pong actor\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned pong actor (ID: %u)\n", pong_id);

    // Spawn ping actor with pong's ID
    actor_config ping_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    ping_cfg.name = "ping";
    ping_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id ping_id;
    if (HIVE_FAILED(hive_spawn(ping_actor, NULL, (void *)(uintptr_t)pong_id,
                               &ping_cfg, &ping_id))) {
        fprintf(stderr, "Failed to spawn ping actor\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned ping actor (ID: %u)\n", ping_id);

    printf("\nStarting scheduler...\n\n");

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
