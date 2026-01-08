#include "acrt_runtime.h"
#include "acrt_ipc.h"
#include <stdio.h>
#include <string.h>

// Message types
typedef struct {
    int count;
} ping_msg;

// Pong actor
static void pong_actor(void *arg) {
    (void)arg; // Unused

    printf("Pong actor started (ID: %u)\n", acrt_self());

    actor_id ping_id = ACTOR_ID_INVALID;

    for (int i = 0; i < 5; i++) {
        // Wait for ping
        acrt_message msg;
        acrt_status status = acrt_ipc_recv(&msg, -1); // Block until message arrives

        if (ACRT_FAILED(status)) {
            printf("Pong: Failed to receive message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        // Get ping actor ID from first message
        if (ping_id == ACTOR_ID_INVALID) {
            ping_id = msg.sender;
        }

        // Decode message payload
        const void *payload;
        size_t payload_len;
        acrt_msg_decode(&msg, NULL, NULL, &payload, &payload_len);

        ping_msg pm_copy = *(ping_msg *)payload;
        printf("Pong: Received ping #%d from actor %u\n", pm_copy.count, msg.sender);

        // Send pong back
        pm_copy.count++;
        status = acrt_ipc_notify(ping_id, &pm_copy, sizeof(ping_msg));

        if (ACRT_FAILED(status)) {
            printf("Pong: Failed to send message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Pong: Sent pong #%d\n", pm_copy.count);
    }

    printf("Pong actor exiting\n");
    acrt_exit();
}

// Ping actor
static void ping_actor(void *arg) {
    actor_id pong_id = (actor_id)(uintptr_t)arg;

    printf("Ping actor started (ID: %u)\n", acrt_self());

    // Send first ping
    ping_msg pm = { .count = 0 };
    acrt_status status = acrt_ipc_notify(pong_id, &pm, sizeof(ping_msg));

    if (ACRT_FAILED(status)) {
        printf("Ping: Failed to send initial message: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("Ping: Sent initial ping #%d\n", pm.count);

    for (int i = 0; i < 5; i++) {
        // Wait for pong
        acrt_message msg;
        status = acrt_ipc_recv(&msg, -1); // Block until message arrives

        if (ACRT_FAILED(status)) {
            printf("Ping: Failed to receive message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        // Decode message payload
        const void *payload;
        acrt_msg_decode(&msg, NULL, NULL, &payload, NULL);

        ping_msg recv_pm = *(ping_msg *)payload;
        printf("Ping: Received pong #%d from actor %u\n", recv_pm.count, msg.sender);

        // Send ping back
        recv_pm.count++;
        status = acrt_ipc_notify(pong_id, &recv_pm, sizeof(ping_msg));

        if (ACRT_FAILED(status)) {
            printf("Ping: Failed to send message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Ping: Sent ping #%d\n", recv_pm.count);
    }

    printf("Ping actor exiting\n");
    acrt_exit();
}

int main(void) {
    printf("=== Actor Runtime Ping-Pong Example ===\n\n");

    // Initialize runtime
    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    printf("Runtime initialized\n");

    // Spawn pong actor first
    actor_config pong_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    pong_cfg.name = "pong";
    pong_cfg.priority = ACRT_PRIORITY_NORMAL;

    actor_id pong_id;
    if (ACRT_FAILED(acrt_spawn_ex(pong_actor, NULL, &pong_cfg, &pong_id))) {
        fprintf(stderr, "Failed to spawn pong actor\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned pong actor (ID: %u)\n", pong_id);

    // Spawn ping actor with pong's ID
    actor_config ping_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    ping_cfg.name = "ping";
    ping_cfg.priority = ACRT_PRIORITY_NORMAL;

    actor_id ping_id;
    if (ACRT_FAILED(acrt_spawn_ex(ping_actor, (void *)(uintptr_t)pong_id, &ping_cfg, &ping_id))) {
        fprintf(stderr, "Failed to spawn ping actor\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned ping actor (ID: %u)\n", ping_id);

    printf("\nStarting scheduler...\n\n");

    // Run scheduler
    acrt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    acrt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
