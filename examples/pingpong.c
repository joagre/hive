#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>
#include <string.h>

// Message types
typedef struct {
    int count;
} ping_msg;

// Pong actor
static void pong_actor(void *arg) {
    (void)arg; // Unused

    printf("Pong actor started (ID: %u)\n", rt_self());

    actor_id ping_id = ACTOR_ID_INVALID;

    for (int i = 0; i < 5; i++) {
        // Wait for ping
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, -1); // Block until message arrives

        if (RT_FAILED(status)) {
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
        rt_msg_decode(&msg, NULL, NULL, &payload, &payload_len);

        ping_msg pm_copy = *(ping_msg *)payload;
        printf("Pong: Received ping #%d from actor %u\n", pm_copy.count, msg.sender);

        // Send pong back
        pm_copy.count++;
        status = rt_ipc_send(ping_id, &pm_copy, sizeof(ping_msg));

        if (RT_FAILED(status)) {
            printf("Pong: Failed to send message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Pong: Sent pong #%d\n", pm_copy.count);
    }

    printf("Pong actor exiting\n");
    rt_exit();
}

// Ping actor
static void ping_actor(void *arg) {
    actor_id pong_id = (actor_id)(uintptr_t)arg;

    printf("Ping actor started (ID: %u)\n", rt_self());

    // Send first ping
    ping_msg pm = { .count = 0 };
    rt_status status = rt_ipc_send(pong_id, &pm, sizeof(ping_msg));

    if (RT_FAILED(status)) {
        printf("Ping: Failed to send initial message: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    printf("Ping: Sent initial ping #%d\n", pm.count);

    for (int i = 0; i < 5; i++) {
        // Wait for pong
        rt_message msg;
        status = rt_ipc_recv(&msg, -1); // Block until message arrives

        if (RT_FAILED(status)) {
            printf("Ping: Failed to receive message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        // Decode message payload
        const void *payload;
        rt_msg_decode(&msg, NULL, NULL, &payload, NULL);

        ping_msg recv_pm = *(ping_msg *)payload;
        printf("Ping: Received pong #%d from actor %u\n", recv_pm.count, msg.sender);

        // Send ping back
        recv_pm.count++;
        status = rt_ipc_send(pong_id, &recv_pm, sizeof(ping_msg));

        if (RT_FAILED(status)) {
            printf("Ping: Failed to send message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Ping: Sent ping #%d\n", recv_pm.count);
    }

    printf("Ping actor exiting\n");
    rt_exit();
}

int main(void) {
    printf("=== Actor Runtime Ping-Pong Example ===\n\n");

    // Initialize runtime
    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    printf("Runtime initialized\n");

    // Spawn pong actor first
    actor_config pong_cfg = RT_ACTOR_CONFIG_DEFAULT;
    pong_cfg.name = "pong";
    pong_cfg.priority = RT_PRIO_NORMAL;

    actor_id pong_id = rt_spawn_ex(pong_actor, NULL, &pong_cfg);
    if (pong_id == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn pong actor\n");
        rt_cleanup();
        return 1;
    }

    printf("Spawned pong actor (ID: %u)\n", pong_id);

    // Spawn ping actor with pong's ID
    actor_config ping_cfg = RT_ACTOR_CONFIG_DEFAULT;
    ping_cfg.name = "ping";
    ping_cfg.priority = RT_PRIO_NORMAL;

    actor_id ping_id = rt_spawn_ex(ping_actor, (void *)(uintptr_t)pong_id, &ping_cfg);
    if (ping_id == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn ping actor\n");
        rt_cleanup();
        return 1;
    }

    printf("Spawned ping actor (ID: %u)\n", ping_id);

    printf("\nStarting scheduler...\n\n");

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    rt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
