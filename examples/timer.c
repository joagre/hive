#include "hive_runtime.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include <stdio.h>

// Timer test actor
static void timer_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Timer actor started (ID: %u)\n", hive_self());

    // Test one-shot timer (500ms)
    printf("Creating one-shot timer (500ms)...\n");
    hive_timer_id_t oneshot;
    hive_status_t status = hive_timer_after(500000, &oneshot);
    if (HIVE_FAILED(status)) {
        printf("Failed to create one-shot timer: %s\n", HIVE_ERR_STR(status));
        return;
    }
    printf("One-shot timer created (ID: %u)\n", oneshot);

    // Test periodic timer (200ms)
    printf("Creating periodic timer (200ms)...\n");
    hive_timer_id_t periodic;
    status = hive_timer_every(200000, &periodic);
    if (HIVE_FAILED(status)) {
        printf("Failed to create periodic timer: %s\n", HIVE_ERR_STR(status));
        return;
    }
    printf("Periodic timer created (ID: %u)\n", periodic);

    // Wait for timer ticks
    int periodic_count = 0;
    bool oneshot_received = false;
    bool done = false;

    while (!done) {
        hive_message_t msg;
        status = hive_ipc_recv(&msg, -1); // Block until message
        if (HIVE_FAILED(status)) {
            printf("Failed to receive message: %s\n", HIVE_ERR_STR(status));
            break;
        }

        if (hive_msg_is_timer(&msg)) {
            // Timer ID is directly available in msg.tag
            printf("Timer tick from timer ID: %u\n", msg.tag);

            if (msg.tag == oneshot) {
                printf("One-shot timer fired!\n");
                oneshot_received = true;
            } else if (msg.tag == periodic) {
                periodic_count++;
                printf("Periodic timer tick #%d\n", periodic_count);

                if (periodic_count >= 5) {
                    printf("Cancelling periodic timer...\n");
                    status = hive_timer_cancel(periodic);
                    if (HIVE_FAILED(status)) {
                        printf("Failed to cancel timer: %s\n",
                               HIVE_ERR_STR(status));
                    } else {
                        printf("Periodic timer cancelled\n");
                    }
                    done = true;
                }
            }
        }
    }

    printf("Timer test completed!\n");
    printf("One-shot received: %s\n", oneshot_received ? "yes" : "no");
    printf("Periodic ticks: %d\n", periodic_count);

    return;
}

int main(void) {
    printf("=== Actor Runtime Timer Example ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn timer test actor
    hive_actor_config_t actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "timer";

    hive_actor_id_t id;
    if (HIVE_FAILED(hive_spawn(timer_actor, NULL, NULL, &actor_cfg, &id))) {
        fprintf(stderr, "Failed to spawn timer actor\n");
        hive_cleanup();
        return 1;
    }

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
