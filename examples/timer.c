#include "acrt_runtime.h"
#include "acrt_timer.h"
#include "acrt_ipc.h"
#include <stdio.h>

// Timer test actor
static void timer_actor(void *arg) {
    (void)arg;

    printf("Timer actor started (ID: %u)\n", acrt_self());

    // Test one-shot timer (500ms)
    printf("Creating one-shot timer (500ms)...\n");
    timer_id oneshot;
    acrt_status status = acrt_timer_after(500000, &oneshot);
    if (ACRT_FAILED(status)) {
        printf("Failed to create one-shot timer: %s\n", ACRT_ERR_STR(status));
        acrt_exit();
    }
    printf("One-shot timer created (ID: %u)\n", oneshot);

    // Test periodic timer (200ms)
    printf("Creating periodic timer (200ms)...\n");
    timer_id periodic;
    status = acrt_timer_every(200000, &periodic);
    if (ACRT_FAILED(status)) {
        printf("Failed to create periodic timer: %s\n", ACRT_ERR_STR(status));
        acrt_exit();
    }
    printf("Periodic timer created (ID: %u)\n", periodic);

    // Wait for timer ticks
    int periodic_count = 0;
    bool oneshot_received = false;
    bool done = false;

    while (!done) {
        acrt_message msg;
        status = acrt_ipc_recv(&msg, -1);  // Block until message
        if (ACRT_FAILED(status)) {
            printf("Failed to receive message: %s\n", ACRT_ERR_STR(status));
            break;
        }

        if (acrt_msg_is_timer(&msg)) {
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
                    status = acrt_timer_cancel(periodic);
                    if (ACRT_FAILED(status)) {
                        printf("Failed to cancel timer: %s\n", ACRT_ERR_STR(status));
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

    acrt_exit();
}

int main(void) {
    printf("=== Actor Runtime Timer Example ===\n\n");

    // Initialize runtime
    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n", ACRT_ERR_STR(status));
        return 1;
    }

    // Spawn timer test actor
    actor_config actor_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "timer";

    actor_id id;
    if (ACRT_FAILED(acrt_spawn_ex(timer_actor, NULL, &actor_cfg, &id))) {
        fprintf(stderr, "Failed to spawn timer actor\n");
        acrt_cleanup();
        return 1;
    }

    // Run scheduler
    acrt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    acrt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
