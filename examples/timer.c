#include "rt_runtime.h"
#include "rt_timer.h"
#include "rt_ipc.h"
#include <stdio.h>

// Timer test actor
static void timer_actor(void *arg) {
    (void)arg;

    printf("Timer actor started (ID: %u)\n", rt_self());

    // Test one-shot timer (500ms)
    printf("Creating one-shot timer (500ms)...\n");
    timer_id oneshot;
    rt_status status = rt_timer_after(500000, &oneshot);
    if (RT_FAILED(status)) {
        printf("Failed to create one-shot timer: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }
    printf("One-shot timer created (ID: %u)\n", oneshot);

    // Test periodic timer (200ms)
    printf("Creating periodic timer (200ms)...\n");
    timer_id periodic;
    status = rt_timer_every(200000, &periodic);
    if (RT_FAILED(status)) {
        printf("Failed to create periodic timer: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }
    printf("Periodic timer created (ID: %u)\n", periodic);

    // Wait for timer ticks
    int periodic_count = 0;
    bool oneshot_received = false;
    bool done = false;

    while (!done) {
        rt_message msg;
        status = rt_ipc_recv(&msg, -1);  // Block until message
        if (RT_FAILED(status)) {
            printf("Failed to receive message: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        if (rt_msg_is_timer(&msg)) {
            // Timer ID is encoded in the message tag
            uint32_t tick_id;
            rt_msg_decode(&msg, NULL, &tick_id, NULL, NULL);
            printf("Timer tick from timer ID: %u\n", tick_id);

            if (tick_id == oneshot) {
                printf("One-shot timer fired!\n");
                oneshot_received = true;
            } else if (tick_id == periodic) {
                periodic_count++;
                printf("Periodic timer tick #%d\n", periodic_count);

                if (periodic_count >= 5) {
                    printf("Cancelling periodic timer...\n");
                    status = rt_timer_cancel(periodic);
                    if (RT_FAILED(status)) {
                        printf("Failed to cancel timer: %s\n",
                               status.msg ? status.msg : "unknown error");
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

    rt_exit();
}

int main(void) {
    printf("=== Actor Runtime Timer Example ===\n\n");

    // Initialize runtime
    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Spawn timer test actor
    actor_config actor_cfg = RT_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "timer";

    actor_id id = rt_spawn_ex(timer_actor, NULL, &actor_cfg);
    if (id == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn timer actor\n");
        rt_cleanup();
        return 1;
    }

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    rt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
