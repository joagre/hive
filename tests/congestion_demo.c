#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>
#include <stdbool.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

#define NUM_WORKERS 3
#define BURST_SIZE 100

typedef struct {
    actor_id workers[NUM_WORKERS];
    int worker_count;
} coordinator_args;

// Worker that processes messages
void worker_actor(void *arg) {
    int id = *(int *)arg;
    int processed = 0;

    while (true) {
        hive_message msg;
        hive_status status = hive_ipc_recv(&msg, 500); // 500ms timeout

        if (status.code == HIVE_ERR_TIMEOUT) {
            // No more work
            break;
        }

        if (HIVE_SUCCEEDED(status)) {
            processed++;
        }
    }

    printf("Worker %d: Processed %d messages\n", id, processed);
    hive_exit();
}

// Coordinator that distributes work with backoff-retry
void coordinator_actor(void *arg) {
    coordinator_args *args = (coordinator_args *)arg;

    printf("\nCoordinator: Distributing %d messages to %d workers...\n",
           BURST_SIZE * NUM_WORKERS, NUM_WORKERS);

    int total_sent = 0;
    int retry_needed = 0;
    int retry_success = 0;

    // Send bursts to each worker
    for (int burst = 0; burst < BURST_SIZE; burst++) {
        for (int w = 0; w < args->worker_count; w++) {
            int data = burst * NUM_WORKERS + w;

            hive_status status =
                hive_ipc_notify(args->workers[w], 0, &data, sizeof(data));

            if (status.code == HIVE_ERR_NOMEM) {
                retry_needed++;

                if (retry_needed == 1) {
                    printf("Coordinator: Pool exhausted! Using "
                           "backoff-retry...\n");
                }

                // Backoff-retry pattern
                hive_message msg;
                hive_ipc_recv(&msg, 5); // Backoff 5ms

                // Retry
                status =
                    hive_ipc_notify(args->workers[w], 0, &data, sizeof(data));
                if (HIVE_SUCCEEDED(status)) {
                    retry_success++;
                    total_sent++;
                } else {
                    // Even retry failed - aggressive backoff
                    hive_ipc_recv(&msg, 20);
                    status = hive_ipc_notify(args->workers[w], 0, &data,
                                             sizeof(data));
                    if (HIVE_SUCCEEDED(status)) {
                        retry_success++;
                        total_sent++;
                    }
                }
            } else if (HIVE_SUCCEEDED(status)) {
                total_sent++;
            }
        }

        // Yield periodically to let workers process
        if (burst % 20 == 0) {
            hive_yield();
        }
    }

    printf("\nCoordinator: Distribution complete\n");
    printf("  Total sent: %d / %d\n", total_sent, BURST_SIZE * NUM_WORKERS);
    printf("  Retries needed: %d\n", retry_needed);
    printf("  Retries succeeded: %d\n", retry_success);

    if (retry_needed > 0) {
        printf("\n[OK] Backoff-retry handled temporary congestion\n");
        printf("  Without retry, %d messages would have been lost\n",
               retry_needed);
    }

    hive_exit();
}

int main(void) {
    printf("=== Congestion Handling with Backoff-Retry ===\n");
    printf("\nScenario: Coordinator sends bursts to multiple workers\n");
    printf("Expected: Temporary pool exhaustion handled by backoff-retry\n");

    hive_init();

    coordinator_args args;
    args.worker_count = NUM_WORKERS;

    // Spawn workers
    static int worker_ids[NUM_WORKERS];
    for (int i = 0; i < NUM_WORKERS; i++) {
        worker_ids[i] = i + 1;
        hive_spawn(worker_actor, &worker_ids[i], &args.workers[i]);
    }
    printf("Main: Spawned %d workers\n", NUM_WORKERS);

    // Spawn coordinator
    actor_id coordinator;
    hive_spawn(coordinator_actor, &args, &coordinator);
    printf("Main: Spawned coordinator\n");

    hive_run();
    hive_cleanup();

    printf("\n=== Demo Complete ===\n");
    return 0;
}
