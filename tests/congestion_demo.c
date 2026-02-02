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
    hive_actor_id_t workers[NUM_WORKERS];
    int worker_count;
} coordinator_args_t;

// Worker that processes messages
void worker_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = *(int *)args;
    int processed = 0;

    while (true) {
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 500); // 500ms timeout

        if (status.code == HIVE_ERR_TIMEOUT) {
            // No more work
            break;
        }

        if (HIVE_SUCCEEDED(status)) {
            processed++;
        }
    }

    printf("Worker %d: Processed %d messages\n", id, processed);
    return;
}

// Coordinator that distributes work with backoff-retry
void coordinator_actor(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    coordinator_args_t *cargs = (coordinator_args_t *)args;

    printf("\nCoordinator: Distributing %d messages to %d workers...\n",
           BURST_SIZE * NUM_WORKERS, NUM_WORKERS);

    int total_sent = 0;
    int retry_needed = 0;
    int retry_success = 0;

    // Notify bursts to each worker
    for (int burst = 0; burst < BURST_SIZE; burst++) {
        for (int w = 0; w < cargs->worker_count; w++) {
            int data = burst * NUM_WORKERS + w;

            hive_status_t status = hive_ipc_notify(
                cargs->workers[w], HIVE_TAG_NONE, &data, sizeof(data));

            if (status.code == HIVE_ERR_NOMEM) {
                retry_needed++;

                if (retry_needed == 1) {
                    printf("Coordinator: Pool exhausted! Using "
                           "backoff-retry...\n");
                }

                // Backoff-retry pattern
                hive_message_t msg;
                hive_ipc_recv(&msg, 5); // Backoff 5ms

                // Retry
                status = hive_ipc_notify(cargs->workers[w], HIVE_TAG_NONE,
                                         &data, sizeof(data));
                if (HIVE_SUCCEEDED(status)) {
                    retry_success++;
                    total_sent++;
                } else {
                    // Even retry failed - aggressive backoff
                    hive_ipc_recv(&msg, 20);
                    status = hive_ipc_notify(cargs->workers[w], HIVE_TAG_NONE,
                                             &data, sizeof(data));
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

    return;
}

int main(void) {
    printf("=== Congestion Handling with Backoff-Retry ===\n");
    printf("\nScenario: Coordinator notifies bursts to multiple workers\n");
    printf("Expected: Temporary pool exhaustion handled by backoff-retry\n");

    hive_init();

    coordinator_args_t args;
    args.worker_count = NUM_WORKERS;

    // Spawn workers
    static int worker_ids[NUM_WORKERS];
    for (int i = 0; i < NUM_WORKERS; i++) {
        worker_ids[i] = i + 1;
        hive_spawn(worker_actor, NULL, &worker_ids[i], NULL, &args.workers[i]);
    }
    printf("Main: Spawned %d workers\n", NUM_WORKERS);

    // Spawn coordinator
    hive_actor_id_t coordinator;
    hive_spawn(coordinator_actor, NULL, &args, NULL, &coordinator);
    printf("Main: Spawned coordinator\n");

    hive_run();
    hive_cleanup();

    printf("\n=== Demo Complete ===\n");
    return 0;
}
