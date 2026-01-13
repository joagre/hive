#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_static_config.h"
#include <stdio.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Slow processor that drains messages gradually
void slow_processor_actor(void *arg) {
    (void)arg;
    int processed = 0;

    printf("Processor: Starting to process messages slowly...\n");

    while (processed < 260) {
        hive_message msg;
        hive_status status = hive_ipc_recv(&msg, 50);  // 50ms timeout

        if (HIVE_SUCCEEDED(status)) {
            processed++;
            if (processed % 50 == 0) {
                printf("Processor: Processed %d messages (freeing pool space)...\n", processed);
            }
            // Process slowly to allow sender to retry
            hive_yield();
            hive_yield();
        } else if (status.code == HIVE_ERR_TIMEOUT) {
            // No more messages for now
            printf("Processor: No messages available, total processed: %d\n", processed);
            break;
        }
    }

    printf("Processor: Finished, processed %d total messages\n", processed);
    hive_exit();
}

void aggressive_sender_actor(void *arg) {
    actor_id processor = *(actor_id *)arg;

    printf("\nSender: Aggressively sending messages until pool exhausts...\n");

    int sent = 0;
    int failed = 0;
    int succeeded_after_retry = 0;

    // Phase 1: Fill the pool
    for (int i = 0; i < 300; i++) {
        int data = i;
        hive_status status = hive_ipc_notify(processor, 0, &data, sizeof(data));

        if (HIVE_SUCCEEDED(status)) {
            sent++;
        } else if (status.code == HIVE_ERR_NOMEM) {
            failed++;

            if (failed == 1) {
                printf("\nSender: ✓ Pool exhausted after %d successful sends\n", sent);
                printf("Sender: Beginning backoff-retry pattern...\n\n");
            }

            // Backoff-retry pattern
            hive_message msg;
            hive_status recv_status = hive_ipc_recv(&msg, 15);  // Backoff 15ms

            if (recv_status.code == HIVE_ERR_TIMEOUT) {
                // No messages during backoff - just retry
            } else if (HIVE_SUCCEEDED(recv_status)) {
                // Got a message during backoff
                printf("Sender: Received message during backoff from actor %u\n", msg.sender);
            }

            // Retry the send
            status = hive_ipc_notify(processor, 0, &data, sizeof(data));
            if (HIVE_SUCCEEDED(status)) {
                succeeded_after_retry++;
                if (succeeded_after_retry == 1) {
                    printf("Sender: ✓ First retry succeeded! (pool space became available)\n");
                }
                if (succeeded_after_retry % 20 == 0) {
                    printf("Sender: %d retries succeeded (processor is draining pool)...\n",
                           succeeded_after_retry);
                }
                sent++;
            } else {
                // Still failed after retry - break out
                if (failed > 5) {
                    printf("Sender: Still failing after %d attempts, stopping\n", failed);
                    break;
                }
            }
        }

        // Yield occasionally to let processor run
        if (i % 10 == 0) {
            hive_yield();
        }
    }

    printf("\nSender: Final stats:\n");
    printf("  - Total sent: %d\n", sent);
    printf("  - Initial failures: %d\n", failed);
    printf("  - Succeeded after retry: %d\n", succeeded_after_retry);

    if (succeeded_after_retry > 0) {
        printf("\n✓ Backoff-retry pattern WORKS!\n");
        printf("  Pool space became available as receiver processed messages\n");
    }

    hive_exit();
}

int main(void) {
    printf("=== Simple Backoff-Retry Test ===\n\n");
    printf("Pool: HIVE_MAILBOX_ENTRY_POOL_SIZE = %d\n", HIVE_MAILBOX_ENTRY_POOL_SIZE);
    printf("Strategy: Aggressive sender + slow processor = pool exhaustion + recovery\n");

    hive_init();

    actor_id processor;
    hive_spawn(slow_processor_actor, NULL, &processor);
    printf("Main: Spawned slow processor (ID: %u)\n", processor);

    actor_id sender;
    hive_spawn(aggressive_sender_actor, &processor, &sender);
    printf("Main: Spawned aggressive sender\n");

    hive_run();
    hive_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
