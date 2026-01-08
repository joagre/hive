#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_static_config.h"
#include <stdio.h>

// Slow processor that drains messages gradually
void slow_processor_actor(void *arg) {
    (void)arg;
    int processed = 0;

    printf("Processor: Starting to process messages slowly...\n");

    while (processed < 260) {
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 50);  // 50ms timeout

        if (!RT_FAILED(status)) {
            processed++;
            if (processed % 50 == 0) {
                printf("Processor: Processed %d messages (freeing pool space)...\n", processed);
            }
            // Process slowly to allow sender to retry
            rt_yield();
            rt_yield();
        } else if (status.code == RT_ERR_TIMEOUT) {
            // No more messages for now
            printf("Processor: No messages available, total processed: %d\n", processed);
            break;
        }
    }

    printf("Processor: Finished, processed %d total messages\n", processed);
    rt_exit();
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
        rt_status status = rt_ipc_send(processor, &data, sizeof(data));

        if (!RT_FAILED(status)) {
            sent++;
        } else if (status.code == RT_ERR_NOMEM) {
            failed++;

            if (failed == 1) {
                printf("\nSender: ✓ Pool exhausted after %d successful sends\n", sent);
                printf("Sender: Beginning backoff-retry pattern...\n\n");
            }

            // Backoff-retry pattern
            rt_message msg;
            rt_status recv_status = rt_ipc_recv(&msg, 15);  // Backoff 15ms

            if (recv_status.code == RT_ERR_TIMEOUT) {
                // No messages during backoff - just retry
            } else if (!RT_FAILED(recv_status)) {
                // Got a message during backoff
                printf("Sender: Received message during backoff from actor %u\n", msg.sender);
            }

            // Retry the send
            status = rt_ipc_send(processor, &data, sizeof(data));
            if (!RT_FAILED(status)) {
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
            rt_yield();
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

    rt_exit();
}

int main(void) {
    printf("=== Simple Backoff-Retry Test ===\n\n");
    printf("Pool: RT_MAILBOX_ENTRY_POOL_SIZE = %d\n", RT_MAILBOX_ENTRY_POOL_SIZE);
    printf("Strategy: Aggressive sender + slow processor = pool exhaustion + recovery\n");

    rt_init();

    actor_id processor = rt_spawn(slow_processor_actor, NULL);
    printf("Main: Spawned slow processor (ID: %u)\n", processor);

    rt_spawn(aggressive_sender_actor, &processor);
    printf("Main: Spawned aggressive sender\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
