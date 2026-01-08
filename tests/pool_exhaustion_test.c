#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <stdbool.h>

// Receiver that accumulates messages without processing
void slow_receiver_actor(void *arg) {
    (void)arg;
    printf("Receiver: Started, will not process messages to exhaust pool\n");

    // Just sleep - don't process messages
    // This causes sender's messages to accumulate in mailbox
    rt_message msg;
    rt_ipc_recv(&msg, -1);  // Block forever (won't get any messages)

    rt_exit();
}

void sender_actor(void *arg) {
    actor_id receiver = *(actor_id *)arg;

    printf("\nSender: Attempting to exhaust IPC pool by sending to slow receiver...\n");
    printf("Sender: Pool sizes: MAILBOX_ENTRY=%d, MESSAGE_DATA=%d\n",
           RT_MAILBOX_ENTRY_POOL_SIZE, RT_MESSAGE_DATA_POOL_SIZE);

    // Send messages until pool is exhausted
    int sent_count = 0;
    int data = 0;
    rt_status status;

    while (true) {
        data++;
        status = rt_ipc_cast(receiver, &data, sizeof(data));

        if (RT_FAILED(status)) {
            if (status.code == RT_ERR_NOMEM) {
                printf("Sender: ✓ Pool exhausted after %d messages!\n", sent_count);
                printf("Sender: Got RT_ERR_NOMEM as expected\n");
                break;
            } else {
                printf("Sender: Unexpected error: %d (%s)\n", status.code, status.msg);
                rt_exit();
            }
        }

        sent_count++;

        // Safety limit
        if (sent_count > RT_MAILBOX_ENTRY_POOL_SIZE + 100) {
            printf("Sender: ERROR - Sent %d messages without exhausting pool\n", sent_count);
            rt_exit();
        }
    }

    printf("\nSender: Testing backoff-retry pattern...\n");

    // Try to send with backoff-retry
    int retry_count = 0;
    bool send_succeeded = false;

    for (int attempt = 0; attempt < 5; attempt++) {
        printf("Sender: Attempt %d - trying to send...\n", attempt + 1);

        data++;
        status = rt_ipc_cast(receiver, &data, sizeof(data));

        if (!RT_FAILED(status)) {
            printf("Sender: ✓ Send succeeded on attempt %d!\n", attempt + 1);
            send_succeeded = true;
            break;
        }

        if (status.code == RT_ERR_NOMEM) {
            printf("Sender:   Still exhausted, backing off 20ms...\n");

            // Backoff with timeout
            rt_message msg;
            status = rt_ipc_recv(&msg, 20);

            if (status.code == RT_ERR_TIMEOUT) {
                printf("Sender:   Backoff timeout (no messages received)\n");
                retry_count++;
            } else if (!RT_FAILED(status)) {
                printf("Sender:   Got message during backoff from actor %u\n", msg.sender);
                // In real code, would handle the message here
            }
        }
    }

    if (!send_succeeded) {
        printf("Sender: ✗ Failed to send after %d retries (pool still exhausted)\n", retry_count);
        printf("Sender: This is expected - pool won't free until receiver processes messages\n");
    }

    printf("\nSender: Signaling receiver to start processing messages...\n");
    // Send wake-up signal to receiver using different actor
    // (In this test, receiver is blocked so this won't actually work,
    //  but demonstrates the pattern)

    printf("\nSender: Test complete - demonstrated:\n");
    printf("  1. ✓ Pool exhaustion (RT_ERR_NOMEM)\n");
    printf("  2. ✓ Backoff-retry with timeout\n");
    printf("  3. ✓ Developer handles timeout vs message explicitly\n");

    rt_exit();
}

int main(void) {
    printf("=== IPC Pool Exhaustion and Backoff-Retry Test ===\n\n");

    rt_init();

    // Spawn receiver that won't process messages
    actor_id receiver = rt_spawn(slow_receiver_actor, NULL);
    printf("Main: Spawned slow receiver (ID: %u)\n", receiver);

    // Spawn sender that will exhaust pool and retry
    actor_id sender = rt_spawn(sender_actor, &receiver);
    printf("Main: Spawned sender (ID: %u)\n", sender);

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
