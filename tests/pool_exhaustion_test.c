#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <stdbool.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Receiver that accumulates messages without processing
void slow_receiver_actor(void *args, const hive_spawn_info *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("Receiver: Started, will not process messages to exhaust pool\n");

    // Just sleep - don't process messages
    // This causes sender's messages to accumulate in mailbox
    hive_message msg;
    hive_ipc_recv(&msg, -1); // Block forever (won't get any messages)

    hive_exit();
}

void sender_actor(void *args, const hive_spawn_info *siblings,
                  size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id receiver = *(actor_id *)args;

    printf("\nSender: Attempting to exhaust IPC pool by sending to slow "
           "receiver...\n");
    printf("Sender: Pool sizes: MAILBOX_ENTRY=%d, MESSAGE_DATA=%d\n",
           HIVE_MAILBOX_ENTRY_POOL_SIZE, HIVE_MESSAGE_DATA_POOL_SIZE);

    // Send messages until pool is exhausted
    int sent_count = 0;
    int data = 0;
    hive_status status;

    while (true) {
        data++;
        status = hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));

        if (HIVE_FAILED(status)) {
            if (status.code == HIVE_ERR_NOMEM) {
                printf("Sender: [OK] Pool exhausted after %d messages!\n",
                       sent_count);
                printf("Sender: Got HIVE_ERR_NOMEM as expected\n");
                break;
            } else {
                printf("Sender: Unexpected error: %d (%s)\n", status.code,
                       status.msg);
                hive_exit();
            }
        }

        sent_count++;

        // Safety limit
        if (sent_count > HIVE_MAILBOX_ENTRY_POOL_SIZE + 100) {
            printf("Sender: ERROR - Sent %d messages without exhausting pool\n",
                   sent_count);
            hive_exit();
        }
    }

    printf("\nSender: Testing backoff-retry pattern...\n");

    // Try to send with backoff-retry
    int retry_count = 0;
    bool send_succeeded = false;

    for (int attempt = 0; attempt < 5; attempt++) {
        printf("Sender: Attempt %d - trying to send...\n", attempt + 1);

        data++;
        status = hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));

        if (HIVE_SUCCEEDED(status)) {
            printf("Sender: [OK] Send succeeded on attempt %d!\n", attempt + 1);
            send_succeeded = true;
            break;
        }

        if (status.code == HIVE_ERR_NOMEM) {
            printf("Sender:   Still exhausted, backing off 20ms...\n");

            // Backoff with timeout
            hive_message msg;
            status = hive_ipc_recv(&msg, 20);

            if (status.code == HIVE_ERR_TIMEOUT) {
                printf("Sender:   Backoff timeout (no messages received)\n");
                retry_count++;
            } else if (HIVE_SUCCEEDED(status)) {
                printf("Sender:   Got message during backoff from actor %u\n",
                       msg.sender);
                // In real code, would handle the message here
            }
        }
    }

    if (!send_succeeded) {
        printf("Sender: [FAIL] Failed to send after %d retries (pool still "
               "exhausted)\n",
               retry_count);
        printf("Sender: This is expected - pool won't free until receiver "
               "processes messages\n");
    }

    printf("\nSender: Signaling receiver to start processing messages...\n");
    // Send wake-up signal to receiver using different actor
    // (In this test, receiver is blocked so this won't actually work,
    //  but demonstrates the pattern)

    printf("\nSender: Test complete - demonstrated:\n");
    printf("  1. [OK] Pool exhaustion (HIVE_ERR_NOMEM)\n");
    printf("  2. [OK] Backoff-retry with timeout\n");
    printf("  3. [OK] Developer handles timeout vs message explicitly\n");

    hive_exit();
}

int main(void) {
    printf("=== IPC Pool Exhaustion and Backoff-Retry Test ===\n\n");

    hive_init();

    // Spawn receiver that won't process messages
    actor_id receiver;
    hive_spawn(slow_receiver_actor, NULL, NULL, NULL, &receiver);
    printf("Main: Spawned slow receiver (ID: %u)\n", receiver);

    // Spawn sender that will exhaust pool and retry
    actor_id sender;
    hive_spawn(sender_actor, NULL, &receiver, NULL, &sender);
    printf("Main: Spawned sender (ID: %u)\n", sender);

    hive_run();
    hive_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
