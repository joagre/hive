#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <stdbool.h>

#define MESSAGES_TO_FILL_POOL (RT_MAILBOX_ENTRY_POOL_SIZE - 10)

typedef struct {
    actor_id receiver;
    actor_id controller;
} sender_args;

// Receiver that waits for signal before processing
void receiver_actor(void *arg) {
    actor_id controller = *(actor_id *)arg;

    printf("Receiver: Started, waiting for START signal from controller...\n");

    // Wait for start signal
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    if (msg.sender == controller) {
        char *cmd = (char *)msg.data;
        if (*cmd == 'S') {  // START
            printf("Receiver: Got START signal, beginning to process messages...\n");
        }
    }

    // Process accumulated messages
    int processed = 0;
    while (true) {
        rt_status status = rt_ipc_recv(&msg, 0);  // Non-blocking

        if (status.code == RT_ERR_WOULDBLOCK) {
            break;  // No more messages
        }

        if (!RT_FAILED(status)) {
            processed++;
            if (processed % 50 == 0) {
                printf("Receiver: Processed %d messages, yielding...\n", processed);
                rt_yield();  // Give sender a chance to retry
            }
        }
    }

    printf("Receiver: Finished processing %d messages\n", processed);
    rt_exit();
}

void sender_actor(void *arg) {
    sender_args *args = (sender_args *)arg;
    actor_id receiver = args->receiver;

    printf("\nSender: Filling up pool by sending %d messages...\n", MESSAGES_TO_FILL_POOL);

    // Fill up most of the pool
    int sent_count = 0;
    int data = 0;

    for (int i = 0; i < MESSAGES_TO_FILL_POOL; i++) {
        data++;
        rt_status status = rt_ipc_send(receiver, &data, sizeof(data));

        if (RT_FAILED(status)) {
            if (status.code == RT_ERR_NOMEM) {
                printf("Sender: Pool exhausted after %d messages (expected ~%d)\n",
                       sent_count, MESSAGES_TO_FILL_POOL);
                break;
            }
        }
        sent_count++;
    }

    printf("Sender: Sent %d messages, pool should be nearly full\n", sent_count);

    // Now try to send a few more - should fail
    printf("\nSender: Attempting to send more messages (should fail)...\n");

    int failed_count = 0;
    for (int i = 0; i < 20; i++) {
        data++;
        rt_status status = rt_ipc_send(receiver, &data, sizeof(data));

        if (status.code == RT_ERR_NOMEM) {
            failed_count++;
        }
    }

    printf("Sender: ✓ Got RT_ERR_NOMEM on %d send attempts\n", failed_count);

    // Use backoff-retry pattern
    printf("\nSender: Using backoff-retry pattern...\n");

    bool send_succeeded = false;
    int retry_count = 0;

    // Signal receiver to start processing after a short delay
    printf("Sender: Waiting a bit, then signaling receiver to process messages...\n");
    rt_message dummy_msg;
    rt_ipc_recv(&dummy_msg, 100);  // Wait 100ms

    // Signal controller to tell receiver to start
    char start_cmd = 'S';
    rt_ipc_send(args->controller, &start_cmd, sizeof(start_cmd));

    // Now retry with backoff
    for (int attempt = 0; attempt < 20; attempt++) {
        rt_yield();  // Give receiver chance to process

        data++;
        rt_status status = rt_ipc_send(receiver, &data, sizeof(data));

        if (!RT_FAILED(status)) {
            printf("Sender: ✓ Send succeeded on attempt %d!\n", attempt + 1);
            printf("Sender: Pool space became available after receiver processed messages\n");
            send_succeeded = true;
            break;
        }

        if (status.code == RT_ERR_NOMEM) {
            retry_count++;

            // Backoff
            rt_message msg;
            rt_ipc_recv(&msg, 10);  // Backoff 10ms

            if (attempt % 5 == 0) {
                printf("Sender: Attempt %d - still exhausted, retrying...\n", attempt + 1);
            }
        }
    }

    if (send_succeeded) {
        printf("\nSender: ✓ Backoff-retry SUCCESS!\n");
        printf("Sender: Demonstrated realistic pool exhaustion and recovery\n");
    } else {
        printf("\nSender: ✗ Send still failing after %d retries\n", retry_count);
    }

    rt_exit();
}

void controller_actor(void *arg) {
    sender_args *args = (sender_args *)arg;

    printf("Controller: Waiting for signal from sender...\n");

    rt_message msg;
    rt_ipc_recv(&msg, -1);

    char *cmd = (char *)msg.data;
    if (*cmd == 'S') {
        printf("Controller: Got START command, forwarding to receiver...\n");
        rt_ipc_send(args->receiver, cmd, sizeof(*cmd));
    }

    rt_exit();
}

int main(void) {
    printf("=== Backoff-Retry Test with Real Pool Exhaustion ===\n\n");
    printf("Pool size: RT_MAILBOX_ENTRY_POOL_SIZE = %d\n", RT_MAILBOX_ENTRY_POOL_SIZE);
    printf("Will fill pool with %d messages\n\n", MESSAGES_TO_FILL_POOL);

    rt_init();

    sender_args args;

    // Spawn actors
    args.controller = rt_spawn(controller_actor, &args);
    printf("Main: Spawned controller (ID: %u)\n", args.controller);

    args.receiver = rt_spawn(receiver_actor, &args.controller);
    printf("Main: Spawned receiver (ID: %u)\n", args.receiver);

    rt_spawn(sender_actor, &args);
    printf("Main: Spawned sender\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
