#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/* TEST_STACK_SIZE caps stack for QEMU builds; passes through on native */
#ifndef TEST_STACK_SIZE
#define TEST_STACK_SIZE(x) (x)
#endif

// Leave room for control messages and timers
#define MESSAGES_TO_FILL_POOL (HIVE_MAILBOX_ENTRY_POOL_SIZE / 2)

typedef struct {
    actor_id receiver;
    actor_id sender;
} test_args;

// Message tags for selective receive
#define TAG_DATA 0
#define TAG_START 1
#define TAG_DONE 2

// Receiver that processes messages
void receiver_actor(void *arg) {
    (void)arg;

    printf("Receiver: Started (ID: %u), mailbox count: %zu\n", hive_self(),
           hive_ipc_count());
    fflush(stdout);

    // Debug: Scan messages and show their tags
    printf("Receiver: Scanning mailbox for tags...\n");
    fflush(stdout);

    int scanned = 0;
    int found_start = 0;
    int found_done = 0;
    hive_message msg;

    while (scanned < 300) {
        hive_status status = hive_ipc_recv(&msg, 0); // Non-blocking
        if (status.code == HIVE_ERR_WOULDBLOCK) {
            break;
        }
        if (HIVE_SUCCEEDED(status)) {
            if (scanned < 5 || msg.tag == TAG_START || msg.tag == TAG_DONE) {
                printf("  Message %d: tag=%u\n", scanned, msg.tag);
            }
            if (msg.tag == TAG_START) {
                found_start = 1;
            }
            if (msg.tag == TAG_DONE) {
                found_done = 1;
            }
            scanned++;

            // Yield periodically
            if (scanned % 50 == 0) {
                printf("Receiver: Processed %d messages, yielding...\n",
                       scanned);
                fflush(stdout);
                hive_yield();
            }
        }
    }

    printf("Receiver: Scanned %d messages, found START: %s, found DONE: %s\n",
           scanned, found_start ? "YES" : "NO", found_done ? "YES" : "NO");
    printf("Receiver: Finished processing\n");
    fflush(stdout);

    // Wait for DONE signal if not already received
    if (!found_done) {
        printf("Receiver: Waiting for sender DONE signal...\n");
        fflush(stdout);

        hive_status status = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_ANY,
                                                 TAG_DONE, &msg, 5000);

        if (HIVE_SUCCEEDED(status)) {
            printf("Receiver: Got DONE signal\n");
        } else {
            printf("Receiver: Timeout waiting for DONE (%s)\n",
                   status.msg ? status.msg : "unknown");
        }
    }

    printf("Receiver: Exiting\n");

    hive_exit();
}

void sender_actor(void *arg) {
    test_args *args = (test_args *)arg;
    actor_id receiver = args->receiver;

    printf("Sender: Started (ID: %u), receiver ID: %u\n", hive_self(),
           receiver);
    fflush(stdout);

    printf("\nSender: Filling pool with %d data messages (tag=%d)...\n",
           MESSAGES_TO_FILL_POOL, TAG_DATA);
    fflush(stdout);

    // Fill pool with data messages
    int sent_count = 0;
    int data = 0;

    for (int i = 0; i < MESSAGES_TO_FILL_POOL; i++) {
        data++;
        hive_status status = hive_ipc_notify_ex(receiver, HIVE_MSG_NOTIFY,
                                                TAG_DATA, &data, sizeof(data));
        if (HIVE_FAILED(status)) {
            if (status.code == HIVE_ERR_NOMEM) {
                printf("Sender: Pool exhausted after %d messages\n",
                       sent_count);
                break;
            }
        }
        sent_count++;
    }

    printf("Sender: Sent %d messages\n", sent_count);

    // Try sending more
    printf("\nSender: Attempting 50 more sends...\n");

    int extra_sent = 0;
    int failed_count = 0;
    for (int i = 0; i < 50; i++) {
        data++;
        hive_status status = hive_ipc_notify_ex(receiver, HIVE_MSG_NOTIFY,
                                                TAG_DATA, &data, sizeof(data));
        if (status.code == HIVE_ERR_NOMEM) {
            failed_count++;
        } else if (HIVE_SUCCEEDED(status)) {
            extra_sent++;
        }
    }

    if (failed_count > 0) {
        printf("Sender: [OK] HIVE_ERR_NOMEM on %d attempts (sent %d more)\n",
               failed_count, extra_sent);
    } else {
        printf("Sender: All 50 extra sends succeeded\n");
    }

    // Send START signal
    printf("\nSender: Sending START signal (tag=%d)...\n", TAG_START);
    fflush(stdout);
    hive_status status =
        hive_ipc_notify_ex(receiver, HIVE_MSG_NOTIFY, TAG_START, NULL, 0);
    if (HIVE_FAILED(status)) {
        printf("Sender: Failed to send START: %s\n",
               status.msg ? status.msg : "unknown");
    } else {
        printf("Sender: START signal sent successfully\n");
    }

    // Yield to let receiver process
    printf("\nSender: Yielding to receiver...\n");
    fflush(stdout);
    hive_yield();

    // Retry loop
    printf("Sender: Starting retry loop...\n");
    fflush(stdout);

    bool send_succeeded = false;
    int retry_count = 0;

    for (int attempt = 0; attempt < 30; attempt++) {
        hive_yield();

        data++;
        status = hive_ipc_notify_ex(receiver, HIVE_MSG_NOTIFY, TAG_DATA, &data,
                                    sizeof(data));

        if (HIVE_SUCCEEDED(status)) {
            printf("Sender: [OK] Send succeeded on attempt %d!\n", attempt + 1);
            send_succeeded = true;
            break;
        }

        if (status.code == HIVE_ERR_NOMEM) {
            retry_count++;
            hive_message msg;
            hive_ipc_recv(&msg, 5); // Backoff 5ms
            if (attempt % 10 == 0) {
                printf("Sender: Attempt %d - pool exhausted\n", attempt + 1);
            }
        } else {
            printf("Sender: Send failed: %s\n",
                   status.msg ? status.msg : "unknown");
            break;
        }
    }

    // Send DONE signal
    printf("\nSender: Sending DONE signal...\n");
    hive_ipc_notify_ex(receiver, HIVE_MSG_NOTIFY, TAG_DONE, NULL, 0);

    if (send_succeeded) {
        printf("\nSender: [OK] Backoff-retry SUCCESS!\n");
    } else if (retry_count == 0) {
        printf("\nSender: Pool never exhausted during retry\n");
    } else {
        printf("\nSender: [FAIL] Still failing after %d retries\n",
               retry_count);
    }

    hive_exit();
}

int main(void) {
    printf("=== Backoff-Retry Test ===\n\n");
    printf("Pool size: HIVE_MAILBOX_ENTRY_POOL_SIZE = %d\n",
           HIVE_MAILBOX_ENTRY_POOL_SIZE);
    printf("Messages to send: %d\n\n", MESSAGES_TO_FILL_POOL);
    fflush(stdout);

    hive_init();

    test_args args = {0};

    hive_spawn(receiver_actor, &args, &args.receiver);
    printf("Main: Spawned receiver (ID: %u)\n", args.receiver);

    hive_spawn(sender_actor, &args, &args.sender);
    printf("Main: Spawned sender (ID: %u)\n", args.sender);
    fflush(stdout);

    hive_run();
    hive_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
