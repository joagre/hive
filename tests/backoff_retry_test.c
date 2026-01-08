#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Leave room for control messages and timers
#define MESSAGES_TO_FILL_POOL (RT_MAILBOX_ENTRY_POOL_SIZE / 2)

typedef struct {
    actor_id receiver;
    actor_id sender;
} test_args;

// Message tags for selective receive
#define TAG_DATA    0
#define TAG_START   1
#define TAG_DONE    2

// Receiver that processes messages
void receiver_actor(void *arg) {
    (void)arg;

    printf("Receiver: Started (ID: %u), mailbox count: %zu\n", rt_self(), rt_ipc_count());
    fflush(stdout);

    // Debug: Scan messages and show their tags
    printf("Receiver: Scanning mailbox for tags...\n");
    fflush(stdout);

    int scanned = 0;
    int found_start = 0;
    int found_done = 0;
    rt_message msg;

    while (scanned < 300) {
        rt_status status = rt_ipc_recv(&msg, 0);  // Non-blocking
        if (status.code == RT_ERR_WOULDBLOCK) {
            break;
        }
        if (!RT_FAILED(status)) {
            uint32_t tag;
            rt_msg_decode(&msg, NULL, &tag, NULL, NULL);
            if (scanned < 5 || tag == TAG_START || tag == TAG_DONE) {
                printf("  Message %d: tag=%u\n", scanned, tag);
            }
            if (tag == TAG_START) {
                found_start = 1;
            }
            if (tag == TAG_DONE) {
                found_done = 1;
            }
            scanned++;

            // Yield periodically
            if (scanned % 50 == 0) {
                printf("Receiver: Processed %d messages, yielding...\n", scanned);
                fflush(stdout);
                rt_yield();
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

        uint32_t done_tag = TAG_DONE;
        rt_status status = rt_ipc_recv_match(NULL, NULL, &done_tag, &msg, 5000);

        if (!RT_FAILED(status)) {
            printf("Receiver: Got DONE signal\n");
        } else {
            printf("Receiver: Timeout waiting for DONE (%s)\n",
                   status.msg ? status.msg : "unknown");
        }
    }

    printf("Receiver: Exiting\n");

    rt_exit();
}

// Internal send with custom tag
extern rt_status rt_ipc_cast_ex(actor_id to, actor_id sender, rt_msg_class class,
                                 uint32_t tag, const void *data, size_t len);

void sender_actor(void *arg) {
    test_args *args = (test_args *)arg;
    actor_id receiver = args->receiver;
    actor_id self = rt_self();

    printf("Sender: Started (ID: %u), receiver ID: %u\n", self, receiver);
    fflush(stdout);

    printf("\nSender: Filling pool with %d data messages (tag=%d)...\n", MESSAGES_TO_FILL_POOL, TAG_DATA);
    fflush(stdout);

    // Fill pool with data messages
    int sent_count = 0;
    int data = 0;

    for (int i = 0; i < MESSAGES_TO_FILL_POOL; i++) {
        data++;
        rt_status status = rt_ipc_cast_ex(receiver, self, RT_MSG_CAST, TAG_DATA, &data, sizeof(data));
        if (RT_FAILED(status)) {
            if (status.code == RT_ERR_NOMEM) {
                printf("Sender: Pool exhausted after %d messages\n", sent_count);
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
        rt_status status = rt_ipc_cast_ex(receiver, self, RT_MSG_CAST, TAG_DATA, &data, sizeof(data));
        if (status.code == RT_ERR_NOMEM) {
            failed_count++;
        } else if (!RT_FAILED(status)) {
            extra_sent++;
        }
    }

    if (failed_count > 0) {
        printf("Sender: ✓ RT_ERR_NOMEM on %d attempts (sent %d more)\n", failed_count, extra_sent);
    } else {
        printf("Sender: All 50 extra sends succeeded\n");
    }

    // Send START signal
    printf("\nSender: Sending START signal (tag=%d)...\n", TAG_START);
    fflush(stdout);
    rt_status status = rt_ipc_cast_ex(receiver, self, RT_MSG_CAST, TAG_START, NULL, 0);
    if (RT_FAILED(status)) {
        printf("Sender: Failed to send START: %s\n", status.msg ? status.msg : "unknown");
    } else {
        printf("Sender: START signal sent successfully\n");
    }

    // Yield to let receiver process
    printf("\nSender: Yielding to receiver...\n");
    fflush(stdout);
    rt_yield();

    // Retry loop
    printf("Sender: Starting retry loop...\n");
    fflush(stdout);

    bool send_succeeded = false;
    int retry_count = 0;

    for (int attempt = 0; attempt < 30; attempt++) {
        rt_yield();

        data++;
        status = rt_ipc_cast_ex(receiver, self, RT_MSG_CAST, TAG_DATA, &data, sizeof(data));

        if (!RT_FAILED(status)) {
            printf("Sender: ✓ Send succeeded on attempt %d!\n", attempt + 1);
            send_succeeded = true;
            break;
        }

        if (status.code == RT_ERR_NOMEM) {
            retry_count++;
            rt_message msg;
            rt_ipc_recv(&msg, 5);  // Backoff 5ms
            if (attempt % 10 == 0) {
                printf("Sender: Attempt %d - pool exhausted\n", attempt + 1);
            }
        } else {
            printf("Sender: Send failed: %s\n", status.msg ? status.msg : "unknown");
            break;
        }
    }

    // Send DONE signal
    printf("\nSender: Sending DONE signal...\n");
    rt_ipc_cast_ex(receiver, self, RT_MSG_CAST, TAG_DONE, NULL, 0);

    if (send_succeeded) {
        printf("\nSender: ✓ Backoff-retry SUCCESS!\n");
    } else if (retry_count == 0) {
        printf("\nSender: Pool never exhausted during retry\n");
    } else {
        printf("\nSender: ✗ Still failing after %d retries\n", retry_count);
    }

    rt_exit();
}

int main(void) {
    printf("=== Backoff-Retry Test ===\n\n");
    printf("Pool size: RT_MAILBOX_ENTRY_POOL_SIZE = %d\n", RT_MAILBOX_ENTRY_POOL_SIZE);
    printf("Messages to send: %d\n\n", MESSAGES_TO_FILL_POOL);
    fflush(stdout);

    rt_init();

    test_args args = {0};

    args.receiver = rt_spawn(receiver_actor, &args);
    printf("Main: Spawned receiver (ID: %u)\n", args.receiver);

    args.sender = rt_spawn(sender_actor, &args);
    printf("Main: Spawned sender (ID: %u)\n", args.sender);
    fflush(stdout);

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    return 0;
}
