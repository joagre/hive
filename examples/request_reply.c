/*
 * Request/Reply Example - Request/Response Pattern with Blocking Calls
 *
 * This example demonstrates the request/reply pattern using hive_ipc_request/hive_ipc_reply,
 * which provides natural backpressure by blocking the caller until a reply.
 *
 * KEY CONCEPTS:
 * - Caller blocks until callee replies (natural backpressure)
 * - Tag-based correlation ensures replies match requests
 * - No risk of deadlock from circular calls (each direction is independent)
 *
 * USE CASES:
 * - Request-response patterns (database queries, API calls)
 * - Flow control between fast producer and slow consumer
 * - When requester needs confirmation before proceeding
 */

#include "hive_runtime.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <string.h>

// Work request sent from producer to consumer
typedef struct {
    int job_id;
    int data;
} work_request_t;

// Work result sent back from consumer to producer
typedef struct {
    int job_id;
    int result;
} work_result_t;

// Slow consumer that processes work requests
static void consumer_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Consumer: Started (ID: %u)\n", hive_self());
    printf("Consumer: I process slowly to demonstrate backpressure\n\n");

    for (int jobs_processed = 0; jobs_processed < 5; jobs_processed++) {
        // Wait for work request (HIVE_MSG_REQUEST)
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, 5000); // 5 second timeout

        if (status.code == HIVE_ERR_TIMEOUT) {
            printf("Consumer: Timeout waiting for work, exiting\n");
            break;
        }

        if (HIVE_FAILED(status)) {
            printf("Consumer: Receive failed: %s\n", HIVE_ERR_STR(status));
            break;
        }

        if (msg.class != HIVE_MSG_REQUEST) {
            printf("Consumer: Unexpected message class %d, skipping\n",
                   msg.class);
            continue;
        }

        work_request_t *req = (work_request_t *)msg.data;
        printf("Consumer: Received job #%d (data=%d) from producer %u\n",
               req->job_id, req->data, msg.sender);

        // Simulate processing (producer is BLOCKED during this time)
        printf("Consumer: Processing job #%d...\n", req->job_id);

        // Do some "work" - in real code this would be actual computation
        volatile int sum = 0;
        for (int i = 0; i < 1000000; i++) {
            sum += i;
        }

        // Prepare result
        work_result_t result = {
            .job_id = req->job_id,
            .result = req->data * 2 // Simple processing: double the input
        };

        printf("Consumer: Finished job #%d, replying (result=%d)\n",
               req->job_id, result.result);

        // Reply to unblock the caller
        status = hive_ipc_reply(&msg, &result, sizeof(result));
        if (HIVE_FAILED(status)) {
            printf("Consumer: Failed to reply: %s\n", HIVE_ERR_STR(status));
        }

        printf("Consumer: Producer is now unblocked\n\n");
    }

    printf("Consumer: Done processing, exiting\n");
    return;
}

// Fast producer that requests work processing
static void producer_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t consumer_id = (hive_actor_id_t)(uintptr_t)args;

    printf("Producer: Started (ID: %u)\n", hive_self());
    printf("Producer: Requesting 5 jobs with hive_ipc_request (blocks until "
           "reply)\n\n");

    for (int i = 1; i <= 5; i++) {
        work_request_t req = {.job_id = i, .data = i * 100};

        printf("Producer: Calling consumer with job #%d (will block until "
               "reply)...\n",
               i);

        // Call consumer - this BLOCKS until consumer calls hive_ipc_reply()
        hive_message_t reply;
        hive_status_t status = hive_ipc_request(consumer_id, HIVE_ID_NONE, &req,
                                                sizeof(req), &reply, 10000);

        if (HIVE_FAILED(status)) {
            if (status.code == HIVE_ERR_TIMEOUT) {
                printf("Producer: Timeout waiting for reply on job #%d\n", i);
            } else {
                printf("Producer: Call failed: %s\n", HIVE_ERR_STR(status));
            }
            break;
        }

        work_result_t *result = (work_result_t *)reply.data;

        printf("Producer: Job #%d completed! Result=%d\n\n", result->job_id,
               result->result);
    }

    printf("Producer: All jobs sent and completed, exiting\n");
    return;
}

// Demo simple message passing (async notify vs request/reply)
static void demo_actor(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t peer_id = (hive_actor_id_t)(uintptr_t)args;
    (void)peer_id;

    printf("\n--- Message Passing Patterns Demo ---\n");

    // Pattern 1: Fire-and-forget with hive_ipc_notify()
    printf("Demo: Fire-and-forget (hive_ipc_notify) - caller continues "
           "immediately\n");
    int data = 42;
    hive_status_t status = hive_ipc_notify(hive_self(), 0, &data, sizeof(data));
    if (HIVE_SUCCEEDED(status)) {
        hive_message_t msg;
        hive_ipc_recv(&msg, 0);
        printf("Demo: Received self-sent message: %d\n", *(int *)msg.data);
    }

    printf("--- End Demo ---\n\n");
    return;
}

int main(void) {
    printf("=== Request/Reply Example - Request/Response Pattern ===\n\n");

    printf("This example shows:\n");
    printf("1. Producer requests jobs with hive_ipc_request() (blocks until "
           "reply)\n");
    printf("2. Consumer processes and replies with hive_ipc_reply()\n");
    printf("3. Producer only proceeds after receiving reply\n\n");

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // First, run the demo actor
    hive_actor_id_t demo;
    if (HIVE_FAILED(hive_spawn(demo_actor, NULL, NULL, NULL, &demo))) {
        fprintf(stderr, "Failed to spawn demo actor\n");
        hive_cleanup();
        return 1;
    }

    // Spawn consumer first (it will wait for messages)
    hive_actor_id_t consumer;
    if (HIVE_FAILED(hive_spawn(consumer_actor, NULL, NULL, NULL, &consumer))) {
        fprintf(stderr, "Failed to spawn consumer\n");
        hive_cleanup();
        return 1;
    }

    // Spawn producer with consumer's ID
    hive_actor_id_t producer;
    if (HIVE_FAILED(hive_spawn(producer_actor, NULL,
                               (void *)(uintptr_t)consumer, NULL, &producer))) {
        fprintf(stderr, "Failed to spawn producer\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned actors: demo=%u, consumer=%u, producer=%u\n\n", demo,
           consumer, producer);

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");
    hive_cleanup();

    printf("\n=== Example completed ===\n");
    printf("\nKey takeaways:\n");
    printf("- hive_ipc_request() blocks until hive_ipc_reply() is received\n");
    printf("- Tag-based correlation matches replies to requests\n");
    printf("- Natural backpressure without explicit release calls\n");
    printf("- Simpler than old IPC_SYNC mode\n");

    return 0;
}
