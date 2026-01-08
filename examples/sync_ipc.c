/*
 * RPC Example - Request/Response Pattern with Blocking Calls
 *
 * This example demonstrates the RPC pattern using rt_ipc_call/rt_ipc_reply,
 * which provides natural backpressure by blocking the caller until a reply.
 *
 * KEY CONCEPTS:
 * - Caller blocks until callee sends a reply (natural backpressure)
 * - Tag-based correlation ensures replies match requests
 * - No risk of deadlock from circular calls (each direction is independent)
 *
 * USE CASES:
 * - Request-response patterns (database queries, API calls)
 * - Flow control between fast producer and slow consumer
 * - When sender needs confirmation before proceeding
 */

#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>
#include <string.h>

// Work request sent from producer to consumer
typedef struct {
    int job_id;
    int data;
} work_request;

// Work result sent back from consumer to producer
typedef struct {
    int job_id;
    int result;
} work_result;

// Slow consumer that processes work requests
static void consumer_actor(void *arg) {
    (void)arg;

    printf("Consumer: Started (ID: %u)\n", rt_self());
    printf("Consumer: I process slowly to demonstrate backpressure\n\n");

    for (int jobs_processed = 0; jobs_processed < 5; jobs_processed++) {
        // Wait for work request (RT_MSG_CALL)
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, 5000);  // 5 second timeout

        if (status.code == RT_ERR_TIMEOUT) {
            printf("Consumer: Timeout waiting for work, exiting\n");
            break;
        }

        if (RT_FAILED(status)) {
            printf("Consumer: Receive failed: %s\n", status.msg);
            break;
        }

        // Decode the message
        rt_msg_class class;
        const void *payload;
        size_t payload_len;
        rt_msg_decode(&msg, &class, NULL, &payload, &payload_len);

        if (class != RT_MSG_CALL) {
            printf("Consumer: Unexpected message class %d, skipping\n", class);
            continue;
        }

        work_request *req = (work_request *)payload;
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
        work_result result = {
            .job_id = req->job_id,
            .result = req->data * 2  // Simple processing: double the input
        };

        printf("Consumer: Finished job #%d, sending reply (result=%d)\n",
               req->job_id, result.result);

        // Send reply to unblock the caller
        status = rt_ipc_reply(&msg, &result, sizeof(result));
        if (RT_FAILED(status)) {
            printf("Consumer: Failed to send reply: %s\n", status.msg);
        }

        printf("Consumer: Producer is now unblocked\n\n");
    }

    printf("Consumer: Done processing, exiting\n");
    rt_exit();
}

// Fast producer that sends work requests
static void producer_actor(void *arg) {
    actor_id consumer_id = (actor_id)(uintptr_t)arg;

    printf("Producer: Started (ID: %u)\n", rt_self());
    printf("Producer: Sending 5 jobs with rt_ipc_call (blocks until reply)\n\n");

    for (int i = 1; i <= 5; i++) {
        work_request req = {
            .job_id = i,
            .data = i * 100
        };

        printf("Producer: Calling consumer with job #%d (will block until reply)...\n", i);

        // Call consumer - this BLOCKS until consumer sends rt_ipc_reply()
        rt_message reply;
        rt_status status = rt_ipc_call(consumer_id, &req, sizeof(req), &reply, 10000);

        if (RT_FAILED(status)) {
            if (status.code == RT_ERR_TIMEOUT) {
                printf("Producer: Timeout waiting for reply on job #%d\n", i);
            } else {
                printf("Producer: Call failed: %s\n", status.msg);
            }
            break;
        }

        // Decode and display result
        const void *payload;
        rt_msg_decode(&reply, NULL, NULL, &payload, NULL);
        work_result *result = (work_result *)payload;

        printf("Producer: Job #%d completed! Result=%d\n\n", result->job_id, result->result);
    }

    printf("Producer: All jobs sent and completed, exiting\n");
    rt_exit();
}

// Demo simple message passing (fire-and-forget vs RPC)
static void demo_actor(void *arg) {
    actor_id peer_id = (actor_id)(uintptr_t)arg;
    (void)peer_id;

    printf("\n--- Message Passing Patterns Demo ---\n");

    // Pattern 1: Fire-and-forget with rt_ipc_cast()
    printf("Demo: Fire-and-forget (rt_ipc_cast) - sender continues immediately\n");
    int data = 42;
    rt_status status = rt_ipc_cast(rt_self(), &data, sizeof(data));
    if (!RT_FAILED(status)) {
        rt_message msg;
        rt_ipc_recv(&msg, 0);
        const void *payload;
        rt_msg_decode(&msg, NULL, NULL, &payload, NULL);
        printf("Demo: Received self-sent message: %d\n", *(int *)payload);
    }

    printf("--- End Demo ---\n\n");
    rt_exit();
}

int main(void) {
    printf("=== RPC Example - Request/Response Pattern ===\n\n");

    printf("This example shows:\n");
    printf("1. Producer sends jobs with rt_ipc_call() (blocks until reply)\n");
    printf("2. Consumer processes and replies with rt_ipc_reply()\n");
    printf("3. Producer only proceeds after receiving reply\n\n");

    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n", status.msg);
        return 1;
    }

    // First, run the demo actor
    actor_id demo = rt_spawn(demo_actor, NULL);
    if (demo == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn demo actor\n");
        rt_cleanup();
        return 1;
    }

    // Spawn consumer first (it will wait for messages)
    actor_id consumer = rt_spawn(consumer_actor, NULL);
    if (consumer == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn consumer\n");
        rt_cleanup();
        return 1;
    }

    // Spawn producer with consumer's ID
    actor_id producer = rt_spawn(producer_actor, (void *)(uintptr_t)consumer);
    if (producer == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn producer\n");
        rt_cleanup();
        return 1;
    }

    printf("Spawned actors: demo=%u, consumer=%u, producer=%u\n\n", demo, consumer, producer);

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");
    rt_cleanup();

    printf("\n=== Example completed ===\n");
    printf("\nKey takeaways:\n");
    printf("- rt_ipc_call() blocks until rt_ipc_reply() is received\n");
    printf("- Tag-based correlation matches replies to requests\n");
    printf("- Natural backpressure without explicit release calls\n");
    printf("- Simpler than old IPC_SYNC mode\n");

    return 0;
}
