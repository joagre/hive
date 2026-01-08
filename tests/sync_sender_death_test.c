#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>
#include <string.h>

// Test data pattern
#define MAGIC_VALUE 0xDEADBEEF
#define DATA_SIZE 64

typedef struct {
    uint32_t magic;
    uint32_t counter;
    char message[56];
} test_data;

void receiver_actor(void *arg) {
    (void)arg;
    printf("Receiver: Waiting for SYNC message...\n");

    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, 5000);
    if (RT_FAILED(status)) {
        printf("Receiver: ✗ FAIL - Failed to receive message: %s\n", status.msg);
        rt_exit();
    }

    printf("Receiver: Got SYNC message from sender %u\n", msg.sender);

    // Deliberately wait before accessing data
    // This gives sender time to die and stack to be freed (if it was on stack)
    printf("Receiver: Sleeping 100ms to ensure sender has died...\n");
    rt_message dummy;
    rt_ipc_recv(&dummy, 100);  // Use as sleep

    // Now access the data - this would be UAF if data was on sender's stack
    printf("Receiver: Accessing SYNC data...\n");
    test_data *data = (test_data *)msg.data;

    printf("Receiver: Validating data integrity...\n");
    if (data->magic != MAGIC_VALUE) {
        printf("Receiver: ✗ FAIL - Data corrupted! magic=0x%08x (expected 0x%08x)\n",
               data->magic, MAGIC_VALUE);
        printf("Receiver: This indicates use-after-free!\n");
    } else {
        printf("Receiver: ✓ PASS - Data still valid! magic=0x%08x\n", data->magic);
        printf("Receiver: ✓ PASS - Message: %s\n", data->message);
        printf("Receiver: ✓ PASS - Pinned buffer prevents UAF even though sender died\n");
    }

    // rt_ipc_release removed;
    rt_exit();
}

void sender_actor(void *arg) {
    actor_id receiver = *(actor_id *)arg;

    printf("Sender: Preparing SYNC message...\n");

    // Create test data on sender's stack
    test_data data;
    data.magic = MAGIC_VALUE;
    data.counter = 42;
    snprintf(data.message, sizeof(data.message),
             "This message should survive sender death!");

    printf("Sender: Sending SYNC message to receiver %u...\n", receiver);
    rt_status status = rt_ipc_send(receiver, &data, sizeof(data));

    if (RT_FAILED(status)) {
        printf("Sender: ✗ FAIL - Send failed: %s\n", status.msg);
        rt_exit();
    }

    printf("Sender: SYNC message released by receiver\n");
    printf("Sender: Now exiting immediately (stack will be freed)...\n");

    // Exit immediately - in old implementation, this would free the stack
    // and receiver would have UAF when accessing msg.data
    rt_exit();
}

int main(void) {
    printf("=== SYNC Sender Death Test (UAF Prevention) ===\n");
    printf("Tests that receiver can safely access SYNC data even after sender dies\n");
    printf("This verifies pinned runtime buffers prevent use-after-free\n\n");

    rt_init();

    actor_id receiver = rt_spawn(receiver_actor, NULL);
    printf("Main: Spawned receiver (ID: %u)\n", receiver);

    actor_id sender = rt_spawn(sender_actor, &receiver);
    printf("Main: Spawned sender (ID: %u)\n\n", sender);

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected behavior:\n");
    printf("  1. Sender sends SYNC message and dies\n");
    printf("  2. Receiver accesses data AFTER sender has died\n");
    printf("  3. Data is still valid (pinned buffer persists)\n");
    printf("  4. No use-after-free or data corruption\n");
    printf("Result: PASS if data validation succeeded\n");

    return 0;
}
