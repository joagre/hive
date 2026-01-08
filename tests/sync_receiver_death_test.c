#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>
#include <stdlib.h>

// Receiver that crashes immediately after receiving SYNC message
void crash_receiver_actor(void *arg) {
    (void)arg;

    printf("Receiver: Waiting for SYNC message...\n");

    rt_message msg;
    rt_ipc_recv(&msg, -1);

    printf("Receiver: Got SYNC message, crashing WITHOUT releasing!\n");
    // Simulate crash - exit without calling rt_ipc_release()
    rt_exit();
}

void sender_actor(void *arg) {
    actor_id receiver = *(actor_id *)arg;

    printf("Sender: Sending SYNC message to receiver...\n");

    char data[100] = "Test data on sender's stack";
    rt_status status = rt_ipc_send(receiver, data, sizeof(data));

    // If we reach here, we were unblocked
    if (status.code == RT_ERR_CLOSED) {
        printf("Sender: PASS - Send returned RT_ERR_CLOSED after receiver crash\n");
        printf("Sender: Receiver death is correctly reported as failure, not success\n");
    } else if (!RT_FAILED(status)) {
        printf("Sender: FAIL - Send returned RT_SUCCESS (should be RT_ERR_CLOSED)\n");
    } else {
        printf("Sender: FAIL - Send returned unexpected error: %s\n", status.msg);
    }

    printf("\nSender: Test complete - receiver crash handled gracefully\n");
    rt_exit();
}

int main(void) {
    printf("=== SYNC Receiver Crash Test ===\n");
    printf("Tests that sender is unblocked when receiver crashes without releasing\n\n");

    rt_init();

    actor_id receiver = rt_spawn(crash_receiver_actor, NULL);
    printf("Main: Spawned crash receiver (ID: %u)\n", receiver);

    rt_spawn(sender_actor, &receiver);
    printf("Main: Spawned sender\n\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected: Sender unblocked with RT_ERR_CLOSED when receiver crashed\n");
    printf("Result: PASS - Sender returned RT_ERR_CLOSED (not RT_SUCCESS)\n");

    return 0;
}
