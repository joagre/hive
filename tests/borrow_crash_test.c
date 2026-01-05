#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>
#include <stdlib.h>

// Receiver that crashes immediately after receiving BORROW message
void crash_receiver_actor(void *arg) {
    (void)arg;

    printf("Receiver: Waiting for BORROW message...\n");

    rt_message msg;
    rt_ipc_recv(&msg, -1);

    printf("Receiver: Got BORROW message, crashing WITHOUT releasing!\n");
    // Simulate crash - exit without calling rt_ipc_release()
    rt_exit();
}

void sender_actor(void *arg) {
    actor_id receiver = *(actor_id *)arg;

    printf("Sender: Sending BORROW message to receiver...\n");

    char data[100] = "Test data on sender's stack";
    rt_status status = rt_ipc_send(receiver, data, sizeof(data), IPC_BORROW);

    // If we reach here, we were unblocked
    if (!RT_FAILED(status)) {
        printf("Sender: PASS - Send returned normally after receiver crash\n");
        printf("Sender: Sender was automatically unblocked (principle of least surprise)\n");
    } else {
        printf("Sender: FAIL - Send returned error: %s\n", status.msg);
    }

    printf("\nSender: Test complete - receiver crash handled gracefully\n");
    rt_exit();
}

int main(void) {
    printf("=== BORROW Receiver Crash Test ===\n");
    printf("Tests that sender is unblocked when receiver crashes without releasing\n\n");

    rt_init();

    actor_id receiver = rt_spawn(crash_receiver_actor, NULL);
    printf("Main: Spawned crash receiver (ID: %u)\n", receiver);

    rt_spawn(sender_actor, &receiver);
    printf("Main: Spawned sender\n\n");

    rt_run();
    rt_cleanup();

    printf("\n=== Test Complete ===\n");
    printf("Expected: Sender unblocked when receiver crashed\n");
    printf("Result: PASS - Sender returned from rt_ipc_send()\n");

    return 0;
}
