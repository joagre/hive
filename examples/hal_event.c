// HAL Event Example - Interrupt-driven actor wakeup
//
// This example demonstrates HAL events for ISR-to-actor communication:
// - Creating and destroying HAL events
// - Signaling events (simulating ISR behavior)
// - Waiting on events with hive_event_wait()
// - Combining HAL events with timers in hive_select()
//
// On real hardware (STM32), ISRs call hive_hal_event_signal() to wake
// actors waiting in hive_select(). This example simulates that pattern
// using a separate actor as the "signal source".

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_select.h"
#include "hal/hive_hal_event.h"
#include <stdio.h>

// Shared event ID (in real code, this would be created by hardware init)
static hive_hal_event_id_t s_rx_event = HIVE_HAL_EVENT_INVALID;

// Tags for IPC
#define TAG_START 1
#define TAG_DONE 2

// Signal source actor - simulates hardware interrupt signaling
// In real STM32 code, this would be an ISR calling hive_hal_event_signal()
static void signal_source(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t consumer = *(hive_actor_id_t *)args;

    printf("[Signal] Source actor started\n");

    // Wait for consumer to be ready
    hive_message_t msg;
    hive_ipc_recv_match(consumer, HIVE_MSG_NOTIFY, TAG_START, &msg, -1);
    printf("[Signal] Consumer ready, starting signal sequence\n");

    // Simulate hardware interrupts at irregular intervals
    for (int i = 1; i <= 5; i++) {
        // Variable delay to simulate real hardware behavior
        int delay_ms = 100 + (i * 50);
        hive_sleep(delay_ms * 1000);

        printf("[Signal] Signaling event (interrupt %d)\n", i);
        hive_hal_event_signal(s_rx_event);
    }

    printf("[Signal] Done signaling, notifying consumer\n");
    hive_ipc_notify(consumer, TAG_DONE, NULL, 0);

    return;
}

// Consumer actor - waits for HAL events using hive_event_wait()
static void consumer_simple(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t signal_actor = *(hive_actor_id_t *)args;

    printf("[Consumer] Simple consumer started\n");
    printf("[Consumer] Using hive_event_wait() for single-source waiting\n");

    // Tell signal source we're ready
    hive_ipc_notify(signal_actor, TAG_START, NULL, 0);

    int count = 0;
    while (count < 5) {
        printf("[Consumer] Waiting for event...\n");

        hive_status_t status = hive_event_wait(s_rx_event, 2000); // 2s timeout

        if (HIVE_SUCCEEDED(status)) {
            count++;
            printf("[Consumer] Event received! (count=%d)\n", count);
        } else if (status.code == HIVE_ERR_TIMEOUT) {
            printf("[Consumer] Timeout - no event in 2 seconds\n");
            break;
        } else {
            printf("[Consumer] Wait failed: %s\n", HIVE_ERR_STR(status));
            break;
        }
    }

    printf("[Consumer] Received %d events\n", count);
    return;
}

// Controller actor - demonstrates hive_select() with mixed sources
static void controller(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("\n=== Part 2: hive_select() with HAL event + timer ===\n\n");
    printf("[Controller] Started\n");

    // Create a periodic heartbeat timer
    hive_timer_id_t heartbeat;
    hive_timer_every(300000, &heartbeat); // 300ms heartbeat

    // Set up select sources - HAL event has priority (first in array)
    enum { SEL_EVENT, SEL_HEARTBEAT };
    hive_select_source_t sources[] = {
        [SEL_EVENT] = {.type = HIVE_SEL_HAL_EVENT, .event = s_rx_event},
        [SEL_HEARTBEAT] = {.type = HIVE_SEL_IPC,
                           .ipc = {.class = HIVE_MSG_TIMER, .tag = heartbeat}},
    };

    int event_count = 0;
    int heartbeat_count = 0;
    int iterations = 0;

    printf("[Controller] Waiting on HAL event (priority) + timer...\n");

    // Run for a fixed number of iterations
    while (iterations < 10) {
        hive_select_result_t result;
        hive_status_t status = hive_select(sources, 2, &result, 1000);

        if (HIVE_FAILED(status)) {
            if (status.code == HIVE_ERR_TIMEOUT) {
                printf("[Controller] Timeout - no events for 1 second\n");
            } else {
                printf("[Controller] Select error: %s\n", HIVE_ERR_STR(status));
            }
            break;
        }

        iterations++;

        switch (result.index) {
        case SEL_EVENT:
            event_count++;
            printf("[Controller] HAL event received! (events=%d, hb=%d)\n",
                   event_count, heartbeat_count);
            break;

        case SEL_HEARTBEAT:
            heartbeat_count++;
            printf("[Controller] Heartbeat #%d\n", heartbeat_count);
            // Signal the event every 3rd heartbeat (simulates periodic HW event)
            if (heartbeat_count % 3 == 0) {
                printf("[Controller] Simulating hardware interrupt...\n");
                hive_hal_event_signal(s_rx_event);
            }
            break;
        }
    }

    hive_timer_cancel(heartbeat);
    printf(
        "[Controller] Final: %d HAL events, %d heartbeats in %d iterations\n",
        event_count, heartbeat_count, iterations);

    return;
}

int main(void) {
    printf("=== HAL Event Example ===\n\n");
    printf("This example demonstrates HAL events for interrupt-driven I/O:\n");
    printf("- hive_event_wait() for single-source waiting\n");
    printf("- hive_select() for multi-source waiting (HAL event + timer)\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Create HAL event (simulates hardware RX interrupt event)
    s_rx_event = hive_hal_event_create();
    if (s_rx_event == HIVE_HAL_EVENT_INVALID) {
        fprintf(stderr, "Failed to create HAL event\n");
        hive_cleanup();
        return 1;
    }
    printf("Created HAL event ID: %u\n\n", (unsigned)s_rx_event);

    // ========================================================================
    // Part 1: Simple hive_event_wait() demo
    // ========================================================================
    printf("=== Part 1: Simple hive_event_wait() ===\n\n");

    // Spawn consumer first (will wait for signal source)
    hive_actor_id_t consumer, signal;
    hive_spawn(signal_source, NULL, &consumer, NULL, &signal);
    hive_spawn(consumer_simple, NULL, &signal, NULL, &consumer);

    // Run until both actors exit
    hive_run();

    // ========================================================================
    // Part 2: hive_select() with HAL event + timer
    // ========================================================================
    // Reinitialize for part 2
    status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to reinitialize runtime\n");
        hive_hal_event_destroy(s_rx_event);
        return 1;
    }

    // Clear event state from part 1
    hive_hal_event_clear(s_rx_event);

    hive_actor_id_t ctrl;
    hive_spawn(controller, NULL, NULL, NULL, &ctrl);

    hive_run();

    // Cleanup
    hive_hal_event_destroy(s_rx_event);
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
