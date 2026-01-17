// hive_select() example - Unified event waiting
//
// This example demonstrates waiting on multiple event sources:
// - Sensor bus (simulated sensor data)
// - Timer (periodic heartbeat)
// - Command IPC (control messages)
//
// hive_select() provides a clean event loop that can respond to
// any source immediately without busy-polling.

#include "hive_runtime.h"
#include "hive_select.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_link.h"
#include <stdio.h>
#include <string.h>

// Simulated sensor data
typedef struct {
    float temperature;
    float humidity;
    uint32_t sequence;
} sensor_data_t;

// Command tags
#define CMD_SHUTDOWN 100
#define CMD_STATUS 101

// Global bus for sensor data
static bus_id g_sensor_bus = BUS_ID_INVALID;

// Sensor publisher actor - simulates sensor readings
static void sensor_publisher(void *arg) {
    (void)arg;
    printf("[Sensor] Publisher started\n");

    // Create periodic timer for sensor updates (100ms)
    timer_id tick;
    hive_timer_every(100000, &tick);

    sensor_data_t data = {20.0f, 50.0f, 0};

    for (int i = 0; i < 10; i++) {
        // Wait for timer
        hive_message msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, tick, &msg, -1);

        // Simulate changing sensor readings
        data.temperature += 0.5f;
        data.humidity += 1.0f;
        data.sequence++;

        // Publish to bus
        hive_bus_publish(g_sensor_bus, &data, sizeof(data));
        printf("[Sensor] Published: temp=%.1f, humidity=%.1f, seq=%u\n",
               data.temperature, data.humidity, data.sequence);
    }

    printf("[Sensor] Publisher finished\n");
    hive_timer_cancel(tick);
    hive_exit();
}

// Command sender actor - sends shutdown command after delay
static void command_sender(void *arg) {
    actor_id controller = *(actor_id *)arg;
    printf("[Command] Sender started, will send shutdown after 500ms\n");

    // Wait before sending shutdown
    hive_sleep(500000);

    // Send status request
    printf("[Command] Sending STATUS command\n");
    int status_data = 0;
    hive_ipc_notify(controller, CMD_STATUS, &status_data, sizeof(status_data));

    // Wait a bit more
    hive_sleep(200000);

    // Send shutdown command
    printf("[Command] Sending SHUTDOWN command\n");
    int shutdown_data = 1;
    hive_ipc_notify(controller, CMD_SHUTDOWN, &shutdown_data,
                    sizeof(shutdown_data));

    hive_exit();
}

// Controller actor - uses hive_select() to wait on multiple sources
static void controller(void *arg) {
    (void)arg;
    printf("[Controller] Started\n");

    // Subscribe to sensor bus
    hive_status status = hive_bus_subscribe(g_sensor_bus);
    if (HIVE_FAILED(status)) {
        printf("[Controller] Failed to subscribe: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    // Create heartbeat timer (250ms)
    timer_id heartbeat;
    hive_timer_every(250000, &heartbeat);

    // Spawn sensor publisher
    actor_id publisher;
    hive_spawn(sensor_publisher, NULL, &publisher);
    hive_link(publisher);

    // Spawn command sender
    actor_id self = hive_self();
    actor_id cmd_sender;
    hive_spawn(command_sender, &self, &cmd_sender);
    hive_link(cmd_sender);

    // Set up select sources
    enum { SEL_SENSOR, SEL_HEARTBEAT, SEL_STATUS, SEL_SHUTDOWN };
    hive_select_source sources[] = {
        [SEL_SENSOR] = {HIVE_SEL_BUS, .bus = g_sensor_bus},
        [SEL_HEARTBEAT] = {HIVE_SEL_IPC,
                           .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, heartbeat}},
        [SEL_STATUS] = {HIVE_SEL_IPC,
                        .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_STATUS}},
        [SEL_SHUTDOWN] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY,
                                                HIVE_MSG_NOTIFY, CMD_SHUTDOWN}},
    };

    int sensor_count = 0;
    int heartbeat_count = 0;
    bool running = true;

    printf("[Controller] Entering main loop\n");

    while (running) {
        hive_select_result result;
        status = hive_select(sources, 4, &result, 1000);

        if (HIVE_FAILED(status)) {
            if (status.code == HIVE_ERR_TIMEOUT) {
                printf("[Controller] Timeout - no events for 1 second\n");
                continue;
            }
            printf("[Controller] Select error: %s\n", HIVE_ERR_STR(status));
            break;
        }

        switch (result.index) {
        case SEL_SENSOR: {
            // Bus data has priority - processed first when both ready
            sensor_data_t *data = (sensor_data_t *)result.bus.data;
            sensor_count++;
            printf("[Controller] Sensor: temp=%.1f, seq=%u (count=%d)\n",
                   data->temperature, data->sequence, sensor_count);
            break;
        }
        case SEL_HEARTBEAT:
            heartbeat_count++;
            printf("[Controller] Heartbeat #%d\n", heartbeat_count);
            break;

        case SEL_STATUS:
            printf("[Controller] Status request received - sensors=%d, "
                   "heartbeats=%d\n",
                   sensor_count, heartbeat_count);
            break;

        case SEL_SHUTDOWN:
            printf("[Controller] Shutdown command received\n");
            running = false;
            break;
        }

        // Check for exit messages (publisher finished)
        hive_message msg;
        if (HIVE_SUCCEEDED(hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_EXIT,
                                               HIVE_TAG_ANY, &msg, 0))) {
            if (hive_is_exit_msg(&msg)) {
                hive_exit_msg exit_info;
                hive_decode_exit(&msg, &exit_info);
                printf("[Controller] Actor %u exited (%s)\n", exit_info.actor,
                       hive_exit_reason_str(exit_info.reason));
            }
        }
    }

    // Cleanup
    hive_timer_cancel(heartbeat);
    hive_bus_unsubscribe(g_sensor_bus);

    printf("[Controller] Final stats: %d sensor readings, %d heartbeats\n",
           sensor_count, heartbeat_count);
    printf("[Controller] Finished\n");

    hive_exit();
}

int main(void) {
    printf("=== hive_select() Example ===\n\n");
    printf("This example demonstrates unified event waiting:\n");
    printf("- Sensor bus data (highest priority)\n");
    printf("- Timer heartbeats\n");
    printf("- Command messages\n\n");

    // Initialize runtime
    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Create sensor bus (reduced limits for QEMU compatibility)
    hive_bus_config bus_cfg = HIVE_BUS_CONFIG_DEFAULT;
    bus_cfg.max_subscribers = 2;
    bus_cfg.max_entries = 4;
    bus_cfg.max_entry_size = 64;
    bus_cfg.max_age_ms = 500; // Expire old readings after 500ms
    status = hive_bus_create(&bus_cfg, &g_sensor_bus);
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to create bus: %s\n", HIVE_ERR_STR(status));
        hive_cleanup();
        return 1;
    }

    // Spawn controller
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "controller";

    actor_id id;
    if (HIVE_FAILED(hive_spawn_ex(controller, NULL, &cfg, &id))) {
        fprintf(stderr, "Failed to spawn controller\n");
        hive_bus_destroy(g_sensor_bus);
        hive_cleanup();
        return 1;
    }

    // Run scheduler
    hive_run();

    // Cleanup
    hive_bus_destroy(g_sensor_bus);
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
