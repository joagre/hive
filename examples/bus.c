#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <string.h>

// Shared bus ID
static bus_id g_sensor_bus = BUS_ID_INVALID;

// Sensor data structure
typedef struct {
    uint32_t timestamp;
    float    temperature;
    float    pressure;
} sensor_data;

// Publisher actor - publishes sensor data periodically
static void publisher_actor(void *arg) {
    (void)arg;

    printf("Publisher actor started (ID: %u)\n", hive_self());

    // Create periodic timer (200ms)
    timer_id timer;
    hive_status status = hive_timer_every(200000, &timer);
    if (HIVE_FAILED(status)) {
        printf("Publisher: Failed to create timer: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Publisher: Created periodic timer (200ms)\n");

    // Publish 10 sensor readings
    for (int i = 0; i < 10; i++) {
        // Wait for timer tick
        hive_message msg;
        status = hive_ipc_recv(&msg, -1);
        if (HIVE_FAILED(status)) {
            printf("Publisher: Failed to receive: %s\n", HIVE_ERR_STR(status));
            break;
        }

        if (!hive_msg_is_timer(&msg)) {
            printf("Publisher: Unexpected message\n");
            continue;
        }

        // Create sensor data
        sensor_data data;
        data.timestamp = i;
        data.temperature = 20.0f + i * 0.5f;
        data.pressure = 1013.0f + i * 0.1f;

        // Publish to bus
        status = hive_bus_publish(g_sensor_bus, &data, sizeof(data));
        if (HIVE_FAILED(status)) {
            printf("Publisher: Failed to publish: %s\n", HIVE_ERR_STR(status));
            break;
        }

        printf("Publisher: Published reading #%d\n", i);
    }

    // Cancel timer
    hive_timer_cancel(timer);
    printf("Publisher: Done publishing\n");

    hive_exit();
}

// Subscriber actor - reads sensor data
static void subscriber_actor(void *arg) {
    const char *name = (const char *)arg;

    printf("%s actor started (ID: %u)\n", name, hive_self());

    // Subscribe to bus
    hive_status status = hive_bus_subscribe(g_sensor_bus);
    if (HIVE_FAILED(status)) {
        printf("%s: Failed to subscribe: %s\n", name, HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("%s: Subscribed to sensor bus\n", name);

    // Read sensor data (subscriber A reads all, subscriber B reads first 5)
    int max_reads = (strcmp(name, "Subscriber A") == 0) ? 10 : 5;

    for (int i = 0; i < max_reads; i++) {
        sensor_data data;
        size_t actual_len;

        // Blocking read
        status = hive_bus_read_wait(g_sensor_bus, &data, sizeof(data),
                                  &actual_len, -1);
        if (HIVE_FAILED(status)) {
            printf("%s: Failed to read: %s\n", name, HIVE_ERR_STR(status));
            break;
        }

        if (actual_len != sizeof(data)) {
            printf("%s: Unexpected data size: %zu\n", name, actual_len);
            continue;
        }

        printf("%s: Read data #%u\n", name, data.timestamp);
    }

    // Unsubscribe
    hive_bus_unsubscribe(g_sensor_bus);
    printf("%s: Done reading\n", name);

    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Bus Example ===\n\n");

    // Initialize runtime
    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n", HIVE_ERR_STR(status));
        return 1;
    }

    // Create bus with retention policy
    hive_bus_config bus_cfg = HIVE_BUS_CONFIG_DEFAULT;
    bus_cfg.consume_after_reads = 0;        // Unlimited readers (data persists)
    bus_cfg.max_age_ms = 0;         // No time-based expiry
    bus_cfg.max_entries = 16;       // Ring buffer size
    bus_cfg.max_entry_size = 256;   // Max payload size
    bus_cfg.max_subscribers = 32;   // Maximum concurrent subscribers

    status = hive_bus_create(&bus_cfg, &g_sensor_bus);
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to create bus: %s\n", HIVE_ERR_STR(status));
        hive_cleanup();
        return 1;
    }

    printf("Created sensor bus (ID: %u)\n\n", g_sensor_bus);

    // Spawn subscriber actors
    actor_config actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "subscriber_a";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id sub_a;
    if (HIVE_FAILED(hive_spawn_ex(subscriber_actor, (void *)"Subscriber A", &actor_cfg, &sub_a))) {
        fprintf(stderr, "Failed to spawn subscriber A\n");
        hive_cleanup();
        return 1;
    }

    actor_cfg.name = "subscriber_b";
    actor_id sub_b;
    if (HIVE_FAILED(hive_spawn_ex(subscriber_actor, (void *)"Subscriber B", &actor_cfg, &sub_b))) {
        fprintf(stderr, "Failed to spawn subscriber B\n");
        hive_cleanup();
        return 1;
    }

    // Spawn publisher actor
    actor_cfg.name = "publisher";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id pub;
    if (HIVE_FAILED(hive_spawn_ex(publisher_actor, NULL, &actor_cfg, &pub))) {
        fprintf(stderr, "Failed to spawn publisher\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned actors: publisher=%u, subscriber_a=%u, subscriber_b=%u\n\n",
           pub, sub_a, sub_b);

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Destroy bus
    status = hive_bus_destroy(g_sensor_bus);
    if (HIVE_FAILED(status)) {
        printf("Warning: Failed to destroy bus: %s\n", HIVE_ERR_STR(status));
    }

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
