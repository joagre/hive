#include "acrt_runtime.h"
#include "acrt_bus.h"
#include "acrt_timer.h"
#include "acrt_ipc.h"
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

    printf("Publisher actor started (ID: %u)\n", acrt_self());

    // Create periodic timer (200ms)
    timer_id timer;
    acrt_status status = acrt_timer_every(200000, &timer);
    if (ACRT_FAILED(status)) {
        printf("Publisher: Failed to create timer: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("Publisher: Created periodic timer (200ms)\n");

    // Publish 10 sensor readings
    for (int i = 0; i < 10; i++) {
        // Wait for timer tick
        acrt_message msg;
        status = acrt_ipc_recv(&msg, -1);
        if (ACRT_FAILED(status)) {
            printf("Publisher: Failed to receive: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        if (!acrt_msg_is_timer(&msg)) {
            printf("Publisher: Unexpected message\n");
            continue;
        }

        // Create sensor data
        sensor_data data;
        data.timestamp = i;
        data.temperature = 20.0f + i * 0.5f;
        data.pressure = 1013.0f + i * 0.1f;

        // Publish to bus
        status = acrt_bus_publish(g_sensor_bus, &data, sizeof(data));
        if (ACRT_FAILED(status)) {
            printf("Publisher: Failed to publish: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Publisher: Published reading #%d\n", i);
    }

    // Cancel timer
    acrt_timer_cancel(timer);
    printf("Publisher: Done publishing\n");

    acrt_exit();
}

// Subscriber actor - reads sensor data
static void subscriber_actor(void *arg) {
    const char *name = (const char *)arg;

    printf("%s actor started (ID: %u)\n", name, acrt_self());

    // Subscribe to bus
    acrt_status status = acrt_bus_subscribe(g_sensor_bus);
    if (ACRT_FAILED(status)) {
        printf("%s: Failed to subscribe: %s\n", name,
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("%s: Subscribed to sensor bus\n", name);

    // Read sensor data (subscriber A reads all, subscriber B reads first 5)
    int max_reads = (strcmp(name, "Subscriber A") == 0) ? 10 : 5;

    for (int i = 0; i < max_reads; i++) {
        sensor_data data;
        size_t actual_len;

        // Blocking read
        status = acrt_bus_read_wait(g_sensor_bus, &data, sizeof(data),
                                  &actual_len, -1);
        if (ACRT_FAILED(status)) {
            printf("%s: Failed to read: %s\n", name,
                   status.msg ? status.msg : "unknown error");
            break;
        }

        if (actual_len != sizeof(data)) {
            printf("%s: Unexpected data size: %zu\n", name, actual_len);
            continue;
        }

        printf("%s: Read data #%u\n", name, data.timestamp);
    }

    // Unsubscribe
    acrt_bus_unsubscribe(g_sensor_bus);
    printf("%s: Done reading\n", name);

    acrt_exit();
}

int main(void) {
    printf("=== Actor Runtime Bus Example ===\n\n");

    // Initialize runtime
    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Create bus with retention policy
    acrt_bus_config bus_cfg = ACRT_BUS_CONFIG_DEFAULT;
    bus_cfg.consume_after_reads = 0;        // Unlimited readers (data persists)
    bus_cfg.max_age_ms = 0;         // No time-based expiry
    bus_cfg.max_entries = 16;       // Ring buffer size
    bus_cfg.max_entry_size = 256;   // Max payload size
    bus_cfg.max_subscribers = 32;   // Maximum concurrent subscribers

    status = acrt_bus_create(&bus_cfg, &g_sensor_bus);
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to create bus: %s\n",
                status.msg ? status.msg : "unknown error");
        acrt_cleanup();
        return 1;
    }

    printf("Created sensor bus (ID: %u)\n\n", g_sensor_bus);

    // Spawn subscriber actors
    actor_config actor_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "subscriber_a";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id sub_a;
    if (ACRT_FAILED(acrt_spawn_ex(subscriber_actor, (void *)"Subscriber A", &actor_cfg, &sub_a))) {
        fprintf(stderr, "Failed to spawn subscriber A\n");
        acrt_cleanup();
        return 1;
    }

    actor_cfg.name = "subscriber_b";
    actor_id sub_b;
    if (ACRT_FAILED(acrt_spawn_ex(subscriber_actor, (void *)"Subscriber B", &actor_cfg, &sub_b))) {
        fprintf(stderr, "Failed to spawn subscriber B\n");
        acrt_cleanup();
        return 1;
    }

    // Spawn publisher actor
    actor_cfg.name = "publisher";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id pub;
    if (ACRT_FAILED(acrt_spawn_ex(publisher_actor, NULL, &actor_cfg, &pub))) {
        fprintf(stderr, "Failed to spawn publisher\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned actors: publisher=%u, subscriber_a=%u, subscriber_b=%u\n\n",
           pub, sub_a, sub_b);

    // Run scheduler
    acrt_run();

    printf("\nScheduler finished\n");

    // Destroy bus
    status = acrt_bus_destroy(g_sensor_bus);
    if (ACRT_FAILED(status)) {
        printf("Warning: Failed to destroy bus: %s\n",
               status.msg ? status.msg : "unknown error");
    }

    // Cleanup
    acrt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
