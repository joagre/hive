#include "rt_runtime.h"
#include "rt_bus.h"
#include "rt_timer.h"
#include "rt_ipc.h"
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

    printf("Publisher actor started (ID: %u)\n", rt_self());

    // Create periodic timer (200ms)
    timer_id timer;
    rt_status status = rt_timer_every(200000, &timer);
    if (RT_FAILED(status)) {
        printf("Publisher: Failed to create timer: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    printf("Publisher: Created periodic timer (200ms)\n");

    // Publish 10 sensor readings
    for (int i = 0; i < 10; i++) {
        // Wait for timer tick
        rt_message msg;
        status = rt_ipc_recv(&msg, -1);
        if (RT_FAILED(status)) {
            printf("Publisher: Failed to receive: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        if (!rt_timer_is_tick(&msg)) {
            printf("Publisher: Unexpected message\n");
            continue;
        }

        // Create sensor data
        sensor_data data;
        data.timestamp = i;
        data.temperature = 20.0f + i * 0.5f;
        data.pressure = 1013.0f + i * 0.1f;

        // Publish to bus
        status = rt_bus_publish(g_sensor_bus, &data, sizeof(data));
        if (RT_FAILED(status)) {
            printf("Publisher: Failed to publish: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Publisher: Published reading #%d\n", i);
    }

    // Cancel timer
    rt_timer_cancel(timer);
    printf("Publisher: Done publishing\n");

    rt_exit();
}

// Subscriber actor - reads sensor data
static void subscriber_actor(void *arg) {
    const char *name = (const char *)arg;

    printf("%s actor started (ID: %u)\n", name, rt_self());

    // Subscribe to bus
    rt_status status = rt_bus_subscribe(g_sensor_bus);
    if (RT_FAILED(status)) {
        printf("%s: Failed to subscribe: %s\n", name,
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    printf("%s: Subscribed to sensor bus\n", name);

    // Read sensor data (subscriber A reads all, subscriber B reads first 5)
    int max_reads = (strcmp(name, "Subscriber A") == 0) ? 10 : 5;

    for (int i = 0; i < max_reads; i++) {
        sensor_data data;
        size_t actual_len;

        // Blocking read
        status = rt_bus_read_wait(g_sensor_bus, &data, sizeof(data),
                                  &actual_len, -1);
        if (RT_FAILED(status)) {
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
    rt_bus_unsubscribe(g_sensor_bus);
    printf("%s: Done reading\n", name);

    rt_exit();
}

int main(void) {
    printf("=== Actor Runtime Bus Example ===\n\n");

    // Initialize runtime
    rt_config cfg = RT_CONFIG_DEFAULT;
    cfg.max_actors = 5;

    rt_status status = rt_init(&cfg);
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Create bus with retention policy
    rt_bus_config bus_cfg;
    bus_cfg.max_readers = 0;        // Unlimited readers (data persists)
    bus_cfg.max_age_ms = 0;         // No time-based expiry
    bus_cfg.max_entries = 16;       // Ring buffer size
    bus_cfg.max_entry_size = 256;   // Max payload size

    status = rt_bus_create(&bus_cfg, &g_sensor_bus);
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to create bus: %s\n",
                status.msg ? status.msg : "unknown error");
        rt_cleanup();
        return 1;
    }

    printf("Created sensor bus (ID: %u)\n\n", g_sensor_bus);

    // Spawn subscriber actors
    actor_config actor_cfg = RT_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "subscriber_a";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id sub_a = rt_spawn_ex(subscriber_actor, (void *)"Subscriber A", &actor_cfg);
    if (sub_a == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn subscriber A\n");
        rt_cleanup();
        return 1;
    }

    actor_cfg.name = "subscriber_b";
    actor_id sub_b = rt_spawn_ex(subscriber_actor, (void *)"Subscriber B", &actor_cfg);
    if (sub_b == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn subscriber B\n");
        rt_cleanup();
        return 1;
    }

    // Spawn publisher actor
    actor_cfg.name = "publisher";
    actor_cfg.stack_size = 128 * 1024;  // Increase stack size
    actor_id pub = rt_spawn_ex(publisher_actor, NULL, &actor_cfg);
    if (pub == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn publisher\n");
        rt_cleanup();
        return 1;
    }

    printf("Spawned actors: publisher=%u, subscriber_a=%u, subscriber_b=%u\n\n",
           pub, sub_a, sub_b);

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");

    // Destroy bus
    status = rt_bus_destroy(g_sensor_bus);
    if (RT_FAILED(status)) {
        printf("Warning: Failed to destroy bus: %s\n",
               status.msg ? status.msg : "unknown error");
    }

    // Cleanup
    rt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
