#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include <stdio.h>
#include <string.h>

// Shared bus ID
static bus_id_t s_sensor_bus = BUS_ID_INVALID;

// Sensor data structure
typedef struct {
    uint32_t timestamp;
    float temperature;
    float pressure;
} sensor_data_t;

// Publisher actor - publishes sensor data periodically
static void publisher_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Publisher actor started (ID: %u)\n", hive_self());

    // Create periodic timer (200ms)
    timer_id_t timer;
    hive_status_t status = hive_timer_every(200000, &timer);
    if (HIVE_FAILED(status)) {
        printf("Publisher: Failed to create timer: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Publisher: Created periodic timer (200ms)\n");

    // Publish 10 sensor readings
    for (int i = 0; i < 10; i++) {
        // Wait for timer tick using selective receive with timer_id_t
        hive_message_t msg;
        status = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer,
                                     &msg, -1);
        if (HIVE_FAILED(status)) {
            printf("Publisher: Failed to receive: %s\n", HIVE_ERR_STR(status));
            break;
        }

        // Create sensor data
        sensor_data_t data;
        data.timestamp = i;
        data.temperature = 20.0f + i * 0.5f;
        data.pressure = 1013.0f + i * 0.1f;

        // Publish to bus
        status = hive_bus_publish(s_sensor_bus, &data, sizeof(data));
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
static void subscriber_actor(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    const char *name = (const char *)args;

    printf("%s actor started (ID: %u)\n", name, hive_self());

    // Subscribe to bus
    hive_status_t status = hive_bus_subscribe(s_sensor_bus);
    if (HIVE_FAILED(status)) {
        printf("%s: Failed to subscribe: %s\n", name, HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("%s: Subscribed to sensor bus\n", name);

    // Read sensor data (subscriber A reads all, subscriber B reads first 5)
    int max_reads = (strcmp(name, "Subscriber A") == 0) ? 10 : 5;

    for (int i = 0; i < max_reads; i++) {
        sensor_data_t data;
        size_t actual_len;

        // Blocking read
        status = hive_bus_read_wait(s_sensor_bus, &data, sizeof(data),
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
    hive_bus_unsubscribe(s_sensor_bus);
    printf("%s: Done reading\n", name);

    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Bus Example ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Create bus with retention policy
    hive_bus_config_t bus_cfg = HIVE_BUS_CONFIG_DEFAULT;
    bus_cfg.consume_after_reads = 0; // Unlimited readers (data persists)
    bus_cfg.max_age_ms = 0;          // No time-based expiry
#ifdef QEMU_TEST_STACK_SIZE
    bus_cfg.max_entries = 4; // Reduced for QEMU
    bus_cfg.max_entry_size = 64;
    bus_cfg.max_subscribers = 4;
#else
    bus_cfg.max_entries = 16;     // Ring buffer size
    bus_cfg.max_entry_size = 256; // Max payload size
    bus_cfg.max_subscribers = 32; // Maximum concurrent subscribers
#endif

    status = hive_bus_create(&bus_cfg, &s_sensor_bus);
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to create bus: %s\n", HIVE_ERR_STR(status));
        hive_cleanup();
        return 1;
    }

    printf("Created sensor bus (ID: %u)\n\n", s_sensor_bus);

    // Spawn subscriber actors
    actor_config_t actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "subscriber_a";
#ifdef QEMU_TEST_STACK_SIZE
    actor_cfg.stack_size = 2048; // Reduced for QEMU
#else
    actor_cfg.stack_size = 128 * 1024; // Increase stack size
#endif
    actor_id_t sub_a;
    if (HIVE_FAILED(hive_spawn(subscriber_actor, NULL, (void *)"Subscriber A",
                               &actor_cfg, &sub_a))) {
        fprintf(stderr, "Failed to spawn subscriber A\n");
        hive_cleanup();
        return 1;
    }

    actor_cfg.name = "subscriber_b";
    actor_id_t sub_b;
    if (HIVE_FAILED(hive_spawn(subscriber_actor, NULL, (void *)"Subscriber B",
                               &actor_cfg, &sub_b))) {
        fprintf(stderr, "Failed to spawn subscriber B\n");
        hive_cleanup();
        return 1;
    }

    // Spawn publisher actor
    actor_cfg.name = "publisher";
    // Stack size already set above (reuse actor_cfg)
    actor_id_t pub;
    if (HIVE_FAILED(
            hive_spawn(publisher_actor, NULL, NULL, &actor_cfg, &pub))) {
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
    status = hive_bus_destroy(s_sensor_bus);
    if (HIVE_FAILED(status)) {
        printf("Warning: Failed to destroy bus: %s\n", HIVE_ERR_STR(status));
    }

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
