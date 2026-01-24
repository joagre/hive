// Logging example - Demonstrates structured logging with file output
//
// Shows:
// - HIVE_LOG_* macros at different levels (TRACE, DEBUG, INFO, WARN, ERROR)
// - Opening a binary log file
// - Periodic log sync using timers
// - Closing the log file
//
// The log file is written in binary format. Use tools/decode_log.py to read it:
//   python3 tools/decode_log.py /tmp/hive_logging_example.log
//
// Build with different log levels:
//   make CFLAGS="-DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_DEBUG ..."
//
// Log levels (from most to least verbose):
//   HIVE_LOG_LEVEL_TRACE (0) - Verbose tracing
//   HIVE_LOG_LEVEL_DEBUG (1) - Debug information
//   HIVE_LOG_LEVEL_INFO  (2) - General info (default)
//   HIVE_LOG_LEVEL_WARN  (3) - Warnings
//   HIVE_LOG_LEVEL_ERROR (4) - Errors only
//   HIVE_LOG_LEVEL_NONE  (5) - Disable all logging

#include "hive_runtime.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include <stdio.h>

#define LOG_FILE_PATH "/tmp/hive_logging_example.log"
#define SYNC_INTERVAL_US 500000 // Sync every 500ms
#define NUM_ITERATIONS 5

// Logging demo actor
static void logging_actor(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    HIVE_LOG_INFO("Logging actor started (ID: %u)", hive_self());

    // Open log file for binary output
    // On STM32, this would erase the flash sector (blocks briefly)
    HIVE_LOG_INFO("Opening log file: %s", LOG_FILE_PATH);
    hive_status_t status = hive_log_file_open(LOG_FILE_PATH);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to open log file: %s", HIVE_ERR_STR(status));
        // Continue anyway - console logging still works
    } else {
        HIVE_LOG_INFO("Log file opened successfully");
    }

    // Create periodic timer for log sync
    timer_id_t sync_timer;
    status = hive_timer_every(SYNC_INTERVAL_US, &sync_timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create sync timer: %s", HIVE_ERR_STR(status));
        hive_exit();
    }

    // Demonstrate different log levels
    HIVE_LOG_TRACE("This is a TRACE message (very verbose)");
    HIVE_LOG_DEBUG("This is a DEBUG message (debug info)");
    HIVE_LOG_INFO("This is an INFO message (general info)");
    HIVE_LOG_WARN("This is a WARN message (warning)");
    HIVE_LOG_ERROR("This is an ERROR message (error condition)");

    // Simulate some work with periodic logging and sync
    int iteration = 0;
    while (iteration < NUM_ITERATIONS) {
        hive_message_t msg;
        status = hive_ipc_recv(&msg, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("Failed to receive message: %s",
                           HIVE_ERR_STR(status));
            break;
        }

        if (hive_msg_is_timer(&msg) && msg.tag == sync_timer) {
            iteration++;

            // Log some data (simulating sensor readings, state, etc.)
            HIVE_LOG_INFO("Iteration %d/%d: sensor=%.2f, state=%d", iteration,
                          NUM_ITERATIONS, iteration * 1.5f, iteration * 10);

            // Periodic sync - flush logs to storage
            // On STM32, this writes the ring buffer to flash
            hive_log_file_sync();
            HIVE_LOG_DEBUG("Log synced to storage");
        }
    }

    // Cancel timer and close log file
    hive_timer_cancel(sync_timer);

    HIVE_LOG_INFO("Closing log file...");
    hive_log_file_close();
    HIVE_LOG_INFO("Log file closed");

    printf("\nLogging complete. Decode the binary log with:\n");
    printf("  python3 tools/decode_log.py %s\n\n", LOG_FILE_PATH);

    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Logging Example ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn logging demo actor
    actor_config_t actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    actor_cfg.name = "logging_demo";

    actor_id_t id;
    if (HIVE_FAILED(hive_spawn(logging_actor, NULL, NULL, &actor_cfg, &id))) {
        fprintf(stderr, "Failed to spawn logging actor\n");
        hive_cleanup();
        return 1;
    }

    // Run scheduler
    hive_run();

    printf("Scheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
