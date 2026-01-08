#include "acrt_runtime.h"
#include "acrt_file.h"
#include <stdio.h>
#include <string.h>

// File writer actor
static void writer_actor(void *arg) {
    (void)arg;

    printf("Writer actor started (ID: %u)\n", acrt_self());

    const char *filename = "/tmp/actor_test.txt";
    const char *message = "Hello from actor runtime!\n";

    // Open file for writing
    int fd;
    acrt_status status = acrt_file_open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644, &fd);
    if (ACRT_FAILED(status)) {
        printf("Writer: Failed to open file: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("Writer: Opened file (fd=%d)\n", fd);

    // Write message
    size_t written;
    status = acrt_file_write(fd, message, strlen(message), &written);
    if (ACRT_FAILED(status)) {
        printf("Writer: Failed to write: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_file_close(fd);
        acrt_exit();
    }

    printf("Writer: Wrote %zu bytes\n", written);

    // Sync to disk
    status = acrt_file_sync(fd);
    if (ACRT_FAILED(status)) {
        printf("Writer: Failed to sync: %s\n",
               status.msg ? status.msg : "unknown error");
    }

    // Close file
    status = acrt_file_close(fd);
    if (ACRT_FAILED(status)) {
        printf("Writer: Failed to close: %s\n",
               status.msg ? status.msg : "unknown error");
    }

    printf("Writer: Done!\n");
    acrt_exit();
}

// File reader actor
static void reader_actor(void *arg) {
    (void)arg;

    printf("Reader actor started (ID: %u)\n", acrt_self());

    // Sleep briefly to let writer finish (in real code, use IPC coordination)
    for (int i = 0; i < 100000; i++) {
        acrt_yield();
    }

    const char *filename = "/tmp/actor_test.txt";

    // Open file for reading
    int fd;
    acrt_status status = acrt_file_open(filename, O_RDONLY, 0, &fd);
    if (ACRT_FAILED(status)) {
        printf("Reader: Failed to open file: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_exit();
    }

    printf("Reader: Opened file (fd=%d)\n", fd);

    // Read content
    char buffer[256] = {0};
    size_t nread;
    status = acrt_file_read(fd, buffer, sizeof(buffer) - 1, &nread);
    if (ACRT_FAILED(status)) {
        printf("Reader: Failed to read: %s\n",
               status.msg ? status.msg : "unknown error");
        acrt_file_close(fd);
        acrt_exit();
    }

    printf("Reader: Read %zu bytes: \"%s\"\n", nread, buffer);

    // Close file
    status = acrt_file_close(fd);
    if (ACRT_FAILED(status)) {
        printf("Reader: Failed to close: %s\n",
               status.msg ? status.msg : "unknown error");
    }

    printf("Reader: Done!\n");
    acrt_exit();
}

int main(void) {
    printf("=== Actor Runtime File I/O Example ===\n\n");

    // Initialize runtime
    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    printf("Runtime initialized\n");

    // Spawn writer actor
    actor_config writer_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    writer_cfg.name = "writer";
    writer_cfg.priority = ACRT_PRIORITY_NORMAL;

    actor_id writer_id;
    if (ACRT_FAILED(acrt_spawn_ex(writer_actor, NULL, &writer_cfg, &writer_id))) {
        fprintf(stderr, "Failed to spawn writer actor\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned writer actor (ID: %u)\n", writer_id);

    // Spawn reader actor
    actor_config reader_cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    reader_cfg.name = "reader";
    reader_cfg.priority = ACRT_PRIORITY_NORMAL;

    actor_id reader_id;
    if (ACRT_FAILED(acrt_spawn_ex(reader_actor, NULL, &reader_cfg, &reader_id))) {
        fprintf(stderr, "Failed to spawn reader actor\n");
        acrt_cleanup();
        return 1;
    }

    printf("Spawned reader actor (ID: %u)\n", reader_id);

    printf("\nStarting scheduler...\n\n");

    // Run scheduler
    acrt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    acrt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
