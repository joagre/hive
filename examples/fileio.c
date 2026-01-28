#include "hive_runtime.h"
#include "hive_file.h"
#include <stdio.h>
#include <string.h>

// File writer actor
static void writer_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Writer actor started (ID: %u)\n", hive_self());

    const char *filename = "/tmp/actor_test.txt";
    const char *message = "Hello from actor runtime!\n";

    // Open file for writing
    int fd;
    hive_status_t status = hive_file_open(
        filename, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644, &fd);
    if (HIVE_FAILED(status)) {
        printf("Writer: Failed to open file: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Writer: Opened file (fd=%d)\n", fd);

    // Write message
    size_t written;
    status = hive_file_write(fd, message, strlen(message), &written);
    if (HIVE_FAILED(status)) {
        printf("Writer: Failed to write: %s\n", HIVE_ERR_STR(status));
        hive_file_close(fd);
        hive_exit();
    }

    printf("Writer: Wrote %zu bytes\n", written);

    // Sync to disk
    status = hive_file_sync(fd);
    if (HIVE_FAILED(status)) {
        printf("Writer: Failed to sync: %s\n", HIVE_ERR_STR(status));
    }

    // Close file
    status = hive_file_close(fd);
    if (HIVE_FAILED(status)) {
        printf("Writer: Failed to close: %s\n", HIVE_ERR_STR(status));
    }

    printf("Writer: Done!\n");
    hive_exit();
}

// File reader actor
static void reader_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Reader actor started (ID: %u)\n", hive_self());

    // Sleep briefly to let writer finish (in real code, use IPC coordination)
    for (int i = 0; i < 100000; i++) {
        hive_yield();
    }

    const char *filename = "/tmp/actor_test.txt";

    // Open file for reading
    int fd;
    hive_status_t status = hive_file_open(filename, HIVE_O_RDONLY, 0, &fd);
    if (HIVE_FAILED(status)) {
        printf("Reader: Failed to open file: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Reader: Opened file (fd=%d)\n", fd);

    // Read content
    char buffer[256] = {0};
    size_t nread;
    status = hive_file_read(fd, buffer, sizeof(buffer) - 1, &nread);
    if (HIVE_FAILED(status)) {
        printf("Reader: Failed to read: %s\n", HIVE_ERR_STR(status));
        hive_file_close(fd);
        hive_exit();
    }

    printf("Reader: Read %zu bytes: \"%s\"\n", nread, buffer);

    // Close file
    status = hive_file_close(fd);
    if (HIVE_FAILED(status)) {
        printf("Reader: Failed to close: %s\n", HIVE_ERR_STR(status));
    }

    printf("Reader: Done!\n");
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime File I/O Example ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    printf("Runtime initialized\n");

    // Spawn writer actor
    hive_actor_config_t writer_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    writer_cfg.name = "writer";
    writer_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id_t writer_id;
    if (HIVE_FAILED(
            hive_spawn(writer_actor, NULL, NULL, &writer_cfg, &writer_id))) {
        fprintf(stderr, "Failed to spawn writer actor\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned writer actor (ID: %u)\n", writer_id);

    // Spawn reader actor
    hive_actor_config_t reader_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    reader_cfg.name = "reader";
    reader_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id_t reader_id;
    if (HIVE_FAILED(
            hive_spawn(reader_actor, NULL, NULL, &reader_cfg, &reader_id))) {
        fprintf(stderr, "Failed to spawn reader actor\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned reader actor (ID: %u)\n", reader_id);

    printf("\nStarting scheduler...\n\n");

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
