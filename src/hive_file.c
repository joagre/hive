#include "hive_file.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

// File I/O subsystem state
static struct {
    bool initialized;
} g_file = {0};

// Initialize file I/O subsystem
hive_status hive_file_init(void) {
    HIVE_INIT_GUARD(g_file.initialized);
    g_file.initialized = true;
    return HIVE_SUCCESS;
}

// Cleanup file I/O subsystem
void hive_file_cleanup(void) {
    HIVE_CLEANUP_GUARD(g_file.initialized);
    g_file.initialized = false;
}

// On embedded systems (FATFS/littlefs), these operations are fast (<1ms typically)
// Blocking the scheduler briefly is acceptable

hive_status hive_file_open(const char *path, int flags, int mode, int *fd_out) {
    if (!path || !fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    int fd = open(path, flags, mode);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "open failed");
    }

    *fd_out = fd;
    return HIVE_SUCCESS;
}

hive_status hive_file_close(int fd) {
    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    if (close(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "close failed");
    }

    return HIVE_SUCCESS;
}

hive_status hive_file_read(int fd, void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "read failed");
    }

    *actual = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = pread(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pread failed");
    }

    *actual = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_write(int fd, const void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "write failed");
    }

    *actual = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid arguments");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = pwrite(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pwrite failed");
    }

    *actual = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_sync(int fd) {
    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    if (fsync(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "fsync failed");
    }

    return HIVE_SUCCESS;
}
