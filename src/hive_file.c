// Unified File I/O Implementation
//
// Platform-independent wrapper that handles:
// - Subsystem initialization guards
// - Argument validation
//
// Actual I/O operations are delegated to platform-specific HAL implementations:
// - Linux: POSIX file I/O (in hive_hal_file.c)
// - STM32: Flash-backed virtual files (in hive_hal_file_stm32.c)

#include "hive_file.h"
#include "hive_internal.h"
#include "hal/hive_hal_file.h"
#include "hal/hive_mount.h"

// File I/O subsystem state
static struct {
    bool initialized;
} s_file = {0};

hive_status_t hive_file_init(void) {
    HIVE_INIT_GUARD(s_file.initialized);

    hive_status_t status = hive_hal_file_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    s_file.initialized = true;
    return HIVE_SUCCESS;
}

void hive_file_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_file.initialized);

    hive_hal_file_cleanup();
    s_file.initialized = false;
}

hive_status_t hive_file_open(const char *path, int flags, int mode, int *out) {
    if (!path || !out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL path or out pointer");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_open(path, flags, mode, out);
}

hive_status_t hive_file_close(int fd) {
    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_close(fd);
}

hive_status_t hive_file_read(int fd, void *buf, size_t len,
                             size_t *bytes_read) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_read(fd, buf, len, bytes_read);
}

hive_status_t hive_file_pread(int fd, void *buf, size_t len, size_t offset,
                              size_t *bytes_read) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_pread(fd, buf, len, offset, bytes_read);
}

hive_status_t hive_file_write(int fd, const void *buf, size_t len,
                              size_t *bytes_written) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_write(fd, buf, len, bytes_written);
}

hive_status_t hive_file_pwrite(int fd, const void *buf, size_t len,
                               size_t offset, size_t *bytes_written) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_pwrite(fd, buf, len, offset, bytes_written);
}

hive_status_t hive_file_sync(int fd) {
    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_hal_file_sync(fd);
}

hive_status_t hive_file_mount_available(const char *path) {
    if (!path) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL path");
    }

    HIVE_REQUIRE_INIT(s_file.initialized, "File I/O");

    return hive_mount_available(path);
}
