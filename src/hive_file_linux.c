#include "hive_file.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include <unistd.h>
#include <fcntl.h>

// Map HIVE_O_* flags to POSIX O_* flags
static int hive_flags_to_posix(int hive_flags) {
    int posix_flags = 0;

    // Access mode (mutually exclusive)
    int access = hive_flags & 0x0003;
    if (access == HIVE_O_RDONLY)
        posix_flags |= O_RDONLY;
    else if (access == HIVE_O_WRONLY)
        posix_flags |= O_WRONLY;
    else if (access == HIVE_O_RDWR)
        posix_flags |= O_RDWR;

    // Additional flags
    if (hive_flags & HIVE_O_CREAT)
        posix_flags |= O_CREAT;
    if (hive_flags & HIVE_O_TRUNC)
        posix_flags |= O_TRUNC;
    if (hive_flags & HIVE_O_APPEND)
        posix_flags |= O_APPEND;

    return posix_flags;
}

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

// On embedded systems (FATFS/littlefs), these operations are fast (<1ms
// typically) Blocking the scheduler briefly is acceptable

hive_status hive_file_open(const char *path, int flags, int mode, int *fd_out) {
    if (!path || !fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL path or fd_out pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    // Convert HIVE_O_* flags to POSIX O_* flags
    int posix_flags = hive_flags_to_posix(flags);

    int fd = open(path, posix_flags, mode);
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

hive_status hive_file_read(int fd, void *buf, size_t len, size_t *bytes_read) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "read failed");
    }

    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_pread(int fd, void *buf, size_t len, size_t offset,
                            size_t *bytes_read) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = pread(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pread failed");
    }

    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_write(int fd, const void *buf, size_t len,
                            size_t *bytes_written) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "write failed");
    }

    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_pwrite(int fd, const void *buf, size_t len, size_t offset,
                             size_t *bytes_written) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    ssize_t n = pwrite(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pwrite failed");
    }

    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status hive_file_sync(int fd) {
    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    if (fsync(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "fsync failed");
    }

    return HIVE_SUCCESS;
}
