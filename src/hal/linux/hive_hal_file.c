// Hardware Abstraction Layer - Linux File I/O Implementation
//
// File I/O uses POSIX file operations.

#include "hive_static_config.h"

#if HIVE_ENABLE_FILE

#include "hal/hive_hal_file.h"
#include "hive_file.h"
#include "hive_mount.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

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

hive_status_t hive_hal_file_init(void) {
    // No initialization needed for POSIX file I/O
    return HIVE_SUCCESS;
}

void hive_hal_file_cleanup(void) {
    // No cleanup needed for POSIX file I/O
}

hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *out) {
    // Find mount for path
    size_t prefix_len;
    const hive_mount_t *mount = hive_mount_find(path, &prefix_len);
    if (!mount) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    // Build full path: root + subpath
    const char *root = hive_mount_get_root(mount);
    const char *subpath = path + prefix_len;

    // Handle root "/" prefix specially - subpath includes leading slash
    if (prefix_len == 1 && path[0] == '/') {
        subpath = path; // Keep full path for "/" mount
    }

    char fullpath[512];
    if (root[0] == '\0') {
        // Empty root = direct passthrough
        snprintf(fullpath, sizeof(fullpath), "%s", path);
    } else {
        snprintf(fullpath, sizeof(fullpath), "%s%s", root, subpath);
    }

    int posix_flags = hive_flags_to_posix(flags);
    int fd = open(fullpath, posix_flags, mode);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "open failed");
    }
    *out = fd;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_close(int fd) {
    if (close(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "close failed");
    }
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_read(int fd, void *buf, size_t len,
                                 size_t *bytes_read) {
    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "read failed");
    }
    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                  size_t *bytes_read) {
    ssize_t n = pread(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pread failed");
    }
    *bytes_read = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_write(int fd, const void *buf, size_t len,
                                  size_t *bytes_written) {
    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "write failed");
    }
    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                   size_t offset, size_t *bytes_written) {
    ssize_t n = pwrite(fd, buf, len, offset);
    if (n < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "pwrite failed");
    }
    *bytes_written = (size_t)n;
    return HIVE_SUCCESS;
}

hive_status_t hive_hal_file_sync(int fd) {
    if (fsync(fd) < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "fsync failed");
    }
    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_FILE
