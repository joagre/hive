#include "rt_file.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>

// Forward declarations for internal functions
rt_status rt_file_init(void);
void rt_file_cleanup(void);

// File I/O subsystem state (simplified - no worker thread!)
static struct {
    bool initialized;
} g_file = {0};

// Initialize file I/O subsystem
rt_status rt_file_init(void) {
    RT_INIT_GUARD(g_file.initialized);
    g_file.initialized = true;
    return RT_SUCCESS;
}

// Cleanup file I/O subsystem
void rt_file_cleanup(void) {
    RT_CLEANUP_GUARD(g_file.initialized);
    g_file.initialized = false;
}

// All file operations are now direct synchronous syscalls
// On embedded systems (FATFS/littlefs), these operations are fast (<1ms typically)
// Blocking the scheduler briefly is acceptable

rt_status rt_file_open(const char *path, int flags, int mode, int *fd_out) {
    if (!path || !fd_out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    int fd = open(path, flags, mode);
    if (fd < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    *fd_out = fd;
    return RT_SUCCESS;
}

rt_status rt_file_close(int fd) {
    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    if (close(fd) < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    return RT_SUCCESS;
}

rt_status rt_file_read(int fd, void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    ssize_t n = read(fd, buf, len);
    if (n < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    *actual = (size_t)n;
    return RT_SUCCESS;
}

rt_status rt_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    ssize_t n = pread(fd, buf, len, offset);
    if (n < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    *actual = (size_t)n;
    return RT_SUCCESS;
}

rt_status rt_file_write(int fd, const void *buf, size_t len, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    ssize_t n = write(fd, buf, len);
    if (n < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    *actual = (size_t)n;
    return RT_SUCCESS;
}

rt_status rt_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *actual) {
    if (!buf || !actual) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    ssize_t n = pwrite(fd, buf, len, offset);
    if (n < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    *actual = (size_t)n;
    return RT_SUCCESS;
}

rt_status rt_file_sync(int fd) {
    if (!g_file.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "File I/O subsystem not initialized");
    }

    if (fsync(fd) < 0) {
        return RT_ERROR(RT_ERR_IO, strerror(errno));
    }

    return RT_SUCCESS;
}
