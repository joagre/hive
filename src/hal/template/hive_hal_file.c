// Hardware Abstraction Layer - File I/O Template
//
// This file provides a documented template for file I/O functions.
// Copy to src/hal/<platform>/hive_hal_file.c and implement.
//
// Compile with HIVE_ENABLE_FILE=1 to enable.

#include "hive_static_config.h"

#if HIVE_ENABLE_FILE

#include "hal/hive_hal_file.h"

// Initialize file I/O subsystem.
//
// Examples:
//   - Linux: No-op (POSIX always available)
//   - STM32: Initialize flash file system
//
hive_status_t hive_hal_file_init(void) {
    // TODO: Implement for your platform
    return HIVE_SUCCESS;
}

// Cleanup file I/O subsystem.
void hive_hal_file_cleanup(void) {
    // TODO: Implement for your platform
}

// Open a file.
//
// Parameters:
//   path - File path
//   flags - Open flags (HIVE_O_RDONLY, HIVE_O_WRONLY, HIVE_O_CREAT, etc.)
//   mode - File permissions (for HIVE_O_CREAT)
//   out - Output file descriptor
//
// Note: STM32 only supports virtual paths like "/log", "/config"
//
hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *out) {
    (void)path;
    (void)flags;
    (void)mode;
    (void)out;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Close a file.
hive_status_t hive_hal_file_close(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Read from file (sequential).
hive_status_t hive_hal_file_read(int fd, void *buf, size_t len,
                                 size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Read from file at offset (random access).
hive_status_t hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                  size_t *bytes_read) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_read;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Write to file (sequential).
hive_status_t hive_hal_file_write(int fd, const void *buf, size_t len,
                                  size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Write to file at offset (random access).
hive_status_t hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                   size_t offset, size_t *bytes_written) {
    (void)fd;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

// Sync file to storage.
hive_status_t hive_hal_file_sync(int fd) {
    (void)fd;
    return HIVE_ERROR(HIVE_ERR_INVALID, "File I/O not implemented");
}

#endif // HIVE_ENABLE_FILE
