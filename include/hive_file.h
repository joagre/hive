#ifndef HIVE_FILE_H
#define HIVE_FILE_H

#include "hive_types.h"

// Platform-independent file flags
// Use these instead of POSIX O_* flags for cross-platform compatibility
#define HIVE_O_RDONLY 0x0001
#define HIVE_O_WRONLY 0x0002
#define HIVE_O_RDWR 0x0003
#define HIVE_O_CREAT 0x0100
#define HIVE_O_TRUNC 0x0200
#define HIVE_O_APPEND 0x0400

// File operations
// All operations block the calling actor and yield to scheduler
//
// Linux: Standard filesystem paths
// STM32: Virtual paths mapped to flash sectors (e.g., "/log", "/config")
//        Configured via board -D flags (HIVE_VFILE_LOG_BASE, etc.)

// Open file
// flags: HIVE_O_RDONLY, HIVE_O_WRONLY, HIVE_O_RDWR, HIVE_O_CREAT, HIVE_O_TRUNC,
// etc.
hive_status_t hive_file_open(const char *path, int flags, int mode,
                             int *fd_out);

// Close file
hive_status_t hive_file_close(int fd);

// Read from file (blocks until complete or error)
hive_status_t hive_file_read(int fd, void *buf, size_t len, size_t *bytes_read);

// Read from file at offset (does not change file position)
hive_status_t hive_file_pread(int fd, void *buf, size_t len, size_t offset,
                              size_t *bytes_read);

// Write to file
hive_status_t hive_file_write(int fd, const void *buf, size_t len,
                              size_t *bytes_written);

// Write to file at offset (does not change file position)
hive_status_t hive_file_pwrite(int fd, const void *buf, size_t len,
                               size_t offset, size_t *bytes_written);

// Sync file to disk
hive_status_t hive_file_sync(int fd);

#endif // HIVE_FILE_H
