#ifndef ACRT_FILE_H
#define ACRT_FILE_H

#include "acrt_types.h"
#include <fcntl.h>

// File operations
// All operations block the calling actor and yield to scheduler

// Open file
// flags: O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, etc.
acrt_status acrt_file_open(const char *path, int flags, int mode, int *fd_out);

// Close file
acrt_status acrt_file_close(int fd);

// Read from file (blocks until complete or error)
acrt_status acrt_file_read(int fd, void *buf, size_t len, size_t *bytes_read);

// Read from file at offset (does not change file position)
acrt_status acrt_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read);

// Write to file
acrt_status acrt_file_write(int fd, const void *buf, size_t len, size_t *bytes_written);

// Write to file at offset (does not change file position)
acrt_status acrt_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written);

// Sync file to disk
acrt_status acrt_file_sync(int fd);

#endif // ACRT_FILE_H
