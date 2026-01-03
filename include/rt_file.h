#ifndef RT_FILE_H
#define RT_FILE_H

#include "rt_types.h"
#include <fcntl.h>

// File operations
// All operations block the calling actor and yield to scheduler

// Open file
// flags: O_RDONLY, O_WRONLY, O_RDWR, O_CREAT, O_TRUNC, etc.
rt_status rt_file_open(const char *path, int flags, int mode, int *fd_out);

// Close file
rt_status rt_file_close(int fd);

// Read from file
rt_status rt_file_read(int fd, void *buf, size_t len, size_t *actual);

// Read from file at offset (does not change file position)
rt_status rt_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *actual);

// Write to file
rt_status rt_file_write(int fd, const void *buf, size_t len, size_t *actual);

// Write to file at offset (does not change file position)
rt_status rt_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *actual);

// Sync file to disk
rt_status rt_file_sync(int fd);

#endif // RT_FILE_H
