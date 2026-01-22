// Hardware Abstraction Layer - File I/O
//
// Abstracts platform-specific file operations:
// - Linux: POSIX file I/O
// - STM32: Flash-backed virtual files
//
// The common hive_file.c wrapper handles:
// - Subsystem initialization guards
// - Argument validation
// - HIVE_O_* flag definitions
//
// HAL implementations only need to provide the core operations.

#ifndef HIVE_HAL_FILE_H
#define HIVE_HAL_FILE_H

#include "hive_types.h"
#include <stddef.h>

// Initialize file I/O subsystem.
// Linux: No-op
// STM32: Reset ring buffer and virtual file state
hive_status hive_hal_file_init(void);

// Cleanup file I/O subsystem.
// Linux: No-op
// STM32: Close any open virtual files
void hive_hal_file_cleanup(void);

// Open a file.
// path: File path (Linux: filesystem path, STM32: virtual path like "/log")
// flags: HIVE_O_* flags (HIVE_O_RDONLY, HIVE_O_WRONLY, etc.)
// mode: Permission mode (Linux only, ignored on STM32)
// fd_out: Output file descriptor
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_file_open(const char *path, int flags, int mode,
                               int *fd_out);

// Close a file.
// fd: File descriptor to close
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_file_close(int fd);

// Read from file.
// fd: File descriptor
// buf: Buffer to read into
// len: Maximum bytes to read
// bytes_read: Output - actual bytes read
// Returns: HIVE_SUCCESS or error status
// Note: STM32 returns error (use pread instead)
hive_status hive_hal_file_read(int fd, void *buf, size_t len,
                               size_t *bytes_read);

// Read from file at offset (does not change file position).
// fd: File descriptor
// buf: Buffer to read into
// len: Maximum bytes to read
// offset: Position to read from
// bytes_read: Output - actual bytes read
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                size_t *bytes_read);

// Write to file.
// fd: File descriptor
// buf: Data to write
// len: Bytes to write
// bytes_written: Output - actual bytes written
// Returns: HIVE_SUCCESS or error status
hive_status hive_hal_file_write(int fd, const void *buf, size_t len,
                                size_t *bytes_written);

// Write to file at offset (does not change file position).
// fd: File descriptor
// buf: Data to write
// len: Bytes to write
// offset: Position to write at
// bytes_written: Output - actual bytes written
// Returns: HIVE_SUCCESS or error status
// Note: STM32 returns error (ring buffer doesn't support random writes)
hive_status hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                 size_t offset, size_t *bytes_written);

// Sync file to storage.
// fd: File descriptor
// Returns: HIVE_SUCCESS or error status
// Linux: fsync()
// STM32: Flush ring buffer to flash
hive_status hive_hal_file_sync(int fd);

#endif // HIVE_HAL_FILE_H
