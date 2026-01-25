// Mount Table - File Backend Selection
//
// Provides path-based routing to different storage backends.
// Platform-independent interface with platform-specific implementations.
//
// Backends:
// - POSIX: Linux filesystem (passthrough)
// - FLASH: STM32 internal flash sectors
// - SD: SD card via FatFS (STM32 only, requires HIVE_ENABLE_SD)

#ifndef HIVE_MOUNT_H
#define HIVE_MOUNT_H

#include "hive_types.h"
#include <stddef.h>

// Storage backend types
typedef enum {
    HIVE_BACKEND_POSIX, // Linux: direct POSIX I/O
    HIVE_BACKEND_FLASH, // STM32: internal flash sector
    HIVE_BACKEND_SD,    // STM32: SD card via FatFS
} hive_file_backend_t;

// Forward declaration - actual struct is platform-specific
typedef struct hive_mount hive_mount_t;

// Find mount for a given path.
// Returns the mount with the longest matching prefix, or NULL if no match.
// prefix_len: Output - length of matched prefix (for extracting subpath)
const hive_mount_t *hive_mount_find(const char *path, size_t *prefix_len);

// Get number of configured mounts.
size_t hive_mount_count(void);

// Check if a mount is available (backend ready).
// For SD: checks if card is inserted and FatFS mounted.
// For FLASH/POSIX: always returns HIVE_SUCCESS.
// Returns:
// - HIVE_SUCCESS: Mount exists and backend is ready
// - HIVE_ERR_INVALID: No mount for path
// - HIVE_ERR_IO: Mount exists but backend unavailable
hive_status_t hive_mount_available(const char *path);

// ----------------------------------------------------------------------------
// Accessors (platform-specific struct, common interface)
// ----------------------------------------------------------------------------

// Get the backend type for a mount.
hive_file_backend_t hive_mount_get_backend(const hive_mount_t *mount);

// Get the path prefix for a mount.
const char *hive_mount_get_prefix(const hive_mount_t *mount);

#endif // HIVE_MOUNT_H
