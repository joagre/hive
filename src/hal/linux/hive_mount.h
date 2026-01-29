// Linux Mount Table - Internal Header
//
// Defines the Linux-specific mount structure.
// Included by both hive_mounts.c and hive_hal_file.c.

#ifndef HIVE_MOUNT_LINUX_H
#define HIVE_MOUNT_LINUX_H

#include "hal/hive_mount.h"

// Linux mount structure
struct hive_mount {
    const char *prefix;          // Path prefix to match
    hive_file_backend_t backend; // Always POSIX on Linux
    const char *root;            // Filesystem root (empty = passthrough)
};

// Linux-specific accessor: get root path for building full filesystem path
const char *hive_mount_get_root(const hive_mount_t *mount);

#endif // HIVE_MOUNT_LINUX_H
