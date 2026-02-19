// Webots Crazyflie Mount Table Implementation
//
// Simulation-specific mount configuration for Webots.
// Uses POSIX passthrough to local filesystem.

#include "hive_mount.h"
#include <string.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Mount Table
// ----------------------------------------------------------------------------

static const hive_mount_t g_mounts[] = {
    {
        .prefix = "/tmp",
        .backend = HIVE_BACKEND_POSIX,
        .root = "/tmp",
    },
    {
        .prefix = "/var/tmp",
        .backend = HIVE_BACKEND_POSIX,
        .root = "/var/tmp",
    },
    // Sentinel to avoid empty array
    {.prefix = NULL, .backend = HIVE_BACKEND_POSIX, .root = NULL},
};

// Count excludes the sentinel
#define MOUNT_COUNT (sizeof(g_mounts) / sizeof(g_mounts[0]) - 1)

// ----------------------------------------------------------------------------
// Path Matching
// ----------------------------------------------------------------------------

// Check if path matches prefix with proper boundary handling.
// "/tmp" matches "/tmp", "/tmp/", "/tmp/foo" but NOT "/tmpdir".
static bool prefix_matches(const char *path, const char *prefix,
                           size_t *match_len) {
    if (prefix == NULL) {
        return false; // Sentinel entry
    }

    size_t plen = strlen(prefix);

    // Check prefix matches
    if (strncmp(path, prefix, plen) != 0) {
        return false;
    }

    // Must be exact match OR followed by '/' or end of string
    char next = path[plen];
    if (next == '\0' || next == '/') {
        *match_len = plen;
        return true;
    }

    return false;
}

// ----------------------------------------------------------------------------
// Mount Table API
// ----------------------------------------------------------------------------

const hive_mount_t *hive_mount_find(const char *path, size_t *prefix_len) {
    const hive_mount_t *best = NULL;
    size_t best_len = 0;

    for (size_t i = 0; i < MOUNT_COUNT; i++) {
        size_t len;
        if (prefix_matches(path, g_mounts[i].prefix, &len)) {
            if (len > best_len) {
                best = &g_mounts[i];
                best_len = len;
            }
        }
    }

    if (prefix_len != NULL) {
        *prefix_len = best_len;
    }
    return best;
}

size_t hive_mount_count(void) {
    return MOUNT_COUNT;
}

hive_status_t hive_mount_available(const char *path) {
    size_t prefix_len;
    const hive_mount_t *mount = hive_mount_find(path, &prefix_len);

    if (mount == NULL) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    // POSIX backend is always available
    return HIVE_SUCCESS;
}

// ----------------------------------------------------------------------------
// Common Accessors
// ----------------------------------------------------------------------------

hive_file_backend_t hive_mount_get_backend(const hive_mount_t *mount) {
    return mount->backend;
}

const char *hive_mount_get_prefix(const hive_mount_t *mount) {
    return mount->prefix;
}

// Linux-specific: get root path for building full filesystem path
const char *hive_mount_get_root(const hive_mount_t *mount) {
    return mount->root;
}
