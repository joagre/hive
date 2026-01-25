// STM32 Mount Table - Stub Implementation
//
// Placeholder until full mount table refactoring is complete.
// The existing flash virtual file system continues to work via
// hive_hal_file_stm32.c which handles /log and /config directly.
//
// TODO: Refactor to use mount table when SD card support is added.

#include "hal/hive_mount.h"
#include <stddef.h>

// STM32 mount structure (stub - not used yet)
struct hive_mount {
    const char *prefix;
    hive_file_backend_t backend;
};

// No mounts configured via table yet - flash paths handled directly by HAL
// Dummy entry to avoid empty array (not valid in C11)
static const hive_mount_t g_mounts[] = {
    {.prefix = NULL, .backend = HIVE_BACKEND_FLASH}, // Unused sentinel
};

#define MOUNT_COUNT 0 // Actual count is 0 - sentinel not counted

const hive_mount_t *hive_mount_find(const char *path, size_t *prefix_len) {
    (void)path;
    (void)g_mounts; // Suppress unused warning - sentinel only
    if (prefix_len) {
        *prefix_len = 0;
    }
    // Return NULL - flash paths are handled directly by HAL
    return NULL;
}

size_t hive_mount_count(void) {
    return MOUNT_COUNT;
}

hive_status_t hive_mount_available(const char *path) {
    (void)path;
    // Flash paths are always available, but handled directly by HAL
    // This stub returns INVALID since no mount table entries exist
    // The HAL will still handle /log and /config directly
    return HIVE_ERROR(HIVE_ERR_INVALID, "mount table not configured");
}

hive_file_backend_t hive_mount_get_backend(const hive_mount_t *mount) {
    (void)mount;
    return HIVE_BACKEND_FLASH;
}

const char *hive_mount_get_prefix(const hive_mount_t *mount) {
    (void)mount;
    return "";
}
