// Crazyflie 2.1+ Mount Table Implementation
//
// Board-specific mount configuration for Crazyflie 2.1+ with optional
// Micro SD Card Deck.
//
// Flash mounts configured via -D flags in Makefile.crazyflie-2.1+:
//   HIVE_VFILE_LOG_BASE, HIVE_VFILE_LOG_SIZE, HIVE_VFILE_LOG_SECTOR
//
// SD card deck configuration:
//   SPI3 (PB3=SCK, PB4=MISO, PB5=MOSI)
//   CS: PB6
//
// Reference: https://www.bitcraze.io/products/micro-sd-card-deck/

#include "hive_mount.h"
#include <string.h>
#include <stdbool.h>

// ----------------------------------------------------------------------------
// Crazyflie Micro SD Card Deck Configuration
// ----------------------------------------------------------------------------

// SD card deck uses SPI3 with CS on PB6
// Port numbering: 0=GPIOA, 1=GPIOB, 2=GPIOC, 3=GPIOD
#define CF_SD_SPI_ID 3  // SPI3
#define CF_SD_CS_PORT 1 // GPIOB
#define CF_SD_CS_PIN 6  // Pin 6

// ----------------------------------------------------------------------------
// Mount Table
// ----------------------------------------------------------------------------

static const hive_mount_t g_mounts[] = {
#ifdef HIVE_VFILE_LOG_BASE
    {
        .prefix = "/log",
        .backend = HIVE_BACKEND_FLASH,
        .flash =
            {
                .base = HIVE_VFILE_LOG_BASE,
                .size = HIVE_VFILE_LOG_SIZE,
                .sector = HIVE_VFILE_LOG_SECTOR,
            },
    },
#endif
#ifdef HIVE_VFILE_CONFIG_BASE
    {
        .prefix = "/config",
        .backend = HIVE_BACKEND_FLASH,
        .flash =
            {
                .base = HIVE_VFILE_CONFIG_BASE,
                .size = HIVE_VFILE_CONFIG_SIZE,
                .sector = HIVE_VFILE_CONFIG_SECTOR,
            },
    },
#endif
#if HIVE_ENABLE_SD
    {
        .prefix = "/sd",
        .backend = HIVE_BACKEND_SD,
        .sd =
            {
                .spi_id = CF_SD_SPI_ID,
                .cs_port = CF_SD_CS_PORT,
                .cs_pin = CF_SD_CS_PIN,
            },
    },
#endif
    // Sentinel to avoid empty array (not valid in C11)
    {.prefix = NULL, .backend = HIVE_BACKEND_FLASH, .flash = {0, 0, 0}},
};

// Count excludes the sentinel
#define MOUNT_COUNT (sizeof(g_mounts) / sizeof(g_mounts[0]) - 1)

// ----------------------------------------------------------------------------
// Path Matching
// ----------------------------------------------------------------------------

// Check if path matches prefix with proper boundary handling.
// "/log" matches "/log", "/log/" but NOT "/logger".
static bool prefix_matches(const char *path, const char *prefix,
                           size_t *match_len) {
    if (!prefix) {
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

    if (prefix_len) {
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

    if (!mount) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    switch (mount->backend) {
    case HIVE_BACKEND_FLASH:
        // Flash is always available
        return HIVE_SUCCESS;

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD: {
        // Check if SD card is available (defined in hive_hal_file_stm32.c)
        extern hive_status_t sd_check_available(const hive_mount_t *mount);
        return sd_check_available(mount);
    }
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "unsupported backend");
    }
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
