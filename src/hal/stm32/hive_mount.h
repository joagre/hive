// STM32 Mount Table - Internal Header
//
// Defines the STM32-specific mount structure with flash and SD configurations.
// Included by both hive_mounts.c and hive_hal_file_stm32.c.

#ifndef HIVE_MOUNT_STM32_H
#define HIVE_MOUNT_STM32_H

#include "hal/hive_mount.h"
#include "hive_static_config.h"
#include <stdint.h>

// STM32 mount structure
struct hive_mount {
    const char *prefix;          // Path prefix to match
    hive_file_backend_t backend; // FLASH or SD
    union {
        struct {
            uint32_t base;  // Flash base address
            uint32_t size;  // Flash region size
            uint8_t sector; // Flash sector number
        } flash;
#if HIVE_ENABLE_SD
        struct {
            uint8_t spi_id;  // 1=SPI1, 2=SPI2, 3=SPI3
            uint8_t cs_port; // 0=GPIOA, 1=GPIOB, etc.
            uint8_t cs_pin;  // 0-15
        } sd;
#endif
    };
};

// ----------------------------------------------------------------------------
// File descriptor encoding
// ----------------------------------------------------------------------------

// fd layout: [backend:4][index:12]
// - Flash: fd = 0x1000 + index
// - SD:    fd = 0x2000 + index

#define FD_MAKE(backend, index) (((int)(backend) << 12) | (index))
#define FD_BACKEND(fd) ((hive_file_backend_t)((fd) >> 12))
#define FD_INDEX(fd) ((fd) & 0xFFF)

// Maximum open files per backend
#define HIVE_MAX_FLASH_FILES 4

#ifndef HIVE_MAX_SD_FILES
#define HIVE_MAX_SD_FILES 4
#endif

// ----------------------------------------------------------------------------
// STM32-specific accessors
// ----------------------------------------------------------------------------

// Get flash configuration (only valid for FLASH backend)
static inline uint32_t hive_mount_flash_base(const hive_mount_t *mount) {
    return mount->flash.base;
}

static inline uint32_t hive_mount_flash_size(const hive_mount_t *mount) {
    return mount->flash.size;
}

static inline uint8_t hive_mount_flash_sector(const hive_mount_t *mount) {
    return mount->flash.sector;
}

#if HIVE_ENABLE_SD
// Get SD configuration (only valid for SD backend)
static inline uint8_t hive_mount_sd_spi(const hive_mount_t *mount) {
    return mount->sd.spi_id;
}

static inline uint8_t hive_mount_sd_cs_port(const hive_mount_t *mount) {
    return mount->sd.cs_port;
}

static inline uint8_t hive_mount_sd_cs_pin(const hive_mount_t *mount) {
    return mount->sd.cs_pin;
}
#endif

#endif // HIVE_MOUNT_STM32_H
