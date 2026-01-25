// FatFS Disk I/O Implementation for SD Card via SPI
//
// Implements the diskio interface for FatFS using SPI SD card driver.

#include "diskio.h"
#include "ffconf.h"

#if HIVE_ENABLE_SD

// Forward declarations for SPI SD driver (implemented in spi_sd.c)
extern int spi_sd_init(void);
extern int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count);
extern int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector,
                               uint32_t count);
extern int spi_sd_sync(void);
extern uint32_t spi_sd_get_sector_count(void);
extern int spi_sd_is_present(void);

// ---------------------------------------------------------------------------
// Disk Status
// ---------------------------------------------------------------------------

static DSTATUS s_disk_status = STA_NOINIT;

// ---------------------------------------------------------------------------
// disk_initialize - Initialize SD card
// ---------------------------------------------------------------------------

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv != 0) {
        return STA_NOINIT; // Only drive 0 supported
    }

    if (!spi_sd_is_present()) {
        s_disk_status = STA_NOINIT | STA_NODISK;
        return s_disk_status;
    }

    if (spi_sd_init() != 0) {
        s_disk_status = STA_NOINIT;
        return s_disk_status;
    }

    s_disk_status = 0; // Initialized successfully
    return s_disk_status;
}

// ---------------------------------------------------------------------------
// disk_status - Get disk status
// ---------------------------------------------------------------------------

DSTATUS disk_status(BYTE pdrv) {
    if (pdrv != 0) {
        return STA_NOINIT;
    }
    return s_disk_status;
}

// ---------------------------------------------------------------------------
// disk_read - Read sectors
// ---------------------------------------------------------------------------

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || count == 0) {
        return RES_PARERR;
    }

    if (s_disk_status & STA_NOINIT) {
        return RES_NOTRDY;
    }

    if (spi_sd_read_blocks(buff, sector, count) != 0) {
        return RES_ERROR;
    }

    return RES_OK;
}

// ---------------------------------------------------------------------------
// disk_write - Write sectors
// ---------------------------------------------------------------------------

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || count == 0) {
        return RES_PARERR;
    }

    if (s_disk_status & STA_NOINIT) {
        return RES_NOTRDY;
    }

    if (s_disk_status & STA_PROTECT) {
        return RES_WRPRT;
    }

    if (spi_sd_write_blocks(buff, sector, count) != 0) {
        return RES_ERROR;
    }

    return RES_OK;
}

// ---------------------------------------------------------------------------
// disk_ioctl - Disk control
// ---------------------------------------------------------------------------

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0) {
        return RES_PARERR;
    }

    if (s_disk_status & STA_NOINIT) {
        return RES_NOTRDY;
    }

    switch (cmd) {
    case CTRL_SYNC:
        // Flush write buffer
        if (spi_sd_sync() != 0) {
            return RES_ERROR;
        }
        return RES_OK;

    case GET_SECTOR_COUNT:
        // Get number of sectors
        *(LBA_t *)buff = spi_sd_get_sector_count();
        return RES_OK;

    case GET_SECTOR_SIZE:
        // Sector size is always 512 for SD cards
        *(WORD *)buff = 512;
        return RES_OK;

    case GET_BLOCK_SIZE:
        // Erase block size in sectors (typical for SD)
        *(DWORD *)buff = 128; // 64KB erase block
        return RES_OK;

    case CTRL_TRIM:
        // Not supported
        return RES_OK;

    default:
        return RES_PARERR;
    }
}

// ---------------------------------------------------------------------------
// get_fattime - Get current time for timestamps
// ---------------------------------------------------------------------------

DWORD get_fattime(void) {
    // Return fixed timestamp (FF_FS_NORTC=1 in ffconf.h)
    // Format: YYYYYYY.MMMM.DDDDD.HHHHH.MMMMMM.SSSSS
    // Using values from ffconf.h: 2025-01-01 00:00:00
    return ((DWORD)(FF_NORTC_YEAR - 1980) << 25) | ((DWORD)FF_NORTC_MON << 21) |
           ((DWORD)FF_NORTC_MDAY << 16) | ((DWORD)0 << 11) // Hour
           | ((DWORD)0 << 5)                               // Minute
           | ((DWORD)0 >> 1);                              // Second/2
}

#else // !HIVE_ENABLE_SD

// Stub implementations when SD is disabled
DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;
    return STA_NOINIT;
}
DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return STA_NOINIT;
}
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    (void)buff;
    (void)sector;
    (void)count;
    return RES_NOTRDY;
}
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    (void)pdrv;
    (void)buff;
    (void)sector;
    (void)count;
    return RES_NOTRDY;
}
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    (void)pdrv;
    (void)cmd;
    (void)buff;
    return RES_NOTRDY;
}
DWORD get_fattime(void) {
    return 0;
}

#endif // HIVE_ENABLE_SD
