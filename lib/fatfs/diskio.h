// FatFS Disk I/O Interface
//
// Hardware abstraction for SD card access via SPI.
// See http://elm-chan.org/fsw/ff/doc/dioctl.html

#ifndef DISKIO_H
#define DISKIO_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Status Flags
// ---------------------------------------------------------------------------

typedef uint8_t DSTATUS;

#define STA_NOINIT 0x01  // Drive not initialized
#define STA_NODISK 0x02  // No medium in the drive
#define STA_PROTECT 0x04 // Write protected

// ---------------------------------------------------------------------------
// Result Codes
// ---------------------------------------------------------------------------

typedef enum {
    RES_OK = 0, // Successful
    RES_ERROR,  // R/W error
    RES_WRPRT,  // Write protected
    RES_NOTRDY, // Not ready
    RES_PARERR  // Invalid parameter
} DRESULT;

// ---------------------------------------------------------------------------
// FatFS Types (must match ff.h)
// ---------------------------------------------------------------------------

typedef uint8_t BYTE;
typedef uint16_t WORD;
typedef uint32_t DWORD;
typedef unsigned int UINT;
typedef uint32_t LBA_t; // LBA type (32-bit for FF_LBA64=0)

// ---------------------------------------------------------------------------
// Disk I/O Functions (called by FatFS)
// ---------------------------------------------------------------------------

// Initialize disk drive
DSTATUS disk_initialize(BYTE pdrv);

// Get disk status
DSTATUS disk_status(BYTE pdrv);

// Read sectors
DRESULT disk_read(BYTE pdrv,    // Physical drive number
                  BYTE *buff,   // Data buffer
                  LBA_t sector, // Start sector (LBA)
                  UINT count    // Number of sectors to read
);

// Write sectors
DRESULT disk_write(BYTE pdrv,        // Physical drive number
                   const BYTE *buff, // Data buffer
                   LBA_t sector,     // Start sector (LBA)
                   UINT count        // Number of sectors to write
);

// Disk I/O control
DRESULT disk_ioctl(BYTE pdrv, // Physical drive number
                   BYTE cmd,  // Control command
                   void *buff // Buffer to send/receive data
);

// ---------------------------------------------------------------------------
// Control Commands
// ---------------------------------------------------------------------------

#define CTRL_SYNC 0        // Flush disk write buffer
#define GET_SECTOR_COUNT 1 // Get media size (LBA_t)
#define GET_SECTOR_SIZE 2  // Get sector size (WORD)
#define GET_BLOCK_SIZE 3   // Get erase block size (DWORD)
#define CTRL_TRIM 4        // Notify device of unused sectors

// ---------------------------------------------------------------------------
// Time Function (optional)
// ---------------------------------------------------------------------------

// Get current time for file timestamps
// Returns: bit[31:25]=Year(0-127), bit[24:21]=Month(1-12), bit[20:16]=Day(1-31)
//          bit[15:11]=Hour(0-23), bit[10:5]=Minute(0-59), bit[4:0]=Second/2(0-29)
DWORD get_fattime(void);

#endif // DISKIO_H
