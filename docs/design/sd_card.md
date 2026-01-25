# SD Card Support Design

Investigation into adding SD card support for STM32 alongside existing flash storage.

## Current Architecture

The file I/O uses a layered design:

```
hive_file_open("/log", ...)
        |
        v
+------------------+
|   hive_file.c    |  Unified wrapper (validation, init guards)
+------------------+
        |
        v
+------------------+
| hive_hal_file.h  |  HAL interface (8 functions)
+------------------+
        |
        +------------+------------+
        |                         |
        v                         v
+------------------+     +------------------+
| Linux HAL        |     | STM32 HAL        |
| (POSIX file I/O) |     | (flash virtual   |
|                  |     |  files)          |
+------------------+     +------------------+
```

The STM32 HAL maps virtual paths to flash sectors:
- `/log` -> Flash sector for logging
- `/config` -> Flash sector for configuration

## Proposed Design: Path-Based Routing

Route file operations based on path prefix:

| Path prefix | Backend | Notes |
|-------------|---------|-------|
| `/log`, `/config` | Internal flash | Existing behavior |
| `/sd/...` | SD card (FatFS) | New capability |

Example usage:
```c
// Existing flash-backed files (unchanged)
hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);

// New SD card files
hive_file_open("/sd/flight_001.bin", HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);
hive_file_open("/sd/telemetry.csv", HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);
```

## API Changes

**No changes to public API.** The `hive_file_*` functions remain identical.

Internal HAL changes:
1. `hive_hal_file_open()` detects `/sd/` prefix and routes to SD backend
2. File descriptors encode backend type (e.g., fd 0-15 = flash, 16+ = SD)
3. All other HAL functions dispatch based on fd

## Implementation Components

### 1. SPI Driver

The Micro SD Card Deck uses SPI:
- Default CS: IO4 (configurable via solder bridges)
- Clock: Up to 25 MHz for SD card
- STM32F405 SPI2 or SPI3

```c
// New file: hal/crazyflie-2.1+/spi_sd.c
void spi_sd_init(void);
uint8_t spi_sd_transfer(uint8_t tx);
void spi_sd_cs_low(void);
void spi_sd_cs_high(void);
```

### 2. FatFS Integration

Use ChaN's FatFS library (standard for embedded):
- FAT32 support (required by SD Card Deck)
- Minimal footprint (~10KB flash)
- Already battle-tested

```c
// FatFS disk I/O glue: hal/crazyflie-2.1+/diskio.c
DSTATUS disk_initialize(BYTE pdrv);
DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count);
DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count);
```

### 3. HAL Modifications

```c
// In hive_hal_file_stm32.c

// Backend type encoded in fd
#define FD_FLASH_BASE  0
#define FD_FLASH_MAX   15
#define FD_SD_BASE     16
#define FD_SD_MAX      23

static bool is_sd_path(const char *path) {
    return strncmp(path, "/sd/", 4) == 0;
}

hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *fd_out) {
    if (is_sd_path(path)) {
        return sd_file_open(path + 3, flags, mode, fd_out);  // Skip "/sd"
    } else {
        return flash_file_open(path, flags, mode, fd_out);   // Existing code
    }
}
```

### 4. Deck Detection

Crazyflie decks have 1-wire memory for auto-detection:
- Check for SD deck presence at init
- Fail gracefully if deck not present

```c
bool sd_deck_present(void);  // Check 1-wire memory
```

## File Structure

```
src/hal/crazyflie-2.1+/
    hive_hal_file_stm32.c    # Modified: add path routing
    spi_sd.c                 # New: SPI driver for SD card
    spi_sd.h
    diskio.c                 # New: FatFS disk I/O glue

lib/fatfs/                   # Third-party FatFS library
    ff.c
    ff.h
    ffconf.h                 # FatFS configuration
    diskio.h
```

## Differences: Flash vs SD Card

| Feature | Internal Flash | SD Card |
|---------|----------------|---------|
| Capacity | 128KB sector | 2-32 GB |
| Write speed | ~1ms per 256B block | Variable (buffered) |
| Erase required | Yes (sector erase) | No (FatFS handles) |
| Random write | No (ring buffer) | Yes |
| `hive_file_read()` | Not supported | Supported |
| `hive_file_pwrite()` | Not supported | Supported |
| File creation | Fixed paths only | Any path under `/sd/` |
| FAT32 | No | Yes |

## SD Card Advantages for Telemetry

1. **Capacity** - Log entire flights (hours of data vs seconds)
2. **No wear leveling** - SD card handles internally
3. **Easy extraction** - Remove card, read on PC
4. **Multiple files** - Separate files per flight
5. **Standard format** - FAT32, readable anywhere

## Implementation Steps

1. **Add FatFS library** - Copy ChaN's FatFS to `lib/fatfs/`
2. **Implement SPI driver** - Low-level SPI for SD card
3. **Implement diskio.c** - FatFS disk I/O glue
4. **Modify HAL** - Add path routing in `hive_hal_file_stm32.c`
5. **Add deck detection** - Check for SD deck at init
6. **Test** - Verify read/write/sync operations
7. **Update telemetry_logger_actor.c** - Use `/sd/` path when deck present

## Memory Impact

| Component | Flash | RAM |
|-----------|-------|-----|
| FatFS library | ~10 KB | ~1 KB |
| SPI driver | ~1 KB | ~0.1 KB |
| Disk I/O glue | ~0.5 KB | ~0.5 KB |
| **Total** | ~12 KB | ~1.6 KB |

## Open Questions

1. **Hot-plug support?** - Detect SD card insertion/removal at runtime?
2. **File naming convention?** - Auto-increment flight numbers? Timestamps?
3. **Sync frequency?** - How often to flush to SD? (Currently: on hive_file_sync)
4. **Fallback behavior?** - If SD card missing, fall back to flash? Or error?

## References

- [Micro SD Card Deck](https://www.bitcraze.io/products/micro-sd-card-deck/)
- [FatFS](http://elm-chan.org/fsw/ff/00index_e.html)
- [SD Card SPI Protocol](http://elm-chan.org/docs/mmc/mmc_e.html)
