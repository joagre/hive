# SD Card Support Design

Adding SD card support for STM32 alongside existing flash storage via a unified mount table.

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

**Current limitations:**
- STM32: Paths hardcoded (`/log`, `/config`) via `HIVE_VFILE_*` defines
- No unified way to add new backends (SD card, EEPROM, etc.)
- Linux and STM32 use completely different path handling

---

## Proposed Design: Mount Table

A mount table maps path prefixes to storage backends. Same `hive_file_*` API, paths work across platforms.

### Core Types

```c
typedef enum {
    HIVE_BACKEND_POSIX,   // Linux: direct POSIX I/O
    HIVE_BACKEND_FLASH,   // STM32: internal flash sector
    HIVE_BACKEND_SD,      // STM32: SD card via FatFS
} hive_file_backend_t;

typedef struct {
    uint32_t base;        // Flash base address
    uint32_t size;        // Flash region size
    uint8_t sector;       // Flash sector number
} hive_flash_config_t;

typedef struct {
    SPI_TypeDef *spi;     // SPI peripheral (SPI2, SPI3)
    uint16_t cs_pin;      // Chip select GPIO pin
} hive_sd_config_t;

typedef struct {
    const char *root;     // Filesystem root (e.g., "/tmp/hive")
} hive_posix_config_t;

typedef struct {
    const char *prefix;               // Path prefix to match
    hive_file_backend_t backend;      // Which backend handles this
    union {
        hive_posix_config_t posix;    // Linux config
        hive_flash_config_t flash;    // STM32 flash config
        hive_sd_config_t sd;          // SD card config
    };
} hive_mount_t;
```

### Platform Configurations

**Linux** - Single catch-all mount, POSIX passthrough:

```c
// src/hal/linux/hive_mounts.c
static const hive_mount_t g_mounts[] = {
    {
        .prefix = "/",
        .backend = HIVE_BACKEND_POSIX,
        .posix = { .root = "/tmp/hive" }
    },
};
#define HIVE_MOUNT_COUNT 1
```

Usage:
- `hive_file_open("/log", ...)` -> opens `/tmp/hive/log`
- `hive_file_open("/mydata.bin", ...)` -> opens `/tmp/hive/mydata.bin`

**STM32 Crazyflie** - Flash + optional SD card:

```c
// examples/pilot/hal/crazyflie-2.1+/hive_mounts.c

// Flash addresses still come from Makefile -D flags
// -DHIVE_VFILE_LOG_BASE=0x080E0000 -DHIVE_VFILE_LOG_SIZE=131072 ...

static const hive_mount_t g_mounts[] = {
    {
        .prefix = "/log",
        .backend = HIVE_BACKEND_FLASH,
        .flash = {
            .base = HIVE_VFILE_LOG_BASE,
            .size = HIVE_VFILE_LOG_SIZE,
            .sector = HIVE_VFILE_LOG_SECTOR
        }
    },
    {
        .prefix = "/config",
        .backend = HIVE_BACKEND_FLASH,
        .flash = {
            .base = HIVE_VFILE_CONFIG_BASE,
            .size = HIVE_VFILE_CONFIG_SIZE,
            .sector = HIVE_VFILE_CONFIG_SECTOR
        }
    },
    {
        .prefix = "/sd",
        .backend = HIVE_BACKEND_SD,
        .sd = {
            .spi = SPI2,
            .cs_pin = GPIO_PIN_4   // IO4 default on SD deck
        }
    },
};
#define HIVE_MOUNT_COUNT 3
```

Usage:
- `hive_file_open("/log", ...)` -> internal flash
- `hive_file_open("/config", ...)` -> internal flash
- `hive_file_open("/sd/flight.bin", ...)` -> SD card

### Path Resolution

```c
// Find mount for path (longest prefix match)
static const hive_mount_t *find_mount(const char *path) {
    const hive_mount_t *best = NULL;
    size_t best_len = 0;

    for (size_t i = 0; i < HIVE_MOUNT_COUNT; i++) {
        size_t len = strlen(g_mounts[i].prefix);
        if (strncmp(path, g_mounts[i].prefix, len) == 0) {
            if (len > best_len) {
                best = &g_mounts[i];
                best_len = len;
            }
        }
    }
    return best;
}

// In hive_hal_file_open:
hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *fd_out) {
    const hive_mount_t *mount = find_mount(path);
    if (!mount) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    const char *subpath = path + strlen(mount->prefix);

    switch (mount->backend) {
    case HIVE_BACKEND_POSIX:
        return posix_file_open(mount, subpath, flags, mode, fd_out);
    case HIVE_BACKEND_FLASH:
        return flash_file_open(mount, subpath, flags, mode, fd_out);
    case HIVE_BACKEND_SD:
        return sd_file_open(mount, subpath, flags, mode, fd_out);
    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "unknown backend");
    }
}
```

### File Descriptor Encoding

Encode backend type in fd for dispatch:

```c
// fd layout: [backend:4][index:12]
#define FD_BACKEND_SHIFT  12
#define FD_INDEX_MASK     0x0FFF

#define FD_MAKE(backend, index)  (((backend) << FD_BACKEND_SHIFT) | (index))
#define FD_BACKEND(fd)           ((fd) >> FD_BACKEND_SHIFT)
#define FD_INDEX(fd)             ((fd) & FD_INDEX_MASK)

// Example:
// fd = 0x0002 -> POSIX backend, index 2
// fd = 0x1003 -> Flash backend, index 3
// fd = 0x2000 -> SD backend, index 0
```

---

## Portable Code

Same code works on Linux and STM32:

```c
void telemetry_logger_actor(void *args, ...) {
    int fd;

    // Try SD card first (high capacity)
    hive_status_t status = hive_file_open("/sd/flight.bin",
                                          HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);

    if (HIVE_FAILED(status)) {
        // Fall back to flash (limited capacity)
        status = hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);
    }

    // Same write code regardless of backend
    while (running) {
        hive_file_write(fd, data, len, &written);
        hive_file_sync(fd);  // Flush to storage
    }

    hive_file_close(fd);
}
```

On Linux: `/sd/flight.bin` -> `/tmp/hive/sd/flight.bin`
On STM32: `/sd/flight.bin` -> SD card via FatFS

---

## Configuration Summary

| Item | Where Defined | Notes |
|------|---------------|-------|
| Flash addresses | Makefile `-D` flags | Board-specific, compile-time |
| Flash sectors | Makefile `-D` flags | Board-specific, compile-time |
| SD SPI pins | Mount table | Board-specific, compile-time |
| Mount prefixes | Mount table | Platform-specific |
| POSIX root | Mount table | Linux only |

Flash defines (unchanged from current):
```makefile
# Board Makefile
CFLAGS += -DHIVE_VFILE_LOG_BASE=0x080E0000
CFLAGS += -DHIVE_VFILE_LOG_SIZE=131072
CFLAGS += -DHIVE_VFILE_LOG_SECTOR=11
```

---

## Backend Implementations

### POSIX Backend (Linux)

Thin wrapper around standard POSIX calls:

```c
static hive_status_t posix_file_open(const hive_mount_t *mount,
                                     const char *subpath, int flags,
                                     int mode, int *fd_out) {
    char fullpath[256];
    snprintf(fullpath, sizeof(fullpath), "%s%s", mount->posix.root, subpath);

    int posix_flags = hive_flags_to_posix(flags);
    int fd = open(fullpath, posix_flags, mode);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, strerror(errno));
    }

    *fd_out = FD_MAKE(HIVE_BACKEND_POSIX, fd);
    return HIVE_SUCCESS;
}
```

### Flash Backend (STM32)

Existing implementation, refactored to use mount config:

```c
static hive_status_t flash_file_open(const hive_mount_t *mount,
                                     const char *subpath, int flags,
                                     int mode, int *fd_out) {
    // subpath is empty for flash (whole mount = one file)
    // Use mount->flash.base, mount->flash.size, mount->flash.sector
    // ... existing flash_file_open logic ...
}
```

### SD Backend (STM32)

FatFS wrapper:

```c
static FIL s_sd_files[HIVE_MAX_SD_FILES];
static bool s_sd_initialized = false;

static hive_status_t sd_file_open(const hive_mount_t *mount,
                                  const char *subpath, int flags,
                                  int mode, int *fd_out) {
    if (!s_sd_initialized) {
        // Initialize SPI and mount FatFS
        spi_sd_init(mount->sd.spi, mount->sd.cs_pin);
        FRESULT res = f_mount(&s_fatfs, "", 1);
        if (res != FR_OK) {
            return HIVE_ERROR(HIVE_ERR_IO, "SD mount failed");
        }
        s_sd_initialized = true;
    }

    // Find free slot
    int index = find_free_sd_slot();
    if (index < 0) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "no free SD file slots");
    }

    // Convert flags and open via FatFS
    BYTE fatfs_mode = hive_flags_to_fatfs(flags);
    FRESULT res = f_open(&s_sd_files[index], subpath, fatfs_mode);
    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_open failed");
    }

    *fd_out = FD_MAKE(HIVE_BACKEND_SD, index);
    return HIVE_SUCCESS;
}
```

---

## File Structure

```
include/
    hive_file.h              # Public API (unchanged)
    hal/
        hive_hal_file.h      # HAL interface (unchanged)
        hive_mount.h         # NEW: mount table types

src/
    hive_file.c              # Unified wrapper (unchanged)
    hal/
        linux/
            hive_hal_file_linux.c   # Refactored: use mount table
            hive_mounts.c           # NEW: Linux mount config
        stm32/
            hive_hal_file_stm32.c   # Refactored: multi-backend dispatch
            hive_file_flash.c       # Extracted: flash backend
            hive_file_sd.c          # NEW: SD backend
            hive_mounts.c           # NEW: default STM32 mounts

examples/pilot/hal/crazyflie-2.1+/
    hive_mounts.c            # Crazyflie-specific mount config

lib/fatfs/                   # Third-party FatFS library
    ff.c, ff.h, ffconf.h, diskio.h
```

---

## Differences: Flash vs SD Card

| Feature | Internal Flash | SD Card |
|---------|----------------|---------|
| Capacity | 128KB sector | 2-32 GB |
| Write speed | ~1ms per 256B block | Variable (buffered) |
| Erase required | Yes (sector erase) | No (FatFS handles) |
| Random write | No (ring buffer) | Yes |
| `hive_file_read()` | Not supported | Supported |
| `hive_file_pwrite()` | Not supported | Supported |
| File creation | One file per mount | Any path under mount |
| FAT32 | No | Yes |

---

## Memory Impact

| Component | Flash | RAM |
|-----------|-------|-----|
| Mount table dispatch | ~0.5 KB | ~0.1 KB |
| FatFS library | ~10 KB | ~1 KB |
| SPI driver | ~1 KB | ~0.1 KB |
| SD file slots (4) | ~0.1 KB | ~2 KB |
| **Total (with SD)** | ~12 KB | ~3.2 KB |

Without SD card enabled: negligible overhead (~0.5 KB flash).

---

## Implementation Steps

1. **Add mount table types** - `include/hal/hive_mount.h`
2. **Refactor Linux HAL** - Use mount table, POSIX backend
3. **Refactor STM32 HAL** - Multi-backend dispatch
4. **Extract flash backend** - Move existing flash code to `hive_file_flash.c`
5. **Add FatFS library** - Copy to `lib/fatfs/`
6. **Implement SD backend** - SPI driver + FatFS wrapper
7. **Add Crazyflie mounts** - Board-specific mount config
8. **Test** - Verify all backends work
9. **Update pilot** - Use SD card when available

---

## Open Questions

1. **Runtime mount?** - Allow mounting/unmounting at runtime? (Probably no - keep it simple)
2. **Hot-plug SD?** - Detect card insertion/removal? (Nice to have, not critical)
3. **File naming?** - Auto-increment flight numbers? Timestamps? (Application decides)
4. **Fallback policy?** - Configurable per-application

---

## References

- [Micro SD Card Deck](https://www.bitcraze.io/products/micro-sd-card-deck/)
- [FatFS](http://elm-chan.org/fsw/ff/00index_e.html)
- [SD Card SPI Protocol](http://elm-chan.org/docs/mmc/mmc_e.html)
