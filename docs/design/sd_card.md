# SD Card Support Design

Adding SD card support for STM32 alongside existing flash storage via a unified mount table.

## Current Architecture

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

## Design Principles

1. **No API changes** - Same `hive_file_*` functions
2. **Portable paths** - `/log` works on both Linux and STM32
3. **Platform-specific config stays platform-specific** - No STM32 types in common headers
4. **Compile-time configuration** - No runtime mount/unmount
5. **Graceful degradation** - SD card missing returns error, doesn't crash
6. **Follows project conventions** - `_t` suffix, 1TBS, no unicode

---

## Proposed Design: Mount Table

### Architecture Overview

```
                    hive_file_open("/sd/flight.bin", ...)
                                    |
                                    v
                    +-------------------------------+
                    |   hive_hal_file_open()        |
                    |   - find_mount() by prefix    |
                    |   - dispatch to backend       |
                    +-------------------------------+
                           |         |         |
              +------------+---------+---------+------------+
              |                      |                      |
              v                      v                      v
    +------------------+   +------------------+   +------------------+
    | POSIX backend    |   | Flash backend    |   | SD backend       |
    | (Linux only)     |   | (STM32)          |   | (STM32 + FatFS)  |
    +------------------+   +------------------+   +------------------+
```

### Mount Table Types (Platform-Independent)

```c
// include/hal/hive_mount.h

typedef enum {
    HIVE_BACKEND_POSIX,   // Linux: direct POSIX I/O
    HIVE_BACKEND_FLASH,   // STM32: internal flash sector
    HIVE_BACKEND_SD,      // STM32: SD card via FatFS
} hive_file_backend_t;

// Forward declaration - actual struct is platform-specific
typedef struct hive_mount hive_mount_t;

// Mount table access (implemented per-platform)
const hive_mount_t *hive_mount_find(const char *path, size_t *prefix_len);
size_t hive_mount_count(void);
```

### Platform-Specific Mount Definitions

**Linux:**

```c
// src/hal/linux/hive_mounts.c

typedef struct hive_mount {
    const char *prefix;
    hive_file_backend_t backend;
    const char *root;   // Filesystem root (empty = direct passthrough)
} hive_mount_t;

static const hive_mount_t g_mounts[] = {
    { .prefix = "/", .backend = HIVE_BACKEND_POSIX, .root = "" },
};

// Direct passthrough: hive_file_open("/log") -> opens /log
// To remap for testing, change root to e.g., "/tmp/hive"
```

**STM32:**

```c
// src/hal/stm32/hive_mounts.c (or board-specific override)

typedef struct hive_mount {
    const char *prefix;
    hive_file_backend_t backend;
    union {
        struct {
            uint32_t base;
            uint32_t size;
            uint8_t sector;
        } flash;
#if HIVE_ENABLE_SD
        struct {
            uint8_t spi_id;      // 1=SPI1, 2=SPI2, 3=SPI3
            uint8_t cs_port;     // 0=GPIOA, 1=GPIOB, etc.
            uint8_t cs_pin;      // 0-15
        } sd;
#endif
    };
} hive_mount_t;

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
#if HIVE_ENABLE_SD
    {
        .prefix = "/sd",
        .backend = HIVE_BACKEND_SD,
        .sd = { .spi_id = 2, .cs_port = 1, .cs_pin = 4 }  // SPI2, PB4
    },
#endif
};
```

### Path Matching (Fixed)

Prefix matching must handle boundaries correctly:

```c
// Match rules:
// - "/" matches everything (catch-all, must be last)
// - "/log" matches "/log" exactly, NOT "/logger"
// - "/sd" matches "/sd", "/sd/", "/sd/file.bin"

static bool prefix_matches(const char *path, const char *prefix, size_t *match_len) {
    size_t plen = strlen(prefix);

    // Special case: root matches everything
    if (plen == 1 && prefix[0] == '/') {
        *match_len = 1;
        return true;
    }

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

const hive_mount_t *hive_mount_find(const char *path, size_t *prefix_len) {
    const hive_mount_t *best = NULL;
    size_t best_len = 0;

    for (size_t i = 0; i < ARRAY_SIZE(g_mounts); i++) {
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
```

### File Descriptor Strategy

**Problem:** Can't mangle POSIX fds on Linux - the OS owns those numbers.

**Solution:** Different strategies per platform:

- **Linux:** Use POSIX fds directly (no encoding). Backend is always POSIX.
- **STM32:** Encode backend type in fd for dispatch.

STM32 fd encoding (defined in `hive_mount.h`):

```c
// fd layout: [backend:4][index:12]
// - Flash: fd = 0x1000 + index
// - SD:    fd = 0x2000 + index

#define FD_MAKE(backend, index)  (((int)(backend) << 12) | (index))
#define FD_BACKEND(fd)           ((hive_file_backend_t)((fd) >> 12))
#define FD_INDEX(fd)             ((fd) & 0xFFF)
```

---

## SD Card Availability

Applications need to know if SD card is available:

```c
// New API function
hive_status_t hive_file_mount_available(const char *path);

// Returns:
// - HIVE_OK: Mount exists and backend is ready
// - HIVE_ERR_INVALID: No mount for path
// - HIVE_ERR_IO: Mount exists but backend unavailable (e.g., no SD card)

// Usage:
if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
    hive_file_open("/sd/flight.bin", ...);
} else {
    hive_file_open("/log", ...);  // Fallback to flash
}
```

For SD card, this checks:
1. SPI peripheral initialized
2. Card inserted and responds to commands
3. FatFS mounted successfully

---

## Conditional Compilation

SD support is optional:

```c
// hive_static_config.h or Makefile
#define HIVE_ENABLE_SD 1       // Enable SD card support
#define HIVE_MAX_SD_FILES 4    // Max concurrent open files on SD
```

```makefile
# Board Makefile
CFLAGS += -DHIVE_ENABLE_SD=1
CFLAGS += -DHIVE_MAX_SD_FILES=4
```

When `HIVE_ENABLE_SD=0`:
- SD mount entries skipped
- SD backend code not compiled
- No FatFS dependency
- ~12KB flash saved

---

## Memory Impact (Revised)

| Component | Flash | RAM |
|-----------|-------|-----|
| Mount table dispatch | ~0.5 KB | ~0.1 KB |
| FatFS library | ~10 KB | ~0.5 KB (static) |
| FatFS FIL structs (4x) | - | ~2.4 KB |
| SPI driver | ~1 KB | ~0.1 KB |
| **Total (with SD)** | ~12 KB | ~3.1 KB |

Note: FatFS FIL struct is ~600 bytes each. With 4 file slots, that's 2.4KB RAM.

Without SD: ~0.5 KB flash overhead for mount table.

---

## Configuration Summary

| Item | Where Defined | Notes |
|------|---------------|-------|
| Flash addresses | Makefile `-D` flags | Board-specific, unchanged |
| Flash sectors | Makefile `-D` flags | Board-specific, unchanged |
| SD enable | `HIVE_ENABLE_SD` | Compile-time flag |
| SD SPI/GPIO | Mount table | Board-specific |
| Mount prefixes | Mount table | Platform-specific |
| POSIX root | Mount table | Linux only |

---

## Backend Implementations

### POSIX Backend (Linux)

```c
// src/hal/linux/hive_hal_file.c

hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *out) {
    // Find mount for path
    size_t prefix_len;
    const hive_mount_t *mount = hive_mount_find(path, &prefix_len);
    if (!mount) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    // Build full path: root + subpath
    const char *root = hive_mount_get_root(mount);
    char fullpath[512];
    if (root[0] == '\0') {
        // Empty root = direct passthrough
        snprintf(fullpath, sizeof(fullpath), "%s", path);
    } else {
        const char *subpath = path + prefix_len;
        snprintf(fullpath, sizeof(fullpath), "%s%s", root, subpath);
    }

    int posix_flags = hive_flags_to_posix(flags);
    int fd = open(fullpath, posix_flags, mode);
    if (fd < 0) {
        return HIVE_ERROR(HIVE_ERR_IO, "open failed");
    }

    *out = fd;  // Return POSIX fd directly
    return HIVE_SUCCESS;
}
```

### Flash Backend (STM32)

Flash backend uses mount config for sector info. Flash files don't use subpaths:

```c
static hive_status_t flash_open(const hive_mount_t *mount, int flags,
                                int *out) {
    // Find free slot, validate flags (HIVE_O_TRUNC required for writes)
    // Use hive_mount_flash_base/size/sector() accessors
    // Erase sector if HIVE_O_TRUNC + HIVE_O_WRONLY
    // ... existing flash logic ...

    *out = FD_MAKE(HIVE_BACKEND_FLASH, slot);
    return HIVE_SUCCESS;
}
```

### SD Backend (STM32)

```c
#if HIVE_ENABLE_SD

static FATFS s_fatfs;
static FIL s_sd_files[HIVE_MAX_SD_FILES];
static bool s_sd_file_used[HIVE_MAX_SD_FILES];
static bool s_sd_initialized = false;

// Initialize SD card on first access
static hive_status_t sd_ensure_init(const hive_mount_t *mount) {
    if (s_sd_initialized) {
        return HIVE_SUCCESS;
    }

    // Configure SPI driver with mount settings
    spi_sd_config_t cfg = {
        .spi_id = hive_mount_sd_spi(mount),
        .cs_port = hive_mount_sd_cs_port(mount),
        .cs_pin = hive_mount_sd_cs_pin(mount),
    };
    spi_sd_configure(&cfg);

    // Check card presence
    if (!spi_sd_is_present()) {
        return HIVE_ERROR(HIVE_ERR_IO, "SD card not present");
    }

    // Mount FatFS
    FRESULT res = f_mount(&s_fatfs, "", 1);
    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "SD card mount failed");
    }

    s_sd_initialized = true;
    return HIVE_SUCCESS;
}

static hive_status_t sd_open(const hive_mount_t *mount, const char *subpath,
                             int flags, int *out) {
    hive_status_t status = sd_ensure_init(mount);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Find free slot
    int slot = -1;
    for (int i = 0; i < HIVE_MAX_SD_FILES; i++) {
        if (!s_sd_file_used[i]) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "no free SD file slots");
    }

    // Skip leading slash in subpath
    if (subpath && subpath[0] == '/') {
        subpath++;
    }

    BYTE mode = hive_flags_to_fatfs(flags);
    FRESULT res = f_open(&s_sd_files[slot], subpath, mode);
    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_open failed");
    }

    s_sd_file_used[slot] = true;
    *out = FD_MAKE(HIVE_BACKEND_SD, slot);
    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_SD
```

### STM32 HAL Dispatch

```c
// src/hal/stm32/hive_hal_file.c

hive_status_t hive_hal_file_open(const char *path, int flags, int mode,
                                 int *out) {
    (void)mode;

    size_t prefix_len;
    const hive_mount_t *mount = hive_mount_find(path, &prefix_len);
    if (!mount) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "no mount for path");
    }

    switch (hive_mount_get_backend(mount)) {
    case HIVE_BACKEND_FLASH:
        return flash_open(mount, flags, out);
#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD: {
        const char *subpath = path + prefix_len;
        return sd_open(mount, subpath, flags, out);
    }
#endif
    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "unsupported backend");
    }
}

hive_status_t hive_hal_file_write(int fd, const void *buf, size_t len,
                                  size_t *bytes_written) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_write(index, buf, len, bytes_written);
#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_write(index, buf, len, bytes_written);
#endif
    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }
}

// Similar dispatch for read, pread, pwrite, sync, close
```

---

## File Structure

```
include/
    hive_file.h              # Public API (unchanged)
    hal/
        hive_hal_file.h      # HAL interface (unchanged)
        hive_mount.h         # NEW: backend enum, function declarations

src/
    hive_file.c              # Unified wrapper (unchanged)
    hal/
        linux/
            hive_hal_file.c         # File I/O HAL
            hive_mounts.c           # Linux mount config
            hive_mount.h            # Linux mount structure
        stm32/
            hive_hal_file.c         # Multi-backend dispatch + flash/SD backends
            hive_mounts.c           # Default STM32 mounts
            hive_mount.h            # STM32 mount structure + FD encoding
            spi_sd.c                # SPI SD protocol driver
            spi_sd.h                # SPI SD driver interface
            spi_ll.h                # Low-level SPI interface (board implements)

examples/pilot/hal/crazyflie-2.1+/
    hive_mounts.c            # Board-specific mount table
    spi_ll_sd.c              # Board-specific SPI implementation

lib/fatfs/                   # Third-party (conditional)
    ff.c, ff.h, ffconf.h, diskio.c, diskio.h
```

---

## Usage Examples

### Basic Usage

```c
int fd;
hive_status_t status;

// Try SD card first
status = hive_file_open("/sd/flight_001.bin",
                        HIVE_O_WRONLY | HIVE_O_CREAT, 0, &fd);

if (HIVE_FAILED(status)) {
    // Fall back to flash
    status = hive_file_open("/log", HIVE_O_WRONLY | HIVE_O_TRUNC, 0, &fd);
}

// Write works the same regardless of backend
hive_file_write(fd, data, len, &written);
hive_file_sync(fd);
hive_file_close(fd);
```

### Check Availability First

```c
const char *log_path;

if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
    log_path = "/sd/telemetry.bin";
} else {
    log_path = "/log";
}

hive_file_open(log_path, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0, &fd);
```

---

## Implementation Steps

1. **Add mount table header** - `include/hal/hive_mount.h` (enum + declarations)
2. **Implement Linux mounts** - Simple POSIX passthrough with root prefix
3. **Refactor STM32 HAL** - Extract flash backend, add dispatch layer
4. **Add mount availability API** - `hive_file_mount_available()`
5. **Add FatFS library** - Copy to `lib/fatfs/`, conditional compile
6. **Implement SD backend** - SPI driver + FatFS wrapper
7. **Add board-specific mounts** - Override mount table with board-specific SPI/GPIO config
8. **Test** - Linux passthrough, STM32 flash, STM32 SD

---

## Design Decisions

1. **No automatic directory creation** - `hive_file_open()` with `HIVE_O_CREAT` does not create parent directories. If the directory doesn't exist, return `HIVE_ERR_IO`. Use flat file structures like `/sd/flight_001.bin` for logging. Add `hive_file_mkdir()` later if needed.

2. **No file listing API** - Not needed for telemetry logging. Add `hive_file_readdir()` later if ground station file browsing is required.

3. **No hot-plug detection** - Check SD card presence at init only. If card is removed mid-flight, writes fail with `HIVE_ERR_IO`. Hot-plug adds complexity and creates dangerous edge cases.

---

## References

- [Micro SD Card Deck](https://www.bitcraze.io/products/micro-sd-card-deck/)
- [FatFS](http://elm-chan.org/fsw/ff/00index_e.html)
- [SD Card SPI Protocol](http://elm-chan.org/docs/mmc/mmc_e.html)
