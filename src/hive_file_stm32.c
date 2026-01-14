// STM32 Flash File I/O Implementation
//
// Provides the same API and semantics as hive_file_linux.c for STM32 flash storage.
// Uses a ring buffer for efficiency - most writes complete immediately. When the
// buffer fills up, write() blocks to flush data to flash before continuing.
// This ensures no data loss while maintaining good performance for typical usage.
//
// Key characteristics:
// - Virtual file paths mapped to flash sectors (/log, /config)
// - Ring buffer for efficient writes (most are O(1))
// - Blocks to flush when buffer is full (ensures no data loss)
// - Staged writes with flash programming from RAM
// - Erase on TRUNC flag (required before writing)
//
// Board-specific flash configuration via -D flags:
// - HIVE_VFILE_LOG_BASE, HIVE_VFILE_LOG_SIZE, HIVE_VFILE_LOG_SECTOR
// - HIVE_VFILE_CONFIG_BASE, HIVE_VFILE_CONFIG_SIZE, HIVE_VFILE_CONFIG_SECTOR

#include "hive_file.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// STM32 Flash register definitions
// These are standard for STM32F4xx - included via CMSIS headers in real builds
#ifndef FLASH_BASE
#define FLASH_BASE            0x40023C00UL
#endif

typedef struct {
    volatile uint32_t ACR;      // Access control register
    volatile uint32_t KEYR;     // Key register
    volatile uint32_t OPTKEYR;  // Option key register
    volatile uint32_t SR;       // Status register
    volatile uint32_t CR;       // Control register
    volatile uint32_t OPTCR;    // Option control register
} FLASH_TypeDef;

#define FLASH               ((FLASH_TypeDef *) FLASH_BASE)

// Flash key values for unlock sequence
#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

// Flash status register bits
#define FLASH_SR_BSY        (1UL << 16)
#define FLASH_SR_PGSERR     (1UL << 7)
#define FLASH_SR_PGPERR     (1UL << 6)
#define FLASH_SR_PGAERR     (1UL << 5)
#define FLASH_SR_WRPERR     (1UL << 4)
#define FLASH_SR_OPERR      (1UL << 1)
#define FLASH_SR_EOP        (1UL << 0)
#define FLASH_SR_ERRORS     (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | FLASH_SR_OPERR)

// Flash control register bits
#define FLASH_CR_PG         (1UL << 0)   // Programming
#define FLASH_CR_SER        (1UL << 1)   // Sector erase
#define FLASH_CR_MER        (1UL << 2)   // Mass erase
#define FLASH_CR_SNB_Pos    3            // Sector number position
#define FLASH_CR_SNB_Msk    (0x1FUL << FLASH_CR_SNB_Pos)
#define FLASH_CR_PSIZE_Pos  8            // Program size position
#define FLASH_CR_PSIZE_0    (1UL << 8)   // 8-bit
#define FLASH_CR_PSIZE_1    (1UL << 9)   // 32-bit
#define FLASH_CR_STRT       (1UL << 16)  // Start
#define FLASH_CR_LOCK       (1UL << 31)  // Lock

// ----------------------------------------------------------------------------
// Virtual File Table
// ----------------------------------------------------------------------------

typedef struct {
    const char *path;
    uint32_t flash_base;
    uint32_t flash_size;
    uint8_t  sector;
    // Runtime state
    uint32_t write_pos;
    bool     opened;
    bool     erased_ok;
    bool     write_mode;
} vfile_t;

// Build virtual file table from -D flags
static vfile_t g_vfiles[] = {
#ifdef HIVE_VFILE_LOG_BASE
    {
        .path       = "/log",
        .flash_base = HIVE_VFILE_LOG_BASE,
        .flash_size = HIVE_VFILE_LOG_SIZE,
        .sector     = HIVE_VFILE_LOG_SECTOR,
        .write_pos  = 0,
        .opened     = false,
        .erased_ok  = false,
        .write_mode = false,
    },
#endif
#ifdef HIVE_VFILE_CONFIG_BASE
    {
        .path       = "/config",
        .flash_base = HIVE_VFILE_CONFIG_BASE,
        .flash_size = HIVE_VFILE_CONFIG_SIZE,
        .sector     = HIVE_VFILE_CONFIG_SECTOR,
        .write_pos  = 0,
        .opened     = false,
        .erased_ok  = false,
        .write_mode = false,
    },
#endif
};

#define VFILE_COUNT (sizeof(g_vfiles) / sizeof(g_vfiles[0]))

// ----------------------------------------------------------------------------
// Ring Buffer for Deferred Writes
// ----------------------------------------------------------------------------

static uint8_t g_ring_buf[HIVE_FILE_RING_SIZE];
static volatile uint32_t g_ring_head;  // Write position (producers)
static volatile uint32_t g_ring_tail;  // Read position (sync)

// Staging buffer for flash block commits
static uint8_t g_staging[HIVE_FILE_BLOCK_SIZE];
static uint32_t g_staging_len;

// Current file being written (for ring buffer association)
static int g_ring_fd = -1;

// ----------------------------------------------------------------------------
// Ring Buffer Operations
// ----------------------------------------------------------------------------

static inline uint32_t ring_used(void) {
    return (g_ring_head - g_ring_tail) & (HIVE_FILE_RING_SIZE - 1);
}

static inline uint32_t ring_free(void) {
    // Leave one byte unused to distinguish full from empty
    return HIVE_FILE_RING_SIZE - 1 - ring_used();
}

static inline bool ring_empty(void) {
    return g_ring_head == g_ring_tail;
}

static size_t ring_push(const uint8_t *data, size_t len) {
    size_t free = ring_free();
    size_t to_write = (len < free) ? len : free;

    for (size_t i = 0; i < to_write; i++) {
        g_ring_buf[g_ring_head & (HIVE_FILE_RING_SIZE - 1)] = data[i];
        g_ring_head++;
    }

    return to_write;
}

static size_t ring_pop(uint8_t *data, size_t max_len) {
    size_t used = ring_used();
    size_t to_read = (max_len < used) ? max_len : used;

    for (size_t i = 0; i < to_read; i++) {
        data[i] = g_ring_buf[g_ring_tail & (HIVE_FILE_RING_SIZE - 1)];
        g_ring_tail++;
    }

    return to_read;
}

// ----------------------------------------------------------------------------
// Flash Operations
// ----------------------------------------------------------------------------

static void flash_unlock(void) {
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1;
        FLASH->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void) {
    FLASH->CR |= FLASH_CR_LOCK;
}

static void flash_wait_bsy(void) {
    while (FLASH->SR & FLASH_SR_BSY);
}

static void flash_clear_errors(void) {
    FLASH->SR = FLASH_SR_ERRORS;
}

// Erase a flash sector (blocking, takes 1-4 seconds for 128KB sector!)
static bool flash_erase_sector(uint8_t sector) {
    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    // Set sector erase mode and sector number
    FLASH->CR = FLASH_CR_SER | ((uint32_t)sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;

    // Wait for completion (blocking!)
    flash_wait_bsy();

    // Check for errors
    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

// Program words to flash - MUST execute from RAM during write
// This function is placed in .RamFunc section (copied to RAM at startup)
__attribute__((section(".RamFunc"), noinline))
static void flash_program_words_ram(uint32_t addr, const uint32_t *data, uint32_t words) {
    // Enable programming mode with 32-bit parallelism
    FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    for (uint32_t i = 0; i < words; i++) {
        // Write word
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        // Wait for completion
        while (FLASH->SR & FLASH_SR_BSY);
    }

    // Disable programming mode
    FLASH->CR &= ~FLASH_CR_PG;
}

// Write a block to flash (with IRQ masking)
static bool flash_write_block(uint32_t addr, const void *data, uint32_t len) {
    if (len == 0 || (len & 3) != 0) {
        return false;  // Must be word-aligned
    }

    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    // Disable interrupts during flash programming
    // This keeps the critical section short (~1ms for 256 bytes)
    __asm volatile ("cpsid i" ::: "memory");

    flash_program_words_ram(addr, (const uint32_t *)data, len / 4);

    __asm volatile ("cpsie i" ::: "memory");

    // Check for errors
    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

// ----------------------------------------------------------------------------
// Staging Buffer Operations
// ----------------------------------------------------------------------------

static void staging_reset(void) {
    g_staging_len = 0;
    // Fill with 0xFF (erased flash state)
    memset(g_staging, 0xFF, HIVE_FILE_BLOCK_SIZE);
}

static size_t staging_space(void) {
    return HIVE_FILE_BLOCK_SIZE - g_staging_len;
}

static void staging_append(const uint8_t *data, size_t len) {
    if (g_staging_len + len > HIVE_FILE_BLOCK_SIZE) {
        len = HIVE_FILE_BLOCK_SIZE - g_staging_len;
    }
    memcpy(&g_staging[g_staging_len], data, len);
    g_staging_len += len;
}

// Commit staging buffer to flash
static bool staging_commit(vfile_t *vf) {
    if (g_staging_len == 0) {
        return true;  // Nothing to commit
    }

    // Check if we have space in flash
    if (vf->write_pos + HIVE_FILE_BLOCK_SIZE > vf->flash_size) {
        return false;  // Flash region full
    }

    // Write block to flash
    uint32_t addr = vf->flash_base + vf->write_pos;
    bool ok = flash_write_block(addr, g_staging, HIVE_FILE_BLOCK_SIZE);

    if (ok) {
        vf->write_pos += HIVE_FILE_BLOCK_SIZE;
    }

    staging_reset();
    return ok;
}

// Flush ring buffer contents to flash via staging buffer
// Returns false if flash is full or write error occurs
static bool flush_ring_to_flash(vfile_t *vf) {
    uint8_t temp[64];
    while (!ring_empty()) {
        size_t n = ring_pop(temp, sizeof(temp));

        for (size_t i = 0; i < n; i++) {
            if (staging_space() == 0) {
                if (!staging_commit(vf)) {
                    return false;  // Flash full or write error
                }
            }
            staging_append(&temp[i], 1);
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// File I/O Subsystem State
// ----------------------------------------------------------------------------

static struct {
    bool initialized;
} g_file = {0};

// ----------------------------------------------------------------------------
// API Implementation
// ----------------------------------------------------------------------------

hive_status hive_file_init(void) {
    HIVE_INIT_GUARD(g_file.initialized);

    // Reset ring buffer
    g_ring_head = 0;
    g_ring_tail = 0;
    g_ring_fd = -1;

    // Reset staging
    staging_reset();

    // Reset virtual file state
    for (size_t i = 0; i < VFILE_COUNT; i++) {
        g_vfiles[i].write_pos = 0;
        g_vfiles[i].opened = false;
        g_vfiles[i].erased_ok = false;
        g_vfiles[i].write_mode = false;
    }

    g_file.initialized = true;
    return HIVE_SUCCESS;
}

void hive_file_cleanup(void) {
    HIVE_CLEANUP_GUARD(g_file.initialized);

    // Close any open files
    for (size_t i = 0; i < VFILE_COUNT; i++) {
        g_vfiles[i].opened = false;
    }

    g_file.initialized = false;
}

// Find virtual file by path
static vfile_t *find_vfile(const char *path) {
    for (size_t i = 0; i < VFILE_COUNT; i++) {
        if (strcmp(g_vfiles[i].path, path) == 0) {
            return &g_vfiles[i];
        }
    }
    return NULL;
}

// Get virtual file by fd (fd = index into g_vfiles)
static vfile_t *get_vfile(int fd) {
    if (fd < 0 || (size_t)fd >= VFILE_COUNT) {
        return NULL;
    }
    return &g_vfiles[fd];
}

hive_status hive_file_open(const char *path, int flags, int mode, int *fd_out) {
    (void)mode;  // Not used for flash files

    if (!path || !fd_out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL path or fd_out pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = find_vfile(path);
    if (!vf) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "unknown virtual file path");
    }

    if (vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file already open");
    }

    // Determine access mode
    int access = flags & 0x0003;

    // STM32 flag restrictions:
    // - HIVE_O_RDWR not supported (read() doesn't work, only pread())
    // - HIVE_O_WRONLY requires HIVE_O_TRUNC (flash must be erased first)
    // - HIVE_O_CREAT and HIVE_O_APPEND are silently ignored
    if (access == HIVE_O_RDWR) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
            "HIVE_O_RDWR not supported on STM32; use HIVE_O_RDONLY or HIVE_O_WRONLY");
    }

    bool write_mode = (access == HIVE_O_WRONLY);

    if (write_mode && !(flags & HIVE_O_TRUNC)) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
            "HIVE_O_TRUNC required for flash writes (must erase sector first)");
    }

    // Handle TRUNC flag - erase the sector
    if ((flags & HIVE_O_TRUNC) && write_mode) {
        if (!flash_erase_sector(vf->sector)) {
            return HIVE_ERROR(HIVE_ERR_IO, "flash erase failed");
        }
        vf->erased_ok = true;
        vf->write_pos = 0;
    }

    vf->opened = true;
    vf->write_mode = write_mode;

    // fd is the index into g_vfiles
    *fd_out = (int)(vf - g_vfiles);

    // If this is write mode, associate ring buffer with this fd
    if (write_mode) {
        g_ring_fd = *fd_out;
        g_ring_head = 0;
        g_ring_tail = 0;
        staging_reset();
    }

    return HIVE_SUCCESS;
}

hive_status hive_file_close(int fd) {
    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = get_vfile(fd);
    if (!vf || !vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }

    // Final sync if write mode
    if (vf->write_mode && g_ring_fd == fd) {
        hive_file_sync(fd);
        g_ring_fd = -1;
    }

    vf->opened = false;
    vf->write_mode = false;

    return HIVE_SUCCESS;
}

hive_status hive_file_read(int fd, void *buf, size_t len, size_t *bytes_read) {
    (void)len;  // Read position tracking not implemented

    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = get_vfile(fd);
    if (!vf || !vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }

    // For now, read is not implemented (would need read position tracking)
    // This is primarily a write-optimized implementation for logging
    *bytes_read = 0;
    return HIVE_ERROR(HIVE_ERR_INVALID, "read not implemented for flash files");
}

hive_status hive_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read) {
    if (!buf || !bytes_read) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL buffer or bytes_read pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = get_vfile(fd);
    if (!vf || !vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }

    // Clamp read to available data
    if (offset >= vf->flash_size) {
        *bytes_read = 0;
        return HIVE_SUCCESS;
    }

    size_t avail = vf->flash_size - offset;
    if (len > avail) {
        len = avail;
    }

    // Direct read from flash memory
    memcpy(buf, (const void *)(vf->flash_base + offset), len);
    *bytes_read = len;

    return HIVE_SUCCESS;
}

hive_status hive_file_write(int fd, const void *buf, size_t len, size_t *bytes_written) {
    if (!buf || !bytes_written) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL buffer or bytes_written pointer");
    }

    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = get_vfile(fd);
    if (!vf || !vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }

    if (!vf->write_mode) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not opened for writing");
    }

    if (!vf->erased_ok) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "flash not erased (use HIVE_O_TRUNC)");
    }

    // Write data to ring buffer, flushing to flash when buffer is full.
    // Most writes complete immediately (O(1)). When buffer fills up, we flush
    // to flash (blocking) to make room, ensuring no data loss.
    const uint8_t *data = (const uint8_t *)buf;
    size_t remaining = len;
    *bytes_written = 0;

    while (remaining > 0) {
        size_t pushed = ring_push(data, remaining);
        *bytes_written += pushed;
        data += pushed;
        remaining -= pushed;

        if (remaining > 0) {
            // Ring buffer is full - flush to flash to make room
            if (!flush_ring_to_flash(vf)) {
                // Flash is full or write error - return partial write
                return (*bytes_written > 0) ? HIVE_SUCCESS :
                    HIVE_ERROR(HIVE_ERR_IO, "flash region full");
            }
        }
    }

    return HIVE_SUCCESS;
}

hive_status hive_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written) {
    // pwrite not supported for ring-buffered flash writes
    (void)fd;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "pwrite not supported for flash files");
}

hive_status hive_file_sync(int fd) {
    HIVE_REQUIRE_INIT(g_file.initialized, "File I/O");

    vfile_t *vf = get_vfile(fd);
    if (!vf || !vf->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }

    if (!vf->write_mode || g_ring_fd != fd) {
        return HIVE_SUCCESS;  // Nothing to sync
    }

    if (!vf->erased_ok) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "flash not erased");
    }

    // Drain ring buffer to staging buffer, then to flash
    if (!flush_ring_to_flash(vf)) {
        return HIVE_ERROR(HIVE_ERR_IO, "flash write failed");
    }

    // Commit any remaining data in staging (padded with 0xFF)
    if (g_staging_len > 0) {
        if (!staging_commit(vf)) {
            return HIVE_ERROR(HIVE_ERR_IO, "flash write failed");
        }
    }

    return HIVE_SUCCESS;
}
