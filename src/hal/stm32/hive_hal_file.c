// Hardware Abstraction Layer - STM32 File I/O Implementation
//
// Multi-backend file system for STM32:
// - FLASH: Internal flash sectors (ring-buffered writes)
// - SD: SD card via FatFS (when HIVE_ENABLE_SD is defined)
//
// Uses mount table for path-based backend selection.
// File descriptors encode backend type for dispatch.

#include "hal/hive_hal_file.h"
#include "hive_mount.h"
#include "hive_file.h"
#include "hive_internal.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Flash Backend
// ============================================================================

// STM32 Flash register definitions
// These are standard for STM32F4xx - included via CMSIS headers in real builds
#ifndef FLASH_BASE
#define FLASH_BASE 0x40023C00UL
#endif

typedef struct {
    volatile uint32_t ACR;     // Access control register
    volatile uint32_t KEYR;    // Key register
    volatile uint32_t OPTKEYR; // Option key register
    volatile uint32_t SR;      // Status register
    volatile uint32_t CR;      // Control register
    volatile uint32_t OPTCR;   // Option control register
} FLASH_TypeDef;

#define FLASH ((FLASH_TypeDef *)FLASH_BASE)

// Flash key values for unlock sequence
#define FLASH_KEY1 0x45670123UL
#define FLASH_KEY2 0xCDEF89ABUL

// Flash status register bits
#define FLASH_SR_BSY (1UL << 16)
#define FLASH_SR_PGSERR (1UL << 7)
#define FLASH_SR_PGPERR (1UL << 6)
#define FLASH_SR_PGAERR (1UL << 5)
#define FLASH_SR_WRPERR (1UL << 4)
#define FLASH_SR_OPERR (1UL << 1)
#define FLASH_SR_EOP (1UL << 0)
#define FLASH_SR_ERRORS                                                      \
    (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | \
     FLASH_SR_OPERR)

// Flash control register bits
#define FLASH_CR_PG (1UL << 0)  // Programming
#define FLASH_CR_SER (1UL << 1) // Sector erase
#define FLASH_CR_MER (1UL << 2) // Mass erase
#define FLASH_CR_SNB_Pos 3      // Sector number position
#define FLASH_CR_SNB_Msk (0x1FUL << FLASH_CR_SNB_Pos)
#define FLASH_CR_PSIZE_Pos 8        // Program size position
#define FLASH_CR_PSIZE_0 (1UL << 8) // 8-bit
#define FLASH_CR_PSIZE_1 (1UL << 9) // 32-bit
#define FLASH_CR_STRT (1UL << 16)   // Start
#define FLASH_CR_LOCK (1UL << 31)   // Lock

// ----------------------------------------------------------------------------
// Flash File State
// ----------------------------------------------------------------------------

typedef struct {
    const hive_mount_t *mount; // Mount configuration (NULL = unused)
    uint32_t write_pos;        // Current write position
    bool opened;
    bool erased_ok;
    bool write_mode;
} flash_file_t;

static flash_file_t s_flash_files[HIVE_MAX_FLASH_FILES];

// ----------------------------------------------------------------------------
// Ring Buffer for Deferred Writes
// ----------------------------------------------------------------------------

static uint8_t s_ring_buf[HIVE_FILE_RING_SIZE];
static volatile uint32_t s_ring_head; // Write position
static volatile uint32_t s_ring_tail; // Read position

// Staging buffer for flash block commits
static uint8_t s_staging[HIVE_FILE_BLOCK_SIZE];
static uint32_t s_staging_len;

// Current file being written (index into s_flash_files, or -1)
static int s_ring_file_idx = -1;

// ----------------------------------------------------------------------------
// Ring Buffer Operations
// ----------------------------------------------------------------------------

static inline uint32_t ring_used(void) {
    return (s_ring_head - s_ring_tail) & (HIVE_FILE_RING_SIZE - 1);
}

static inline uint32_t ring_free(void) {
    return HIVE_FILE_RING_SIZE - 1 - ring_used();
}

static inline bool ring_empty(void) {
    return s_ring_head == s_ring_tail;
}

static size_t ring_push(const uint8_t *data, size_t len) {
    size_t free = ring_free();
    size_t to_write = (len < free) ? len : free;

    for (size_t i = 0; i < to_write; i++) {
        s_ring_buf[s_ring_head & (HIVE_FILE_RING_SIZE - 1)] = data[i];
        s_ring_head++;
    }

    return to_write;
}

static size_t ring_pop(uint8_t *data, size_t max_len) {
    size_t used = ring_used();
    size_t to_read = (max_len < used) ? max_len : used;

    for (size_t i = 0; i < to_read; i++) {
        data[i] = s_ring_buf[s_ring_tail & (HIVE_FILE_RING_SIZE - 1)];
        s_ring_tail++;
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
    while (FLASH->SR & FLASH_SR_BSY)
        ;
}

static void flash_clear_errors(void) {
    FLASH->SR = FLASH_SR_ERRORS;
}

static bool flash_erase_sector(uint8_t sector) {
    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    FLASH->CR = FLASH_CR_SER | ((uint32_t)sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;

    flash_wait_bsy();

    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

__attribute__((section(".ramfunc"), noinline)) static void
flash_program_words_ram(uint32_t addr, const uint32_t *data, uint32_t words) {
    FLASH->CR = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    for (uint32_t i = 0; i < words; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        while (FLASH->SR & FLASH_SR_BSY)
            ;
    }

    FLASH->CR &= ~FLASH_CR_PG;
}

static bool flash_write_block(uint32_t addr, const void *data, uint32_t len) {
    if (len == 0 || (len & 3) != 0) {
        return false;
    }

    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    __asm volatile("cpsid i" ::: "memory");
    flash_program_words_ram(addr, (const uint32_t *)data, len / 4);
    __asm volatile("cpsie i" ::: "memory");

    bool ok = (FLASH->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

// ----------------------------------------------------------------------------
// Staging Buffer Operations
// ----------------------------------------------------------------------------

static void staging_reset(void) {
    s_staging_len = 0;
    memset(s_staging, 0xFF, HIVE_FILE_BLOCK_SIZE);
}

static size_t staging_space(void) {
    return HIVE_FILE_BLOCK_SIZE - s_staging_len;
}

static void staging_append(const uint8_t *data, size_t len) {
    if (s_staging_len + len > HIVE_FILE_BLOCK_SIZE) {
        len = HIVE_FILE_BLOCK_SIZE - s_staging_len;
    }
    memcpy(&s_staging[s_staging_len], data, len);
    s_staging_len += len;
}

static bool staging_commit(flash_file_t *ff) {
    if (s_staging_len == 0) {
        return true;
    }

    uint32_t flash_size = hive_mount_flash_size(ff->mount);
    if (ff->write_pos + HIVE_FILE_BLOCK_SIZE > flash_size) {
        return false;
    }

    uint32_t addr = hive_mount_flash_base(ff->mount) + ff->write_pos;
    bool ok = flash_write_block(addr, s_staging, HIVE_FILE_BLOCK_SIZE);

    if (ok) {
        ff->write_pos += HIVE_FILE_BLOCK_SIZE;
    }

    staging_reset();
    return ok;
}

static bool flush_ring_to_flash(flash_file_t *ff) {
    uint8_t temp[64];
    while (!ring_empty()) {
        size_t n = ring_pop(temp, sizeof(temp));

        for (size_t i = 0; i < n; i++) {
            if (staging_space() == 0) {
                if (!staging_commit(ff)) {
                    return false;
                }
            }
            staging_append(&temp[i], 1);
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// Flash Backend API
// ----------------------------------------------------------------------------

static void flash_init(void) {
    s_ring_head = 0;
    s_ring_tail = 0;
    s_ring_file_idx = -1;
    staging_reset();

    for (int i = 0; i < HIVE_MAX_FLASH_FILES; i++) {
        s_flash_files[i].mount = NULL;
        s_flash_files[i].opened = false;
    }
}

static void flash_cleanup(void) {
    for (int i = 0; i < HIVE_MAX_FLASH_FILES; i++) {
        s_flash_files[i].opened = false;
    }
}

static hive_status_t flash_open(const hive_mount_t *mount, int flags,
                                int *out) {
    // Find free slot
    int slot = -1;
    for (int i = 0; i < HIVE_MAX_FLASH_FILES; i++) {
        if (!s_flash_files[i].opened) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "no free flash file slots");
    }

    flash_file_t *ff = &s_flash_files[slot];

    // Check if already open (same mount)
    for (int i = 0; i < HIVE_MAX_FLASH_FILES; i++) {
        if (s_flash_files[i].opened && s_flash_files[i].mount == mount) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "file already open");
        }
    }

    int access = flags & 0x0003;

    if (access == HIVE_O_RDWR) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "HIVE_O_RDWR not supported; use RDONLY or WRONLY");
    }

    bool write_mode = (access == HIVE_O_WRONLY);

    if (write_mode && !(flags & HIVE_O_TRUNC)) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "HIVE_O_TRUNC required for flash writes");
    }

    if ((flags & HIVE_O_TRUNC) && write_mode) {
        if (!flash_erase_sector(hive_mount_flash_sector(mount))) {
            return HIVE_ERROR(HIVE_ERR_IO, "flash erase failed");
        }
        ff->erased_ok = true;
        ff->write_pos = 0;
    } else {
        ff->erased_ok = false;
        ff->write_pos = 0;
    }

    ff->mount = mount;
    ff->opened = true;
    ff->write_mode = write_mode;

    if (write_mode) {
        s_ring_file_idx = slot;
        s_ring_head = 0;
        s_ring_tail = 0;
        staging_reset();
    }

    *out = FD_MAKE(HIVE_BACKEND_FLASH, slot);
    return HIVE_SUCCESS;
}

static hive_status_t flash_close(int index) {
    if (index < 0 || index >= HIVE_MAX_FLASH_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid flash fd");
    }

    flash_file_t *ff = &s_flash_files[index];
    if (!ff->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    if (ff->write_mode && s_ring_file_idx == index) {
        // Sync before close
        if (ff->erased_ok) {
            flush_ring_to_flash(ff);
            if (s_staging_len > 0) {
                staging_commit(ff);
            }
        }
        s_ring_file_idx = -1;
    }

    ff->opened = false;
    ff->mount = NULL;

    return HIVE_SUCCESS;
}

static hive_status_t flash_read(int index, void *buf, size_t len,
                                size_t *bytes_read) {
    (void)index;
    (void)buf;
    (void)len;
    *bytes_read = 0;
    return HIVE_ERROR(HIVE_ERR_INVALID, "read not supported for flash");
}

static hive_status_t flash_pread(int index, void *buf, size_t len,
                                 size_t offset, size_t *bytes_read) {
    if (index < 0 || index >= HIVE_MAX_FLASH_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid flash fd");
    }

    flash_file_t *ff = &s_flash_files[index];
    if (!ff->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    uint32_t flash_size = hive_mount_flash_size(ff->mount);
    if (offset >= flash_size) {
        *bytes_read = 0;
        return HIVE_SUCCESS;
    }

    size_t avail = flash_size - offset;
    if (len > avail) {
        len = avail;
    }

    uint32_t flash_base = hive_mount_flash_base(ff->mount);
    memcpy(buf, (const void *)(flash_base + offset), len);
    *bytes_read = len;

    return HIVE_SUCCESS;
}

static hive_status_t flash_write(int index, const void *buf, size_t len,
                                 size_t *bytes_written) {
    if (index < 0 || index >= HIVE_MAX_FLASH_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid flash fd");
    }

    flash_file_t *ff = &s_flash_files[index];
    if (!ff->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    if (!ff->write_mode) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not opened for writing");
    }

    if (!ff->erased_ok) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "flash not erased");
    }

    const uint8_t *data = (const uint8_t *)buf;
    size_t remaining = len;
    *bytes_written = 0;

    while (remaining > 0) {
        size_t pushed = ring_push(data, remaining);
        *bytes_written += pushed;
        data += pushed;
        remaining -= pushed;

        if (remaining > 0) {
            if (!flush_ring_to_flash(ff)) {
                return (*bytes_written > 0)
                           ? HIVE_SUCCESS
                           : HIVE_ERROR(HIVE_ERR_IO, "flash region full");
            }
        }
    }

    return HIVE_SUCCESS;
}

static hive_status_t flash_pwrite(int index, const void *buf, size_t len,
                                  size_t offset, size_t *bytes_written) {
    (void)index;
    (void)buf;
    (void)len;
    (void)offset;
    (void)bytes_written;
    return HIVE_ERROR(HIVE_ERR_INVALID, "pwrite not supported for flash");
}

static hive_status_t flash_sync(int index) {
    if (index < 0 || index >= HIVE_MAX_FLASH_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid flash fd");
    }

    flash_file_t *ff = &s_flash_files[index];
    if (!ff->opened) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    if (!ff->write_mode || s_ring_file_idx != index) {
        return HIVE_SUCCESS;
    }

    if (!ff->erased_ok) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "flash not erased");
    }

    if (!flush_ring_to_flash(ff)) {
        return HIVE_ERROR(HIVE_ERR_IO, "flash write failed");
    }

    if (s_staging_len > 0) {
        if (!staging_commit(ff)) {
            return HIVE_ERROR(HIVE_ERR_IO, "flash write failed");
        }
    }

    return HIVE_SUCCESS;
}

// ============================================================================
// SD Backend (FatFS)
// ============================================================================

#if HIVE_ENABLE_SD

#include "spi_sd.h"
#include "ff.h"

// FatFS filesystem and file handles
static FATFS s_fatfs;
static FIL s_sd_files[HIVE_MAX_SD_FILES];
static bool s_sd_file_used[HIVE_MAX_SD_FILES];
static bool s_sd_initialized = false;

// Convert HIVE flags to FatFS mode
static BYTE hive_flags_to_fatfs(int flags) {
    BYTE mode = 0;
    int access = flags & 0x0003;

    if (access == HIVE_O_RDONLY) {
        mode = FA_READ;
    } else if (access == HIVE_O_WRONLY) {
        mode = FA_WRITE;
    } else if (access == HIVE_O_RDWR) {
        mode = FA_READ | FA_WRITE;
    }

    if (flags & HIVE_O_CREAT) {
        mode |= FA_OPEN_ALWAYS;
    } else {
        mode |= FA_OPEN_EXISTING;
    }

    if (flags & HIVE_O_TRUNC) {
        mode |= FA_CREATE_ALWAYS;
    }

    if (flags & HIVE_O_APPEND) {
        mode |= FA_OPEN_APPEND;
    }

    return mode;
}

static void sd_init(void) {
    for (int i = 0; i < HIVE_MAX_SD_FILES; i++) {
        s_sd_file_used[i] = false;
    }
    s_sd_initialized = false;
}

static void sd_cleanup(void) {
    // Close any open files
    for (int i = 0; i < HIVE_MAX_SD_FILES; i++) {
        if (s_sd_file_used[i]) {
            f_close(&s_sd_files[i]);
            s_sd_file_used[i] = false;
        }
    }

    // Unmount FatFS
    if (s_sd_initialized) {
        f_mount(NULL, "", 0);
        s_sd_initialized = false;
    }
}

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
    FRESULT res = f_mount(&s_fatfs, "", 1); // Force mount
    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "SD card mount failed");
    }

    s_sd_initialized = true;
    return HIVE_SUCCESS;
}

static hive_status_t sd_open(const hive_mount_t *mount, const char *subpath,
                             int flags, int *out) {
    // Initialize SD if needed
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

    // Open file
    BYTE mode = hive_flags_to_fatfs(flags);
    FRESULT res = f_open(&s_sd_files[slot], subpath, mode);
    if (res != FR_OK) {
        if (res == FR_NO_FILE || res == FR_NO_PATH) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "file not found");
        }
        return HIVE_ERROR(HIVE_ERR_IO, "f_open failed");
    }

    s_sd_file_used[slot] = true;
    *out = FD_MAKE(HIVE_BACKEND_SD, slot);
    return HIVE_SUCCESS;
}

static hive_status_t sd_close(int index) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    FRESULT res = f_close(&s_sd_files[index]);
    s_sd_file_used[index] = false;

    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_close failed");
    }

    return HIVE_SUCCESS;
}

static hive_status_t sd_read(int index, void *buf, size_t len,
                             size_t *bytes_read) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    UINT br;
    FRESULT res = f_read(&s_sd_files[index], buf, len, &br);
    *bytes_read = br;

    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_read failed");
    }

    return HIVE_SUCCESS;
}

static hive_status_t sd_pread(int index, void *buf, size_t len, size_t offset,
                              size_t *bytes_read) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    // Seek to offset
    FRESULT res = f_lseek(&s_sd_files[index], offset);
    if (res != FR_OK) {
        *bytes_read = 0;
        return HIVE_ERROR(HIVE_ERR_IO, "f_lseek failed");
    }

    // Read data
    UINT br;
    res = f_read(&s_sd_files[index], buf, len, &br);
    *bytes_read = br;

    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_read failed");
    }

    return HIVE_SUCCESS;
}

static hive_status_t sd_write(int index, const void *buf, size_t len,
                              size_t *bytes_written) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    UINT bw;
    FRESULT res = f_write(&s_sd_files[index], buf, len, &bw);
    *bytes_written = bw;

    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_write failed");
    }

    return HIVE_SUCCESS;
}

static hive_status_t sd_pwrite(int index, const void *buf, size_t len,
                               size_t offset, size_t *bytes_written) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    // Seek to offset
    FRESULT res = f_lseek(&s_sd_files[index], offset);
    if (res != FR_OK) {
        *bytes_written = 0;
        return HIVE_ERROR(HIVE_ERR_IO, "f_lseek failed");
    }

    // Write data
    UINT bw;
    res = f_write(&s_sd_files[index], buf, len, &bw);
    *bytes_written = bw;

    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_write failed");
    }

    return HIVE_SUCCESS;
}

static hive_status_t sd_sync(int index) {
    if (index < 0 || index >= HIVE_MAX_SD_FILES) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid SD fd");
    }

    if (!s_sd_file_used[index]) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "file not open");
    }

    FRESULT res = f_sync(&s_sd_files[index]);
    if (res != FR_OK) {
        return HIVE_ERROR(HIVE_ERR_IO, "f_sync failed");
    }

    return HIVE_SUCCESS;
}

// Check if SD card is available (for mount_available API)
hive_status_t sd_check_available(const hive_mount_t *mount) {
    // If already initialized, we're good
    if (s_sd_initialized) {
        return HIVE_SUCCESS;
    }

    // Try to initialize (but don't keep it mounted)
    spi_sd_config_t cfg = {
        .spi_id = hive_mount_sd_spi(mount),
        .cs_port = hive_mount_sd_cs_port(mount),
        .cs_pin = hive_mount_sd_cs_pin(mount),
    };
    spi_sd_configure(&cfg);

    if (!spi_sd_is_present()) {
        return HIVE_ERROR(HIVE_ERR_IO, "SD card not present");
    }

    return HIVE_SUCCESS;
}

#endif // HIVE_ENABLE_SD

// ============================================================================
// HAL File API - Dispatch Layer
// ============================================================================

hive_status_t hive_hal_file_init(void) {
    flash_init();
#if HIVE_ENABLE_SD
    sd_init();
#endif
    return HIVE_SUCCESS;
}

void hive_hal_file_cleanup(void) {
    flash_cleanup();
#if HIVE_ENABLE_SD
    sd_cleanup();
#endif
}

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

hive_status_t hive_hal_file_close(int fd) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_close(index);

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_close(index);
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }
}

hive_status_t hive_hal_file_read(int fd, void *buf, size_t len,
                                 size_t *bytes_read) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_read(index, buf, len, bytes_read);

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_read(index, buf, len, bytes_read);
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }
}

hive_status_t hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset,
                                  size_t *bytes_read) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_pread(index, buf, len, offset, bytes_read);

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_pread(index, buf, len, offset, bytes_read);
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
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

hive_status_t hive_hal_file_pwrite(int fd, const void *buf, size_t len,
                                   size_t offset, size_t *bytes_written) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_pwrite(index, buf, len, offset, bytes_written);

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_pwrite(index, buf, len, offset, bytes_written);
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }
}

hive_status_t hive_hal_file_sync(int fd) {
    hive_file_backend_t backend = FD_BACKEND(fd);
    int index = FD_INDEX(fd);

    switch (backend) {
    case HIVE_BACKEND_FLASH:
        return flash_sync(index);

#if HIVE_ENABLE_SD
    case HIVE_BACKEND_SD:
        return sd_sync(index);
#endif

    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid fd");
    }
}
