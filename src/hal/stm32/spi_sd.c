// SPI SD Card Driver - SD Protocol Implementation
//
// Platform-independent SD card protocol over SPI.
// Board-specific SPI functions provided via spi_ll.h interface.
//
// Sector data transfers (512 bytes) use DMA + scheduler yield so other
// actors run during the SPI transfer. Card busy-wait after writes also
// yields via periodic timer polling. Commands, tokens, and CRC bytes
// remain byte-by-byte (fast, no yield needed).
//
// If called outside actor context (e.g., during FatFS mount from main()
// before the scheduler starts), sector transfers fall back to byte-by-byte
// and busy-waits use polling instead of timer yield.
//
// Reference: http://elm-chan.org/docs/mmc/mmc_e.html

#include "spi_sd.h"
#include "spi_ll.h"
#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include "hal/hive_hal_event.h"
#include "hive_select.h"
#include "hive_timer.h"
#include <string.h>
#include <stdbool.h>

// Forward declaration - detect whether scheduler is running.
// Returns NULL when called from main() before hive_run().
// DMA + yield requires actor context; falls back to byte-by-byte otherwise.
struct actor;
extern struct actor *hive_actor_current(void);

static inline bool in_actor_context(void) {
    return hive_actor_current() != NULL;
}

// Debug output - uses platform debug printf
// Enable SD_DEBUG_ENABLE to see initialization details
#if defined(SD_DEBUG_ENABLE) && SD_DEBUG_ENABLE
extern void platform_debug_printf(const char *fmt, ...);
#define SD_DEBUG(...) platform_debug_printf("[SD] " __VA_ARGS__)
#else
#define SD_DEBUG(...) ((void)0)
#endif

// ---------------------------------------------------------------------------
// SD Card Commands (SPI Mode)
// ---------------------------------------------------------------------------

#define CMD0 0    // GO_IDLE_STATE
#define CMD1 1    // SEND_OP_COND (MMC)
#define CMD8 8    // SEND_IF_COND
#define CMD9 9    // SEND_CSD
#define CMD10 10  // SEND_CID
#define CMD12 12  // STOP_TRANSMISSION
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD18 18  // READ_MULTIPLE_BLOCK
#define CMD24 24  // WRITE_BLOCK
#define CMD25 25  // WRITE_MULTIPLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SD_SEND_OP_COND (app-specific)

// R1 response flags
#define R1_IDLE_STATE 0x01
#define R1_ERASE_RESET 0x02
#define R1_ILLEGAL_CMD 0x04
#define R1_CRC_ERROR 0x08
#define R1_ERASE_SEQ_ERROR 0x10
#define R1_ADDRESS_ERROR 0x20
#define R1_PARAMETER_ERROR 0x40

// Data tokens
#define TOKEN_SINGLE_BLOCK 0xFE
#define TOKEN_MULTI_BLOCK 0xFC
#define TOKEN_STOP_TRAN 0xFD

// ---------------------------------------------------------------------------
// DMA + Yield Configuration
// ---------------------------------------------------------------------------

#define SD_DMA_TIMEOUT_MS 100 // DMA transfer timeout (512B at 21MHz ~195us)
#define SD_BUSY_POLL_INTERVAL_US 1000 // 1ms between busy polls
#define SD_BUSY_TIMEOUT_MS 500        // 500ms max card busy time
#define SD_BUSY_MAX_POLLS (SD_BUSY_TIMEOUT_MS * 1000 / SD_BUSY_POLL_INTERVAL_US)

// ---------------------------------------------------------------------------
// Driver State
// ---------------------------------------------------------------------------

static bool s_initialized = false;
static sd_card_type_t s_card_type = SD_CARD_NONE;
static uint32_t s_sector_count = 0;

// HAL event for DMA completion (signaled by ISR in spi_ll_sd.c)
static hive_hal_event_id_t s_dma_event = HIVE_HAL_EVENT_INVALID;

// ---------------------------------------------------------------------------
// SPI Helper Functions
// ---------------------------------------------------------------------------

// Send multiple bytes (discard received)
static void spi_send(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        spi_ll_xfer(buf[i]);
    }
}

// Receive multiple bytes (send 0xFF)
static void spi_recv(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_ll_xfer(0xFF);
    }
}

// ---------------------------------------------------------------------------
// SD Protocol Helpers
// ---------------------------------------------------------------------------

// Wait for card ready (not busy)
static int wait_ready(uint32_t timeout_ms) {
    (void)timeout_ms;
    for (uint32_t i = 0; i < 500000; i++) {
        if (spi_ll_xfer(0xFF) == 0xFF) {
            return 0;
        }
    }
    return -1;
}

// Send SD command
static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t buf[6];
    uint8_t crc = 0x01; // Dummy CRC (CRC disabled after init)

    // Pre-defined CRCs for required commands
    if (cmd == CMD0)
        crc = 0x95;
    if (cmd == CMD8)
        crc = 0x87;

    buf[0] = 0x40 | cmd;
    buf[1] = (arg >> 24) & 0xFF;
    buf[2] = (arg >> 16) & 0xFF;
    buf[3] = (arg >> 8) & 0xFF;
    buf[4] = arg & 0xFF;
    buf[5] = crc;

    // Send dummy clock
    spi_ll_xfer(0xFF);

    // Wait for card ready (skip for CMD0/CMD12)
    if (cmd != CMD0 && cmd != CMD12) {
        wait_ready(500);
    }

    // Send command
    spi_send(buf, 6);

    // Wait for response (not 0xFF) - need many iterations for slow cards
    // Bitcraze uses 0x1FFF (~8191) iterations
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 8192; i++) {
        r1 = spi_ll_xfer(0xFF);
        if ((r1 & 0x80) == 0) {
            break;
        }
    }

    return r1;
}

// Send application-specific command
static uint8_t send_acmd(uint8_t cmd, uint32_t arg) {
    send_cmd(CMD55, 0);
    return send_cmd(cmd, arg);
}

// Wait for data token
static int wait_data_token(uint8_t token, uint32_t timeout_ms) {
    (void)timeout_ms;
    for (uint32_t i = 0; i < 100000; i++) {
        uint8_t r = spi_ll_xfer(0xFF);
        if (r == token) {
            return 0;
        }
        if (r != 0xFF) {
            return -1; // Error token
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// DMA Transfer + Yield
// ---------------------------------------------------------------------------

// Transfer one sector (512 bytes) via DMA, yielding to scheduler.
// Locks SPI bus during DMA to prevent flow deck access.
// CS must already be asserted by caller.
// Falls back to byte-by-byte if not in actor context (e.g., during mount).
static int dma_xfer(void *buf, uint32_t len, spi_ll_dma_dir_t dir) {
    if (!in_actor_context()) {
        // No scheduler running - use synchronous byte-by-byte transfer
        if (dir == SPI_LL_DIR_RX) {
            spi_recv(buf, len);
        } else {
            spi_send(buf, len);
        }
        return 0;
    }

    spi_ll_lock();
    spi_ll_dma_start(buf, len, dir);
    hive_status_t s = hive_event_wait(s_dma_event, SD_DMA_TIMEOUT_MS);
    spi_ll_unlock();

    if (HIVE_FAILED(s) || !spi_ll_dma_ok()) {
        return -1;
    }
    return 0;
}

// Wait for card ready with scheduler yield.
// Deasserts CS between polls so other SPI devices can use the bus.
// Each poll briefly locks the bus for one byte transfer (~0.4us).
// Falls back to synchronous busy-wait if not in actor context.
static int wait_ready_yield(void) {
    if (!in_actor_context()) {
        // No scheduler - use synchronous busy-wait (same as init phase)
        spi_ll_cs_low();
        int ret = wait_ready(SD_BUSY_TIMEOUT_MS);
        spi_ll_cs_high();
        return ret;
    }

    // Check immediately before starting timer
    spi_ll_lock();
    spi_ll_cs_low();
    uint8_t r = spi_ll_xfer(0xFF);
    spi_ll_cs_high();
    spi_ll_unlock();
    if (r == 0xFF) {
        return 0;
    }

    hive_timer_id_t poll_timer;
    hive_status_t s = hive_timer_every(SD_BUSY_POLL_INTERVAL_US, &poll_timer);
    if (HIVE_FAILED(s)) {
        return -1;
    }

    int result = -1;
    hive_message_t msg;

    for (int i = 0; i < SD_BUSY_MAX_POLLS; i++) {
        s = hive_timer_recv(poll_timer, &msg, SD_BUSY_TIMEOUT_MS);
        if (HIVE_FAILED(s)) {
            break; // Timer failure or timeout
        }

        spi_ll_lock();
        spi_ll_cs_low();
        r = spi_ll_xfer(0xFF);
        spi_ll_cs_high();
        spi_ll_unlock();

        if (r == 0xFF) {
            result = 0;
            break;
        }
    }

    hive_timer_cancel(poll_timer);
    return result;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void spi_sd_configure(const spi_sd_config_t *config) {
    // Board-specific SPI configuration is hardcoded in spi_ll_sd.c
    // This function exists for API compatibility with mount table
    (void)config;
}

int spi_sd_is_present(void) {
    // Board-specific detection could be added to spi_ll
    return 1;
}

int spi_sd_init(void) {
    if (s_initialized) {
        return 0;
    }

    uint8_t r1;
    uint8_t ocr[4];

    SD_DEBUG("Initializing SPI SD card...\n");

    // Initialize SPI hardware at low speed
    spi_ll_init();
    spi_ll_set_slow();

    // Add a longer power-on delay for card stabilization
    SD_DEBUG("Power-on delay (100ms equivalent)...\n");
    for (volatile int d = 0; d < 2000000; d++)
        __asm__("nop");

    // Abort any in-progress transfer from a previous boot.
    // If the MCU reset mid-transfer (e.g., st-flash), the card is still
    // powered and waiting for clocks to complete. Send CMD12 (STOP) with
    // CS asserted, then flush with 4096+ clocks (CS high) to drain any
    // pending data and return the card to idle native mode.
    SD_DEBUG("Aborting stale transfer (CMD12 + flush)...\n");
    spi_ll_cs_low();
    send_cmd(CMD12, 0); // STOP_TRANSMISSION - abort any multi-block op
    spi_ll_cs_high();

    // 4096 clocks (512 bytes) with CS high drains any pending read data
    // and satisfies the SD spec's 74-clock minimum for power-on reset.
    for (int i = 0; i < 512; i++) {
        spi_ll_xfer(0xFF);
    }

    // Power-on delay
    for (volatile int d = 0; d < 200000; d++)
        __asm__("nop");

    // Enter SPI mode (CMD0) with retries
    r1 = 0xFF;
    for (int attempt = 0; attempt < 5 && r1 != R1_IDLE_STATE; attempt++) {
        if (attempt > 0) {
            SD_DEBUG("CMD0 attempt %d...\n", attempt + 1);
            // Flush again between retries
            spi_ll_cs_high();
            for (int i = 0; i < 128; i++) {
                spi_ll_xfer(0xFF);
            }
            for (volatile int d = 0; d < 500000; d++)
                __asm__("nop");
        }

        spi_ll_cs_low();
        for (volatile int d = 0; d < 10000; d++)
            __asm__("nop");
        r1 = send_cmd(CMD0, 0);
        spi_ll_cs_high();
        SD_DEBUG("CMD0 response: 0x%02X (expect 0x01)\n", r1);
    }

    if (r1 != R1_IDLE_STATE) {
        SD_DEBUG("CMD0 failed after 5 attempts (last=0x%02X)\n", r1);
        return -1;
    }

    // Check voltage (CMD8)
    SD_DEBUG("Sending CMD8 (SEND_IF_COND)...\n");
    spi_ll_cs_low();
    r1 = send_cmd(CMD8, 0x1AA);
    SD_DEBUG("CMD8 response: 0x%02X\n", r1);
    if (r1 == R1_IDLE_STATE) {
        // SD v2.0+
        spi_recv(ocr, 4);
        spi_ll_cs_high();

        SD_DEBUG("CMD8 OCR: %02X %02X %02X %02X\n", ocr[0], ocr[1], ocr[2],
                 ocr[3]);

        if (ocr[2] != 0x01 || ocr[3] != 0xAA) {
            SD_DEBUG("CMD8 voltage check failed\n");
            return -1; // Voltage not accepted
        }

        // Wait for card ready (ACMD41 with HCS=1)
        SD_DEBUG("Sending ACMD41 (SD_SEND_OP_COND)...\n");
        for (int i = 0; i < 1000; i++) {
            spi_ll_cs_low();
            r1 = send_acmd(ACMD41, 0x40000000);
            spi_ll_cs_high();
            if (r1 == 0)
                break;
        }

        SD_DEBUG("ACMD41 final response: 0x%02X\n", r1);

        if (r1 != 0) {
            SD_DEBUG("ACMD41 failed: card not ready\n");
            return -1;
        }

        // Check CCS bit (high capacity)
        spi_ll_cs_low();
        r1 = send_cmd(CMD58, 0);
        spi_recv(ocr, 4);
        spi_ll_cs_high();

        s_card_type = (ocr[0] & 0x40) ? SD_CARD_V2_HC : SD_CARD_V2_SC;
        SD_DEBUG("Card type: %s\n",
                 s_card_type == SD_CARD_V2_HC ? "SDHC" : "SD v2.0");
    } else {
        spi_ll_cs_high();
        SD_DEBUG("Card is SD v1.x or MMC\n");
        // SD v1.x or MMC
        spi_ll_cs_low();
        r1 = send_acmd(ACMD41, 0);
        spi_ll_cs_high();

        if ((r1 & R1_ILLEGAL_CMD) == 0) {
            // SD v1.x
            for (int i = 0; i < 1000; i++) {
                spi_ll_cs_low();
                r1 = send_acmd(ACMD41, 0);
                spi_ll_cs_high();
                if (r1 == 0)
                    break;
            }
            s_card_type = SD_CARD_V1;
        } else {
            // MMC (not supported)
            SD_DEBUG("MMC cards not supported\n");
            return -1;
        }

        if (r1 != 0) {
            SD_DEBUG("ACMD41 failed for SD v1.x\n");
            return -1;
        }
    }

    // Set block size to 512 for v1/v2-SC cards
    if (s_card_type != SD_CARD_V2_HC) {
        spi_ll_cs_low();
        r1 = send_cmd(CMD16, 512);
        spi_ll_cs_high();
        if (r1 != 0) {
            return -1;
        }
    }

    // Read CSD to get sector count
    spi_ll_cs_low();
    r1 = send_cmd(CMD9, 0);
    if (r1 == 0 && wait_data_token(TOKEN_SINGLE_BLOCK, 500) == 0) {
        uint8_t csd[16];
        spi_recv(csd, 16);
        spi_ll_xfer(0xFF); // CRC
        spi_ll_xfer(0xFF);

        // Parse CSD for capacity
        if ((csd[0] >> 6) == 1) {
            // CSD v2.0 (SDHC/SDXC)
            uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) |
                              ((uint32_t)csd[8] << 8) | (uint32_t)csd[9];
            s_sector_count = (c_size + 1) * 1024;
        } else {
            // CSD v1.0
            uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                              ((uint32_t)csd[7] << 2) |
                              ((uint32_t)(csd[8] >> 6) & 0x03);
            uint32_t c_size_mult =
                ((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01);
            uint32_t read_bl_len = csd[5] & 0x0F;
            s_sector_count = (c_size + 1)
                             << (c_size_mult + read_bl_len + 2 - 9);
        }
    }
    spi_ll_cs_high();

    // Switch to high speed
    spi_ll_set_fast();

    // Initialize DMA for non-blocking sector transfers
    s_dma_event = hive_hal_event_create();
    if (s_dma_event == HIVE_HAL_EVENT_INVALID) {
        SD_DEBUG("Failed to create DMA event\n");
        return -1;
    }
    spi_ll_dma_init(s_dma_event);

    s_initialized = true;
    SD_DEBUG("SD card initialized: %lu sectors (%lu MB)\n", s_sector_count,
             s_sector_count / 2048);
    return 0;
}

int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count) {
    if (!s_initialized || count == 0) {
        SD_DEBUG("read_blocks: not initialized or count=0\n");
        return -1;
    }
    SD_DEBUG("read_blocks: sector=%lu count=%lu\n", sector, count);

    // Convert to byte address for v1/v2-SC cards
    uint32_t addr = sector;
    if (s_card_type != SD_CARD_V2_HC) {
        addr *= 512;
    }

    spi_ll_cs_low();

    if (count == 1) {
        // Single block read
        uint8_t r = send_cmd(CMD17, addr);
        if (r != 0) {
            SD_DEBUG("CMD17 failed: 0x%02X\n", r);
            spi_ll_cs_high();
            return -1;
        }

        if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
            SD_DEBUG("wait_data_token failed\n");
            spi_ll_cs_high();
            return -1;
        }

        if (dma_xfer(buf, 512, SPI_LL_DIR_RX) != 0) {
            spi_ll_cs_high();
            return -1;
        }
        spi_ll_xfer(0xFF); // CRC
        spi_ll_xfer(0xFF);
    } else {
        // Multi-block read
        if (send_cmd(CMD18, addr) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
                send_cmd(CMD12, 0);
                spi_ll_cs_high();
                return -1;
            }

            if (dma_xfer(buf + i * 512, 512, SPI_LL_DIR_RX) != 0) {
                send_cmd(CMD12, 0);
                spi_ll_cs_high();
                return -1;
            }
            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);
        }

        send_cmd(CMD12, 0);
    }

    spi_ll_cs_high();
    return 0;
}

int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count) {
    if (!s_initialized || count == 0) {
        return -1;
    }

    // Convert to byte address for v1/v2-SC cards
    uint32_t addr = sector;
    if (s_card_type != SD_CARD_V2_HC) {
        addr *= 512;
    }

    spi_ll_cs_low();

    if (count == 1) {
        // Single block write
        if (send_cmd(CMD24, addr) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        spi_ll_xfer(0xFF);               // Gap
        spi_ll_xfer(TOKEN_SINGLE_BLOCK); // Data token
        if (dma_xfer((void *)buf, 512, SPI_LL_DIR_TX) != 0) {
            spi_ll_cs_high();
            return -1;
        }
        spi_ll_xfer(0xFF); // CRC
        spi_ll_xfer(0xFF);

        // Check data response
        uint8_t resp = spi_ll_xfer(0xFF);
        if ((resp & 0x1F) != 0x05) {
            spi_ll_cs_high();
            return -1;
        }

        // Deassert CS before yielding busy-wait (frees SPI for flow deck)
        spi_ll_cs_high();
        if (wait_ready_yield() != 0) {
            return -1;
        }
    } else {
        // Multi-block write
        if (send_cmd(CMD25, addr) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            spi_ll_xfer(0xFF);              // Gap
            spi_ll_xfer(TOKEN_MULTI_BLOCK); // Data token
            if (dma_xfer((void *)(buf + i * 512), 512, SPI_LL_DIR_TX) != 0) {
                spi_ll_cs_high();
                return -1;
            }
            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);

            // Check data response
            uint8_t resp = spi_ll_xfer(0xFF);
            if ((resp & 0x1F) != 0x05) {
                spi_ll_xfer(TOKEN_STOP_TRAN);
                spi_ll_cs_high();
                return -1;
            }

            // Deassert CS before yielding busy-wait
            spi_ll_cs_high();
            if (wait_ready_yield() != 0) {
                return -1;
            }
            // Reassert CS for next block or stop command
            spi_ll_cs_low();
        }

        spi_ll_xfer(TOKEN_STOP_TRAN);
        spi_ll_cs_high();
        if (wait_ready_yield() != 0) {
            return -1;
        }
    }

    return 0;
}

int spi_sd_sync(void) {
    if (!s_initialized) {
        return -1;
    }

    return wait_ready_yield();
}

uint32_t spi_sd_get_sector_count(void) {
    return s_sector_count;
}

sd_card_type_t spi_sd_get_card_type(void) {
    return s_card_type;
}

#else // !HIVE_ENABLE_SD

// Stubs when SD is disabled
void spi_sd_configure(const spi_sd_config_t *config) {
    (void)config;
}
int spi_sd_is_present(void) {
    return 0;
}
int spi_sd_init(void) {
    return -1;
}
int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count) {
    (void)buf;
    (void)sector;
    (void)count;
    return -1;
}
int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count) {
    (void)buf;
    (void)sector;
    (void)count;
    return -1;
}
int spi_sd_sync(void) {
    return -1;
}
uint32_t spi_sd_get_sector_count(void) {
    return 0;
}
sd_card_type_t spi_sd_get_card_type(void) {
    return SD_CARD_NONE;
}

#endif // HIVE_ENABLE_SD
