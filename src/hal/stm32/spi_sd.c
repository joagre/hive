// SPI SD Card Driver - SD Protocol Implementation
//
// Platform-independent SD card protocol over SPI.
// Board-specific SPI functions provided via spi_ll.h interface.
//
// Sector data transfers (512 bytes) use DMA with spin-wait (~24us at
// 21 MHz, ~23x faster than byte-by-byte). Card busy-wait after writes
// yields via hive_sleep() with 1ms polls, matching the Bitcraze
// waitForCardReady() pattern (CS high, yield, CS low with SPI1
// reconfigure, poll). SPI bus is locked for the entire SD card
// transaction (command + data + CRC + response) to prevent flow deck
// SPI1 access on shared-bus boards.
//
// Before the scheduler starts (e.g., FatFS mount from main()), DMA
// falls back to synchronous byte-by-byte and busy-waits use polling.
//
// Reference: http://elm-chan.org/docs/mmc/mmc_e.html

#include "spi_sd.h"
#include "spi_ll.h"
#include "hive_static_config.h"
#include "hive_runtime.h"
#include "hive_timer.h"
#include "hal/hive_hal_event.h"

#if HIVE_ENABLE_SD

#include <string.h>
#include <stdbool.h>

// Debug output - uses platform debug printf
// Enable SD_DEBUG_ENABLE to see initialization details
#define SD_DEBUG_ENABLE 0
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
// Configuration
// ---------------------------------------------------------------------------

#define SD_BUSY_TIMEOUT_MS 500 // 500ms max card busy time

// ---------------------------------------------------------------------------
// Driver State
// ---------------------------------------------------------------------------

static bool s_initialized = false;
static sd_card_type_t s_card_type = SD_CARD_NONE;
static uint32_t s_sector_count = 0;

// DMA completion event (signaled by ISR, waited on by actor via hive_select)
static hive_hal_event_id_t s_dma_event = HIVE_HAL_EVENT_INVALID;

// DMA wait timeout (512 bytes at 328 kHz slow = ~12ms, at 21 MHz fast = ~0.2ms)
#define SD_DMA_TIMEOUT_MS 50

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
// DMA Transfer + Spin-Wait
// ---------------------------------------------------------------------------

// Check if running in actor context (scheduler started).
// Before scheduler, hive_self() returns HIVE_ACTOR_ID_INVALID.
static bool in_actor_context(void) {
    return hive_self() != HIVE_ACTOR_ID_INVALID;
}

// Transfer sector data via DMA with spin-wait.
// DMA is ~23x faster than byte-by-byte (24us vs 700us per 512-byte block at
// 21 MHz) but the transfer is too short to benefit from yielding to the
// scheduler. The meaningful yields happen in wait_ready_yield() where card
// programming takes 10-200ms.
// Caller must hold the SPI bus lock (spi_ll_lock) before calling.
// CS must already be asserted by caller.
// Falls back to byte-by-byte before scheduler starts (e.g., during mount).
static int dma_xfer(void *buf, uint32_t len, spi_ll_dma_dir_t dir) {
    if (s_dma_event == HIVE_HAL_EVENT_INVALID) {
        // DMA not initialized: synchronous fallback
        if (dir == SPI_LL_DIR_RX) {
            spi_recv(buf, len);
        } else {
            spi_send(buf, len);
        }
        return 0;
    }

    // DMA transfer with spin-wait for completion.
    // 512 bytes at 21 MHz = ~24us - too short to justify a context switch.
    spi_ll_dma_start(buf, len, dir);

    uint32_t timeout = 2000000; // ~48ms at 168 MHz (generous for 24us xfer)
    while (!spi_ll_dma_ok() && !spi_ll_dma_error() && timeout > 0) {
        timeout--;
    }

    if (!spi_ll_dma_ok()) {
        SD_DEBUG("DMA %s: %s\n", dir == SPI_LL_DIR_TX ? "TX" : "RX",
                 spi_ll_dma_error() ? "ERROR" : "TIMEOUT");
        spi_ll_dma_cleanup();
        return -1;
    }

    // STM32 SPI DMA requirement: wait for BSY to clear and drain residual
    // RX data before any byte-by-byte SPI access. Without this, the next
    // spi_ll_xfer() may see stale data or corrupt the bus.
    spi_ll_dma_cleanup();
    return 0;
}

// Wait for card ready after a write operation, yielding to the scheduler.
// Releases the SPI bus between polls so the flow deck can read at 250Hz.
// Uses spi_ll_cs_low_poll() (lightweight reconfigure, no RCC reset) instead
// of spi_ll_cs_low() (full RCC reset required for DMA but corrupts MSB).
//
// Before the scheduler starts (FatFS mount), falls back to tight polling
// since hive_sleep() is a no-op and the 500 polls would finish in
// microseconds - not enough time for card programming (~200ms).
static int wait_ready_yield(void) {
    if (!in_actor_context()) {
        // Pre-scheduler: tight poll with CS held (no bus sharing needed)
        spi_ll_cs_low();
        int rc = wait_ready(SD_BUSY_TIMEOUT_MS);
        spi_ll_cs_high();
        return rc;
    }

    uint32_t polls = SD_BUSY_TIMEOUT_MS;

    spi_ll_lock();
    spi_ll_cs_low_poll();
    spi_ll_xfer(0xFF); // Discard stale byte after SPE toggle
    uint8_t r = spi_ll_xfer(0xFF);

    while (r != 0xFF && polls > 0) {
        // Release bus: no dummy clock (avoids clock glitch on next cs_low)
        spi_ll_cs_high_no_dummy();
        spi_ll_unlock();

        // Yield to scheduler for 1ms (flow deck can use SPI1)
        hive_sleep(1000);

        // Reacquire bus: lightweight reconfigure (no RCC reset, no MSB
        // corruption). Flow deck may have changed SPI1 baud rate.
        spi_ll_lock();
        spi_ll_cs_low_poll();
        spi_ll_xfer(0xFF); // Discard stale byte after SPE toggle
        r = spi_ll_xfer(0xFF);
        polls--;
    }

    spi_ll_cs_high();
    spi_ll_unlock();

    return (r == 0xFF) ? 0 : -1;
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

    // Abort any in-progress transfer from a previous boot.
    // If the MCU reset mid-transfer (e.g., st-flash), the card is still
    // powered and waiting for clocks to complete. The card could be in
    // any state: waiting for command, receiving data bytes, sending data,
    // or busy programming. Recovery sequence:
    //
    // 1. Flush 520 bytes with CS LOW to complete any partial data block
    //    (512 data + 2 CRC + padding). If the card was mid-data-receive,
    //    it interprets incoming bytes as data, not commands, so CMD12
    //    won't work until the data phase completes.
    // 2. Send CMD12 (STOP_TRANSMISSION) to abort multi-block operations.
    // 3. Deassert CS.
    // 4. Send 1024 bytes (8192 clocks) with CS HIGH to drain any pending
    //    read data and satisfy the SD spec's 74-clock power-on minimum.
    SD_DEBUG("Aborting stale transfer (flush + CMD12)...\n");

    // Phase 1: Flush 2048 bytes with CS LOW. The card might be in mid-data
    // receive state expecting more data bytes. 2048 bytes covers multiple
    // blocks (4 x 512) plus overhead, ensuring the card finishes any pending
    // data phase regardless of multi-block position.
    spi_ll_cs_low();
    for (int i = 0; i < 2048; i++) {
        spi_ll_xfer(0xFF);
    }

    // Phase 2: Send CMD12 to abort any multi-block operation, then wait
    // for the card to finish programming before deassering CS.
    send_cmd(CMD12, 0);
    wait_ready(500);
    spi_ll_cs_high();

    // Phase 3: 8192 clocks with CS HIGH to drain pending read data
    for (int i = 0; i < 1024; i++) {
        spi_ll_xfer(0xFF);
    }

    // Power-on delay (~100ms at 168 MHz) - card may need time to finish
    // internal operations (flash programming, wear leveling) after recovery.
    for (volatile int d = 0; d < 5000000; d++)
        __asm__("nop");

    // Enter SPI mode (CMD0) with retries. Some cards need many attempts
    // after an unclean shutdown (e.g., power loss during write). Retry with
    // full flush and delay between attempts.
    r1 = 0xFF;
    for (int attempt = 0; attempt < 20 && r1 != R1_IDLE_STATE; attempt++) {
        if (attempt > 0) {
            if (attempt <= 5 || (attempt % 5) == 0) {
                SD_DEBUG("CMD0 attempt %d...\n", attempt + 1);
            }
            // Flush again between retries
            spi_ll_cs_high();
            for (int i = 0; i < 1024; i++) {
                spi_ll_xfer(0xFF);
            }
            for (volatile int d = 0; d < 5000000; d++)
                __asm__("nop"); // ~100ms
        }

        spi_ll_cs_low();
        for (volatile int d = 0; d < 10000; d++)
            __asm__("nop");
        r1 = send_cmd(CMD0, 0);
        spi_ll_cs_high();
        SD_DEBUG("CMD0 response: 0x%02X (expect 0x01)\n", r1);
    }

    if (r1 != R1_IDLE_STATE) {
        SD_DEBUG("CMD0 failed after 20 attempts (last=0x%02X)\n", r1);
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

    // Initialize DMA for sector transfers (ISR signals event on completion)
    s_dma_event = hive_hal_event_create();
    if (s_dma_event != HIVE_HAL_EVENT_INVALID) {
        spi_ll_dma_init(s_dma_event);
    }

    s_initialized = true;
    SD_DEBUG("SD card initialized: %lu sectors (%lu MB)\n", s_sector_count,
             s_sector_count / 2048);
    return 0;
}

int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count) {
    if (!s_initialized || count == 0) {
        return -1;
    }

    // Convert to byte address for v1/v2-SC cards
    uint32_t addr = sector;
    if (s_card_type != SD_CARD_V2_HC) {
        addr *= 512;
    }

    // Lock bus for entire transaction (command + data + CRC).
    // Flow deck checks spi_ll_is_locked() before SPI1 access.
    spi_ll_lock();
    spi_ll_cs_low();

    if (count == 1) {
        // Single block read
        uint8_t r = send_cmd(CMD17, addr);
        if (r != 0) {
            SD_DEBUG("CMD17 fail: r=0x%02X s=%lu\n", r, (unsigned long)sector);
            spi_ll_cs_high();
            spi_ll_unlock();
            return -1;
        }

        if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
            SD_DEBUG("R token fail: s=%lu\n", (unsigned long)sector);
            spi_ll_cs_high();
            spi_ll_unlock();
            return -1;
        }

        if (dma_xfer(buf, 512, SPI_LL_DIR_RX) != 0) {
            SD_DEBUG("R DMA fail: s=%lu\n", (unsigned long)sector);
            spi_ll_cs_high();
            spi_ll_unlock();
            return -1;
        }
        spi_ll_xfer(0xFF); // CRC
        spi_ll_xfer(0xFF);
    } else {
        // Multi-block read
        if (send_cmd(CMD18, addr) != 0) {
            spi_ll_cs_high();
            spi_ll_unlock();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
                send_cmd(CMD12, 0);
                spi_ll_cs_high();
                spi_ll_unlock();
                return -1;
            }

            if (dma_xfer(buf + i * 512, 512, SPI_LL_DIR_RX) != 0) {
                send_cmd(CMD12, 0);
                spi_ll_cs_high();
                spi_ll_unlock();
                return -1;
            }
            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);
        }

        send_cmd(CMD12, 0);
    }

    spi_ll_cs_high();
    spi_ll_unlock();
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

    if (count == 1) {
        // Single block write with retry on data response failure.
        // Rapid same-sector rewrites (FatFS file creation) can get dresp=0x00
        // when the card's internal state hasn't fully settled. Recovery is
        // conservative: just deassert CS and wait for card ready before retry.
        // IMPORTANT: Do NOT flush extra bytes with CS LOW during recovery -
        // that can put the card in an unrecoverable state requiring power cycle.
        int ok = 0;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            // Lock bus for entire transaction (command + data + response).
            // Flow deck checks spi_ll_is_locked() before SPI1 access.
            spi_ll_lock();
            spi_ll_cs_low();

            uint8_t r1 = send_cmd(CMD24, addr);
            if (r1 != 0) {
                SD_DEBUG("W CMD24 fail: r=0x%02X s=%lu\n", r1,
                         (unsigned long)sector);
                spi_ll_cs_high();
                spi_ll_unlock();
                continue;
            }

            spi_ll_xfer(0xFF);               // Gap
            spi_ll_xfer(TOKEN_SINGLE_BLOCK); // Data token
            if (dma_xfer((void *)buf, 512, SPI_LL_DIR_TX) != 0) {
                SD_DEBUG("W DMA fail: s=%lu\n", (unsigned long)sector);
                spi_ll_cs_high();
                spi_ll_unlock();
                continue;
            }

            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);

            // Check data response
            uint8_t resp = spi_ll_xfer(0xFF);
            if ((resp & 0x1F) != 0x05) {
                SD_DEBUG("W dresp: 0x%02X s=%lu try=%d\n", resp,
                         (unsigned long)sector, attempt);
                // Conservative recovery: deassert CS, wait for card to finish
                // any internal operations, then retry. No extra clocking with
                // CS LOW (that risks corrupting card state).
                spi_ll_cs_high();
                spi_ll_unlock();
                wait_ready_yield();
                continue;
            }

            // Deassert CS and unlock before yielding busy-wait (frees SPI
            // for flow deck during card programming time)
            spi_ll_cs_high();
            spi_ll_unlock();
            if (wait_ready_yield() != 0) {
                SD_DEBUG("W busy fail: s=%lu\n", (unsigned long)sector);
                continue;
            }
            ok = 1;
        }
        if (!ok) {
            return -1;
        }
    } else {
        // Multi-block write
        spi_ll_lock();
        spi_ll_cs_low();
        if (send_cmd(CMD25, addr) != 0) {
            spi_ll_cs_high();
            spi_ll_unlock();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            spi_ll_xfer(0xFF);              // Gap
            spi_ll_xfer(TOKEN_MULTI_BLOCK); // Data token
            if (dma_xfer((void *)(buf + i * 512), 512, SPI_LL_DIR_TX) != 0) {
                spi_ll_cs_high();
                spi_ll_unlock();
                return -1;
            }
            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);

            // Check data response
            uint8_t resp = spi_ll_xfer(0xFF);
            if ((resp & 0x1F) != 0x05) {
                spi_ll_xfer(TOKEN_STOP_TRAN);
                spi_ll_cs_high();
                spi_ll_unlock();
                return -1;
            }

            // Deassert CS and unlock before yielding busy-wait
            spi_ll_cs_high();
            spi_ll_unlock();
            if (wait_ready_yield() != 0) {
                return -1;
            }
            // Reassert CS for next block or stop command
            spi_ll_lock();
            spi_ll_cs_low();
        }

        spi_ll_xfer(TOKEN_STOP_TRAN);
        spi_ll_cs_high();
        spi_ll_unlock();
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
