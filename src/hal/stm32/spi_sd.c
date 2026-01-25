// SPI SD Card Driver - SD Protocol Implementation
//
// Platform-independent SD card protocol over SPI.
// Board-specific SPI functions provided via spi_ll.h interface.
//
// Reference: http://elm-chan.org/docs/mmc/mmc_e.html

#include "spi_sd.h"
#include "spi_ll.h"
#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include <string.h>
#include <stdbool.h>

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
// Driver State
// ---------------------------------------------------------------------------

static bool s_initialized = false;
static sd_card_type_t s_card_type = SD_CARD_NONE;
static uint32_t s_sector_count = 0;

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

    // Wait for card ready
    if (cmd != CMD0 && cmd != CMD12) {
        wait_ready(500);
    }

    // Send command
    spi_send(buf, 6);

    // Wait for response (not 0xFF)
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 10; i++) {
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

    // Initialize SPI hardware at low speed
    spi_ll_init();
    spi_ll_set_slow();

    // CS high, send 80+ clocks to enter native mode
    spi_ll_cs_high();
    for (int i = 0; i < 10; i++) {
        spi_ll_xfer(0xFF);
    }

    // Enter SPI mode (CMD0)
    spi_ll_cs_low();
    r1 = send_cmd(CMD0, 0);
    spi_ll_cs_high();

    if (r1 != R1_IDLE_STATE) {
        return -1;
    }

    // Check voltage (CMD8)
    spi_ll_cs_low();
    r1 = send_cmd(CMD8, 0x1AA);
    if (r1 == R1_IDLE_STATE) {
        // SD v2.0+
        spi_recv(ocr, 4);
        spi_ll_cs_high();

        if (ocr[2] != 0x01 || ocr[3] != 0xAA) {
            return -1; // Voltage not accepted
        }

        // Wait for card ready (ACMD41 with HCS=1)
        for (int i = 0; i < 1000; i++) {
            spi_ll_cs_low();
            r1 = send_acmd(ACMD41, 0x40000000);
            spi_ll_cs_high();
            if (r1 == 0)
                break;
        }

        if (r1 != 0) {
            return -1;
        }

        // Check CCS bit (high capacity)
        spi_ll_cs_low();
        r1 = send_cmd(CMD58, 0);
        spi_recv(ocr, 4);
        spi_ll_cs_high();

        s_card_type = (ocr[0] & 0x40) ? SD_CARD_V2_HC : SD_CARD_V2_SC;
    } else {
        spi_ll_cs_high();
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
            return -1;
        }

        if (r1 != 0) {
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

    s_initialized = true;
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

    spi_ll_cs_low();

    if (count == 1) {
        // Single block read
        if (send_cmd(CMD17, addr) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        spi_recv(buf, 512);
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

            spi_recv(buf + i * 512, 512);
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
        spi_send(buf, 512);
        spi_ll_xfer(0xFF); // CRC
        spi_ll_xfer(0xFF);

        // Check data response
        uint8_t resp = spi_ll_xfer(0xFF);
        if ((resp & 0x1F) != 0x05) {
            spi_ll_cs_high();
            return -1;
        }

        wait_ready(500);
    } else {
        // Multi-block write
        if (send_cmd(CMD25, addr) != 0) {
            spi_ll_cs_high();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            spi_ll_xfer(0xFF);              // Gap
            spi_ll_xfer(TOKEN_MULTI_BLOCK); // Data token
            spi_send(buf + i * 512, 512);
            spi_ll_xfer(0xFF); // CRC
            spi_ll_xfer(0xFF);

            // Check data response
            uint8_t resp = spi_ll_xfer(0xFF);
            if ((resp & 0x1F) != 0x05) {
                spi_ll_xfer(TOKEN_STOP_TRAN);
                spi_ll_cs_high();
                return -1;
            }

            wait_ready(500);
        }

        spi_ll_xfer(TOKEN_STOP_TRAN);
        wait_ready(500);
    }

    spi_ll_cs_high();
    return 0;
}

int spi_sd_sync(void) {
    if (!s_initialized) {
        return -1;
    }

    spi_ll_cs_low();
    int result = wait_ready(500);
    spi_ll_cs_high();

    return result;
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
