// SPI SD Card Driver - Header
//
// Low-level SPI driver for SD card communication.
// Implements the interface required by FatFS diskio.c.

#ifndef SPI_SD_H
#define SPI_SD_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// SPI port configuration (set via mount table)
typedef struct {
    uint8_t spi_id;  // 1=SPI1, 2=SPI2, 3=SPI3
    uint8_t cs_port; // 0=GPIOA, 1=GPIOB, etc.
    uint8_t cs_pin;  // 0-15
} spi_sd_config_t;

// ---------------------------------------------------------------------------
// Driver API (called by diskio.c)
// ---------------------------------------------------------------------------

// Configure SPI port (must be called before init)
void spi_sd_configure(const spi_sd_config_t *config);

// Check if SD card is present (card detect signal)
int spi_sd_is_present(void);

// Initialize SD card (SPI mode)
// Returns: 0 on success, -1 on error
int spi_sd_init(void);

// Read blocks from SD card
// Returns: 0 on success, -1 on error
int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count);

// Write blocks to SD card
// Returns: 0 on success, -1 on error
int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count);

// Sync (wait for write completion)
// Returns: 0 on success, -1 on error
int spi_sd_sync(void);

// Get total sector count
uint32_t spi_sd_get_sector_count(void);

// ---------------------------------------------------------------------------
// Card Types
// ---------------------------------------------------------------------------

typedef enum {
    SD_CARD_NONE = 0,
    SD_CARD_V1,    // SD v1.x (byte addressing)
    SD_CARD_V2_SC, // SD v2+ Standard Capacity (byte addressing)
    SD_CARD_V2_HC, // SD v2+ High Capacity (block addressing)
} sd_card_type_t;

// Get detected card type
sd_card_type_t spi_sd_get_card_type(void);

#endif // SPI_SD_H
