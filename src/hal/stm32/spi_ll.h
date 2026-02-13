// SPI Low-Level Interface
//
// Board-specific SPI implementation must provide these functions.
// Used by spi_sd.c for SD card communication.

#ifndef SPI_LL_H
#define SPI_LL_H

#include <stdint.h>
#include <stdbool.h>
#include "hive_types.h"

// ---------------------------------------------------------------------------
// Low-Level SPI Functions (implemented by board BSP)
// ---------------------------------------------------------------------------

// Initialize SPI peripheral and GPIO pins
// Called once during SD card initialization
void spi_ll_init(void);

// Set SPI clock to slow speed (~100-400 kHz) for card initialization
void spi_ll_set_slow(void);

// Set SPI clock to fast speed (up to 25 MHz) for data transfer
void spi_ll_set_fast(void);

// Assert chip select (drive low)
void spi_ll_cs_low(void);

// Deassert chip select (drive high)
void spi_ll_cs_high(void);

// Transfer single byte (full duplex)
// Sends 'data' and returns received byte
uint8_t spi_ll_xfer(uint8_t data);

// ---------------------------------------------------------------------------
// DMA Bulk Transfer (implemented by board BSP)
// ---------------------------------------------------------------------------

// DMA transfer direction
typedef enum {
    SPI_LL_DIR_TX, // Memory -> SPI (write to card)
    SPI_LL_DIR_RX, // SPI -> Memory (read from card)
} spi_ll_dma_dir_t;

// Initialize DMA streams for SPI SD transfers.
// event_id: HAL event to signal from ISR when DMA completes.
// Called once during SD card initialization, after spi_ll_init().
void spi_ll_dma_init(hive_hal_event_id_t event_id);

// Start a DMA transfer (returns immediately).
// buf: data buffer (TX: source data, RX: destination buffer)
// len: number of bytes (typically 512 for one sector)
// dir: transfer direction
// SPI is full-duplex so both TX and RX DMA streams run for every transfer.
// For TX, RX captures discarded bytes. For RX, TX sends 0xFF.
void spi_ll_dma_start(void *buf, uint32_t len, spi_ll_dma_dir_t dir);

// Check if last DMA transfer completed successfully.
// Returns true if transfer finished without error.
bool spi_ll_dma_ok(void);

// ---------------------------------------------------------------------------
// SPI Bus Locking (for shared bus protection during DMA)
// ---------------------------------------------------------------------------
// On boards where SPI is shared between SD card and other devices (e.g.
// flow deck), the bus must be locked during DMA transfers to prevent
// concurrent access. These are simple boolean flags - no yield, no blocking.

// Set bus locked flag
void spi_ll_lock(void);

// Clear bus locked flag
void spi_ll_unlock(void);

// Check if bus is locked (other devices should skip SPI access)
bool spi_ll_is_locked(void);

#endif // SPI_LL_H
