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
// Full SPI peripheral reset via RCC - required before DMA transfers.
// Corrupts MSB of first byte read (0xFF reads as 0x7F).
void spi_ll_cs_low(void);

// Assert chip select (drive low) - lightweight reconfiguration.
// Disables SPE, writes CR1, re-enables SPE. No RCC reset, no MSB
// corruption. Safe for byte-by-byte polling but NOT for DMA transfers.
// Used by wait_ready_yield() after yielding the bus to the flow deck.
void spi_ll_cs_low_poll(void);

// Deassert chip select (drive high) with dummy clock for multi-slave bus
void spi_ll_cs_high(void);

// Deassert chip select without dummy clock (for busy polling between yields)
void spi_ll_cs_high_no_dummy(void);

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
// event_id: HAL event signaled by ISR on DMA completion. The ISR sets
// internal completion flags (checked via spi_ll_dma_ok/spi_ll_dma_error)
// and signals this event. Currently polled via spin-wait, not event_wait.
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

// Check if last DMA transfer had an error (vs timeout).
// Returns true if DMA reported a transfer/FIFO error.
bool spi_ll_dma_error(void);

// Clean up SPI state after DMA transfer.
// Waits for BSY flag to clear and drains residual RX data.
// Must be called after spi_ll_dma_ok() before any byte-by-byte SPI access.
void spi_ll_dma_cleanup(void);

// ---------------------------------------------------------------------------
// SPI Bus Locking (for shared bus protection)
// ---------------------------------------------------------------------------
// On boards where SPI is shared between SD card and other devices (e.g.
// flow deck), the bus must be locked for the entire SD card transaction
// (command + data + CRC + response), not just during DMA. Other devices
// check spi_ll_is_locked() and skip SPI access when locked.
// Simple boolean flags - no yield, no blocking.

// Set bus locked flag
void spi_ll_lock(void);

// Clear bus locked flag
void spi_ll_unlock(void);

// Check if bus is locked (other devices should skip SPI access)
bool spi_ll_is_locked(void);

#endif // SPI_LL_H
