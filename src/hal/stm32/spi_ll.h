// SPI Low-Level Interface
//
// Board-specific SPI implementation must provide these functions.
// Used by spi_sd.c for SD card communication.

#ifndef SPI_LL_H
#define SPI_LL_H

#include <stdint.h>

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

#endif // SPI_LL_H
