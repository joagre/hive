// Crazyflie 2.1+ Bring-Up - SD Card Test
//
// Tests SD card via SPI (Micro SD Card Deck):
// - SPI3 initialization (PB3=SCK, PB4=MISO, PB5=MOSI, PB6=CS)
// - Card detection and identification
// - Write/read verification
//
// Requires: Micro SD Card Deck attached with FAT32 formatted card

#ifndef BRINGUP_SD_H
#define BRINGUP_SD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// SD card test results
typedef struct {
    bool spi_init_ok;      // SPI peripheral initialized
    bool card_present;     // Card detected
    bool card_init_ok;     // Card responded to init sequence
    bool write_ok;         // Write test data succeeded
    bool read_ok;          // Read test data succeeded
    bool verify_ok;        // Data integrity verified
    uint32_t card_type;    // 0=unknown, 1=SDv1, 2=SDv2, 3=SDHC
    uint32_t card_size_mb; // Card capacity in MB (if detected)
    uint32_t write_time_ms;
    uint32_t read_time_ms;
} sd_test_results_t;

// Initialize SD test (configures SPI3)
void sd_test_init(void);

// Run all SD card tests
// Returns true if all tests passed, false if deck not present or tests failed
bool sd_run_test(sd_test_results_t *results);

// Individual tests (for debugging)
bool sd_test_spi_init(void);
bool sd_test_card_detect(uint32_t *card_type, uint32_t *card_size_mb);
bool sd_test_write(uint32_t *time_ms);
bool sd_test_read_verify(uint32_t *time_ms);

#endif // BRINGUP_SD_H
