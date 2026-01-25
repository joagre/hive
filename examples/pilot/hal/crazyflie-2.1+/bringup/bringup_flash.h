// Crazyflie 2.1+ Bring-Up - Internal Flash Test
//
// Tests internal flash storage for telemetry logging:
// - Flash erase (sector 8, 128KB)
// - Flash write (ring buffer -> flash)
// - Flash read verification (pread)

#ifndef BRINGUP_FLASH_H
#define BRINGUP_FLASH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Flash test results
typedef struct {
    bool erase_ok;  // Sector erase succeeded
    bool write_ok;  // Write test pattern succeeded
    bool read_ok;   // Read back succeeded
    bool verify_ok; // Data integrity verified
    uint32_t bytes_written;
    uint32_t bytes_read;
    uint32_t write_time_ms;
    uint32_t read_time_ms;
} flash_test_results_t;

// Initialize flash test (configures flash peripheral)
void flash_test_init(void);

// Run all flash tests
// Returns true if all tests passed
bool flash_run_test(flash_test_results_t *results);

// Individual tests (for debugging)
bool flash_test_erase(void);
bool flash_test_write(uint32_t *bytes_written, uint32_t *time_ms);
bool flash_test_read_verify(uint32_t *bytes_read, uint32_t *time_ms);

#endif // BRINGUP_FLASH_H
