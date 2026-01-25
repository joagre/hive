// Crazyflie 2.1+ Bring-Up - Configuration EEPROM Test
//
// Tests the onboard I2C EEPROM used for configuration storage:
// - AT24C64 (8 KB) at address 0x50 on I2C1
// - Stores radio address, calibration data, etc.
//
// Reference: https://www.bitcraze.io/documentation/

#ifndef BRINGUP_EEPROM_H
#define BRINGUP_EEPROM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// EEPROM test results
typedef struct {
    bool i2c_init_ok;    // I2C1 initialized
    bool device_found;   // EEPROM responds at expected address
    bool write_ok;       // Write test pattern succeeded
    bool read_ok;        // Read test pattern succeeded
    bool verify_ok;      // Data integrity verified
    uint8_t device_addr; // Detected I2C address
    uint32_t write_time_ms;
    uint32_t read_time_ms;
} eeprom_test_results_t;

// Initialize EEPROM test (configures I2C1)
void eeprom_test_init(void);

// Run all EEPROM tests
// Returns true if all tests passed
bool eeprom_run_test(eeprom_test_results_t *results);

// Individual tests
bool eeprom_test_detect(uint8_t *addr_out);
bool eeprom_test_write(uint32_t *time_ms);
bool eeprom_test_read_verify(uint32_t *time_ms);

#endif // BRINGUP_EEPROM_H
