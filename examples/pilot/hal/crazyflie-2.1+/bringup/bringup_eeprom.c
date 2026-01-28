// Crazyflie 2.1+ Bring-Up - Configuration EEPROM Test
//
// Tests the AT24C64 I2C EEPROM (8 KB) used for configuration storage.
// The EEPROM is on I2C1 (PB6=SCL, PB7=SDA) at address 0x50.
//
// Test sequence:
// 1. Initialize I2C1
// 2. Detect EEPROM at 0x50 (or scan 0x50-0x57)
// 3. Write test pattern to a safe location (last page)
// 4. Read back and verify
//
// WARNING: This test writes to the EEPROM! Uses last page to avoid
// overwriting important configuration data.

#include "bringup_eeprom.h"
#include "bringup_i2c1.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// EEPROM configuration
#define EEPROM_ADDR_BASE 0x50 // Base I2C address (A0-A2 = 0)
#define EEPROM_SIZE 8192      // 8 KB (AT24C64)
#define EEPROM_PAGE_SIZE 32   // 32 bytes per page
#define EEPROM_TEST_ADDR (EEPROM_SIZE - EEPROM_PAGE_SIZE) // Last page

// Test pattern
#define TEST_PATTERN_SIZE 16
static const uint8_t s_test_pattern[TEST_PATTERN_SIZE] = {
    0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE,
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};

// Timing
static uint32_t get_ticks(void) {
    return swo_get_ticks();
}

// Simple delay
static void delay_ms(uint32_t ms) {
    uint32_t start = get_ticks();
    while ((get_ticks() - start) < ms)
        ;
}

// ----------------------------------------------------------------------------
// EEPROM Operations
// ----------------------------------------------------------------------------

static uint8_t s_eeprom_addr = 0;

// Write to EEPROM (address is 16-bit for AT24C64)
static bool eeprom_write(uint16_t mem_addr, const uint8_t *data, size_t len) {
    // EEPROM write: [I2C addr][mem addr high][mem addr low][data...]
    uint8_t buf[2 + TEST_PATTERN_SIZE];
    buf[0] = (uint8_t)(mem_addr >> 8);
    buf[1] = (uint8_t)(mem_addr & 0xFF);
    memcpy(&buf[2], data, len);

    if (!i2c1_write(s_eeprom_addr, buf, 2 + len)) {
        return false;
    }

    // Wait for write cycle (max 5ms for AT24C64)
    delay_ms(6);
    return true;
}

// Read from EEPROM
static bool eeprom_read(uint16_t mem_addr, uint8_t *data, size_t len) {
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)(mem_addr >> 8);
    addr_buf[1] = (uint8_t)(mem_addr & 0xFF);

    return i2c1_write_read(s_eeprom_addr, addr_buf, 2, data, len);
}

// ----------------------------------------------------------------------------
// EEPROM Test Implementation
// ----------------------------------------------------------------------------

void eeprom_test_init(void) {
    i2c1_init();
}

bool eeprom_test_detect(uint8_t *addr_out) {
    swo_puts("[EEPROM] Scanning I2C1 for EEPROM (0x50-0x57)...\n");

    for (uint8_t addr = EEPROM_ADDR_BASE; addr <= EEPROM_ADDR_BASE + 7;
         addr++) {
        if (i2c1_probe(addr)) {
            swo_printf("[EEPROM] Found EEPROM at 0x%02X... OK\n", addr);
            s_eeprom_addr = addr;
            *addr_out = addr;
            return true;
        }
    }

    swo_puts("[EEPROM] No EEPROM found... FAIL\n");
    *addr_out = 0;
    return false;
}

bool eeprom_test_write(uint32_t *time_ms) {
    swo_printf("[EEPROM] Writing %d bytes to address 0x%04X... ",
               TEST_PATTERN_SIZE, EEPROM_TEST_ADDR);

    uint32_t start = get_ticks();

    if (!eeprom_write(EEPROM_TEST_ADDR, s_test_pattern, TEST_PATTERN_SIZE)) {
        swo_puts("FAIL\n");
        *time_ms = 0;
        return false;
    }

    *time_ms = get_ticks() - start;
    swo_printf("OK (%u ms)\n", *time_ms);
    return true;
}

bool eeprom_test_read_verify(uint32_t *time_ms) {
    swo_printf("[EEPROM] Reading and verifying %d bytes... ",
               TEST_PATTERN_SIZE);

    uint32_t start = get_ticks();

    uint8_t read_buf[TEST_PATTERN_SIZE];
    if (!eeprom_read(EEPROM_TEST_ADDR, read_buf, TEST_PATTERN_SIZE)) {
        swo_puts("FAIL (read error)\n");
        *time_ms = 0;
        return false;
    }

    *time_ms = get_ticks() - start;

    // Verify
    int errors = 0;
    for (int i = 0; i < TEST_PATTERN_SIZE; i++) {
        if (read_buf[i] != s_test_pattern[i]) {
            if (errors < 3) {
                swo_printf(
                    "\n[EEPROM]   Mismatch at %d: got 0x%02X, expected 0x%02X",
                    i, read_buf[i], s_test_pattern[i]);
            }
            errors++;
        }
    }

    if (errors > 0) {
        swo_printf("\nFAIL (%d errors)\n", errors);
        return false;
    }

    swo_printf("OK (%u ms)\n", *time_ms);
    return true;
}

bool eeprom_run_test(eeprom_test_results_t *results) {
    memset(results, 0, sizeof(*results));

    swo_puts("\n=== EEPROM Test ===\n");
    swo_puts("[EEPROM] Configuration EEPROM (I2C1: PB6=SCL, PB7=SDA)\n");

    // Initialize I2C1
    swo_puts("[EEPROM] Initializing I2C1... ");
    eeprom_test_init();
    results->i2c_init_ok = true;
    swo_puts("OK\n");

    // Detect EEPROM
    results->device_found = eeprom_test_detect(&results->device_addr);
    if (!results->device_found) {
        swo_puts("[EEPROM] EEPROM not detected\n");
        return false;
    }

    // Write test
    results->write_ok = eeprom_test_write(&results->write_time_ms);
    if (!results->write_ok) {
        return false;
    }

    // Read and verify
    results->read_ok = eeprom_test_read_verify(&results->read_time_ms);
    results->verify_ok = results->read_ok;

    if (results->verify_ok) {
        swo_puts("[EEPROM] All tests passed!\n");
    }

    return results->verify_ok;
}
