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
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// EEPROM configuration
#define EEPROM_ADDR_BASE 0x50 // Base I2C address (A0-A2 = 0)
#define EEPROM_SIZE 8192      // 8 KB (AT24C64)
#define EEPROM_PAGE_SIZE 32   // 32 bytes per page
#define EEPROM_TEST_ADDR (EEPROM_SIZE - EEPROM_PAGE_SIZE) // Last page

// I2C1 configuration (PB6=SCL, PB7=SDA)
// APB1 = 42 MHz, I2C clock = 400 kHz (fast mode)
#define I2C_CCR_VALUE 53   // 42 MHz / (2 * 400 kHz)
#define I2C_TRISE_VALUE 13 // (42 MHz * 300 ns) + 1
#define I2C_TIMEOUT 100000

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
// I2C1 Low-Level (separate from I2C3 used for sensors)
// ----------------------------------------------------------------------------

static bool i2c1_wait_flag(volatile uint32_t *reg, uint32_t flag, bool set) {
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        bool is_set = (*reg & flag) != 0;
        if (is_set == set) {
            return true;
        }
    }
    return false;
}

static void i2c1_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Reset I2C1
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

    // Configure PB6 (SCL) as AF4 open-drain
    GPIOB->MODER &= ~GPIO_MODER_MODER6;
    GPIOB->MODER |= GPIO_MODER_MODER6_1;      // AF mode
    GPIOB->OTYPER |= GPIO_OTYPER_OT_6;        // Open-drain
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6; // High speed
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR6;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR6_0; // Pull-up
    GPIOB->AFR[0] &= ~(0xFU << (6 * 4));
    GPIOB->AFR[0] |= (4U << (6 * 4)); // AF4 = I2C1

    // Configure PB7 (SDA) as AF4 open-drain
    GPIOB->MODER &= ~GPIO_MODER_MODER7;
    GPIOB->MODER |= GPIO_MODER_MODER7_1;
    GPIOB->OTYPER |= GPIO_OTYPER_OT_7;
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7;
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0;
    GPIOB->AFR[0] &= ~(0xFU << (7 * 4));
    GPIOB->AFR[0] |= (4U << (7 * 4));

    // Configure I2C1
    I2C1->CR1 = 0;                          // Disable
    I2C1->CR2 = 42;                         // APB1 frequency in MHz
    I2C1->CCR = I2C_CCR_VALUE | I2C_CCR_FS; // Fast mode
    I2C1->TRISE = I2C_TRISE_VALUE;
    I2C1->CR1 = I2C_CR1_PE; // Enable
}

static bool i2c1_probe(uint8_t addr) {
    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write mode)
    I2C1->DR = (addr << 1) | 0;

    // Wait for address ACK
    uint32_t timeout = I2C_TIMEOUT;
    while (timeout--) {
        uint32_t sr1 = I2C1->SR1;
        if (sr1 & I2C_SR1_AF) {
            I2C1->SR1 = ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        if (sr1 & I2C_SR1_ADDR) {
            (void)I2C1->SR2;
            I2C1->CR1 |= I2C_CR1_STOP;
            return true;
        }
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
}

static bool i2c1_write(uint8_t addr, const uint8_t *data, size_t len) {
    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    // Send address (write)
    I2C1->DR = (addr << 1) | 0;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }
    (void)I2C1->SR2;

    // Send data
    for (size_t i = 0; i < len; i++) {
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true)) {
            I2C1->CR1 |= I2C_CR1_STOP;
            return false;
        }
        I2C1->DR = data[i];
    }

    // Wait for transfer complete
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
        I2C1->CR1 |= I2C_CR1_STOP;
        return false;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return true;
}

// Simple delay
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 42;
    while (count--)
        ;
}

static bool i2c1_write_read(uint8_t addr, const uint8_t *wdata, size_t wlen,
                            uint8_t *rdata, size_t rlen) {
    // Generate start
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (write)
    I2C1->DR = (addr << 1) | 0;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }
    (void)I2C1->SR2;

    // Send write data
    for (size_t i = 0; i < wlen; i++) {
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_TXE, true)) {
            goto error;
        }
        I2C1->DR = wdata[i];
    }

    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
        goto error;
    }

    // Repeated start
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_START;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_SB, true)) {
        goto error;
    }

    // Send address (read)
    I2C1->DR = (addr << 1) | 1;
    if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, true)) {
        goto error;
    }

    if (rlen == 1) {
        // N=1: Disable ACK before clearing ADDR, then set STOP
        I2C1->CR1 &= ~I2C_CR1_ACK;
        (void)I2C1->SR2; // Clear ADDR
        I2C1->CR1 |= I2C_CR1_STOP;

        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        rdata[0] = (uint8_t)I2C1->DR;
    } else if (rlen == 2) {
        // N=2: Use POS bit per STM32 reference manual
        I2C1->CR1 |= I2C_CR1_POS;
        (void)I2C1->SR2; // Clear ADDR
        I2C1->CR1 &= ~I2C_CR1_ACK;

        // Wait for BTF (both bytes received)
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }

        // Set STOP before reading
        I2C1->CR1 |= I2C_CR1_STOP;
        rdata[0] = (uint8_t)I2C1->DR;
        rdata[1] = (uint8_t)I2C1->DR;

        // Clear POS
        I2C1->CR1 &= ~I2C_CR1_POS;
    } else {
        // N>2: Per RM0090 section 27.3.3
        (void)I2C1->SR2; // Clear ADDR

        // Read bytes 0 to N-3 normally
        for (size_t i = 0; i < rlen - 2; i++) {
            if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
                goto error;
            }
            rdata[i] = (uint8_t)I2C1->DR;
        }

        // Wait for BTF (byte N-2 in DR, byte N-1 in shift register)
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_BTF, true)) {
            goto error;
        }
        // Clear ACK, set STOP
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_STOP;
        // Read DataN-2 (shifts DataN-1 to DR)
        rdata[rlen - 2] = (uint8_t)I2C1->DR;
        // Wait for DataN-1 to be ready
        if (!i2c1_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, true)) {
            goto error;
        }
        // Read DataN-1
        rdata[rlen - 1] = (uint8_t)I2C1->DR;
    }

    // Wait for STOP bit to be cleared by hardware
    if (!i2c1_wait_flag(&I2C1->CR1, I2C_CR1_STOP, false)) {
        // STOP didn't clear - do full software reset with bus recovery
        I2C1->CR1 = 0; // Disable
        RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
        delay_us(10);
        RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

        // Bus recovery: toggle SCL 9 times while I2C is disabled
        GPIOB->MODER &= ~GPIO_MODER_MODER6;
        GPIOB->MODER |= GPIO_MODER_MODER6_0; // Output mode
        GPIOB->BSRR = GPIO_BSRR_BS_6;        // Start high
        for (int i = 0; i < 9; i++) {
            GPIOB->BSRR = GPIO_BSRR_BR_6; // SCL low
            delay_us(5);
            GPIOB->BSRR = GPIO_BSRR_BS_6; // SCL high
            delay_us(5);
        }
        // Reconfigure PB6 as AF
        GPIOB->MODER &= ~GPIO_MODER_MODER6;
        GPIOB->MODER |= GPIO_MODER_MODER6_1;
        // Reconfigure PB7 as AF
        GPIOB->MODER &= ~GPIO_MODER_MODER7;
        GPIOB->MODER |= GPIO_MODER_MODER7_1;

        // Re-configure I2C
        I2C1->CR2 = 42;
        I2C1->CCR = I2C_CCR_VALUE | I2C_CCR_FS;
        I2C1->TRISE = I2C_TRISE_VALUE;
        I2C1->CR1 = I2C_CR1_PE;
    }
    // Wait for BUSY flag cleared
    i2c1_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false);
    return true;

error:
    I2C1->CR1 &= ~I2C_CR1_POS; // Ensure POS is cleared
    I2C1->CR1 |= I2C_CR1_STOP;
    i2c1_wait_flag(&I2C1->CR1, I2C_CR1_STOP, false);
    i2c1_wait_flag(&I2C1->SR2, I2C_SR2_BUSY, false);
    return false;
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
