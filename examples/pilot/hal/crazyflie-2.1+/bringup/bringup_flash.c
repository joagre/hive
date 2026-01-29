// Crazyflie 2.1+ Bring-Up - Internal Flash Test
//
// Tests internal flash storage on sector 8 (128KB at 0x08080000).
// This is the same sector used for /log in the pilot firmware.
//
// Test sequence:
// 1. Erase sector 8
// 2. Write test pattern (1KB blocks up to 4KB)
// 3. Read back and verify
//
// WARNING: This test erases sector 8! Any existing log data will be lost.

#include "bringup_flash.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// Flash configuration for log sector
#define FLASH_LOG_BASE 0x08080000UL // Sector 8 base address
#define FLASH_LOG_SIZE 131072UL     // 128KB
#define FLASH_LOG_SECTOR 8

// Test parameters
#define TEST_BLOCK_SIZE 256  // Write block size (must be multiple of 4)
#define TEST_TOTAL_SIZE 4096 // Total bytes to write/verify
#define TEST_PATTERN 0xA5    // Fill pattern

// Flash register definitions
#define FLASH_BASE_ADDR 0x40023C00UL

typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t OPTCR;
} FLASH_Regs;

#define FLASH_REGS ((FLASH_Regs *)FLASH_BASE_ADDR)

// Flash key values
#define FLASH_KEY1 0x45670123UL
#define FLASH_KEY2 0xCDEF89ABUL

// Flash status register bits
#define FLASH_SR_BSY (1UL << 16)
#define FLASH_SR_PGSERR (1UL << 7)
#define FLASH_SR_PGPERR (1UL << 6)
#define FLASH_SR_PGAERR (1UL << 5)
#define FLASH_SR_WRPERR (1UL << 4)
#define FLASH_SR_OPERR (1UL << 1)
#define FLASH_SR_EOP (1UL << 0)
#define FLASH_SR_ERRORS                                                      \
    (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR | FLASH_SR_WRPERR | \
     FLASH_SR_OPERR)

// Flash control register bits
#define FLASH_CR_PG (1UL << 0)
#define FLASH_CR_SER (1UL << 1)
#define FLASH_CR_SNB_Pos 3
#define FLASH_CR_PSIZE_1 (1UL << 9) // 32-bit program
#define FLASH_CR_STRT (1UL << 16)
#define FLASH_CR_LOCK (1UL << 31)

// Timing (SysTick based, configured in bringup_swo.c)
static uint32_t get_ticks(void) {
    return swo_get_ticks();
}

// ----------------------------------------------------------------------------
// Flash Low-Level Operations
// ----------------------------------------------------------------------------

static void flash_unlock(void) {
    if (FLASH_REGS->CR & FLASH_CR_LOCK) {
        FLASH_REGS->KEYR = FLASH_KEY1;
        FLASH_REGS->KEYR = FLASH_KEY2;
    }
}

static void flash_lock(void) {
    FLASH_REGS->CR |= FLASH_CR_LOCK;
}

static void flash_wait_bsy(void) {
    while (FLASH_REGS->SR & FLASH_SR_BSY)
        ;
}

static void flash_clear_errors(void) {
    FLASH_REGS->SR = FLASH_SR_ERRORS;
}

static bool flash_erase_sector(uint8_t sector) {
    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    FLASH_REGS->CR = FLASH_CR_SER | ((uint32_t)sector << FLASH_CR_SNB_Pos);
    FLASH_REGS->CR |= FLASH_CR_STRT;

    flash_wait_bsy();

    bool ok = (FLASH_REGS->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

// Program flash in 32-bit words (must run from RAM for safety)
__attribute__((section(".ramfunc"), noinline)) static void
flash_program_words_ram(uint32_t addr, const uint32_t *data, uint32_t words) {
    FLASH_REGS->CR = FLASH_CR_PG | FLASH_CR_PSIZE_1;

    for (uint32_t i = 0; i < words; i++) {
        *(volatile uint32_t *)(addr + i * 4) = data[i];
        while (FLASH_REGS->SR & FLASH_SR_BSY)
            ;
    }

    FLASH_REGS->CR &= ~FLASH_CR_PG;
}

static bool flash_write_block(uint32_t addr, const void *data, uint32_t len) {
    if (len == 0 || (len & 3) != 0) {
        return false;
    }

    flash_unlock();
    flash_clear_errors();
    flash_wait_bsy();

    __asm volatile("cpsid i" ::: "memory");
    flash_program_words_ram(addr, (const uint32_t *)data, len / 4);
    __asm volatile("cpsie i" ::: "memory");

    bool ok = (FLASH_REGS->SR & FLASH_SR_ERRORS) == 0;

    flash_lock();
    return ok;
}

// ----------------------------------------------------------------------------
// Flash Test Implementation
// ----------------------------------------------------------------------------

void flash_test_init(void) {
    // Nothing special needed - flash peripheral always available
}

bool flash_test_erase(void) {
    swo_printf("[FLASH] Erasing sector %d at 0x%08X (%u KB)... ",
               FLASH_LOG_SECTOR, FLASH_LOG_BASE, FLASH_LOG_SIZE / 1024);

    bool ok = flash_erase_sector(FLASH_LOG_SECTOR);

    if (ok) {
        // Verify erased (should be all 0xFF)
        uint32_t *ptr = (uint32_t *)FLASH_LOG_BASE;
        for (uint32_t i = 0; i < 256; i++) { // Check first 1KB
            if (ptr[i] != 0xFFFFFFFF) {
                swo_puts("FAIL (not erased)\n");
                return false;
            }
        }
        swo_puts("OK\n");
    } else {
        swo_puts("FAIL\n");
    }

    return ok;
}

bool flash_test_write(uint32_t *bytes_written, uint32_t *time_ms) {
    static uint8_t test_buf[TEST_BLOCK_SIZE];

    swo_printf("[FLASH] Writing %u bytes in %u-byte blocks... ",
               TEST_TOTAL_SIZE, TEST_BLOCK_SIZE);

    // Fill test buffer with pattern
    for (int i = 0; i < TEST_BLOCK_SIZE; i++) {
        test_buf[i] = (uint8_t)(TEST_PATTERN ^ i);
    }

    uint32_t start = get_ticks();
    uint32_t written = 0;
    uint32_t addr = FLASH_LOG_BASE;

    while (written < TEST_TOTAL_SIZE) {
        if (!flash_write_block(addr, test_buf, TEST_BLOCK_SIZE)) {
            swo_puts("FAIL\n");
            *bytes_written = written;
            *time_ms = get_ticks() - start;
            return false;
        }
        addr += TEST_BLOCK_SIZE;
        written += TEST_BLOCK_SIZE;
    }

    uint32_t elapsed = get_ticks() - start;
    swo_printf("OK (%u ms)\n", elapsed);

    *bytes_written = written;
    *time_ms = elapsed;
    return true;
}

bool flash_test_read_verify(uint32_t *bytes_read, uint32_t *time_ms) {
    swo_printf("[FLASH] Reading and verifying %u bytes... ", TEST_TOTAL_SIZE);

    uint32_t start = get_ticks();
    uint32_t read = 0;
    const uint8_t *ptr = (const uint8_t *)FLASH_LOG_BASE;
    uint32_t errors = 0;

    while (read < TEST_TOTAL_SIZE) {
        uint8_t expected = (uint8_t)(TEST_PATTERN ^ (read % TEST_BLOCK_SIZE));
        if (ptr[read] != expected) {
            if (errors < 3) {
                swo_printf("\n[FLASH]   Mismatch at offset %u: got 0x%02X, "
                           "expected 0x%02X",
                           read, ptr[read], expected);
            }
            errors++;
        }
        read++;
    }

    uint32_t elapsed = get_ticks() - start;

    if (errors > 0) {
        swo_printf("\nFAIL (%u errors)\n", errors);
        *bytes_read = read;
        *time_ms = elapsed;
        return false;
    }

    swo_printf("OK (%u ms)\n", elapsed);
    *bytes_read = read;
    *time_ms = elapsed;
    return true;
}

bool flash_run_test(flash_test_results_t *results) {
    swo_printf("[FLASH] Test sector: %d (0x%08X, %u KB)\n", FLASH_LOG_SECTOR,
               FLASH_LOG_BASE, FLASH_LOG_SIZE / 1024);
    swo_puts("[FLASH] WARNING: This will erase existing log data!\n");

    flash_test_init();

    // Erase
    results->erase_ok = flash_test_erase();
    if (!results->erase_ok) {
        results->write_ok = false;
        results->read_ok = false;
        results->verify_ok = false;
        return false;
    }

    // Write
    results->write_ok =
        flash_test_write(&results->bytes_written, &results->write_time_ms);
    if (!results->write_ok) {
        results->read_ok = false;
        results->verify_ok = false;
        return false;
    }

    // Read and verify
    results->read_ok =
        flash_test_read_verify(&results->bytes_read, &results->read_time_ms);
    results->verify_ok = results->read_ok;

    // Summary
    swo_printf("[FLASH] Write speed: %u bytes in %u ms", results->bytes_written,
               results->write_time_ms);
    if (results->write_time_ms > 0) {
        swo_printf(" (%u bytes/sec)",
                   results->bytes_written * 1000 / results->write_time_ms);
    }
    swo_puts("\n");

    return results->verify_ok;
}
