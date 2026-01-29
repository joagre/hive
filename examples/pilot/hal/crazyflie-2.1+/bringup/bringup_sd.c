// Crazyflie 2.1+ Bring-Up - SD Card Test
//
// Low-level SD card test via SPI. Tests card presence and basic read/write
// without depending on FatFS filesystem.
//
// Micro SD Card Deck pinout:
//   PB3 = SCK  (AF6, SPI3)
//   PB4 = MISO (AF6, SPI3)
//   PB5 = MOSI (AF6, SPI3)
//   PB6 = CS   (GPIO output)
//
// Reference: http://elm-chan.org/docs/mmc/mmc_e.html

#include "bringup_sd.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// ----------------------------------------------------------------------------
// SPI3 Configuration for SD Card Deck
// ----------------------------------------------------------------------------

// GPIO pins
#define SD_SCK_PIN 3  // PB3
#define SD_MISO_PIN 4 // PB4
#define SD_MOSI_PIN 5 // PB5
#define SD_CS_PIN 6   // PB6
#define SPI3_AF 6     // Alternate function for SPI3

// RCC registers
#define RCC_BASE 0x40023800UL
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_APB1ENR_SPI3EN (1UL << 15)

// GPIO GPIOB registers
typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_Regs;

#define GPIOB_BASE 0x40020400UL
#define GPIOB ((GPIO_Regs *)GPIOB_BASE)

// SPI3 registers
typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_Regs;

#define SPI3_BASE 0x40003C00UL
#define SPI3 ((SPI_Regs *)SPI3_BASE)

// SPI CR1 bits
#define SPI_CR1_MSTR (1UL << 2)
#define SPI_CR1_BR_0 (1UL << 3)
#define SPI_CR1_BR_1 (1UL << 4)
#define SPI_CR1_BR_2 (1UL << 5)
#define SPI_CR1_SPE (1UL << 6)
#define SPI_CR1_SSI (1UL << 8)
#define SPI_CR1_SSM (1UL << 9)

// SPI SR bits
#define SPI_SR_RXNE (1UL << 0)
#define SPI_SR_TXE (1UL << 1)

// ----------------------------------------------------------------------------
// SD Card Commands
// ----------------------------------------------------------------------------

#define CMD0 0    // GO_IDLE_STATE
#define CMD1 1    // SEND_OP_COND (MMC)
#define CMD8 8    // SEND_IF_COND
#define CMD9 9    // SEND_CSD
#define CMD10 10  // SEND_CID
#define CMD12 12  // STOP_TRANSMISSION
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD24 24  // WRITE_SINGLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SD_SEND_OP_COND

// Card types
#define CT_UNKNOWN 0
#define CT_SDv1 1
#define CT_SDv2 2
#define CT_SDHC 3

// Timing (SysTick based)
static uint32_t get_ticks(void) {
    return swo_get_ticks();
}

// Test block number (use a high block to avoid damaging FAT)
// Block 65536 = 32MB offset, should be safe on most cards
#define TEST_BLOCK_NUM 65536

// Test data buffer
static uint8_t s_test_buf[512];
static uint8_t s_read_buf[512];

// ----------------------------------------------------------------------------
// SPI Low-Level
// ----------------------------------------------------------------------------

static void spi_cs_low(void) {
    GPIOB->ODR &= ~(1UL << SD_CS_PIN);
}

static void spi_cs_high(void) {
    GPIOB->ODR |= (1UL << SD_CS_PIN);
}

static uint8_t spi_xfer(uint8_t data) {
    while (!(SPI3->SR & SPI_SR_TXE))
        ;
    SPI3->DR = data;
    while (!(SPI3->SR & SPI_SR_RXNE))
        ;
    return (uint8_t)SPI3->DR;
}

static void spi_set_slow(void) {
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    SPI3->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // div 256
    SPI3->CR1 |= SPI_CR1_SPE;
}

static void spi_set_fast(void) {
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    SPI3->CR1 |= SPI_CR1_BR_0; // div 4 (~10.5 MHz)
    SPI3->CR1 |= SPI_CR1_SPE;
}

// ----------------------------------------------------------------------------
// SD Card Protocol
// ----------------------------------------------------------------------------

static void sd_delay(void) {
    for (int i = 0; i < 10; i++) {
        spi_xfer(0xFF);
    }
}

static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t crc = 0xFF;

    // CMD0 and CMD8 require valid CRC
    if (cmd == CMD0)
        crc = 0x95;
    if (cmd == CMD8)
        crc = 0x87;

    spi_xfer(0x40 | cmd);
    spi_xfer((uint8_t)(arg >> 24));
    spi_xfer((uint8_t)(arg >> 16));
    spi_xfer((uint8_t)(arg >> 8));
    spi_xfer((uint8_t)(arg));
    spi_xfer(crc);

    // Wait for response (R1)
    uint8_t r1;
    for (int i = 0; i < 10; i++) {
        r1 = spi_xfer(0xFF);
        if (!(r1 & 0x80))
            break;
    }

    return r1;
}

static bool sd_wait_ready(uint32_t timeout_ms) {
    uint32_t start = get_ticks();
    while ((get_ticks() - start) < timeout_ms) {
        if (spi_xfer(0xFF) == 0xFF) {
            return true;
        }
    }
    return false;
}

// ----------------------------------------------------------------------------
// SD Test Implementation
// ----------------------------------------------------------------------------

void sd_test_init(void) {
    // Enable clocks
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_SPI3EN;

    // Configure SPI3 pins as alternate function
    GPIOB->MODER &= ~((3UL << (SD_SCK_PIN * 2)) | (3UL << (SD_MISO_PIN * 2)) |
                      (3UL << (SD_MOSI_PIN * 2)));
    GPIOB->MODER |= ((2UL << (SD_SCK_PIN * 2)) | (2UL << (SD_MISO_PIN * 2)) |
                     (2UL << (SD_MOSI_PIN * 2)));

    // High speed output
    GPIOB->OSPEEDR |= ((3UL << (SD_SCK_PIN * 2)) | (3UL << (SD_MISO_PIN * 2)) |
                       (3UL << (SD_MOSI_PIN * 2)));

    // Set alternate function 6 (SPI3)
    GPIOB->AFR[0] &=
        ~((0xFUL << (SD_SCK_PIN * 4)) | (0xFUL << (SD_MISO_PIN * 4)) |
          (0xFUL << (SD_MOSI_PIN * 4)));
    GPIOB->AFR[0] |=
        ((SPI3_AF << (SD_SCK_PIN * 4)) | (SPI3_AF << (SD_MISO_PIN * 4)) |
         (SPI3_AF << (SD_MOSI_PIN * 4)));

    // Pull-up on MISO
    GPIOB->PUPDR &= ~(3UL << (SD_MISO_PIN * 2));
    GPIOB->PUPDR |= (1UL << (SD_MISO_PIN * 2));

    // Configure CS pin as output, high (deselected)
    GPIOB->MODER &= ~(3UL << (SD_CS_PIN * 2));
    GPIOB->MODER |= (1UL << (SD_CS_PIN * 2));
    GPIOB->OSPEEDR |= (3UL << (SD_CS_PIN * 2));
    GPIOB->ODR |= (1UL << SD_CS_PIN);

    // Configure SPI3: Master, 8-bit, Mode 0
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 |
                SPI_CR1_SSM | SPI_CR1_SSI;
    SPI3->CR1 |= SPI_CR1_SPE;
}

bool sd_test_spi_init(void) {
    swo_puts(
        "[SD] Initializing SPI3 (PB3=SCK, PB4=MISO, PB5=MOSI, PB6=CS)... ");

    sd_test_init();

    // Test SPI by sending dummy bytes
    spi_cs_high();
    for (int i = 0; i < 10; i++) {
        spi_xfer(0xFF);
    }

    swo_puts("OK\n");
    return true;
}

bool sd_test_card_detect(uint32_t *card_type, uint32_t *card_size_mb) {
    swo_puts("[SD] Detecting card...\n");

    *card_type = CT_UNKNOWN;
    *card_size_mb = 0;

    // Start with slow clock for init
    spi_set_slow();

    // Send 80+ clock pulses with CS high
    spi_cs_high();
    for (int i = 0; i < 10; i++) {
        spi_xfer(0xFF);
    }

    // CMD0: GO_IDLE_STATE
    spi_cs_low();
    sd_delay();

    swo_puts("[SD]   CMD0 (GO_IDLE)... ");
    uint8_t r1 = sd_send_cmd(CMD0, 0);
    spi_cs_high();
    spi_xfer(0xFF);

    if (r1 != 0x01) {
        swo_printf("FAIL (R1=0x%02X, expected 0x01)\n", r1);
        swo_puts("[SD]   Card not detected or not responding\n");
        return false;
    }
    swo_puts("OK\n");

    // CMD8: SEND_IF_COND (check SD v2)
    spi_cs_low();
    sd_delay();

    swo_puts("[SD]   CMD8 (IF_COND)... ");
    r1 = sd_send_cmd(CMD8, 0x1AA);

    if (r1 == 0x01) {
        // SDv2 or later
        uint32_t resp = 0;
        resp = (uint32_t)spi_xfer(0xFF) << 24;
        resp |= (uint32_t)spi_xfer(0xFF) << 16;
        resp |= (uint32_t)spi_xfer(0xFF) << 8;
        resp |= (uint32_t)spi_xfer(0xFF);
        spi_cs_high();
        spi_xfer(0xFF);

        if ((resp & 0xFFF) != 0x1AA) {
            swo_printf("FAIL (bad echo: 0x%03X)\n", resp & 0xFFF);
            return false;
        }
        swo_puts("OK (SDv2)\n");

        // ACMD41: Initialize SDv2
        swo_puts("[SD]   ACMD41 (SD_SEND_OP_COND)... ");
        uint32_t start = get_ticks();
        bool initialized = false;

        while ((get_ticks() - start) < 1000) {
            spi_cs_low();
            sd_delay();
            sd_send_cmd(CMD55, 0);
            spi_cs_high();
            spi_xfer(0xFF);

            spi_cs_low();
            sd_delay();
            r1 = sd_send_cmd(ACMD41, 0x40000000); // HCS bit set
            spi_cs_high();
            spi_xfer(0xFF);

            if (r1 == 0x00) {
                initialized = true;
                break;
            }
        }

        if (!initialized) {
            swo_puts("FAIL (timeout)\n");
            return false;
        }
        swo_puts("OK\n");

        // CMD58: Read OCR to check for SDHC
        swo_puts("[SD]   CMD58 (READ_OCR)... ");
        spi_cs_low();
        sd_delay();
        r1 = sd_send_cmd(CMD58, 0);

        if (r1 == 0x00) {
            uint32_t ocr = 0;
            ocr = (uint32_t)spi_xfer(0xFF) << 24;
            ocr |= (uint32_t)spi_xfer(0xFF) << 16;
            ocr |= (uint32_t)spi_xfer(0xFF) << 8;
            ocr |= (uint32_t)spi_xfer(0xFF);
            spi_cs_high();
            spi_xfer(0xFF);

            if (ocr & 0x40000000) {
                *card_type = CT_SDHC;
                swo_puts("OK (SDHC/SDXC)\n");
            } else {
                *card_type = CT_SDv2;
                swo_puts("OK (SDSC)\n");
            }
        } else {
            spi_cs_high();
            spi_xfer(0xFF);
            *card_type = CT_SDv2;
            swo_puts("OK (assumed SDv2)\n");
        }

    } else {
        // SDv1 or MMC
        spi_cs_high();
        spi_xfer(0xFF);
        swo_puts("not supported (SDv1/MMC)\n");

        // Try ACMD41 for SDv1
        swo_puts("[SD]   ACMD41 (SD_SEND_OP_COND)... ");
        uint32_t start = get_ticks();
        bool initialized = false;

        while ((get_ticks() - start) < 1000) {
            spi_cs_low();
            sd_delay();
            sd_send_cmd(CMD55, 0);
            spi_cs_high();
            spi_xfer(0xFF);

            spi_cs_low();
            sd_delay();
            r1 = sd_send_cmd(ACMD41, 0);
            spi_cs_high();
            spi_xfer(0xFF);

            if (r1 == 0x00) {
                initialized = true;
                *card_type = CT_SDv1;
                break;
            }
        }

        if (!initialized) {
            swo_puts("FAIL\n");
            return false;
        }
        swo_puts("OK (SDv1)\n");
    }

    // Switch to fast clock
    spi_set_fast();

    swo_printf("[SD] Card type: %s\n", *card_type == CT_SDHC   ? "SDHC"
                                       : *card_type == CT_SDv2 ? "SDv2"
                                       : *card_type == CT_SDv1 ? "SDv1"
                                                               : "Unknown");

    return true;
}

bool sd_test_write(uint32_t *time_ms) {
    swo_printf("[SD] Writing test block %u (512 bytes)... ", TEST_BLOCK_NUM);

    // Fill test buffer with pattern
    for (int i = 0; i < 512; i++) {
        s_test_buf[i] = (uint8_t)(i ^ 0xA5);
    }

    uint32_t start = get_ticks();

    // Wait for card ready
    spi_cs_low();
    if (!sd_wait_ready(500)) {
        spi_cs_high();
        swo_puts("FAIL (card not ready)\n");
        *time_ms = 0;
        return false;
    }

    // CMD24: Write single block
    uint32_t addr = TEST_BLOCK_NUM; // SDHC uses block addressing
    uint8_t r1 = sd_send_cmd(CMD24, addr);

    if (r1 != 0x00) {
        spi_cs_high();
        spi_xfer(0xFF);
        swo_printf("FAIL (CMD24 R1=0x%02X)\n", r1);
        *time_ms = 0;
        return false;
    }

    // Send data token
    spi_xfer(0xFE);

    // Send data
    for (int i = 0; i < 512; i++) {
        spi_xfer(s_test_buf[i]);
    }

    // Send dummy CRC
    spi_xfer(0xFF);
    spi_xfer(0xFF);

    // Check data response
    uint8_t resp = spi_xfer(0xFF) & 0x1F;
    if (resp != 0x05) {
        spi_cs_high();
        spi_xfer(0xFF);
        swo_printf("FAIL (data response=0x%02X)\n", resp);
        *time_ms = 0;
        return false;
    }

    // Wait for write to complete
    if (!sd_wait_ready(500)) {
        spi_cs_high();
        swo_puts("FAIL (write timeout)\n");
        *time_ms = 0;
        return false;
    }

    spi_cs_high();
    spi_xfer(0xFF);

    *time_ms = get_ticks() - start;
    swo_printf("OK (%u ms)\n", *time_ms);
    return true;
}

bool sd_test_read_verify(uint32_t *time_ms) {
    swo_printf("[SD] Reading and verifying block %u... ", TEST_BLOCK_NUM);

    uint32_t start = get_ticks();

    // Wait for card ready
    spi_cs_low();
    if (!sd_wait_ready(500)) {
        spi_cs_high();
        swo_puts("FAIL (card not ready)\n");
        *time_ms = 0;
        return false;
    }

    // CMD17: Read single block
    uint32_t addr = TEST_BLOCK_NUM;
    uint8_t r1 = sd_send_cmd(CMD17, addr);

    if (r1 != 0x00) {
        spi_cs_high();
        spi_xfer(0xFF);
        swo_printf("FAIL (CMD17 R1=0x%02X)\n", r1);
        *time_ms = 0;
        return false;
    }

    // Wait for data token
    uint8_t token;
    uint32_t timeout_start = get_ticks();
    do {
        token = spi_xfer(0xFF);
        if ((get_ticks() - timeout_start) > 500) {
            spi_cs_high();
            swo_puts("FAIL (data token timeout)\n");
            *time_ms = 0;
            return false;
        }
    } while (token == 0xFF);

    if (token != 0xFE) {
        spi_cs_high();
        spi_xfer(0xFF);
        swo_printf("FAIL (bad token=0x%02X)\n", token);
        *time_ms = 0;
        return false;
    }

    // Read data
    for (int i = 0; i < 512; i++) {
        s_read_buf[i] = spi_xfer(0xFF);
    }

    // Read CRC (ignore)
    spi_xfer(0xFF);
    spi_xfer(0xFF);

    spi_cs_high();
    spi_xfer(0xFF);

    *time_ms = get_ticks() - start;

    // Verify data
    int errors = 0;
    for (int i = 0; i < 512; i++) {
        uint8_t expected = (uint8_t)(i ^ 0xA5);
        if (s_read_buf[i] != expected) {
            if (errors < 3) {
                swo_printf(
                    "\n[SD]   Mismatch at %d: got 0x%02X, expected 0x%02X", i,
                    s_read_buf[i], expected);
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

bool sd_run_test(sd_test_results_t *results) {
    memset(results, 0, sizeof(*results));

    swo_puts("[SD] Micro SD Card Deck (SPI3)\n");
    swo_puts("[SD] Pins: PB3=SCK, PB4=MISO, PB5=MOSI, PB6=CS\n");

    // SPI init
    results->spi_init_ok = sd_test_spi_init();
    if (!results->spi_init_ok) {
        swo_puts("[SD] SPI initialization failed\n");
        return false;
    }

    // Card detect and init
    results->card_init_ok =
        sd_test_card_detect(&results->card_type, &results->card_size_mb);
    results->card_present = (results->card_type != CT_UNKNOWN);

    if (!results->card_init_ok) {
        swo_puts("[SD] Card not detected - deck not attached?\n");
        return false;
    }

    // Write test
    results->write_ok = sd_test_write(&results->write_time_ms);
    if (!results->write_ok) {
        swo_puts("[SD] Write test failed\n");
        return false;
    }

    // Read and verify
    results->read_ok = sd_test_read_verify(&results->read_time_ms);
    results->verify_ok = results->read_ok;

    if (!results->verify_ok) {
        swo_puts("[SD] Read/verify failed\n");
        return false;
    }

    swo_puts("[SD] All tests passed!\n");
    return true;
}
