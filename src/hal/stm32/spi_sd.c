// SPI SD Card Driver - STM32 Implementation
//
// Low-level SPI driver for SD card communication in SPI mode.
// Reference: http://elm-chan.org/docs/mmc/mmc_e.html

#include "spi_sd.h"
#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include <string.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// STM32 Register Definitions
// ---------------------------------------------------------------------------

// RCC registers
#define RCC_BASE 0x40023800UL
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))

// RCC enable bits
#define RCC_AHB1ENR_GPIOAEN (1UL << 0)
#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_AHB1ENR_GPIOCEN (1UL << 2)
#define RCC_AHB1ENR_GPIODEN (1UL << 3)
#define RCC_APB1ENR_SPI2EN (1UL << 14)
#define RCC_APB1ENR_SPI3EN (1UL << 15)

// GPIO registers (offset from port base)
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
} GPIO_TypeDef;

#define GPIOA_BASE 0x40020000UL
#define GPIOB_BASE 0x40020400UL
#define GPIOC_BASE 0x40020800UL
#define GPIOD_BASE 0x40020C00UL

#define GPIOA ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef *)GPIOD_BASE)

// SPI registers
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
} SPI_TypeDef;

#define SPI2_BASE 0x40003800UL
#define SPI3_BASE 0x40003C00UL

#define SPI2 ((SPI_TypeDef *)SPI2_BASE)
#define SPI3 ((SPI_TypeDef *)SPI3_BASE)

// SPI CR1 bits
#define SPI_CR1_CPHA (1UL << 0)
#define SPI_CR1_CPOL (1UL << 1)
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
#define SPI_SR_BSY (1UL << 7)

// SPI3 pin configuration (fixed for Crazyflie SD deck)
// PB3 = SCK (AF6), PB4 = MISO (AF6), PB5 = MOSI (AF6)
#define SPI3_AF 6

// Pointers set during init based on config
static GPIO_TypeDef *s_cs_gpio = NULL;
static uint32_t s_cs_pin = 0;
static SPI_TypeDef *s_spi = NULL;

// ---------------------------------------------------------------------------
// SD Card Commands (SPI Mode)
// ---------------------------------------------------------------------------

#define CMD0 0    // GO_IDLE_STATE
#define CMD1 1    // SEND_OP_COND (MMC)
#define CMD8 8    // SEND_IF_COND
#define CMD9 9    // SEND_CSD
#define CMD10 10  // SEND_CID
#define CMD12 12  // STOP_TRANSMISSION
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD18 18  // READ_MULTIPLE_BLOCK
#define CMD24 24  // WRITE_BLOCK
#define CMD25 25  // WRITE_MULTIPLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SD_SEND_OP_COND (app-specific)

// R1 response flags
#define R1_IDLE_STATE 0x01
#define R1_ERASE_RESET 0x02
#define R1_ILLEGAL_CMD 0x04
#define R1_CRC_ERROR 0x08
#define R1_ERASE_SEQ_ERROR 0x10
#define R1_ADDRESS_ERROR 0x20
#define R1_PARAMETER_ERROR 0x40

// Data tokens
#define TOKEN_SINGLE_BLOCK 0xFE
#define TOKEN_MULTI_BLOCK 0xFC
#define TOKEN_STOP_TRAN 0xFD

// ---------------------------------------------------------------------------
// Driver State
// ---------------------------------------------------------------------------

static spi_sd_config_t s_config;
static bool s_configured = false;
static bool s_initialized = false;
static sd_card_type_t s_card_type = SD_CARD_NONE;
static uint32_t s_sector_count = 0;

// ---------------------------------------------------------------------------
// SPI Low-Level Operations (Hardware-Specific)
// ---------------------------------------------------------------------------

// Get GPIO port from config port number
static GPIO_TypeDef *get_gpio_port(uint8_t port) {
    switch (port) {
    case 0:
        return GPIOA;
    case 1:
        return GPIOB;
    case 2:
        return GPIOC;
    case 3:
        return GPIOD;
    default:
        return GPIOB;
    }
}

// Initialize SPI peripheral
static void spi_init(void) {
    // Enable GPIO clocks (all ports we might use)
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                   RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;

    // Select SPI peripheral based on config
    if (s_config.spi_id == 3) {
        // Enable SPI3 clock
        RCC_APB1ENR |= RCC_APB1ENR_SPI3EN;
        s_spi = SPI3;

        // Configure SPI3 pins: PB3=SCK, PB4=MISO, PB5=MOSI (AF6)
        // Set pins to alternate function mode
        GPIOB->MODER &=
            ~((3UL << (3 * 2)) | (3UL << (4 * 2)) | (3UL << (5 * 2)));
        GPIOB->MODER |=
            ((2UL << (3 * 2)) | (2UL << (4 * 2)) | (2UL << (5 * 2)));

        // High speed
        GPIOB->OSPEEDR |=
            ((3UL << (3 * 2)) | (3UL << (4 * 2)) | (3UL << (5 * 2)));

        // Set AF6 for SPI3 (pins 3,4,5 are in AFR[0])
        GPIOB->AFR[0] &=
            ~((0xFUL << (3 * 4)) | (0xFUL << (4 * 4)) | (0xFUL << (5 * 4)));
        GPIOB->AFR[0] |= ((SPI3_AF << (3 * 4)) | (SPI3_AF << (4 * 4)) |
                          (SPI3_AF << (5 * 4)));

        // Pull-up on MISO
        GPIOB->PUPDR &= ~(3UL << (4 * 2));
        GPIOB->PUPDR |= (1UL << (4 * 2));

    } else if (s_config.spi_id == 2) {
        // Enable SPI2 clock
        RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;
        s_spi = SPI2;

        // Configure SPI2 pins: PB13=SCK, PB14=MISO, PB15=MOSI (AF5)
        GPIOB->MODER &=
            ~((3UL << (13 * 2)) | (3UL << (14 * 2)) | (3UL << (15 * 2)));
        GPIOB->MODER |=
            ((2UL << (13 * 2)) | (2UL << (14 * 2)) | (2UL << (15 * 2)));

        GPIOB->OSPEEDR |=
            ((3UL << (13 * 2)) | (3UL << (14 * 2)) | (3UL << (15 * 2)));

        // AF5 for SPI2 (pins 13,14,15 are in AFR[1])
        GPIOB->AFR[1] &=
            ~((0xFUL << ((13 - 8) * 4)) | (0xFUL << ((14 - 8) * 4)) |
              (0xFUL << ((15 - 8) * 4)));
        GPIOB->AFR[1] |= ((5UL << ((13 - 8) * 4)) | (5UL << ((14 - 8) * 4)) |
                          (5UL << ((15 - 8) * 4)));

        // Pull-up on MISO
        GPIOB->PUPDR &= ~(3UL << (14 * 2));
        GPIOB->PUPDR |= (1UL << (14 * 2));
    }

    // Configure CS pin as output
    s_cs_gpio = get_gpio_port(s_config.cs_port);
    s_cs_pin = s_config.cs_pin;

    s_cs_gpio->MODER &= ~(3UL << (s_cs_pin * 2));
    s_cs_gpio->MODER |= (1UL << (s_cs_pin * 2));   // Output mode
    s_cs_gpio->OSPEEDR |= (3UL << (s_cs_pin * 2)); // High speed
    s_cs_gpio->ODR |= (1UL << s_cs_pin);           // CS high (deselected)

    // Configure SPI: Master, 8-bit, Mode 0 (CPOL=0, CPHA=0), slow speed
    // APB1 = 42 MHz, BR=111 (div 256) = 164 kHz for init
    s_spi->CR1 = SPI_CR1_MSTR | // Master mode
                 SPI_CR1_BR_2 | SPI_CR1_BR_1 |
                 SPI_CR1_BR_0 | // Baud = fPCLK/256
                 SPI_CR1_SSM |  // Software slave management
                 SPI_CR1_SSI;   // Internal slave select

    s_spi->CR1 |= SPI_CR1_SPE; // Enable SPI
}

// Set SPI clock speed for initialization (~400 kHz)
static void spi_set_speed_low(void) {
    s_spi->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    s_spi->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    s_spi->CR1 |=
        SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // div 256 = 164 kHz
    s_spi->CR1 |= SPI_CR1_SPE;                      // Enable SPI
}

// Set SPI clock speed for data transfer (~21 MHz)
static void spi_set_speed_high(void) {
    s_spi->CR1 &= ~SPI_CR1_SPE; // Disable SPI
    s_spi->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    s_spi->CR1 |= SPI_CR1_BR_0; // div 4 = 10.5 MHz (safe for SD)
    s_spi->CR1 |= SPI_CR1_SPE;  // Enable SPI
}

// Chip select control
static void cs_low(void) {
    s_cs_gpio->ODR &= ~(1UL << s_cs_pin);
}

static void cs_high(void) {
    s_cs_gpio->ODR |= (1UL << s_cs_pin);
}

// Transfer single byte (full duplex)
static uint8_t spi_xfer(uint8_t data) {
    // Wait for TX buffer empty
    while (!(s_spi->SR & SPI_SR_TXE))
        ;
    // Send data
    s_spi->DR = data;
    // Wait for RX buffer not empty
    while (!(s_spi->SR & SPI_SR_RXNE))
        ;
    // Return received data
    return (uint8_t)s_spi->DR;
}

// Send multiple bytes (discard received)
static void spi_send(const uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        spi_xfer(buf[i]);
    }
}

// Receive multiple bytes (send 0xFF)
static void spi_recv(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = spi_xfer(0xFF);
    }
}

// ---------------------------------------------------------------------------
// SD Protocol Helpers
// ---------------------------------------------------------------------------

// Wait for card ready (not busy)
static int wait_ready(uint32_t timeout_ms) {
    // TODO: Use actual timer
    (void)timeout_ms;
    for (uint32_t i = 0; i < 500000; i++) {
        if (spi_xfer(0xFF) == 0xFF) {
            return 0;
        }
    }
    return -1;
}

// Send SD command
static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t buf[6];
    uint8_t crc = 0x01; // Dummy CRC (CRC disabled after init)

    // Pre-defined CRCs for required commands
    if (cmd == CMD0)
        crc = 0x95;
    if (cmd == CMD8)
        crc = 0x87;

    buf[0] = 0x40 | cmd; // Start bit + command
    buf[1] = (arg >> 24) & 0xFF;
    buf[2] = (arg >> 16) & 0xFF;
    buf[3] = (arg >> 8) & 0xFF;
    buf[4] = arg & 0xFF;
    buf[5] = crc;

    // Send dummy clocks if needed
    spi_xfer(0xFF);

    // Wait for card ready
    if (cmd != CMD0 && cmd != CMD12) {
        wait_ready(500);
    }

    // Send command
    spi_send(buf, 6);

    // Wait for response (not 0xFF)
    uint8_t r1 = 0xFF;
    for (int i = 0; i < 10; i++) {
        r1 = spi_xfer(0xFF);
        if ((r1 & 0x80) == 0) {
            break;
        }
    }

    return r1;
}

// Send application-specific command
static uint8_t send_acmd(uint8_t cmd, uint32_t arg) {
    send_cmd(CMD55, 0);
    return send_cmd(cmd, arg);
}

// Wait for data token
static int wait_data_token(uint8_t token, uint32_t timeout_ms) {
    (void)timeout_ms;
    for (uint32_t i = 0; i < 100000; i++) {
        uint8_t r = spi_xfer(0xFF);
        if (r == token) {
            return 0;
        }
        if (r != 0xFF) {
            return -1; // Error token
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void spi_sd_configure(const spi_sd_config_t *config) {
    if (config) {
        s_config = *config;
        s_configured = true;
    }
}

int spi_sd_is_present(void) {
    // TODO: Check card detect GPIO if available
    // For now, always return present
    return s_configured ? 1 : 0;
}

int spi_sd_init(void) {
    if (!s_configured) {
        return -1;
    }

    if (s_initialized) {
        return 0;
    }

    uint8_t r1;
    uint8_t ocr[4];

    // Initialize SPI at low speed
    spi_init();
    spi_set_speed_low();

    // CS high, send 80+ clocks to enter native mode
    cs_high();
    for (int i = 0; i < 10; i++) {
        spi_xfer(0xFF);
    }

    // Enter SPI mode (CMD0)
    cs_low();
    r1 = send_cmd(CMD0, 0);
    cs_high();

    if (r1 != R1_IDLE_STATE) {
        return -1;
    }

    // Check voltage (CMD8)
    cs_low();
    r1 = send_cmd(CMD8, 0x1AA);
    if (r1 == R1_IDLE_STATE) {
        // SD v2.0+
        spi_recv(ocr, 4);
        cs_high();

        if (ocr[2] != 0x01 || ocr[3] != 0xAA) {
            return -1; // Voltage not accepted
        }

        // Wait for card ready (ACMD41 with HCS=1)
        for (int i = 0; i < 1000; i++) {
            cs_low();
            r1 = send_acmd(ACMD41, 0x40000000);
            cs_high();
            if (r1 == 0)
                break;
        }

        if (r1 != 0) {
            return -1;
        }

        // Check CCS bit (high capacity)
        cs_low();
        r1 = send_cmd(CMD58, 0);
        spi_recv(ocr, 4);
        cs_high();

        s_card_type = (ocr[0] & 0x40) ? SD_CARD_V2_HC : SD_CARD_V2_SC;
    } else {
        cs_high();
        // SD v1.x or MMC
        cs_low();
        r1 = send_acmd(ACMD41, 0);
        cs_high();

        if ((r1 & R1_ILLEGAL_CMD) == 0) {
            // SD v1.x
            for (int i = 0; i < 1000; i++) {
                cs_low();
                r1 = send_acmd(ACMD41, 0);
                cs_high();
                if (r1 == 0)
                    break;
            }
            s_card_type = SD_CARD_V1;
        } else {
            // MMC (not supported)
            return -1;
        }

        if (r1 != 0) {
            return -1;
        }
    }

    // Set block size to 512 for v1/v2-SC cards
    if (s_card_type != SD_CARD_V2_HC) {
        cs_low();
        r1 = send_cmd(CMD16, 512);
        cs_high();
        if (r1 != 0) {
            return -1;
        }
    }

    // Read CSD to get sector count
    cs_low();
    r1 = send_cmd(CMD9, 0);
    if (r1 == 0 && wait_data_token(TOKEN_SINGLE_BLOCK, 500) == 0) {
        uint8_t csd[16];
        spi_recv(csd, 16);
        spi_xfer(0xFF); // CRC
        spi_xfer(0xFF);

        // Parse CSD for capacity
        if ((csd[0] >> 6) == 1) {
            // CSD v2.0 (SDHC/SDXC)
            uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) |
                              ((uint32_t)csd[8] << 8) | (uint32_t)csd[9];
            s_sector_count = (c_size + 1) * 1024;
        } else {
            // CSD v1.0
            uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) |
                              ((uint32_t)csd[7] << 2) |
                              ((uint32_t)(csd[8] >> 6) & 0x03);
            uint32_t c_size_mult =
                ((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01);
            uint32_t read_bl_len = csd[5] & 0x0F;
            s_sector_count = (c_size + 1)
                             << (c_size_mult + read_bl_len + 2 - 9);
        }
    }
    cs_high();

    // Switch to high speed
    spi_set_speed_high();

    s_initialized = true;
    return 0;
}

int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count) {
    if (!s_initialized || count == 0) {
        return -1;
    }

    // Convert to byte address for v1/v2-SC cards
    uint32_t addr = sector;
    if (s_card_type != SD_CARD_V2_HC) {
        addr *= 512;
    }

    cs_low();

    if (count == 1) {
        // Single block read
        if (send_cmd(CMD17, addr) != 0) {
            cs_high();
            return -1;
        }

        if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
            cs_high();
            return -1;
        }

        spi_recv(buf, 512);
        spi_xfer(0xFF); // CRC
        spi_xfer(0xFF);
    } else {
        // Multi-block read
        if (send_cmd(CMD18, addr) != 0) {
            cs_high();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            if (wait_data_token(TOKEN_SINGLE_BLOCK, 500) != 0) {
                send_cmd(CMD12, 0); // Stop transmission
                cs_high();
                return -1;
            }

            spi_recv(buf + i * 512, 512);
            spi_xfer(0xFF); // CRC
            spi_xfer(0xFF);
        }

        send_cmd(CMD12, 0); // Stop transmission
    }

    cs_high();
    return 0;
}

int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count) {
    if (!s_initialized || count == 0) {
        return -1;
    }

    // Convert to byte address for v1/v2-SC cards
    uint32_t addr = sector;
    if (s_card_type != SD_CARD_V2_HC) {
        addr *= 512;
    }

    cs_low();

    if (count == 1) {
        // Single block write
        if (send_cmd(CMD24, addr) != 0) {
            cs_high();
            return -1;
        }

        spi_xfer(0xFF);               // Gap
        spi_xfer(TOKEN_SINGLE_BLOCK); // Data token
        spi_send(buf, 512);
        spi_xfer(0xFF); // CRC
        spi_xfer(0xFF);

        // Check data response
        uint8_t resp = spi_xfer(0xFF);
        if ((resp & 0x1F) != 0x05) {
            cs_high();
            return -1;
        }

        // Wait for write completion
        wait_ready(500);
    } else {
        // Multi-block write
        if (send_cmd(CMD25, addr) != 0) {
            cs_high();
            return -1;
        }

        for (uint32_t i = 0; i < count; i++) {
            spi_xfer(0xFF);              // Gap
            spi_xfer(TOKEN_MULTI_BLOCK); // Data token
            spi_send(buf + i * 512, 512);
            spi_xfer(0xFF); // CRC
            spi_xfer(0xFF);

            // Check data response
            uint8_t resp = spi_xfer(0xFF);
            if ((resp & 0x1F) != 0x05) {
                spi_xfer(TOKEN_STOP_TRAN);
                cs_high();
                return -1;
            }

            wait_ready(500);
        }

        spi_xfer(TOKEN_STOP_TRAN); // Stop token
        wait_ready(500);
    }

    cs_high();
    return 0;
}

int spi_sd_sync(void) {
    if (!s_initialized) {
        return -1;
    }

    cs_low();
    int result = wait_ready(500);
    cs_high();

    return result;
}

uint32_t spi_sd_get_sector_count(void) {
    return s_sector_count;
}

sd_card_type_t spi_sd_get_card_type(void) {
    return s_card_type;
}

#else // !HIVE_ENABLE_SD

// Stubs when SD is disabled
void spi_sd_configure(const spi_sd_config_t *config) {
    (void)config;
}
int spi_sd_is_present(void) {
    return 0;
}
int spi_sd_init(void) {
    return -1;
}
int spi_sd_read_blocks(uint8_t *buf, uint32_t sector, uint32_t count) {
    (void)buf;
    (void)sector;
    (void)count;
    return -1;
}
int spi_sd_write_blocks(const uint8_t *buf, uint32_t sector, uint32_t count) {
    (void)buf;
    (void)sector;
    (void)count;
    return -1;
}
int spi_sd_sync(void) {
    return -1;
}
uint32_t spi_sd_get_sector_count(void) {
    return 0;
}
sd_card_type_t spi_sd_get_card_type(void) {
    return SD_CARD_NONE;
}

#endif // HIVE_ENABLE_SD
