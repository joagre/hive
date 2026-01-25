// SPI Low-Level Implementation for Crazyflie 2.1+ SD Card Deck
//
// Micro SD Card Deck uses SPI3:
//   PB3 = SCK  (AF6)
//   PB4 = MISO (AF6)
//   PB5 = MOSI (AF6)
//   PB6 = CS   (GPIO output)
//
// Reference: https://www.bitcraze.io/products/micro-sd-card-deck/

#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include <stdint.h>

// ---------------------------------------------------------------------------
// STM32F405 Register Definitions
// ---------------------------------------------------------------------------

// RCC
#define RCC_BASE 0x40023800UL
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40))

#define RCC_AHB1ENR_GPIOBEN (1UL << 1)
#define RCC_APB1ENR_SPI3EN (1UL << 15)

// GPIO
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

#define GPIOB_BASE 0x40020400UL
#define GPIOB ((GPIO_TypeDef *)GPIOB_BASE)

// SPI
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

#define SPI3_BASE 0x40003C00UL
#define SPI3 ((SPI_TypeDef *)SPI3_BASE)

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

// ---------------------------------------------------------------------------
// Pin Configuration
// ---------------------------------------------------------------------------

// SPI3 pins on Crazyflie SD deck
#define SD_SCK_PIN 3  // PB3
#define SD_MISO_PIN 4 // PB4
#define SD_MOSI_PIN 5 // PB5
#define SD_CS_PIN 6   // PB6
#define SPI3_AF 6     // Alternate function 6 for SPI3

// ---------------------------------------------------------------------------
// SPI Low-Level Implementation
// ---------------------------------------------------------------------------

void spi_ll_init(void) {
    // Enable clocks
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC_APB1ENR |= RCC_APB1ENR_SPI3EN;

    // Configure SPI3 pins as alternate function
    // PB3, PB4, PB5 -> AF6 (SPI3)
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

    // Configure SPI3: Master, 8-bit, Mode 0 (CPOL=0, CPHA=0)
    // Start with slow clock for initialization
    SPI3->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2 | SPI_CR1_BR_1 |
                SPI_CR1_BR_0 | // div 256
                SPI_CR1_SSM | SPI_CR1_SSI;

    SPI3->CR1 |= SPI_CR1_SPE;
}

void spi_ll_set_slow(void) {
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    SPI3->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0; // div 256 ~164 kHz
    SPI3->CR1 |= SPI_CR1_SPE;
}

void spi_ll_set_fast(void) {
    SPI3->CR1 &= ~SPI_CR1_SPE;
    SPI3->CR1 &= ~(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    SPI3->CR1 |= SPI_CR1_BR_0; // div 4 ~10.5 MHz
    SPI3->CR1 |= SPI_CR1_SPE;
}

void spi_ll_cs_low(void) {
    GPIOB->ODR &= ~(1UL << SD_CS_PIN);
}

void spi_ll_cs_high(void) {
    GPIOB->ODR |= (1UL << SD_CS_PIN);
}

uint8_t spi_ll_xfer(uint8_t data) {
    while (!(SPI3->SR & SPI_SR_TXE))
        ;
    SPI3->DR = data;
    while (!(SPI3->SR & SPI_SR_RXNE))
        ;
    return (uint8_t)SPI3->DR;
}

#endif // HIVE_ENABLE_SD
