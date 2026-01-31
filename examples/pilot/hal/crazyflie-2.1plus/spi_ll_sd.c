// SPI Low-Level Implementation for Crazyflie 2.1+ SD Card Deck
//
// Micro SD Card Deck shares SPI1 with Flow Deck (PMW3901):
//   PA5 = SCK  (AF5) - shared
//   PA6 = MISO (AF5) - shared
//   PA7 = MOSI (AF5) - shared
//   PC12 = CS  (GPIO output, IO4 on deck connector)
//
// Reference: https://www.bitcraze.io/products/micro-sd-card-deck/

#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include "stm32f4xx.h"
#include <stdint.h>

// ---------------------------------------------------------------------------
// Pin Configuration
// ---------------------------------------------------------------------------

// SD card CS pin on PC12 (IO4 on deck connector)
#define SD_CS_PIN GPIO_Pin_12
#define SD_CS_PORT GPIOC

// ---------------------------------------------------------------------------
// SPI Low-Level Implementation (shares SPI1 with flow deck)
// ---------------------------------------------------------------------------

void spi_ll_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure SPI1 GPIO using direct register access
    // PA5=SCK, PA6=MISO, PA7=MOSI all as AF5 (alternate function mode)

    // Clear mode bits for PA5, PA6, PA7, then set AF mode (10)
    GPIOA->MODER &=
        ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOA->MODER |=
        (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);

    // Set AF5 (SPI1) for PA5, PA6, PA7
    GPIOA->AFR[0] &= ~((0xF << (5 * 4)) | (0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOA->AFR[0] |= ((5 << (5 * 4)) | (5 << (6 * 4)) | (5 << (7 * 4)));

    // Set high speed for PA5, PA6, PA7
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 |
                       GPIO_OSPEEDER_OSPEEDR7);

    // Pull-down on SCK (PA5) and MOSI (PA7) for Mode 0 idle, no pull on MISO (PA6)
    GPIOA->PUPDR &=
        ~(GPIO_PUPDR_PUPDR5 | GPIO_PUPDR_PUPDR6 | GPIO_PUPDR_PUPDR7);
    GPIOA->PUPDR |=
        (GPIO_PUPDR_PUPDR5_1 | GPIO_PUPDR_PUPDR7_1); // Pull-down = 10

    // Configure SD CS pin (PC12) as push-pull output, high speed
    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_0;
    GPIOC->OTYPER &= ~GPIO_OTYPER_OT_12;
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;

    // Start with CS high (deselected)
    GPIO_SetBits(SD_CS_PORT, SD_CS_PIN);
}

void spi_ll_set_slow(void) {
    // Wait for SPI not busy then disable
    while (SPI1->SR & SPI_SR_BSY)
        ;
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Full reconfigure: Mode 0, Master, 8-bit, div 256 ~328 kHz
    SPI1->CR1 = SPI_CR1_MSTR |                               // Master mode
                SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | // div 256
                SPI_CR1_SSM | SPI_CR1_SSI;                   // Software SS
    // CPOL=0, CPHA=0 (Mode 0) - both bits are 0

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

void spi_ll_set_fast(void) {
    // Wait for SPI not busy then disable
    while (SPI1->SR & SPI_SR_BSY)
        ;
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Mode 0, Master, 8-bit, div 4 ~21 MHz (max for SD high speed)
    SPI1->CR1 = SPI_CR1_MSTR |             // Master mode
                SPI_CR1_BR_0 |             // div 4
                SPI_CR1_SSM | SPI_CR1_SSI; // Software SS

    SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI
}

void spi_ll_cs_low(void) {
    GPIO_ResetBits(SD_CS_PORT, SD_CS_PIN);
}

void spi_ll_cs_high(void) {
    GPIO_SetBits(SD_CS_PORT, SD_CS_PIN);
}

uint8_t spi_ll_xfer(uint8_t data) {
    // Clear any pending RX data first
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    // Wait for TX empty
    while (!(SPI1->SR & SPI_SR_TXE))
        ;

    // Write data (byte access for 8-bit mode)
    *((volatile uint8_t *)&SPI1->DR) = data;

    // Wait for RX not empty
    while (!(SPI1->SR & SPI_SR_RXNE))
        ;

    // Read received byte
    return *((volatile uint8_t *)&SPI1->DR);
}

#else

// Avoid empty translation unit warning when SD is disabled
typedef int spi_ll_sd_unused;

#endif // HIVE_ENABLE_SD
