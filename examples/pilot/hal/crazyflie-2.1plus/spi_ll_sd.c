// SPI Low-Level Implementation for Crazyflie 2.1+ SD Card Deck
//
// Micro SD Card Deck shares SPI1 with Flow Deck (PMW3901):
//   PA5 = SCK  (AF5) - shared
//   PA6 = MISO (AF5) - shared
//   PA7 = MOSI (AF5) - shared
//   PC12 = CS  (GPIO output, IO4 on deck connector)
//
// DMA allocation (STM32F405):
//   DMA2 Stream 0, Channel 3 = SPI1_RX (SD card)
//   DMA2 Stream 3, Channel 3 = SPI1_TX (SD card)
//   DMA2 Stream 2, Channel 5 = USART6_RX (syslink - not ours)
//
// WARNING: This driver reconfigures SPI1 clock speed. If used alongside
// flow deck, ensure proper CS management - only one device active at a time.
// Flow deck uses ~1.3 MHz (Mode 0), SD card uses 328 kHz init / 21 MHz fast.
//
// Reference: https://www.bitcraze.io/products/micro-sd-card-deck/

#include "hive_static_config.h"

#if HIVE_ENABLE_SD

#include "spi_ll.h"
#include "stm32f4xx.h"
#include "hal/hive_hal_event.h"
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Pin Configuration
// ---------------------------------------------------------------------------

// SD card CS pin on PC12 (IO4 on deck connector)
#define SD_CS_PIN GPIO_Pin_12
#define SD_CS_PORT GPIOC

// ---------------------------------------------------------------------------
// DMA State
// ---------------------------------------------------------------------------

// DMA2 Stream 0, Channel 3 = SPI1_RX
// DMA2 Stream 3, Channel 3 = SPI1_TX
#define SD_DMA_CHANNEL 3

// HAL event for DMA complete notification (set by spi_sd.c)
static hive_hal_event_id_t s_dma_event_id = HIVE_HAL_EVENT_INVALID;

// DMA completion status (set by ISR, read by spi_sd.c via spi_ll_dma_ok)
static volatile bool s_dma_complete = false;
static volatile bool s_dma_error = false;

// SPI bus lock flag (checked by flow deck before SPI access)
static bool s_bus_locked = false;

// Dummy byte for DMA (TX sends 0xFF for reads, RX discards data for writes)
static uint8_t s_dma_dummy_tx = 0xFF;
static uint8_t s_dma_dummy_rx;

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

// ---------------------------------------------------------------------------
// DMA Bulk Transfer
// ---------------------------------------------------------------------------

void spi_ll_dma_init(hive_hal_event_id_t event_id) {
    s_dma_event_id = event_id;

    // Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Enable NVIC for DMA2 Stream 0 (SPI1_RX complete signals transfer done)
    NVIC_SetPriority(DMA2_Stream0_IRQn, 5);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void spi_ll_dma_start(void *buf, uint32_t len, spi_ll_dma_dir_t dir) {
    s_dma_complete = false;
    s_dma_error = false;

    // Disable both streams before configuration
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream0->CR & DMA_SxCR_EN)
        ;
    while (DMA2_Stream3->CR & DMA_SxCR_EN)
        ;

    // Clear all interrupt flags for Stream 0 (RX) and Stream 3 (TX)
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    DMA2->LIFCR = DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 |
                  DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3;

    // Configure RX DMA (Stream 0, Channel 3) - peripheral to memory
    DMA2_Stream0->CR = (SD_DMA_CHANNEL << 25) | // Channel select
                       DMA_SxCR_TCIE;           // Transfer complete interrupt
    DMA2_Stream0->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream0->NDTR = len;

    if (dir == SPI_LL_DIR_RX) {
        // RX: capture received data into caller's buffer
        DMA2_Stream0->M0AR = (uint32_t)buf;
        DMA2_Stream0->CR |= DMA_SxCR_MINC; // Memory increment
    } else {
        // TX: discard received bytes into dummy variable
        DMA2_Stream0->M0AR = (uint32_t)&s_dma_dummy_rx;
        // No MINC - write same location repeatedly
    }

    // Configure TX DMA (Stream 3, Channel 3) - memory to peripheral
    DMA2_Stream3->CR = (SD_DMA_CHANNEL << 25) | // Channel select
                       DMA_SxCR_DIR_0;          // Memory-to-peripheral
    DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
    DMA2_Stream3->NDTR = len;

    if (dir == SPI_LL_DIR_TX) {
        // TX: send actual data from caller's buffer
        DMA2_Stream3->M0AR = (uint32_t)buf;
        DMA2_Stream3->CR |= DMA_SxCR_MINC; // Memory increment
    } else {
        // RX: send 0xFF dummy bytes to clock data in
        s_dma_dummy_tx = 0xFF;
        DMA2_Stream3->M0AR = (uint32_t)&s_dma_dummy_tx;
        // No MINC - read same 0xFF byte repeatedly
    }

    // Enable SPI DMA requests
    SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

    // Start RX first (must be ready before TX clocks data out)
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2_Stream3->CR |= DMA_SxCR_EN;
}

bool spi_ll_dma_ok(void) {
    return s_dma_complete && !s_dma_error;
}

// ---------------------------------------------------------------------------
// SPI Bus Locking
// ---------------------------------------------------------------------------

void spi_ll_lock(void) {
    s_bus_locked = true;
}

void spi_ll_unlock(void) {
    s_bus_locked = false;
}

bool spi_ll_is_locked(void) {
    return s_bus_locked;
}

// ---------------------------------------------------------------------------
// DMA ISR
// ---------------------------------------------------------------------------

// SPI1 RX DMA complete (DMA2 Stream 0)
// RX finishes after TX, so this signals "entire transfer done"
void DMA2_Stream0_IRQHandler(void) {
    // Transfer complete
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;
        SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        s_dma_complete = true;
        hive_hal_event_signal(s_dma_event_id);
    }

    // Transfer error or FIFO error
    if (DMA2->LISR & (DMA_LISR_TEIF0 | DMA_LISR_DMEIF0 | DMA_LISR_FEIF0)) {
        DMA2->LIFCR = DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
        SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
        s_dma_error = true;
        s_dma_complete = true;
        hive_hal_event_signal(s_dma_event_id);
    }
}

#else

// Avoid empty translation unit warning when SD is disabled
typedef int spi_ll_sd_unused;

#endif // HIVE_ENABLE_SD
