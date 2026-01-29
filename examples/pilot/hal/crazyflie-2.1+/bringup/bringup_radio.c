// Crazyflie 2.1+ Bring-Up - Radio (Syslink) Test
//
// Uses DMA for reliable reception at 1Mbaud (same as production HAL)

#include "bringup_radio.h"
#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// Syslink protocol constants
#define SYSLINK_START1 0xBC
#define SYSLINK_START2 0xCF
#define SYSLINK_PM_BATTERY 0x13
#define SYSLINK_MTU 64

// USART6 baud rate = 1,000,000
// APB2 = 84 MHz
// BRR = 84000000 / 1000000 = 84
#define USART6_BRR 84

// DMA Configuration
// DMA2 Stream 2, Channel 5 = USART6_RX
#define DMA_RX_STREAM DMA2_Stream2
#define DMA_RX_CHANNEL 5
#define DMA_RX_BUFFER_SIZE 256

static uint8_t s_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
static volatile uint32_t s_dma_rx_read_pos = 0;

// RX state machine
typedef enum {
    RX_START1,
    RX_START2,
    RX_TYPE,
    RX_LENGTH,
    RX_DATA,
    RX_CKSUM_A,
    RX_CKSUM_B
} rx_state_t;

static rx_state_t s_rx_state = RX_START1;
static uint8_t s_rx_type;
static uint8_t s_rx_length;
static uint8_t s_rx_data[SYSLINK_MTU];
static uint8_t s_rx_index;
static uint8_t s_rx_ck_a, s_rx_ck_b;

static volatile float s_battery_voltage = 0.0f;
static volatile bool s_has_battery_data = false;

// Diagnostic counters
static uint32_t s_rx_byte_count = 0;
static uint32_t s_start1_count = 0;
static uint32_t s_packet_count = 0;
static uint32_t s_checksum_errors = 0;
static uint8_t s_last_type = 0;

static void process_syslink_packet(void) {
    if (s_rx_type == SYSLINK_PM_BATTERY && s_rx_length >= 5) {
        // Battery packet: flags (1 byte) + voltage (4 bytes float)
        float voltage;
        memcpy(&voltage, &s_rx_data[1], sizeof(float));
        s_battery_voltage = voltage;
        s_has_battery_data = true;
    }
}

static void rx_byte(uint8_t byte) {
    s_rx_byte_count++;

    switch (s_rx_state) {
    case RX_START1:
        if (byte == SYSLINK_START1) {
            s_start1_count++;
            s_rx_state = RX_START2;
        }
        break;

    case RX_START2:
        s_rx_state = (byte == SYSLINK_START2) ? RX_TYPE : RX_START1;
        break;

    case RX_TYPE:
        s_rx_type = byte;
        s_rx_ck_a = byte;
        s_rx_ck_b = byte;
        s_rx_state = RX_LENGTH;
        break;

    case RX_LENGTH:
        s_rx_length = byte;
        s_rx_ck_a += byte;
        s_rx_ck_b += s_rx_ck_a;
        s_rx_index = 0;
        if (s_rx_length > SYSLINK_MTU) {
            s_rx_state = RX_START1;
        } else if (s_rx_length > 0) {
            s_rx_state = RX_DATA;
        } else {
            s_rx_state = RX_CKSUM_A;
        }
        break;

    case RX_DATA:
        s_rx_data[s_rx_index++] = byte;
        s_rx_ck_a += byte;
        s_rx_ck_b += s_rx_ck_a;
        if (s_rx_index >= s_rx_length) {
            s_rx_state = RX_CKSUM_A;
        }
        break;

    case RX_CKSUM_A:
        if (byte == s_rx_ck_a) {
            s_rx_state = RX_CKSUM_B;
        } else {
            s_checksum_errors++;
            s_rx_state = RX_START1;
        }
        break;

    case RX_CKSUM_B:
        if (byte == s_rx_ck_b) {
            s_packet_count++;
            s_last_type = s_rx_type;
            process_syslink_packet();
        } else {
            s_checksum_errors++;
        }
        s_rx_state = RX_START1;
        break;
    }
}

static void dma_rx_init(void) {
    // Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable stream before configuration
    DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while (DMA_RX_STREAM->CR & DMA_SxCR_EN)
        ; // Wait for disable

    // Clear all interrupt flags for Stream 2
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;

    // Configure DMA stream
    // - Channel 5 (bits 27:25)
    // - Circular mode (CIRC)
    // - Memory increment (MINC)
    // - Peripheral to memory (DIR = 00)
    DMA_RX_STREAM->CR =
        (DMA_RX_CHANNEL << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_CIRC | DMA_SxCR_MINC;

    // Peripheral address (USART6 data register)
    DMA_RX_STREAM->PAR = (uint32_t)&USART6->DR;

    // Memory address (our buffer)
    DMA_RX_STREAM->M0AR = (uint32_t)s_dma_rx_buffer;

    // Number of data items
    DMA_RX_STREAM->NDTR = DMA_RX_BUFFER_SIZE;

    // Enable the stream
    DMA_RX_STREAM->CR |= DMA_SxCR_EN;

    // Reset read position
    s_dma_rx_read_pos = 0;
}

void radio_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    // Configure PC6 (TX) and PC7 (RX) as alternate function AF8
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
    GPIOC->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC->AFR[0] |= (8U << (6 * 4)) | (8U << (7 * 4)); // AF8 = USART6
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;

    // Configure PA4 (TXEN) as input with pull-up
    // NRF51 uses this for flow control - HIGH = ready to receive
    GPIOA->MODER &= ~GPIO_MODER_MODER4;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0; // Pull-up

    // Initialize DMA before USART
    dma_rx_init();

    // Configure USART6: 1Mbaud, 8N1, DMA RX
    USART6->CR1 = 0;
    USART6->BRR = USART6_BRR;
    USART6->CR2 = 0;
    USART6->CR3 = USART_CR3_DMAR; // Enable DMA for RX
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    s_rx_state = RX_START1;
    s_battery_voltage = 0.0f;
    s_has_battery_data = false;
}

void radio_poll(void) {
    // Get current DMA write position (NDTR counts down)
    uint32_t write_pos = DMA_RX_BUFFER_SIZE - DMA_RX_STREAM->NDTR;
    uint32_t read_pos = s_dma_rx_read_pos;

    // Process all bytes between read and write positions
    while (read_pos != write_pos) {
        uint8_t byte = s_dma_rx_buffer[read_pos];
        rx_byte(byte);

        // Advance read position with wrap
        read_pos = (read_pos + 1) & (DMA_RX_BUFFER_SIZE - 1);
    }

    // Update read position
    s_dma_rx_read_pos = read_pos;
}

float radio_get_battery_voltage(void) {
    return s_battery_voltage;
}

bool radio_has_battery_data(void) {
    return s_has_battery_data;
}

bool radio_run_test(int timeout_ms) {
    swo_puts("[RADIO] Initializing syslink (USART6)... ");

    radio_init();

    // Reset diagnostics
    s_rx_byte_count = 0;
    s_start1_count = 0;
    s_packet_count = 0;
    s_checksum_errors = 0;
    s_last_type = 0;

    swo_puts("OK\n");

    // Check initial TXEN state
    bool txen = (GPIOA->IDR & (1 << 4)) != 0;
    swo_printf("[RADIO] PA4 (TXEN) initial state: %s\n", txen ? "HIGH" : "LOW");

    // Try sending a version request to wake up nRF51
    // SYSLINK_OW_GETINFO = 0x20 (one-wire info request)
    swo_puts("[RADIO] Sending wakeup packet to nRF51...\n");
    {
        // Send empty RADIO_RAW packet to trigger communication
        uint8_t type = 0x00; // SYSLINK_RADIO_RAW
        uint8_t len = 0;
        uint8_t ck_a = type;
        uint8_t ck_b = ck_a;
        ck_a += len;
        ck_b += ck_a;

        // Wait for TX empty
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = SYSLINK_START1;
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = SYSLINK_START2;
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = type;
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = len;
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = ck_a;
        while (!(USART6->SR & USART_SR_TXE))
            ;
        USART6->DR = ck_b;

        // Small delay for nRF51 to process
        volatile uint32_t d = 100000;
        while (d--)
            ;
    }

    swo_puts("[RADIO] Waiting for battery packet...\n");

    // Poll for battery packet
    int elapsed = 0;
    const int poll_interval = 10;
    int last_report = 0;

    while (elapsed < timeout_ms) {
        radio_poll();

        if (s_has_battery_data) {
            swo_printf("[RADIO] Battery voltage: %.2fV... OK\n",
                       (double)s_battery_voltage);
            return true;
        }

        // Progress report every second
        if (elapsed - last_report >= 1000) {
            swo_printf("[RADIO] %ds: %u bytes, %u packets\n", elapsed / 1000,
                       s_rx_byte_count, s_packet_count);
            last_report = elapsed;
        }

        // Simple delay
        volatile uint32_t count = poll_interval * 42000;
        while (count--)
            ;
        elapsed += poll_interval;
    }

    swo_puts("[RADIO] Timeout waiting for battery packet... FAIL\n");
    swo_printf(
        "[RADIO] Diagnostics: %u bytes, %u start1, %u packets, %u ckerr\n",
        s_rx_byte_count, s_start1_count, s_packet_count, s_checksum_errors);
    if (s_packet_count > 0) {
        swo_printf("[RADIO] Last packet type: 0x%02X\n", s_last_type);
    }

    // Check TXEN pin state
    txen = (GPIOA->IDR & (1 << 4)) != 0;
    swo_printf("[RADIO] PA4 (TXEN) final state: %s\n", txen ? "HIGH" : "LOW");

    // Debug DMA state
    uint32_t dma_cr = DMA_RX_STREAM->CR;
    uint32_t dma_ndtr = DMA_RX_STREAM->NDTR;
    uint32_t write_pos = DMA_RX_BUFFER_SIZE - dma_ndtr;
    swo_printf("[RADIO] DMA CR=0x%08X NDTR=%u write_pos=%u\n", dma_cr, dma_ndtr,
               write_pos);

    // Dump first few bytes from DMA buffer if any
    if (write_pos > 0 || s_dma_rx_read_pos > 0) {
        swo_puts("[RADIO] DMA buffer (first 16 bytes): ");
        for (int i = 0; i < 16 && i < DMA_RX_BUFFER_SIZE; i++) {
            swo_printf("%02X ", s_dma_rx_buffer[i]);
        }
        swo_puts("\n");
    }

    // Also check USART status
    uint32_t usart_sr = USART6->SR;
    uint32_t usart_cr1 = USART6->CR1;
    uint32_t usart_cr3 = USART6->CR3;
    swo_printf("[RADIO] USART SR=0x%04X CR1=0x%04X CR3=0x%04X\n", usart_sr,
               usart_cr1, usart_cr3);

    return false;
}
