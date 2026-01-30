// Crazyflie 2.1+ HAL - Syslink
//
// Implements radio communication via syslink protocol to the nRF51822.
// The nRF51 handles ESB radio protocol to Crazyradio PA on the ground.
//
// UART: USART6 at 1Mbaud (PC6=TX, PC7=RX)
// Flow control: PA4 (TXEN) indicates nRF51 ready to receive
//
// RX uses DMA2 Stream 2 in circular buffer mode for reliable reception
// at 1Mbaud. TX uses polling (we control timing, so no overrun risk).
//
// Protocol: After receiving one RADIO_RAW packet from nRF51, we may send one.
// The nRF51 periodically sends empty packets to enable TX.
//
// Blocking: Each send blocks for ~370us (37 bytes at 1Mbaud).
// Run telemetry actor at LOW priority to avoid affecting control loops.

#include "platform.h"
#include "stm32f4xx.h"
#include <stdbool.h>
#include <string.h>

// ----------------------------------------------------------------------------
// Syslink Constants
// ----------------------------------------------------------------------------

#define SYSLINK_START1 0xBC
#define SYSLINK_START2 0xCF
#define SYSLINK_RADIO_RAW 0x00
#define SYSLINK_PM_BATTERY 0x13
#define SYSLINK_MTU 64
#define RADIO_MTU 31 // Max payload for ESB packet

// ----------------------------------------------------------------------------
// UART Configuration (USART6)
// ----------------------------------------------------------------------------

// USART6 is on APB2 (84 MHz after our clock setup)
// Baud rate = 1,000,000
// BRR = APB2_CLK / BAUD = 84,000,000 / 1,000,000 = 84

#define SYSLINK_USART USART6
#define SYSLINK_BAUD_DIV 84

// GPIO pins (matching Bitcraze firmware)
#define SYSLINK_TX_PIN 6   // PC6
#define SYSLINK_RX_PIN 7   // PC7
#define SYSLINK_TXEN_PIN 4 // PA4 (flow control from nRF51)

// ----------------------------------------------------------------------------
// DMA Configuration
// ----------------------------------------------------------------------------

// DMA2 Stream 2, Channel 5 = USART6_RX
// Circular buffer mode for continuous reception
#define DMA_RX_STREAM DMA2_Stream2
#define DMA_RX_CHANNEL 5
#define DMA_RX_BUFFER_SIZE 256 // Must be power of 2 for efficient wrap handling

// DMA RX circular buffer
static uint8_t s_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
static volatile uint32_t s_dma_rx_read_pos = 0;

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

static volatile bool s_initialized = false;
static volatile bool s_tx_allowed = false;
static volatile float s_battery_voltage = 0.0f;

// RX callback with user data
typedef void (*radio_rx_callback_t)(const void *data, size_t len,
                                    void *user_data);
static radio_rx_callback_t s_rx_callback = NULL;
static void *s_rx_callback_user_data = NULL;

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

// ----------------------------------------------------------------------------
// DMA Initialization
// ----------------------------------------------------------------------------

static void dma_rx_init(void) {
    // Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Disable stream before configuration
    DMA_RX_STREAM->CR &= ~DMA_SxCR_EN;
    while (DMA_RX_STREAM->CR & DMA_SxCR_EN)
        ; // Wait for disable

    // Clear all interrupt flags for Stream 2
    // Stream 2 uses LIFCR (low interrupt flag clear register)
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;

    // Configure DMA stream
    // - Channel 5 (bits 27:25)
    // - Circular mode (CIRC)
    // - Memory increment (MINC)
    // - Peripheral to memory (DIR = 00)
    // - Byte size for both (MSIZE = 00, PSIZE = 00)
    DMA_RX_STREAM->CR = (DMA_RX_CHANNEL << DMA_SxCR_CHSEL_Pos) | // Channel 5
                        DMA_SxCR_CIRC | // Circular mode
                        DMA_SxCR_MINC;  // Memory increment

    // Peripheral address (USART6 data register)
    DMA_RX_STREAM->PAR = (uint32_t)&SYSLINK_USART->DR;

    // Memory address (our buffer)
    DMA_RX_STREAM->M0AR = (uint32_t)s_dma_rx_buffer;

    // Number of data items
    DMA_RX_STREAM->NDTR = DMA_RX_BUFFER_SIZE;

    // Enable the stream
    DMA_RX_STREAM->CR |= DMA_SxCR_EN;

    // Reset read position
    s_dma_rx_read_pos = 0;
}

// ----------------------------------------------------------------------------
// Low-Level UART with DMA
// ----------------------------------------------------------------------------

static void uart_init(void) {
    // Enable USART6 clock (APB2)
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // Configure PC6 (TX) and PC7 (RX) for USART6 (AF8)
    GPIOC->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);
    GPIOC->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // AF mode
    GPIOC->AFR[0] &= ~((0xFU << (6 * 4)) | (0xFU << (7 * 4)));
    GPIOC->AFR[0] |= (8U << (6 * 4)) | (8U << (7 * 4)); // AF8 = USART6
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;

    // Configure PA4 (TXEN) as input with pull-up
    // TXEN high = nRF51 ready to receive (matching Bitcraze firmware)
    GPIOA->MODER &= ~GPIO_MODER_MODER4; // Input mode
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_0; // Pull-up

    // Configure USART6: 1Mbaud, 8N1
    SYSLINK_USART->CR1 = 0; // Disable USART first
    SYSLINK_USART->BRR = SYSLINK_BAUD_DIV;
    SYSLINK_USART->CR2 = 0;              // 1 stop bit
    SYSLINK_USART->CR3 = USART_CR3_DMAR; // Enable DMA for RX

    // Initialize DMA before enabling USART
    dma_rx_init();

    // Enable USART, TX, RX
    SYSLINK_USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static inline void uart_putc(uint8_t c) {
    while (!(SYSLINK_USART->SR & USART_SR_TXE))
        ;
    SYSLINK_USART->DR = c;
}

static inline bool txen_ready(void) {
    return (GPIOA->IDR & (1U << SYSLINK_TXEN_PIN)) != 0;
}

// Get current DMA write position in buffer
static inline uint32_t dma_get_write_pos(void) {
    // NDTR counts down from buffer size
    // Write position = buffer_size - NDTR
    return DMA_RX_BUFFER_SIZE - DMA_RX_STREAM->NDTR;
}

// ----------------------------------------------------------------------------
// Syslink TX
// ----------------------------------------------------------------------------

static int syslink_send(uint8_t type, const void *data, size_t len) {
    if (len > SYSLINK_MTU) {
        return -1;
    }

    // Check hardware flow control
    if (!txen_ready()) {
        return -1;
    }

    // Calculate Fletcher-8 checksum over type + length + data
    uint8_t ck_a = type;
    uint8_t ck_b = ck_a;
    ck_a += (uint8_t)len;
    ck_b += ck_a;

    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        ck_a += bytes[i];
        ck_b += ck_a;
    }

    // Send packet
    uart_putc(SYSLINK_START1);
    uart_putc(SYSLINK_START2);
    uart_putc(type);
    uart_putc((uint8_t)len);
    for (size_t i = 0; i < len; i++) {
        uart_putc(bytes[i]);
    }
    uart_putc(ck_a);
    uart_putc(ck_b);

    return 0;
}

// ----------------------------------------------------------------------------
// Syslink RX
// ----------------------------------------------------------------------------

static void syslink_process_packet(void) {
    switch (s_rx_type) {
    case SYSLINK_RADIO_RAW:
        // Flow control: receiving a packet allows us to send one
        s_tx_allowed = true;

        // Pass to application if callback registered and data present
        if (s_rx_callback && s_rx_length > 0) {
            s_rx_callback(s_rx_data, s_rx_length, s_rx_callback_user_data);
        }
        break;

    case SYSLINK_PM_BATTERY:
        // Battery packet: flags (1 byte) + voltage (4 bytes float)
        if (s_rx_length >= 5) {
            float voltage;
            memcpy(&voltage, &s_rx_data[1], sizeof(float));
            s_battery_voltage = voltage;
        }
        break;

    default:
        // Ignore other packet types
        break;
    }
}

static void syslink_rx_byte(uint8_t byte) {
    switch (s_rx_state) {
    case RX_START1:
        if (byte == SYSLINK_START1) {
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
            // Invalid length, reset
            s_rx_state = RX_START1;
        } else if (s_rx_length > 0) {
            s_rx_state = RX_DATA;
        } else {
            s_rx_state = RX_CKSUM_A;
        }
        break;

    case RX_DATA:
        // Bounds check to prevent buffer overflow (defense in depth)
        if (s_rx_index >= SYSLINK_MTU) {
            s_rx_state = RX_START1;
            break;
        }
        s_rx_data[s_rx_index++] = byte;
        s_rx_ck_a += byte;
        s_rx_ck_b += s_rx_ck_a;
        if (s_rx_index >= s_rx_length) {
            s_rx_state = RX_CKSUM_A;
        }
        break;

    case RX_CKSUM_A:
        s_rx_state = (byte == s_rx_ck_a) ? RX_CKSUM_B : RX_START1;
        break;

    case RX_CKSUM_B:
        if (byte == s_rx_ck_b) {
            syslink_process_packet();
        }
        s_rx_state = RX_START1;
        break;
    }
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

int hal_esb_init(void) {
    uart_init();

    s_tx_allowed = false;
    s_rx_state = RX_START1;
    s_rx_callback = NULL;
    s_rx_callback_user_data = NULL;
    s_battery_voltage = 0.0f;
    s_initialized = true;

    return 0;
}

int hal_esb_send(const void *data, size_t len) {
    if (!s_initialized) {
        return -1;
    }

    if (len > RADIO_MTU) {
        return -1; // Too large for ESB packet
    }

    if (!s_tx_allowed) {
        return -1; // Flow control: not allowed to send yet
    }

    s_tx_allowed = false; // Consumed our TX slot

    return syslink_send(SYSLINK_RADIO_RAW, data, len);
}

bool hal_esb_tx_ready(void) {
    return s_initialized && s_tx_allowed;
}

void hal_esb_poll(void) {
    if (!s_initialized) {
        return;
    }

    // Get current DMA write position
    uint32_t write_pos = dma_get_write_pos();
    uint32_t read_pos = s_dma_rx_read_pos;

    // Process all bytes between read and write positions
    while (read_pos != write_pos) {
        uint8_t byte = s_dma_rx_buffer[read_pos];
        syslink_rx_byte(byte);

        // Advance read position with wrap
        read_pos = (read_pos + 1) & (DMA_RX_BUFFER_SIZE - 1);
    }

    // Update read position
    s_dma_rx_read_pos = read_pos;
}

void hal_esb_set_rx_callback(void (*callback)(const void *data, size_t len,
                                              void *user_data),
                             void *user_data) {
    s_rx_callback = callback;
    s_rx_callback_user_data = user_data;
}

float hal_power_get_battery(void) {
    return s_battery_voltage;
}
