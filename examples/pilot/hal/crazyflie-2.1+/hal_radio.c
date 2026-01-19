// Crazyflie 2.1+ Radio HAL - Syslink Implementation
//
// Implements radio communication via syslink protocol to the nRF51822.
// The nRF51 handles ESB radio protocol to Crazyradio PA on the ground.
//
// UART: USART2 at 1Mbaud (PA2=TX, PA3=RX)
// Flow control: PC4 (TXEN) indicates nRF51 ready to receive
//
// Protocol: After receiving one RADIO_RAW packet from nRF51, we may send one.
// The nRF51 periodically sends empty packets to enable TX.
//
// Blocking: Each send blocks for ~370us (37 bytes at 1Mbaud).
// Run telemetry actor at LOW priority to avoid affecting control loops.

#include "platform_crazyflie.h"
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
// UART Configuration (USART2)
// ----------------------------------------------------------------------------

// USART2 is on APB1 (42 MHz after our clock setup)
// Baud rate = 1,000,000
// BRR = APB1_CLK / BAUD = 42,000,000 / 1,000,000 = 42

#define SYSLINK_USART USART2
#define SYSLINK_BAUD_DIV 42

// GPIO pins
#define SYSLINK_TX_PIN 2   // PA2
#define SYSLINK_RX_PIN 3   // PA3
#define SYSLINK_TXEN_PIN 4 // PC4 (flow control from nRF51)

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

static volatile bool s_initialized = false;
static volatile bool s_tx_allowed = false;
static volatile float s_battery_voltage = 0.0f;

// RX callback
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
// Low-Level UART
// ----------------------------------------------------------------------------

static void uart_init(void) {
    // Enable USART2 clock (APB1)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Enable GPIOA clock (already done in platform_crazyflie.c, but be safe)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    // Configure PA2 (TX) and PA3 (RX) for USART2 (AF7)
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1); // AF mode
    GPIOA->AFR[0] &= ~((0xFU << (2 * 4)) | (0xFU << (3 * 4)));
    GPIOA->AFR[0] |= (7U << (2 * 4)) | (7U << (3 * 4)); // AF7 = USART2
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3;

    // Configure PC4 (TXEN) as input with pull-down
    GPIOC->MODER &= ~GPIO_MODER_MODER4; // Input mode
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOC->PUPDR |= GPIO_PUPDR_PUPDR4_1; // Pull-down

    // Configure USART2: 1Mbaud, 8N1
    SYSLINK_USART->CR1 = 0; // Disable USART first
    SYSLINK_USART->BRR = SYSLINK_BAUD_DIV;
    SYSLINK_USART->CR2 = 0; // 1 stop bit
    SYSLINK_USART->CR3 = 0; // No flow control (we handle TXEN manually)

    // Enable USART, TX, RX
    SYSLINK_USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static inline void uart_putc(uint8_t c) {
    while (!(SYSLINK_USART->SR & USART_SR_TXE))
        ;
    SYSLINK_USART->DR = c;
}

static inline bool uart_rxne(void) {
    return (SYSLINK_USART->SR & USART_SR_RXNE) != 0;
}

static inline uint8_t uart_getc(void) {
    return (uint8_t)SYSLINK_USART->DR;
}

static inline bool txen_ready(void) {
    return (GPIOC->IDR & (1U << SYSLINK_TXEN_PIN)) != 0;
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
            memcpy(&s_battery_voltage, &s_rx_data[1], sizeof(float));
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

int hal_radio_init(void) {
    uart_init();

    s_tx_allowed = false;
    s_rx_state = RX_START1;
    s_rx_callback = NULL;
    s_battery_voltage = 0.0f;
    s_initialized = true;

    return 0;
}

int hal_radio_send(const void *data, size_t len) {
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

bool hal_radio_tx_ready(void) {
    return s_initialized && s_tx_allowed;
}

void hal_radio_poll(void) {
    if (!s_initialized) {
        return;
    }

    // Process all available bytes
    while (uart_rxne()) {
        uint8_t byte = uart_getc();
        syslink_rx_byte(byte);
    }
}

void hal_radio_set_rx_callback(void (*callback)(const void *data, size_t len,
                                                void *user_data),
                               void *user_data) {
    s_rx_callback = callback;
    s_rx_callback_user_data = user_data;
}

float hal_radio_get_battery(void) {
    return s_battery_voltage;
}
