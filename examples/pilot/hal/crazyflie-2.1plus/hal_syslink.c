// Crazyflie 2.1+ HAL - Syslink
//
// Implements radio communication via syslink protocol to the nRF51822.
// The nRF51 handles ESB radio protocol to Crazyradio PA on the ground.
//
// UART: USART6 at 1Mbaud (PC6=TX, PC7=RX)
// Flow control: PA4 (TXEN) indicates nRF51 ready to receive
//
// RX uses interrupt-driven receive (RXNE) matching Bitcraze's default.
// TX uses polling (we control timing, so no overrun risk).
//
// Blocking: Each send blocks for ~370us (37 bytes at 1Mbaud).
// Run telemetry actor at LOW priority to avoid affecting control loops.

#include "platform.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <stdbool.h>
#include <string.h>

// ----------------------------------------------------------------------------
// Syslink Constants
// ----------------------------------------------------------------------------

#define SYSLINK_START1 0xBC
#define SYSLINK_START2 0xCF
#define SYSLINK_RADIO_RAW 0x00
#define SYSLINK_PM_BATTERY_STATE 0x13
#define SYSLINK_PM_BATTERY_AUTOUPDATE 0x14
#define SYSLINK_MTU 64
#define RADIO_MTU 31 // Max payload for ESB packet

// ----------------------------------------------------------------------------
// UART Configuration (USART6)
// ----------------------------------------------------------------------------

// USART6 is on APB2 (84 MHz after our clock setup)
// Baud rate = 1,000,000
// BRR = APB2_CLK / BAUD = 84,000,000 / 1,000,000 = 84

#define SYSLINK_USART USART6

// GPIO pins (matching Bitcraze firmware)
#define SYSLINK_TX_PIN 6   // PC6
#define SYSLINK_RX_PIN 7   // PC7
#define SYSLINK_TXEN_PIN 4 // PA4 (flow control: LOW=ready, HIGH=busy)

// ----------------------------------------------------------------------------
// RX Ring Buffer (filled by interrupt)
// ----------------------------------------------------------------------------

#define RX_BUFFER_SIZE 256 // Must be power of 2 for efficient wrap handling

static volatile uint8_t s_rx_buffer[RX_BUFFER_SIZE];
static volatile uint32_t s_rx_write_pos = 0;
static volatile uint32_t s_rx_read_pos = 0;

// ----------------------------------------------------------------------------
// State
// ----------------------------------------------------------------------------

static volatile bool s_initialized = false;
static volatile float s_battery_voltage = 0.0f;

// Debug counters for RX diagnostics
static volatile uint32_t s_isr_count = 0;
static volatile uint32_t s_rx_byte_count = 0;
static volatile uint32_t s_rx_errors = 0;
static volatile uint32_t s_packet_count = 0;

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
// USART6 Interrupt Handler
// ----------------------------------------------------------------------------

void __attribute__((used)) USART6_IRQHandler(void) {
    s_isr_count++; // Track ISR calls for debugging

    // Check RXNE (receive not empty) flag
    if (SYSLINK_USART->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(SYSLINK_USART->DR & 0xFF);
        s_rx_byte_count++; // Track bytes received

        // Store in ring buffer (drop if full)
        uint32_t next_write = (s_rx_write_pos + 1) & (RX_BUFFER_SIZE - 1);
        if (next_write != s_rx_read_pos) {
            s_rx_buffer[s_rx_write_pos] = byte;
            s_rx_write_pos = next_write;
        }
    }

    // Clear any error flags by reading SR then DR
    if (SYSLINK_USART->SR &
        (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        s_rx_errors++;           // Track errors
        (void)SYSLINK_USART->DR; // Clear error flags
    }
}

// ----------------------------------------------------------------------------
// Low-Level UART (interrupt RX, polled TX)
// ----------------------------------------------------------------------------

static void uart_init(void) {
    // Use STM32 Standard Peripheral Library - exactly matching Bitcraze firmware
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable GPIO and USART clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

    // Configure PC7 (RX) as alternate function with pull-up
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure PC6 (TX) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Map PC6 and PC7 to USART6 (AF8)
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    // Configure USART6: 1Mbaud, 8N1
    USART_InitStructure.USART_BaudRate = 1000000;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl =
        USART_HardwareFlowControl_None;
    USART_Init(USART6, &USART_InitStructure);

    // Configure NVIC for USART6 interrupt (matching Bitcraze priority 5)
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable RXNE interrupt
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    // Configure PA4 (TXEN) as input with pull-up for flow control
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Enable USART6
    USART_Cmd(USART6, ENABLE);
}

static inline void uart_putc(uint8_t c) {
    while (!(SYSLINK_USART->SR & USART_SR_TXE))
        ;
    SYSLINK_USART->DR = c;
}

static inline bool txen_ready(void) {
    // TXEN LOW = nRF51 ready to receive (Bitcraze convention)
    // TXEN HIGH = nRF51 buffer full, don't send
    return (GPIOA->IDR & (1U << SYSLINK_TXEN_PIN)) == 0;
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
        // Pass to application if callback registered and data present
        if (s_rx_callback && s_rx_length > 0) {
            s_rx_callback(s_rx_data, s_rx_length, s_rx_callback_user_data);
        }
        break;

    case SYSLINK_PM_BATTERY_STATE:
        // Battery state packet: flags (1 byte) + voltage (4 bytes float)
        // Type 0x13 = SYSLINK_PM_BATTERY_STATE in Bitcraze firmware
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
            s_packet_count++; // Track successful packets
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

    s_rx_state = RX_START1;
    s_rx_callback = NULL;
    s_rx_callback_user_data = NULL;
    s_battery_voltage = 0.0f;
    s_initialized = true;

    // Enable battery status autoupdate from nRF51.
    // The nRF51 won't send battery packets until this command is received.
    // Delay ~600us at 168MHz for UART to stabilize after init.
    for (volatile int i = 0; i < 100000; i++) {
        __NOP();
    }

    syslink_send(SYSLINK_PM_BATTERY_AUTOUPDATE, NULL, 0);

    return 0;
}

int hal_esb_send(const void *data, size_t len) {
    if (!s_initialized) {
        return -1;
    }

    if (len > RADIO_MTU) {
        return -1; // Too large for ESB packet
    }

    // Flow control via TXEN hardware line (checked in syslink_send)
    return syslink_send(SYSLINK_RADIO_RAW, data, len);
}

bool hal_esb_tx_ready(void) {
    // Ready when initialized and nRF51 TXEN line is LOW
    return s_initialized && txen_ready();
}

void hal_esb_poll(void) {
    if (!s_initialized) {
        return;
    }

    // Process all bytes in ring buffer
    while (s_rx_read_pos != s_rx_write_pos) {
        uint8_t byte = s_rx_buffer[s_rx_read_pos];
        syslink_rx_byte(byte);

        // Advance read position with wrap
        s_rx_read_pos = (s_rx_read_pos + 1) & (RX_BUFFER_SIZE - 1);
    }
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

// ----------------------------------------------------------------------------
// Debug API (for RX diagnostics)
// ----------------------------------------------------------------------------

uint32_t hal_esb_debug_isr_count(void) {
    return s_isr_count;
}

uint32_t hal_esb_debug_rx_bytes(void) {
    return s_rx_byte_count;
}

uint32_t hal_esb_debug_errors(void) {
    return s_rx_errors;
}

uint32_t hal_esb_debug_packets(void) {
    return s_packet_count;
}
