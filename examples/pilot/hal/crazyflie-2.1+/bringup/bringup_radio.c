// Crazyflie 2.1+ Bring-Up - Radio (Syslink) Test

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
            process_syslink_packet();
        }
        s_rx_state = RX_START1;
        break;
    }
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

    // Configure PA4 (TXEN) as input with pull-down
    GPIOA->MODER &= ~GPIO_MODER_MODER4;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;
    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR4_1; // Pull-down

    // Configure USART6: 1Mbaud, 8N1
    USART6->CR1 = 0;
    USART6->BRR = USART6_BRR;
    USART6->CR2 = 0;
    USART6->CR3 = 0;
    USART6->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    s_rx_state = RX_START1;
    s_battery_voltage = 0.0f;
    s_has_battery_data = false;
}

void radio_poll(void) {
    while (USART6->SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART6->DR;
        rx_byte(byte);
    }
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
    swo_puts("OK\n");

    swo_puts("[RADIO] Waiting for battery packet...\n");

    // Poll for battery packet
    int elapsed = 0;
    const int poll_interval = 10;

    while (elapsed < timeout_ms) {
        radio_poll();

        if (s_has_battery_data) {
            swo_printf("[RADIO] Battery voltage: %fV... OK\n",
                       s_battery_voltage);
            return true;
        }

        // Simple delay
        volatile uint32_t count = poll_interval * 42000;
        while (count--)
            ;
        elapsed += poll_interval;
    }

    swo_puts("[RADIO] Timeout waiting for battery packet... FAIL\n");
    return false;
}
