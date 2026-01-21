// Crazyflie 2.1+ Bring-Up - USART1 Debug Output

#include "bringup_uart.h"
#include "stm32f4xx.h"
#include <string.h>

// USART1 configuration
// APB2 = 84 MHz, Baud = 115200
// BRR = 84000000 / 115200 = 729.17 -> mantissa=729, fraction=0.17*16=2.7->3
// BRR = (729 << 4) | 3 = 0x2D93
#define USART1_BRR 0x2D93

// Simple delay using SysTick
static volatile uint32_t s_ticks = 0;

void SysTick_Handler(void) {
    s_ticks++;
}

// Used by SysTick-based timing
__attribute__((unused)) static void delay_ms(uint32_t ms) {
    uint32_t start = s_ticks;
    while ((s_ticks - start) < ms) {
        __WFI();
    }
}

void uart_init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure PA9 (TX) and PA10 (RX) as alternate function
    // AF7 = USART1
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    GPIOA->AFR[1] &= ~((0xFU << ((9 - 8) * 4)) | (0xFU << ((10 - 8) * 4)));
    GPIOA->AFR[1] |= (7U << ((9 - 8) * 4)) | (7U << ((10 - 8) * 4));
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;

    // Configure USART1: 115200 8N1
    USART1->CR1 = 0;
    USART1->BRR = USART1_BRR;
    USART1->CR2 = 0;
    USART1->CR3 = 0;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    // Configure SysTick for 1ms ticks
    SysTick_Config(SystemCoreClock / 1000);
}

void uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE))
        ;
    USART1->DR = (uint8_t)c;
}

void uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s++);
    }
}

bool uart_available(void) {
    return (USART1->SR & USART_SR_RXNE) != 0;
}

char uart_getc(void) {
    while (!uart_available())
        ;
    return (char)USART1->DR;
}

int uart_getc_timeout(int timeout_ms) {
    if (timeout_ms == 0) {
        // Non-blocking
        if (uart_available()) {
            return (int)(unsigned char)USART1->DR;
        }
        return -1;
    }

    if (timeout_ms < 0) {
        // Infinite wait
        return (int)(unsigned char)uart_getc();
    }

    // Timed wait
    uint32_t start = s_ticks;
    while ((s_ticks - start) < (uint32_t)timeout_ms) {
        if (uart_available()) {
            return (int)(unsigned char)USART1->DR;
        }
    }
    return -1;
}

// Simple number to string conversion
static void uart_print_uint(uint32_t val, int base, int width, char pad) {
    char buf[12];
    int i = 0;
    const char *digits = "0123456789ABCDEF";

    if (val == 0) {
        buf[i++] = '0';
    } else {
        while (val > 0) {
            buf[i++] = digits[val % base];
            val /= base;
        }
    }

    // Padding
    while (i < width) {
        buf[i++] = pad;
    }

    // Reverse and print
    while (i > 0) {
        uart_putc(buf[--i]);
    }
}

static void uart_print_int(int32_t val) {
    if (val < 0) {
        uart_putc('-');
        val = -val;
    }
    uart_print_uint((uint32_t)val, 10, 0, '0');
}

// Simple float to string (2 decimal places)
static void uart_print_float(float val) {
    if (val < 0) {
        uart_putc('-');
        val = -val;
    }
    int32_t integer = (int32_t)val;
    int32_t frac = (int32_t)((val - integer) * 100 + 0.5f);
    if (frac >= 100) {
        integer++;
        frac -= 100;
    }
    uart_print_uint((uint32_t)integer, 10, 0, '0');
    uart_putc('.');
    uart_print_uint((uint32_t)frac, 10, 2, '0');
}

void uart_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            int width = 0;
            char pad = ' ';

            // Check for zero padding
            if (*fmt == '0') {
                pad = '0';
                fmt++;
            }

            // Parse width
            while (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
                fmt++;
            }

            switch (*fmt) {
            case 'd':
            case 'i':
                uart_print_int(va_arg(args, int));
                break;
            case 'u':
                uart_print_uint(va_arg(args, unsigned int), 10, width, pad);
                break;
            case 'x':
            case 'X':
                uart_print_uint(va_arg(args, unsigned int), 16, width, pad);
                break;
            case 'f':
                uart_print_float((float)va_arg(args, double));
                break;
            case 's':
                uart_puts(va_arg(args, const char *));
                break;
            case 'c':
                uart_putc((char)va_arg(args, int));
                break;
            case '%':
                uart_putc('%');
                break;
            default:
                uart_putc('%');
                uart_putc(*fmt);
                break;
            }
        } else {
            if (*fmt == '\n') {
                uart_putc('\r');
            }
            uart_putc(*fmt);
        }
        fmt++;
    }

    va_end(args);
}

void uart_print_result(const char *phase, const char *test, bool passed) {
    uart_printf("[%s] %s... %s\n", phase, test, passed ? "OK" : "FAIL");
}

void uart_hex_dump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (i > 0 && (i % 16) == 0) {
            uart_puts("\n");
        }
        uart_printf("%02X ", data[i]);
    }
    uart_puts("\n");
}
