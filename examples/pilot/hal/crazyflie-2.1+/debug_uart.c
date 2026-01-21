// Crazyflie 2.1+ Debug UART Output
//
// USART1 on PA9 (TX) and PA10 (RX) at 115200 baud.
// Extracted from bringup/bringup_uart.c for use in HAL.

#include "debug_uart.h"
#include "stm32f4xx.h"

// USART1 configuration
// APB2 = 84 MHz, Baud = 115200
// BRR = 84000000 / 115200 = 729.17 -> mantissa=729, fraction=0.17*16=2.7->3
// BRR = (729 << 4) | 3 = 0x2D93
#define USART1_BRR 0x2D93

void debug_uart_init(void) {
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
}

static void debug_uart_putc(char c) {
    while (!(USART1->SR & USART_SR_TXE))
        ;
    USART1->DR = (uint8_t)c;
}

static void debug_uart_puts(const char *s) {
    while (*s) {
        if (*s == '\n') {
            debug_uart_putc('\r');
        }
        debug_uart_putc(*s++);
    }
}

// Simple number to string conversion
static void debug_uart_print_uint(uint32_t val, int base, int width, char pad) {
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
        debug_uart_putc(buf[--i]);
    }
}

static void debug_uart_print_int(int32_t val) {
    if (val < 0) {
        debug_uart_putc('-');
        val = -val;
    }
    debug_uart_print_uint((uint32_t)val, 10, 0, '0');
}

// Simple float to string (2 decimal places)
static void debug_uart_print_float(float val) {
    if (val < 0) {
        debug_uart_putc('-');
        val = -val;
    }
    int32_t integer = (int32_t)val;
    int32_t frac = (int32_t)((val - integer) * 100 + 0.5f);
    if (frac >= 100) {
        integer++;
        frac -= 100;
    }
    debug_uart_print_uint((uint32_t)integer, 10, 0, '0');
    debug_uart_putc('.');
    debug_uart_print_uint((uint32_t)frac, 10, 2, '0');
}

void debug_uart_vprintf(const char *fmt, va_list args) {
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
                debug_uart_print_int(va_arg(args, int));
                break;
            case 'u':
                debug_uart_print_uint(va_arg(args, unsigned int), 10, width,
                                      pad);
                break;
            case 'x':
            case 'X':
                debug_uart_print_uint(va_arg(args, unsigned int), 16, width,
                                      pad);
                break;
            case 'f':
                debug_uart_print_float((float)va_arg(args, double));
                break;
            case 's':
                debug_uart_puts(va_arg(args, const char *));
                break;
            case 'c':
                debug_uart_putc((char)va_arg(args, int));
                break;
            case '%':
                debug_uart_putc('%');
                break;
            default:
                debug_uart_putc('%');
                debug_uart_putc(*fmt);
                break;
            }
        } else {
            if (*fmt == '\n') {
                debug_uart_putc('\r');
            }
            debug_uart_putc(*fmt);
        }
        fmt++;
    }
}

void debug_uart_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_uart_vprintf(fmt, args);
    va_end(args);
}
