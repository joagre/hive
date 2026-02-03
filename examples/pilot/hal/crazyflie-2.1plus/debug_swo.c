// Crazyflie 2.1+ Debug SWO Output
//
// Serial Wire Output (SWO) via ITM stimulus port 0.
// Requires STLink V3 or compatible debugger with SWO support.
//
// Use st-trace to view output:
//   st-trace -c 168
//
// This module ONLY handles SWO output. Early log buffering is handled
// at the hal_printf() layer in hal_debug.c.

#include "debug_swo.h"
#include "stm32f4xx.h"
#include <stdint.h>

// SWO configuration
static uint32_t s_swo_initialized = 0;

void debug_swo_init(uint32_t cpu_freq_hz, uint32_t swo_baud) {
    // Enable DBGMCU clock and trace output
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Enable trace and keep debug active during sleep/stop/standby
    // Without DBG_SLEEP, WFI breaks the ST-Link debug connection
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN | DBGMCU_CR_DBG_SLEEP |
                  DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;

    // Configure PB3 as SWO output (AF0 = TRACESWO)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER &= ~GPIO_MODER_MODER3;
    GPIOB->MODER |= GPIO_MODER_MODER3_1;      // AF mode
    GPIOB->AFR[0] &= ~(0xFUL << (3 * 4));     // Clear AF bits for PB3
    GPIOB->AFR[0] |= (0x0UL << (3 * 4));      // AF0 = TRACESWO
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3; // High speed

    // Enable trace in core debug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Unlock ITM
    ITM->LAR = 0xC5ACCE55;

    // Configure TPIU for async SWO (NRZ/UART-like encoding)
    TPI->SPPR = 2; // NRZ encoding
    TPI->ACPR = (cpu_freq_hz / swo_baud) - 1;

    // Disable formatter (just raw ITM output)
    TPI->FFCR = 0;

    // Enable ITM with trace bus ID 1
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | (1 << 16);

    // Enable stimulus port 0
    ITM->TER = 0x1;

    s_swo_initialized = 1;
}

int debug_swo_enabled(void) {
    // Check if debugger is actually connected (C_DEBUGEN bit in DHCSR)
    // Without debugger, SWO output has nowhere to go - skip it entirely
    if (!(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)) {
        return 0;
    }
    return s_swo_initialized && (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
           (ITM->TER & 1);
}

void debug_swo_putc(char c) {
    if (!debug_swo_enabled()) {
        return;
    }
    // Wait for ITM FIFO ready (debugger drains it)
    while (ITM->PORT[0].u32 == 0)
        ;
    ITM->PORT[0].u8 = (uint8_t)c;
}

void debug_swo_puts(const char *s) {
    while (*s) {
        if (*s == '\n') {
            debug_swo_putc('\r');
        }
        debug_swo_putc(*s++);
    }
}

// Simple number to string conversion
static void debug_swo_print_uint(uint32_t val, int base, int width, char pad) {
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
        debug_swo_putc(buf[--i]);
    }
}

static void debug_swo_print_int(int32_t val) {
    if (val < 0) {
        debug_swo_putc('-');
        val = -val;
    }
    debug_swo_print_uint((uint32_t)val, 10, 0, '0');
}

// Float to string with configurable precision (default 2)
static void debug_swo_print_float(float val, int precision) {
    if (precision < 0 || precision > 6)
        precision = 2;

    if (val < 0) {
        debug_swo_putc('-');
        val = -val;
    }

    // Calculate multiplier for precision (10^precision)
    int32_t mult = 1;
    for (int i = 0; i < precision; i++)
        mult *= 10;

    int32_t integer = (int32_t)val;
    int32_t frac = (int32_t)((val - integer) * mult + 0.5f);
    if (frac >= mult) {
        integer++;
        frac -= mult;
    }
    if (frac < 0) {
        frac = 0; // Clamp negative rounding errors
    }
    debug_swo_print_uint((uint32_t)integer, 10, 0, '0');
    if (precision > 0) {
        debug_swo_putc('.');
        debug_swo_print_uint((uint32_t)frac, 10, precision, '0');
    }
}

// Print string with optional width and justification
static void debug_swo_print_string(const char *s, int width, int left_justify) {
    int len = 0;
    const char *p = s;
    while (*p++)
        len++;

    // Right padding (left justify): print string first
    if (left_justify) {
        debug_swo_puts(s);
        for (int i = len; i < width; i++)
            debug_swo_putc(' ');
    } else {
        // Left padding (right justify): pad first
        for (int i = len; i < width; i++)
            debug_swo_putc(' ');
        debug_swo_puts(s);
    }
}

void debug_swo_vprintf(const char *fmt, va_list args) {
    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            int width = 0;
            int precision = -1; // -1 means not specified
            char pad = ' ';
            int left_justify = 0;

            // Check for flags
            if (*fmt == '-') {
                left_justify = 1;
                fmt++;
            }

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

            // Parse precision
            if (*fmt == '.') {
                fmt++;
                precision = 0;
                while (*fmt >= '0' && *fmt <= '9') {
                    precision = precision * 10 + (*fmt - '0');
                    fmt++;
                }
            }

            switch (*fmt) {
            case 'd':
            case 'i':
                debug_swo_print_int(va_arg(args, int));
                break;
            case 'u':
                debug_swo_print_uint(va_arg(args, unsigned int), 10, width,
                                     pad);
                break;
            case 'x':
            case 'X':
                debug_swo_print_uint(va_arg(args, unsigned int), 16, width,
                                     pad);
                break;
            case 'f':
                debug_swo_print_float((float)va_arg(args, double),
                                      precision >= 0 ? precision : 2);
                break;
            case 's':
                debug_swo_print_string(va_arg(args, const char *), width,
                                       left_justify);
                break;
            case 'c':
                debug_swo_putc((char)va_arg(args, int));
                break;
            case 'p': {
                // Pointer: print as 0x followed by hex
                debug_swo_puts("0x");
                debug_swo_print_uint((uint32_t)(uintptr_t)va_arg(args, void *),
                                     16, 8, '0');
                break;
            }
            case 'l':
                // Handle %ld, %lu, %lx (same as 32-bit on Cortex-M)
                fmt++;
                if (*fmt == 'd' || *fmt == 'i') {
                    debug_swo_print_int(va_arg(args, long));
                } else if (*fmt == 'u') {
                    debug_swo_print_uint(va_arg(args, unsigned long), 10, width,
                                         pad);
                } else if (*fmt == 'x' || *fmt == 'X') {
                    debug_swo_print_uint(va_arg(args, unsigned long), 16, width,
                                         pad);
                } else {
                    // Unknown %l? - print literally
                    debug_swo_putc('%');
                    debug_swo_putc('l');
                    debug_swo_putc(*fmt);
                }
                break;
            case '%':
                debug_swo_putc('%');
                break;
            default:
                debug_swo_putc('%');
                debug_swo_putc(*fmt);
                break;
            }
        } else {
            if (*fmt == '\n') {
                debug_swo_putc('\r');
            }
            debug_swo_putc(*fmt);
        }
        fmt++;
    }
}

void debug_swo_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_swo_vprintf(fmt, args);
    va_end(args);
}
