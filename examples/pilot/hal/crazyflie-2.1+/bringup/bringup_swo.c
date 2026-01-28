// Crazyflie 2.1+ Bring-Up - SWO Debug Output
//
// Serial Wire Output (SWO) via ITM stimulus port 0.
// Use st-trace to view output: st-trace -c 168

#include "bringup_swo.h"
#include "stm32f4xx.h"
#include <string.h>

// Crazyflie 2.1+ runs at 168 MHz
#define CPU_FREQ_HZ 168000000
#define SWO_BAUD 2000000

// Simple delay using SysTick
static volatile uint32_t s_ticks = 0;

void SysTick_Handler(void) {
    s_ticks++;
}

void swo_init(void) {
    // Configure SysTick for 1ms ticks
    SysTick_Config(SystemCoreClock / 1000);

    // Enable DBGMCU clock and trace output
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;

    // Configure PB3 as SWO output (AF0 = TRACESWO)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // Set PB3 to alternate function mode
    GPIOB->MODER &= ~GPIO_MODER_MODER3;
    GPIOB->MODER |= GPIO_MODER_MODER3_1; // AF mode
    // Set AF0 for PB3 (TRACESWO)
    GPIOB->AFR[0] &= ~(0xF << (3 * 4)); // Clear AF bits for PB3
    GPIOB->AFR[0] |= (0x0 << (3 * 4));  // AF0 = TRACESWO
    // High speed output
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR3;

    // Enable trace in core debug
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Unlock ITM
    ITM->LAR = 0xC5ACCE55;

    // Configure TPIU for async SWO (NRZ/UART-like encoding)
    TPI->SPPR = 2; // NRZ encoding
    TPI->ACPR = (CPU_FREQ_HZ / SWO_BAUD) - 1;

    // Disable formatter (just raw ITM output)
    TPI->FFCR = 0;

    // Enable ITM with trace bus ID 1
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk | (1 << 16);

    // Enable stimulus port 0
    ITM->TER = 0x1;
}

static int swo_enabled(void) {
    return (ITM->TCR & ITM_TCR_ITMENA_Msk) && (ITM->TER & 1);
}

void swo_putc(char c) {
    if (swo_enabled()) {
        while (ITM->PORT[0].u32 == 0)
            ;
        ITM->PORT[0].u8 = (uint8_t)c;
    }
}

void swo_puts(const char *s) {
    while (*s) {
        if (*s == '\n') {
            swo_putc('\r');
        }
        swo_putc(*s++);
    }
}

// SWO is output-only - input functions always fail/timeout
bool swo_available(void) {
    return false;
}

char swo_getc(void) {
    // No input available - return null
    return '\0';
}

int swo_getc_timeout(int timeout_ms) {
    // Wait the requested time, then return timeout
    if (timeout_ms > 0) {
        uint32_t start = s_ticks;
        while ((s_ticks - start) < (uint32_t)timeout_ms) {
            __WFI();
        }
    }
    return -1; // Always timeout
}

// Simple number to string conversion
static void swo_print_uint(uint32_t val, int base, int width, char pad) {
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
        swo_putc(buf[--i]);
    }
}

static void swo_print_int(int32_t val) {
    if (val < 0) {
        swo_putc('-');
        val = -val;
    }
    swo_print_uint((uint32_t)val, 10, 0, '0');
}

// Simple float to string (2 decimal places)
static void swo_print_float(float val) {
    if (val < 0) {
        swo_putc('-');
        val = -val;
    }
    int32_t integer = (int32_t)val;
    int32_t frac = (int32_t)((val - integer) * 100 + 0.5f);
    if (frac >= 100) {
        integer++;
        frac -= 100;
    }
    swo_print_uint((uint32_t)integer, 10, 0, '0');
    swo_putc('.');
    swo_print_uint((uint32_t)frac, 10, 2, '0');
}

void swo_printf(const char *fmt, ...) {
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

            // Skip precision specifier (e.g., ".2" in "%.2f")
            if (*fmt == '.') {
                fmt++;
                while (*fmt >= '0' && *fmt <= '9') {
                    fmt++;
                }
            }

            switch (*fmt) {
            case 'd':
            case 'i':
                swo_print_int(va_arg(args, int));
                break;
            case 'u':
                swo_print_uint(va_arg(args, unsigned int), 10, width, pad);
                break;
            case 'x':
            case 'X':
                swo_print_uint(va_arg(args, unsigned int), 16, width, pad);
                break;
            case 'f':
                swo_print_float((float)va_arg(args, double));
                break;
            case 's':
                swo_puts(va_arg(args, const char *));
                break;
            case 'c':
                swo_putc((char)va_arg(args, int));
                break;
            case '%':
                swo_putc('%');
                break;
            default:
                swo_putc('%');
                swo_putc(*fmt);
                break;
            }
        } else {
            if (*fmt == '\n') {
                swo_putc('\r');
            }
            swo_putc(*fmt);
        }
        fmt++;
    }

    va_end(args);
}

void swo_print_result(const char *phase, const char *test, bool passed) {
    swo_printf("[%s] %s... %s\n", phase, test, passed ? "OK" : "FAIL");
}

void swo_hex_dump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (i > 0 && (i % 16) == 0) {
            swo_puts("\n");
        }
        swo_printf("%02X ", data[i]);
    }
    swo_puts("\n");
}

uint32_t swo_get_ticks(void) {
    return s_ticks;
}
