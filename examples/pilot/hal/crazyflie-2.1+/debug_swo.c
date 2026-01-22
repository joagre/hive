// Crazyflie 2.1+ Debug SWO Output
//
// Serial Wire Output (SWO) via ITM stimulus port 0.
// Requires STLink V3 or compatible debugger with SWO support.
//
// Use st-trace to view output:
//   st-trace -c 168

#include "debug_swo.h"
#include "stm32f4xx.h"

// SWO configuration
static uint32_t s_swo_initialized = 0;

void debug_swo_init(uint32_t cpu_freq_hz, uint32_t swo_baud) {
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
    return s_swo_initialized && (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
           (ITM->TER & 1);
}

void debug_swo_putc(char c) {
    if (debug_swo_enabled()) {
        while (ITM->PORT[0].u32 == 0)
            ;
        ITM->PORT[0].u8 = (uint8_t)c;
    }
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

// Simple float to string (2 decimal places)
static void debug_swo_print_float(float val) {
    if (val < 0) {
        debug_swo_putc('-');
        val = -val;
    }
    int32_t integer = (int32_t)val;
    int32_t frac = (int32_t)((val - integer) * 100 + 0.5f);
    if (frac >= 100) {
        integer++;
        frac -= 100;
    }
    debug_swo_print_uint((uint32_t)integer, 10, 0, '0');
    debug_swo_putc('.');
    debug_swo_print_uint((uint32_t)frac, 10, 2, '0');
}

void debug_swo_vprintf(const char *fmt, va_list args) {
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
                debug_swo_print_float((float)va_arg(args, double));
                break;
            case 's':
                debug_swo_puts(va_arg(args, const char *));
                break;
            case 'c':
                debug_swo_putc((char)va_arg(args, int));
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
