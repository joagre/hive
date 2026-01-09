// USART1 debug serial driver for STM32F401 (STEVAL-DRONE01)
//
// Direct register access implementation for debug output.
// Uses polling mode for simplicity.

#include "usart1.h"
#include "system_config.h"
#include "gpio_config.h"
#include <stdarg.h>

// ----------------------------------------------------------------------------
// USART1 Register Definitions
// ----------------------------------------------------------------------------

#define USART1_BASE         0x40011000U

#define USART1_SR           (*(volatile uint32_t *)(USART1_BASE + 0x00))
#define USART1_DR           (*(volatile uint32_t *)(USART1_BASE + 0x04))
#define USART1_BRR          (*(volatile uint32_t *)(USART1_BASE + 0x08))
#define USART1_CR1          (*(volatile uint32_t *)(USART1_BASE + 0x0C))
#define USART1_CR2          (*(volatile uint32_t *)(USART1_BASE + 0x10))
#define USART1_CR3          (*(volatile uint32_t *)(USART1_BASE + 0x14))
#define USART1_GTPR         (*(volatile uint32_t *)(USART1_BASE + 0x18))

// USART_SR bits
#define USART_SR_PE         (1U << 0)   // Parity error
#define USART_SR_FE         (1U << 1)   // Framing error
#define USART_SR_NF         (1U << 2)   // Noise detected
#define USART_SR_ORE        (1U << 3)   // Overrun error
#define USART_SR_IDLE       (1U << 4)   // Idle line detected
#define USART_SR_RXNE       (1U << 5)   // Read data register not empty
#define USART_SR_TC         (1U << 6)   // Transmission complete
#define USART_SR_TXE        (1U << 7)   // Transmit data register empty
#define USART_SR_LBD        (1U << 8)   // LIN break detection
#define USART_SR_CTS        (1U << 9)   // CTS flag

// USART_CR1 bits
#define USART_CR1_SBK       (1U << 0)   // Send break
#define USART_CR1_RWU       (1U << 1)   // Receiver wakeup
#define USART_CR1_RE        (1U << 2)   // Receiver enable
#define USART_CR1_TE        (1U << 3)   // Transmitter enable
#define USART_CR1_IDLEIE    (1U << 4)   // IDLE interrupt enable
#define USART_CR1_RXNEIE    (1U << 5)   // RXNE interrupt enable
#define USART_CR1_TCIE      (1U << 6)   // TC interrupt enable
#define USART_CR1_TXEIE     (1U << 7)   // TXE interrupt enable
#define USART_CR1_PEIE      (1U << 8)   // PE interrupt enable
#define USART_CR1_PS        (1U << 9)   // Parity selection (0=even, 1=odd)
#define USART_CR1_PCE       (1U << 10)  // Parity control enable
#define USART_CR1_WAKE      (1U << 11)  // Wakeup method
#define USART_CR1_M         (1U << 12)  // Word length (0=8bit, 1=9bit)
#define USART_CR1_UE        (1U << 13)  // USART enable
#define USART_CR1_OVER8     (1U << 15)  // Oversampling mode (0=16, 1=8)

// USART_CR2 bits
#define USART_CR2_STOP_MASK (3U << 12)  // Stop bits
#define USART_CR2_STOP_1    (0U << 12)  // 1 stop bit
#define USART_CR2_STOP_0_5  (1U << 12)  // 0.5 stop bit
#define USART_CR2_STOP_2    (2U << 12)  // 2 stop bits
#define USART_CR2_STOP_1_5  (3U << 12)  // 1.5 stop bits

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static usart1_config_t s_config;

// Buffer for printf
#define PRINTF_BUFFER_SIZE  256
static char s_printf_buf[PRINTF_BUFFER_SIZE];

// ----------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------

static void set_baud_rate(uint32_t baud) {
    // USART1 is on APB2 (84 MHz)
    // BRR = fck / baud
    // For oversampling by 16: BRR = fck / baud
    // Mantissa = integer part, Fraction = fractional part * 16

    uint32_t pclk2 = PCLK2_FREQ;  // 84 MHz

    // Calculate BRR value
    // BRR = (pclk2 + baud/2) / baud  (rounded)
    uint32_t brr = (pclk2 + baud / 2) / baud;

    USART1_BRR = brr;
}

// Simple integer to string conversion
static char *itoa_simple(int32_t value, char *buf, int base) {
    char *p = buf;
    char *p1, *p2;
    uint32_t uvalue;
    int negative = 0;

    if (value < 0 && base == 10) {
        negative = 1;
        uvalue = (uint32_t)(-value);
    } else {
        uvalue = (uint32_t)value;
    }

    // Generate digits in reverse order
    do {
        int digit = uvalue % base;
        *p++ = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
        uvalue /= base;
    } while (uvalue > 0);

    if (negative) {
        *p++ = '-';
    }

    *p = '\0';

    // Reverse string
    p1 = buf;
    p2 = p - 1;
    while (p1 < p2) {
        char tmp = *p1;
        *p1++ = *p2;
        *p2-- = tmp;
    }

    return buf;
}

static char *utoa_simple(uint32_t value, char *buf, int base) {
    char *p = buf;
    char *p1, *p2;

    do {
        int digit = value % base;
        *p++ = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
        value /= base;
    } while (value > 0);

    *p = '\0';

    // Reverse string
    p1 = buf;
    p2 = p - 1;
    while (p1 < p2) {
        char tmp = *p1;
        *p1++ = *p2;
        *p2-- = tmp;
    }

    return buf;
}

// Simple vsnprintf implementation (subset of format specifiers)
static int simple_vsnprintf(char *buf, size_t size, const char *fmt, va_list args) {
    char *p = buf;
    char *end = buf + size - 1;
    char tmp[32];

    while (*fmt && p < end) {
        if (*fmt != '%') {
            *p++ = *fmt++;
            continue;
        }

        fmt++;  // Skip '%'

        // Handle format specifiers
        switch (*fmt) {
            case '%':
                *p++ = '%';
                break;

            case 'c': {
                char c = (char)va_arg(args, int);
                *p++ = c;
                break;
            }

            case 's': {
                const char *s = va_arg(args, const char *);
                if (!s) s = "(null)";
                while (*s && p < end) {
                    *p++ = *s++;
                }
                break;
            }

            case 'd':
            case 'i': {
                int32_t val = va_arg(args, int32_t);
                itoa_simple(val, tmp, 10);
                char *s = tmp;
                while (*s && p < end) {
                    *p++ = *s++;
                }
                break;
            }

            case 'u': {
                uint32_t val = va_arg(args, uint32_t);
                utoa_simple(val, tmp, 10);
                char *s = tmp;
                while (*s && p < end) {
                    *p++ = *s++;
                }
                break;
            }

            case 'x':
            case 'X': {
                uint32_t val = va_arg(args, uint32_t);
                utoa_simple(val, tmp, 16);
                char *s = tmp;
                while (*s && p < end) {
                    *p++ = *s++;
                }
                break;
            }

            case 'p': {
                uintptr_t val = (uintptr_t)va_arg(args, void *);
                *p++ = '0';
                if (p < end) *p++ = 'x';
                utoa_simple((uint32_t)val, tmp, 16);
                char *s = tmp;
                while (*s && p < end) {
                    *p++ = *s++;
                }
                break;
            }

            case 'f': {
                // Simple float handling (limited precision)
                double val = va_arg(args, double);
                if (val < 0) {
                    *p++ = '-';
                    val = -val;
                }
                int32_t ipart = (int32_t)val;
                float fpart = (float)(val - ipart);

                itoa_simple(ipart, tmp, 10);
                char *s = tmp;
                while (*s && p < end) {
                    *p++ = *s++;
                }

                if (p < end) {
                    *p++ = '.';
                    // 3 decimal places
                    for (int i = 0; i < 3 && p < end; i++) {
                        fpart *= 10;
                        int digit = (int)fpart;
                        *p++ = '0' + digit;
                        fpart -= digit;
                    }
                }
                break;
            }

            default:
                // Unknown format, just copy it
                *p++ = '%';
                if (p < end) *p++ = *fmt;
                break;
        }

        fmt++;
    }

    *p = '\0';
    return (int)(p - buf);
}

// ----------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------

void usart1_init(const usart1_config_t *config) {
    // Use provided config or defaults
    if (config) {
        s_config = *config;
    } else {
        s_config = (usart1_config_t)USART1_CONFIG_DEFAULT;
    }

    // Enable USART1 clock
    system_enable_usart1();

    // Initialize GPIO pins
    gpio_init_usart1();

    // Disable USART before configuration
    USART1_CR1 = 0;

    // Configure baud rate
    set_baud_rate(s_config.baud_rate);

    // Configure CR2: 1 stop bit (default)
    USART1_CR2 = USART_CR2_STOP_1;

    // Configure CR3: No flow control
    USART1_CR3 = 0;

    // Configure CR1: 8N1, enable TX/RX as configured
    uint32_t cr1 = 0;
    if (s_config.tx_enable) {
        cr1 |= USART_CR1_TE;
    }
    if (s_config.rx_enable) {
        cr1 |= USART_CR1_RE;
    }

    // Enable USART
    cr1 |= USART_CR1_UE;

    USART1_CR1 = cr1;
}

void usart1_deinit(void) {
    // Wait for transmission to complete
    usart1_flush();

    // Disable USART
    USART1_CR1 = 0;
}

void usart1_set_baud(uint32_t baud_rate) {
    s_config.baud_rate = baud_rate;

    // Disable USART
    uint32_t cr1 = USART1_CR1;
    USART1_CR1 = cr1 & ~USART_CR1_UE;

    // Set new baud rate
    set_baud_rate(baud_rate);

    // Re-enable USART
    USART1_CR1 = cr1;
}

// ----------------------------------------------------------------------------
// Transmit functions
// ----------------------------------------------------------------------------

void usart1_putc(char c) {
    // Wait until TX buffer is empty
    while (!(USART1_SR & USART_SR_TXE));

    // Write data
    USART1_DR = (uint32_t)c;
}

void usart1_puts(const char *str) {
    while (*str) {
        usart1_putc(*str++);
    }
}

void usart1_write(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    while (len--) {
        usart1_putc(*p++);
    }
}

int usart1_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    int len = simple_vsnprintf(s_printf_buf, PRINTF_BUFFER_SIZE, fmt, args);

    va_end(args);

    usart1_puts(s_printf_buf);

    return len;
}

bool usart1_tx_ready(void) {
    return (USART1_SR & USART_SR_TXE) != 0;
}

void usart1_flush(void) {
    // Wait for transmission complete
    while (!(USART1_SR & USART_SR_TC));
}

// ----------------------------------------------------------------------------
// Receive functions
// ----------------------------------------------------------------------------

char usart1_getc(void) {
    // Wait until RX buffer has data
    while (!(USART1_SR & USART_SR_RXNE));

    // Read data
    return (char)USART1_DR;
}

bool usart1_getc_timeout(char *c, uint32_t timeout_ms) {
    uint32_t start = system_get_tick();

    while (!(USART1_SR & USART_SR_RXNE)) {
        if ((system_get_tick() - start) >= timeout_ms) {
            return false;
        }
    }

    *c = (char)USART1_DR;
    return true;
}

bool usart1_rx_ready(void) {
    return (USART1_SR & USART_SR_RXNE) != 0;
}

size_t usart1_read(void *buf, size_t max_len) {
    uint8_t *p = (uint8_t *)buf;
    size_t count = 0;

    while (count < max_len && usart1_rx_ready()) {
        *p++ = (uint8_t)USART1_DR;
        count++;
    }

    return count;
}

// ----------------------------------------------------------------------------
// Debug helpers
// ----------------------------------------------------------------------------

void usart1_hexdump(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    char hex[3];

    for (size_t i = 0; i < len; i++) {
        if (i > 0 && (i % 16) == 0) {
            usart1_newline();
        } else if (i > 0) {
            usart1_putc(' ');
        }

        // Convert byte to hex
        uint8_t b = p[i];
        hex[0] = "0123456789abcdef"[b >> 4];
        hex[1] = "0123456789abcdef"[b & 0x0f];
        hex[2] = '\0';
        usart1_puts(hex);
    }
    usart1_newline();
}

void usart1_print_int(int32_t value) {
    char buf[12];
    itoa_simple(value, buf, 10);
    usart1_puts(buf);
}

void usart1_print_uint(uint32_t value) {
    char buf[12];
    utoa_simple(value, buf, 10);
    usart1_puts(buf);
}

void usart1_print_hex(uint32_t value) {
    char buf[12];
    usart1_puts("0x");
    utoa_simple(value, buf, 16);
    usart1_puts(buf);
}

void usart1_print_float(float value, int decimals) {
    if (value < 0) {
        usart1_putc('-');
        value = -value;
    }

    int32_t ipart = (int32_t)value;
    float fpart = value - ipart;

    usart1_print_int(ipart);
    usart1_putc('.');

    // Print decimal places
    for (int i = 0; i < decimals; i++) {
        fpart *= 10;
        int digit = (int)fpart;
        usart1_putc('0' + digit);
        fpart -= digit;
    }
}

void usart1_newline(void) {
    usart1_puts("\r\n");
}
