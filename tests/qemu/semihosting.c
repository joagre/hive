/**
 * ARM Semihosting implementation for QEMU
 *
 * Semihosting allows the target to communicate with the host debugger/emulator.
 * QEMU supports semihosting with -semihosting-config enable=on
 *
 * Reference: ARM Semihosting v2.0 Specification
 */

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

/* Semihosting operation numbers */
#define SYS_WRITE0 0x04 /* Write null-terminated string to debug console */
#define SYS_WRITEC 0x03 /* Write single character to debug console */
#define SYS_EXIT 0x18   /* Exit the application */

/* ARM Semihosting call via BKPT instruction */
static inline int32_t semihosting_call(uint32_t op, void *arg) {
    register uint32_t r0 __asm__("r0") = op;
    register void *r1 __asm__("r1") = arg;

    __asm__ volatile("bkpt #0xAB" : "+r"(r0) : "r"(r1) : "memory");

    return (int32_t)r0;
}

/* Write a null-terminated string to debug console */
void semihosting_puts(const char *str) {
    semihosting_call(SYS_WRITE0, (void *)str);
}

/* Write a single character to debug console */
void semihosting_putc(char c) {
    semihosting_call(SYS_WRITEC, &c);
}

/* Exit the application with status code */
void semihosting_exit(int status) {
    /* ARM semihosting exit uses a parameter block */
    struct {
        uint32_t reason;
        uint32_t status;
    } exit_params = {.reason = 0x20026, /* ADP_Stopped_ApplicationExit */
                     .status = (uint32_t)status};

    semihosting_call(SYS_EXIT, &exit_params);

    /* Should not return, but loop just in case */
    while (1) {
    }
}

/* Simple number to string conversion */
static void int_to_str(int32_t val, char *buf, int base) {
    char tmp[12];
    int i = 0;
    int neg = 0;
    uint32_t uval;

    if (val < 0 && base == 10) {
        neg = 1;
        uval = (uint32_t)(-val);
    } else {
        uval = (uint32_t)val;
    }

    if (uval == 0) {
        tmp[i++] = '0';
    } else {
        while (uval > 0) {
            int digit = uval % base;
            tmp[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
            uval /= base;
        }
    }

    int j = 0;
    if (neg) {
        buf[j++] = '-';
    }
    while (i > 0) {
        buf[j++] = tmp[--i];
    }
    buf[j] = '\0';
}

static void uint_to_str(uint32_t val, char *buf, int base) {
    char tmp[12];
    int i = 0;

    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            int digit = val % base;
            tmp[i++] = (digit < 10) ? ('0' + digit) : ('a' + digit - 10);
            val /= base;
        }
    }

    int j = 0;
    while (i > 0) {
        buf[j++] = tmp[--i];
    }
    buf[j] = '\0';
}

/* Minimal printf implementation for semihosting output */
int semihosting_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    char buf[128];
    int buf_idx = 0;

    while (*fmt) {
        if (*fmt == '%' && *(fmt + 1)) {
            fmt++;
            char num_buf[24]; /* Increased for long values */
            int is_long = 0;

            /* Check for 'l' modifier */
            if (*fmt == 'l') {
                is_long = 1;
                fmt++;
            }

            switch (*fmt) {
            case 'd':
            case 'i': {
                int32_t val = is_long ? (int32_t)va_arg(args, long)
                                      : va_arg(args, int32_t);
                int_to_str(val, num_buf, 10);
                for (char *p = num_buf; *p && buf_idx < 126; p++) {
                    buf[buf_idx++] = *p;
                }
                break;
            }
            case 'u': {
                uint32_t val = is_long ? (uint32_t)va_arg(args, unsigned long)
                                       : va_arg(args, uint32_t);
                uint_to_str(val, num_buf, 10);
                for (char *p = num_buf; *p && buf_idx < 126; p++) {
                    buf[buf_idx++] = *p;
                }
                break;
            }
            case 'x':
            case 'X': {
                uint32_t val = is_long ? (uint32_t)va_arg(args, unsigned long)
                                       : va_arg(args, uint32_t);
                uint_to_str(val, num_buf, 16);
                for (char *p = num_buf; *p && buf_idx < 126; p++) {
                    buf[buf_idx++] = *p;
                }
                break;
            }
            case 's': {
                const char *s = va_arg(args, const char *);
                if (s) {
                    while (*s && buf_idx < 126) {
                        buf[buf_idx++] = *s++;
                    }
                }
                break;
            }
            case 'c': {
                char c = (char)va_arg(args, int);
                if (buf_idx < 126) {
                    buf[buf_idx++] = c;
                }
                break;
            }
            case '%': {
                if (buf_idx < 126) {
                    buf[buf_idx++] = '%';
                }
                break;
            }
            default:
                /* Unknown format, skip */
                break;
            }
            fmt++;
        } else {
            if (buf_idx < 126) {
                buf[buf_idx++] = *fmt;
            }
            fmt++;
        }
    }

    buf[buf_idx] = '\0';
    semihosting_puts(buf);

    va_end(args);
    return buf_idx;
}
