// Crazyflie 2.1+ Bring-Up - USART1 Debug Output
//
// USART1 on PA9 (TX) and PA10 (RX) at 115200 baud.
// Connected via debug adapter to ST-Link V3 VCP.

#ifndef BRINGUP_UART_H
#define BRINGUP_UART_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize USART1 for debug output
void uart_init(void);

// Send a single character
void uart_putc(char c);

// Send a null-terminated string
void uart_puts(const char *s);

// Formatted print (subset of printf)
void uart_printf(const char *fmt, ...);

// Check if a character is available
bool uart_available(void);

// Read a character (blocking)
char uart_getc(void);

// Read a character with timeout (returns -1 on timeout)
// timeout_ms = 0 for non-blocking, -1 for infinite
int uart_getc_timeout(int timeout_ms);

// Print test result
void uart_print_result(const char *phase, const char *test, bool passed);

// Print hex dump
void uart_hex_dump(const uint8_t *data, size_t len);

#endif // BRINGUP_UART_H
