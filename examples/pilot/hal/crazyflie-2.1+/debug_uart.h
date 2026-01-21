// Crazyflie 2.1+ Debug UART Output
//
// USART1 on PA9 (TX) and PA10 (RX) at 115200 baud.
// Connected via debug adapter to ST-Link V3 VCP.
//
// Note: This module does NOT configure SysTick. The application must provide
// a SysTick_Handler or use a different timing source.

#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdarg.h>

// Initialize USART1 for debug output (115200 8N1)
void debug_uart_init(void);

// Formatted print (subset of printf: %d, %i, %u, %x, %X, %f, %s, %c, %%)
void debug_uart_printf(const char *fmt, ...);

// Formatted print with va_list (for forwarding from other variadic functions)
void debug_uart_vprintf(const char *fmt, va_list args);

#endif // DEBUG_UART_H
