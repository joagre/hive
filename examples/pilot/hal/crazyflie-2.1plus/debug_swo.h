// Crazyflie 2.1+ Debug SWO Output
//
// Serial Wire Output (SWO) via ITM stimulus port 0.
// Requires STLink V3 or compatible debugger with SWO support.
//
// Use st-trace to view output:
//   st-trace -c 168
//
// SWO is available on the 20-pin JTAG connector (pin 13).

#ifndef DEBUG_SWO_H
#define DEBUG_SWO_H

#include <stdarg.h>
#include <stdint.h>

// Initialize SWO/ITM for debug output
// cpu_freq_hz: CPU frequency in Hz (168000000 for Crazyflie)
// swo_baud: SWO baud rate (typically 2000000)
void debug_swo_init(uint32_t cpu_freq_hz, uint32_t swo_baud);

// Check if SWO is enabled and ready
int debug_swo_enabled(void);

// Formatted print (subset of printf: %d, %i, %u, %x, %X, %f, %s, %c, %%)
void debug_swo_printf(const char *fmt, ...);

// Formatted print with va_list (for forwarding from other variadic functions)
void debug_swo_vprintf(const char *fmt, va_list args);

// Low-level character output
void debug_swo_putc(char c);

// String output
void debug_swo_puts(const char *s);

#endif // DEBUG_SWO_H
