// Crazyflie 2.1+ Bring-Up - SWO Debug Output
//
// Serial Wire Output (SWO) via ITM stimulus port 0.
// Use st-trace to view output: st-trace -c 168
//
// Note: SWO is output-only. Input functions always return timeout.

#ifndef BRINGUP_SWO_H
#define BRINGUP_SWO_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize SWO for debug output
void swo_init(void);

// Send a single character
void swo_putc(char c);

// Send a null-terminated string
void swo_puts(const char *s);

// Formatted print (subset of printf)
void swo_printf(const char *fmt, ...);

// Check if a character is available (always returns false - SWO is output-only)
bool swo_available(void);

// Read a character (always returns immediately - SWO is output-only)
char swo_getc(void);

// Read a character with timeout (always returns -1 - SWO is output-only)
int swo_getc_timeout(int timeout_ms);

// Print test result
void swo_print_result(const char *phase, const char *test, bool passed);

// Print hex dump
void swo_hex_dump(const uint8_t *data, size_t len);

// Get current tick count (milliseconds since init)
uint32_t swo_get_ticks(void);

#endif // BRINGUP_SWO_H
