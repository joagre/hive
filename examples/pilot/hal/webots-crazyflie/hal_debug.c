// Webots Crazyflie HAL - Debug
//
// Debug output via printf.

#include "../hal.h"
#include <stdarg.h>
#include <stdio.h>

void hal_debug_init(void) {
    // No-op for Webots, printf always works
}

void hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

void hal_flush_early_log(void) {
    // No-op for Webots - no early buffering needed
}
