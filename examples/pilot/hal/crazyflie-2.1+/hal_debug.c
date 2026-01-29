// Crazyflie 2.1+ HAL - Debug
//
// Debug output via SWO (Serial Wire Output).

#include "../hal.h"
#include "debug_swo.h"
#include <stdarg.h>

void hal_debug_init(void) {
    // SWO already initialized in hal_init(), nothing to do here
}

void hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_swo_vprintf(fmt, args);
    va_end(args);
}
