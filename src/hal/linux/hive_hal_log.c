// Hive HAL - Logging (Linux)
//
// Console output via stderr.

#include "hal/hive_hal_log.h"
#include <stdarg.h>
#include <stdio.h>

void hive_hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
}
