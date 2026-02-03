// Crazyflie 2.1+ HAL - Debug
//
// Debug output via SWO (Serial Wire Output) and early log buffer.
//
// hal_printf() writes to both:
// - SWO trace (real-time, if debugger connected)
// - Early log buffer (captured for replay to hive log file)
//
// After hive_log_file_open(), call early_log_flush() to replay
// buffered messages to the log file.

#include "../hal.h"
#include "debug_swo.h"
#include "early_log.h"
#include "printf.h"
#include <stdarg.h>

void hal_debug_init(void) {
    // SWO already initialized in hal_init(), nothing to do here
}

// Format buffer for hal_printf output
// Shared between hal_printf and hive_hal_printf (not reentrant, but
// these are only called from main thread before scheduler starts)
static char s_printf_buf[256];

void hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf_(s_printf_buf, sizeof(s_printf_buf), fmt, args);
    va_end(args);

    if (len < 0) {
        return;
    }
    if ((size_t)len >= sizeof(s_printf_buf)) {
        len = sizeof(s_printf_buf) - 1;
    }

    // Output to SWO (real-time trace if debugger connected)
    debug_swo_puts(s_printf_buf);

    // Capture to early log buffer (for replay to hive log file)
    early_log_write(s_printf_buf, (size_t)len);
}

// Implement Hive's hive_hal_printf (required for HIVE_LOG_TO_STDOUT on STM32)
// This is called by HIVE_LOG_* macros - after early_log_flush(), these
// go directly to the log file, so we only need SWO output here.
void hive_hal_printf(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    debug_swo_vprintf(fmt, args);
    va_end(args);
}

// Flush early log buffer to hive log file.
// Call after hive_log_file_open() succeeds.
void hal_flush_early_log(void) {
    early_log_flush();
}
