// printf_config.h - Configuration for eyalroz/printf lightweight library
//
// Optimized for embedded systems with limited stack space.
// See printf.h for all available options.
//
// Note: Hive only uses vsnprintf_() for log formatting. The printf_() family
// requires putchar_() which is provided in hive_log.c as a no-op stub.

#ifndef PRINTF_CONFIG_H
#define PRINTF_CONFIG_H

// Buffer sizes - smaller than defaults to reduce stack usage
// Default is 32 for both; we use 24 which handles up to 64-bit values
#define PRINTF_INTEGER_BUFFER_SIZE 24
#define PRINTF_DECIMAL_BUFFER_SIZE 24

// Default float precision (matches libc default)
#define PRINTF_DEFAULT_FLOAT_PRECISION 6

// Enable long long support (needed for 64-bit timestamps)
#define PRINTF_SUPPORT_LONG_LONG 1

// Disable features we don't need
#define PRINTF_SUPPORT_MSVC_STYLE_INTEGER_SPECIFIERS 0

// Don't alias to standard names - we explicitly use the _() suffixed versions
// This avoids conflicts with libc on Linux while allowing STM32 to work
#define PRINTF_ALIAS_STANDARD_FUNCTION_NAMES_SOFT 0
#define PRINTF_ALIAS_STANDARD_FUNCTION_NAMES_HARD 0

#endif // PRINTF_CONFIG_H
