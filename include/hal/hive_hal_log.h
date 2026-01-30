// Hive HAL - Logging
//
// Platform-specific console/debug output for logging.

#ifndef HIVE_HAL_LOG_H
#define HIVE_HAL_LOG_H

/**
 * Print formatted output to the platform's debug console.
 *
 * Platform implementations:
 * - Linux: Provided by Hive HAL, writes to stderr
 * - STM32: Must be provided by the application (e.g., route to SWO or UART)
 *
 * If HIVE_LOG_TO_STDOUT is enabled on STM32 and this function is not
 * provided, linking will fail with an undefined reference error.
 *
 * @param fmt Printf-style format string
 * @param ... Format arguments
 */
void hive_hal_printf(const char *fmt, ...);

#endif // HIVE_HAL_LOG_H
