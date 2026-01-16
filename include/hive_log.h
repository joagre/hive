#ifndef HIVE_LOG_H
#define HIVE_LOG_H

#include <stdint.h>
#include "hive_types.h"

// Log level constants for compile-time filtering (preprocessor needs numeric
// values)
#define HIVE_LOG_LEVEL_TRACE 0
#define HIVE_LOG_LEVEL_DEBUG 1
#define HIVE_LOG_LEVEL_INFO 2
#define HIVE_LOG_LEVEL_WARN 3
#define HIVE_LOG_LEVEL_ERROR 4
#define HIVE_LOG_LEVEL_NONE 5

// Log level type for runtime use
typedef int hive_log_level_t;

// -----------------------------------------------------------------------------
// Log File Entry Format (for binary log files)
// -----------------------------------------------------------------------------
// Each log entry in a file is prefixed with a 12-byte header, followed by text
// payload. Magic bytes allow recovery of entries from corrupted flash.
//
// Wire format (little-endian, explicit byte serialization for portability):
//   Offset  Size  Field
//   0       2     magic       (0x47, 0x4C = "LG" in little-endian)
//   2       2     seq         Monotonic sequence number (detects drops)
//   4       4     timestamp   Microseconds since boot
//   8       2     len         Payload length (text, excluding null terminator)
//   10      1     level       Log level (TRACE=0, DEBUG=1, INFO=2, WARN=3,
//   ERROR=4) 11      1     reserved    Padding (always 0) 12      len   payload
//   Log message text (no null terminator)

#define HIVE_LOG_MAGIC 0x4C47 // "LG" - Log entry marker
#define HIVE_LOG_HEADER_SIZE 12

// -----------------------------------------------------------------------------
// Log File API (lifecycle managed by supervisor/application)
// -----------------------------------------------------------------------------

// Initialize logging subsystem (call once at startup)
hive_status hive_log_init(void);

// Open log file for writing (call before flight, e.g., during ARM)
// On STM32 with HIVE_O_TRUNC, this triggers flash erase (blocks 1-4 seconds)
hive_status hive_log_file_open(const char *path);

// Sync log file to storage (call periodically, e.g., every 100ms)
// On STM32, this flushes the ring buffer to flash
hive_status hive_log_file_sync(void);

// Close log file (call after flight, e.g., during DISARM)
// Performs final sync before closing
hive_status hive_log_file_close(void);

// Cleanup logging subsystem (call at shutdown)
void hive_log_cleanup(void);

// Default compile-time log level (can override with -DRT_LOG_LEVEL=...)
#ifndef HIVE_LOG_LEVEL
#define HIVE_LOG_LEVEL HIVE_LOG_LEVEL_INFO
#endif

// Core logging function (not typically called directly)
void hive_log_write(hive_log_level_t level, const char *file, int line,
                    const char *fmt, ...) __attribute__((format(printf, 4, 5)));

// Logging macros that compile out based on HIVE_LOG_LEVEL
#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_TRACE
#define HIVE_LOG_TRACE(...) \
    hive_log_write(HIVE_LOG_LEVEL_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_TRACE(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_DEBUG
#define HIVE_LOG_DEBUG(...) \
    hive_log_write(HIVE_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_DEBUG(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_INFO
#define HIVE_LOG_INFO(...) \
    hive_log_write(HIVE_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_INFO(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_WARN
#define HIVE_LOG_WARN(...) \
    hive_log_write(HIVE_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_WARN(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_ERROR
#define HIVE_LOG_ERROR(...) \
    hive_log_write(HIVE_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_ERROR(...) ((void)0)
#endif

#endif // HIVE_LOG_H
