#ifndef HIVE_LOG_H
#define HIVE_LOG_H

#include <stdio.h>

// Log level constants for compile-time filtering (preprocessor needs numeric values)
#define HIVE_LOG_LEVEL_TRACE 0
#define HIVE_LOG_LEVEL_DEBUG 1
#define HIVE_LOG_LEVEL_INFO  2
#define HIVE_LOG_LEVEL_WARN  3
#define HIVE_LOG_LEVEL_ERROR 4
#define HIVE_LOG_LEVEL_NONE  5

// Log level type for runtime use
typedef int hive_log_level_t;

// Default compile-time log level (can override with -DRT_LOG_LEVEL=...)
#ifndef HIVE_LOG_LEVEL
#define HIVE_LOG_LEVEL HIVE_LOG_LEVEL_INFO
#endif

// Core logging function (not typically called directly)
void hive_log_write(hive_log_level_t level, const char *file, int line,
                  const char *fmt, ...) __attribute__((format(printf, 4, 5)));

// Logging macros that compile out based on HIVE_LOG_LEVEL
#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_TRACE
#define HIVE_LOG_TRACE(...) hive_log_write(HIVE_LOG_LEVEL_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_TRACE(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_DEBUG
#define HIVE_LOG_DEBUG(...) hive_log_write(HIVE_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_DEBUG(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_INFO
#define HIVE_LOG_INFO(...) hive_log_write(HIVE_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_INFO(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_WARN
#define HIVE_LOG_WARN(...) hive_log_write(HIVE_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_WARN(...) ((void)0)
#endif

#if HIVE_LOG_LEVEL <= HIVE_LOG_LEVEL_ERROR
#define HIVE_LOG_ERROR(...) hive_log_write(HIVE_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define HIVE_LOG_ERROR(...) ((void)0)
#endif

#endif // HIVE_LOG_H
