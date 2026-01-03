#ifndef RT_LOG_H
#define RT_LOG_H

#include <stdio.h>

// Log levels (lower number = more severe)
typedef enum {
    RT_LOG_LEVEL_TRACE = 0,
    RT_LOG_LEVEL_DEBUG = 1,
    RT_LOG_LEVEL_INFO = 2,
    RT_LOG_LEVEL_WARN = 3,
    RT_LOG_LEVEL_ERROR = 4,
    RT_LOG_LEVEL_NONE = 5
} rt_log_level_t;

// Default compile-time log level (can override with -DRT_LOG_LEVEL=...)
#ifndef RT_LOG_LEVEL
#define RT_LOG_LEVEL RT_LOG_LEVEL_INFO
#endif

// Core logging function (not typically called directly)
void rt_log_write(rt_log_level_t level, const char *file, int line,
                  const char *fmt, ...) __attribute__((format(printf, 4, 5)));

// Logging macros that compile out based on RT_LOG_LEVEL
#if RT_LOG_LEVEL <= RT_LOG_LEVEL_TRACE
#define RT_LOG_TRACE(...) rt_log_write(RT_LOG_LEVEL_TRACE, __FILE__, __LINE__, __VA_ARGS__)
#else
#define RT_LOG_TRACE(...) ((void)0)
#endif

#if RT_LOG_LEVEL <= RT_LOG_LEVEL_DEBUG
#define RT_LOG_DEBUG(...) rt_log_write(RT_LOG_LEVEL_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#else
#define RT_LOG_DEBUG(...) ((void)0)
#endif

#if RT_LOG_LEVEL <= RT_LOG_LEVEL_INFO
#define RT_LOG_INFO(...) rt_log_write(RT_LOG_LEVEL_INFO, __FILE__, __LINE__, __VA_ARGS__)
#else
#define RT_LOG_INFO(...) ((void)0)
#endif

#if RT_LOG_LEVEL <= RT_LOG_LEVEL_WARN
#define RT_LOG_WARN(...) rt_log_write(RT_LOG_LEVEL_WARN, __FILE__, __LINE__, __VA_ARGS__)
#else
#define RT_LOG_WARN(...) ((void)0)
#endif

#if RT_LOG_LEVEL <= RT_LOG_LEVEL_ERROR
#define RT_LOG_ERROR(...) rt_log_write(RT_LOG_LEVEL_ERROR, __FILE__, __LINE__, __VA_ARGS__)
#else
#define RT_LOG_ERROR(...) ((void)0)
#endif

#endif // RT_LOG_H
