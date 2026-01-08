#include "hive_log.h"
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <unistd.h>

// Level names for output
static const char *level_names[] = {
    "TRACE", "DEBUG", "INFO", "WARN", "ERROR"
};

// Level colors for terminal output (ANSI escape codes)
static const char *level_colors[] = {
    "\x1b[36m",  // TRACE: Cyan
    "\x1b[35m",  // DEBUG: Magenta
    "\x1b[32m",  // INFO: Green
    "\x1b[33m",  // WARN: Yellow
    "\x1b[31m"   // ERROR: Red
};

#define COLOR_RESET "\x1b[0m"

// Extract basename from file path
static const char *basename_simple(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

void hive_log_write(hive_log_level_t level, const char *file, int line,
                  const char *fmt, ...) {
    // Check if stderr is a terminal for colored output
    static int use_colors = -1;
    if (use_colors == -1) {
        use_colors = isatty(fileno(stderr));
    }

    // Print log level with optional color
    if (use_colors) {
        fprintf(stderr, "%s%-5s%s ", level_colors[level], level_names[level], COLOR_RESET);
    } else {
        fprintf(stderr, "%-5s ", level_names[level]);
    }

    // Print file:line for DEBUG and TRACE
    if (level <= HIVE_LOG_LEVEL_DEBUG) {
        fprintf(stderr, "%s:%d: ", basename_simple(file), line);
    }

    // Print the message
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);

    fprintf(stderr, "\n");
}
