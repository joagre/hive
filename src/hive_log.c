// Hive Logging Implementation
//
// Supports dual output (controlled by compile-time flags):
// - Console (stderr) with optional ANSI colors (HIVE_LOG_TO_STDOUT)
// - Binary log file with explicit little-endian serialization
// (HIVE_LOG_TO_FILE)

#include "hive_log.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "printf.h" // Lightweight printf for embedded (low stack usage)
#include <string.h>
#include <stdbool.h>

// Required by lightweight printf library - stub since we only use vsnprintf_()
void putchar_(char c) {
    (void)c;
}

#if HIVE_LOG_TO_STDOUT
#include <stdio.h>
#include <unistd.h>
#endif

#if HIVE_LOG_TO_FILE
#include "hive_timer.h"
#include "hive_file.h"
#endif

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static bool s_initialized = false;

#if HIVE_LOG_TO_FILE
static int s_log_fd = -1;  // File descriptor for log file (-1 = not open)
static uint16_t s_seq = 0; // Monotonic sequence number
#endif

// -----------------------------------------------------------------------------
// Console output helpers
// -----------------------------------------------------------------------------

#if HIVE_LOG_TO_STDOUT

// Level names for output
static const char *s_level_names[] = {"TRACE", "DEBUG", "INFO", "WARN",
                                      "ERROR"};

// Level colors for terminal output (ANSI escape codes)
static const char *s_level_colors[] = {
    "\x1b[36m", // TRACE: Cyan
    "\x1b[35m", // DEBUG: Magenta
    "\x1b[32m", // INFO: Green
    "\x1b[33m", // WARN: Yellow
    "\x1b[31m"  // ERROR: Red
};

#define COLOR_RESET "\x1b[0m"

// Extract basename from file path
static const char *basename_simple(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

// Output callback for fctprintf - writes to stderr
static void stderr_putc(char c, void *arg) {
    (void)arg;
    fputc(c, stderr);
}

static void log_to_console(hive_log_level_t level, const char *file, int line,
                           const char *text) {
    // Check if stderr is a terminal for colored output
    static int s_use_colors = -1;
    if (s_use_colors == -1) {
        s_use_colors = isatty(fileno(stderr));
    }

    // Print log level with optional color (using lightweight printf)
    if (s_use_colors) {
        fctprintf(stderr_putc, NULL, "%s%-5s%s ", s_level_colors[level],
                  s_level_names[level], COLOR_RESET);
    } else {
        fctprintf(stderr_putc, NULL, "%-5s ", s_level_names[level]);
    }

    // Print file:line for DEBUG and TRACE
    if (level <= HIVE_LOG_LEVEL_DEBUG) {
        fctprintf(stderr_putc, NULL, "%s:%d: ", basename_simple(file), line);
    }

    // Print the message
    fctprintf(stderr_putc, NULL, "%s\n", text);
}

#endif // HIVE_LOG_TO_STDOUT

// -----------------------------------------------------------------------------
// Binary log file helpers (explicit little-endian serialization)
// -----------------------------------------------------------------------------

#if HIVE_LOG_TO_FILE

// Write 16-bit little-endian value to buffer
static void write_u16_le(uint8_t *buf, uint16_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
}

// Write 32-bit little-endian value to buffer
static void write_u32_le(uint8_t *buf, uint32_t val) {
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

static void log_to_file(hive_log_level_t level, const char *text,
                        size_t text_len) {
    if (s_log_fd < 0)
        return;

    // Build header with explicit byte serialization
    uint8_t header[HIVE_LOG_HEADER_SIZE];
    uint32_t timestamp = (uint32_t)hive_get_time(); // Truncate to 32 bits
    uint16_t len = (uint16_t)(text_len > 0xFFFF ? 0xFFFF : text_len);

    write_u16_le(&header[0], HIVE_LOG_MAGIC); // magic
    write_u16_le(&header[2], s_seq++);        // seq
    write_u32_le(&header[4], timestamp);      // timestamp
    write_u16_le(&header[8], len);            // len
    header[10] = (uint8_t)level;              // level
    header[11] = 0;                           // reserved

    // Write header and payload
    size_t written;
    hive_file_write(s_log_fd, header, HIVE_LOG_HEADER_SIZE, &written);
    if (written == HIVE_LOG_HEADER_SIZE) {
        hive_file_write(s_log_fd, text, len, &written);
    }
}

#endif // HIVE_LOG_TO_FILE

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

hive_status_t hive_log_init(void) {
    HIVE_INIT_GUARD(s_initialized);
    s_initialized = true;
#if HIVE_LOG_TO_FILE
    s_log_fd = -1;
    s_seq = 0;
#endif
    return HIVE_SUCCESS;
}

hive_status_t hive_log_file_open(const char *path) {
#if HIVE_LOG_TO_FILE
    if (!s_initialized) {
        hive_log_init();
    }

    // Close existing file if open
    if (s_log_fd >= 0) {
        hive_log_file_close();
    }

    // Open with create+truncate (TRUNC also erases flash sector on STM32)
    hive_status_t s = hive_file_open(
        path, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644, &s_log_fd);
    if (HIVE_FAILED(s)) {
        s_log_fd = -1;
        return s;
    }

    s_seq = 0; // Reset sequence number for new file
    return HIVE_SUCCESS;
#else
    (void)path;
    return HIVE_SUCCESS; // No-op when file logging disabled
#endif
}

hive_status_t hive_log_file_sync(void) {
#if HIVE_LOG_TO_FILE
    if (s_log_fd < 0) {
        return HIVE_SUCCESS; // No file open, nothing to sync
    }
    return hive_file_sync(s_log_fd);
#else
    return HIVE_SUCCESS;
#endif
}

hive_status_t hive_log_file_close(void) {
#if HIVE_LOG_TO_FILE
    if (s_log_fd < 0) {
        return HIVE_SUCCESS; // No file open
    }

    // Final sync before close
    hive_file_sync(s_log_fd);
    hive_status_t s = hive_file_close(s_log_fd);
    s_log_fd = -1;
    return s;
#else
    return HIVE_SUCCESS;
#endif
}

void hive_log_cleanup(void) {
#if HIVE_LOG_TO_FILE
    if (s_log_fd >= 0) {
        hive_log_file_close();
    }
#endif
    s_initialized = false;
}

void hive_log_write(hive_log_level_t level, const char *file, int line,
                    const char *fmt, ...) {
    // Format the message into a buffer
    char buf[HIVE_LOG_MAX_ENTRY_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf_(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Clamp length to buffer size
    if (len < 0)
        len = 0;
    if ((size_t)len >= sizeof(buf))
        len = sizeof(buf) - 1;

#if HIVE_LOG_TO_STDOUT
    log_to_console(level, file, line, buf);
#else
    (void)file;
    (void)line;
#endif

#if HIVE_LOG_TO_FILE
    // Write to file if open
    if (s_log_fd >= 0) {
        log_to_file(level, buf, (size_t)len);
    }
#endif
}
