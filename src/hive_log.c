// Hive Logging Implementation
//
// Supports dual output (controlled by compile-time flags):
// - Console (stderr) with optional ANSI colors (HIVE_LOG_TO_STDOUT)
// - Plain text log file (HIVE_LOG_TO_FILE)

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
#include "hal/hive_hal_log.h"
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
// Common helpers
// -----------------------------------------------------------------------------

// Level names for output
#if HIVE_LOG_TO_STDOUT || HIVE_LOG_TO_FILE
static const char *s_level_names[] = {"TRACE", "DEBUG", "INFO", "WARN",
                                      "ERROR"};
#endif

// -----------------------------------------------------------------------------
// Console output helpers
// -----------------------------------------------------------------------------

#if HIVE_LOG_TO_STDOUT

// Extract basename from file path
static const char *basename_simple(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? slash + 1 : path;
}

// Console output via HAL - platform handles colors/formatting
static void log_to_console(hive_log_level_t level, const char *file, int line,
                           const char *text) {
    if (level <= HIVE_LOG_LEVEL_DEBUG) {
        hive_hal_printf("%-5s %s:%d: %s\n", s_level_names[level],
                        basename_simple(file), line, text);
    } else {
        hive_hal_printf("%-5s %s\n", s_level_names[level], text);
    }
}

#endif // HIVE_LOG_TO_STDOUT

// -----------------------------------------------------------------------------
// Plain text log file helpers
// -----------------------------------------------------------------------------

#if HIVE_LOG_TO_FILE

static void log_to_file(hive_log_level_t level, const char *text,
                        size_t text_len) {
    if (s_log_fd < 0)
        return;

    // Format: [MM:SS.mmm] LEVEL text\n
    uint64_t timestamp_us = hive_get_time();
    uint32_t total_ms = (uint32_t)(timestamp_us / 1000);
    uint32_t minutes = total_ms / 60000;
    uint32_t seconds = (total_ms / 1000) % 60;
    uint32_t millis = total_ms % 1000;

    // Build line: [MM:SS.mmm] LEVEL text\n
    char line[HIVE_LOG_MAX_ENTRY_SIZE + 32];
    int header_len = snprintf_(line, sizeof(line), "[%02u:%02u.%03u] %-5s ",
                               (unsigned)minutes, (unsigned)seconds,
                               (unsigned)millis, s_level_names[level]);
    if (header_len < 0)
        header_len = 0;

    // Append text (truncate if needed)
    size_t remaining = sizeof(line) - (size_t)header_len - 2; // -2 for \n\0
    size_t copy_len = text_len < remaining ? text_len : remaining;
    memcpy(line + header_len, text, copy_len);
    line[header_len + copy_len] = '\n';

    // Write line
    size_t written;
    hive_file_write(s_log_fd, line, (size_t)header_len + copy_len + 1,
                    &written);
    s_seq++; // Keep sequence for potential debugging
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

    // Open with create+truncate (clears existing content)
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

hive_status_t hive_log_file_truncate(void) {
#if HIVE_LOG_TO_FILE
    if (s_log_fd < 0) {
        return HIVE_SUCCESS; // No file open
    }

    // Sync pending data before truncating
    hive_file_sync(s_log_fd);

    hive_status_t s = hive_file_truncate(s_log_fd);
    if (HIVE_FAILED(s)) {
        return s;
    }

    s_seq = 0; // Reset sequence number for fresh file
    return HIVE_SUCCESS;
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

#if !HIVE_LOG_TO_STDOUT && !HIVE_LOG_TO_FILE
    (void)level;
    (void)len;
#endif
}
