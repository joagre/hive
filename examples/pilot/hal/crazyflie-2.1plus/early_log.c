// Early Log Buffer - Implementation
//
// Captures hal_printf() output before Hive log system is ready.
// Buffer is flushed to log file after hive_log_file_open() in pilot.c.

#include "early_log.h"
#include "hive_log.h"
#include <stdbool.h>
#include <string.h>

// 2KB buffer - enough for HAL init, self-test, and calibration messages
#define EARLY_LOG_BUFFER_SIZE 2048

static char s_buffer[EARLY_LOG_BUFFER_SIZE];
static size_t s_write_pos = 0;
static bool s_active = true; // Disabled after flush

void early_log_write(const char *msg, size_t len) {
    if (!s_active || len == 0) {
        return;
    }

    // If message fits, append it
    if (s_write_pos + len < EARLY_LOG_BUFFER_SIZE) {
        memcpy(s_buffer + s_write_pos, msg, len);
        s_write_pos += len;
    } else {
        // Buffer full - keep most recent by shifting
        // This is O(n) but only happens when buffer fills, which is rare
        size_t keep = EARLY_LOG_BUFFER_SIZE / 2;
        if (s_write_pos > keep) {
            memmove(s_buffer, s_buffer + s_write_pos - keep, keep);
            s_write_pos = keep;
        }
        // Now try to fit the new message
        if (s_write_pos + len < EARLY_LOG_BUFFER_SIZE) {
            memcpy(s_buffer + s_write_pos, msg, len);
            s_write_pos += len;
        }
    }
}

void early_log_flush(void) {
    if (!s_active || s_write_pos == 0) {
        s_active = false;
        return;
    }

    // Write buffered content to log file line by line
    // HIVE_LOG_INFO expects single lines without newlines
    char *start = s_buffer;
    char *end = s_buffer + s_write_pos;

    while (start < end) {
        // Find end of line
        char *newline = start;
        while (newline < end && *newline != '\n') {
            newline++;
        }

        // Temporarily null-terminate the line
        char saved = *newline;
        *newline = '\0';

        // Skip empty lines and write non-empty ones
        if (newline > start) {
            HIVE_LOG_INFO("%s", start);
        }

        *newline = saved;
        start = newline + 1;
    }

    // Disable buffering - all future messages go directly to HIVE_LOG_*
    s_active = false;
    s_write_pos = 0;
}
