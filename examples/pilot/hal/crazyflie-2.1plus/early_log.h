// Early Log Buffer - Captures messages before Hive runtime is ready
//
// HAL init happens before hive_init(), so HIVE_LOG_* isn't available.
// This module buffers early messages and flushes them to the log file
// once the Hive log system is initialized.
//
// Usage:
// - hal_printf() calls early_log_write() to capture formatted output
// - After hive_log_file_open(), call early_log_flush() to replay buffer

#ifndef EARLY_LOG_H
#define EARLY_LOG_H

#include <stddef.h>

// Write message to early log buffer (called by hal_printf)
void early_log_write(const char *msg, size_t len);

// Flush early log buffer to Hive log file and disable buffering
// Call this in main() after hive_log_file_open()
void early_log_flush(void);

#endif // EARLY_LOG_H
