#ifndef HIVE_SCHEDULER_H
#define HIVE_SCHEDULER_H

#include "hive_types.h"
#include <stdbool.h>

// Initialize scheduler
hive_status hive_scheduler_init(void);

// Cleanup scheduler
void hive_scheduler_cleanup(void);

// Run scheduler (blocks until all actors exit or shutdown requested)
void hive_scheduler_run(void);

// Request shutdown
void hive_scheduler_shutdown(void);

// Yield control back to scheduler (called by actors)
void hive_scheduler_yield(void);

// Check if shutdown was requested
bool hive_scheduler_should_stop(void);

// Get epoll file descriptor for event loop (for subsystems to register I/O)
int hive_scheduler_get_epoll_fd(void);

#endif // HIVE_SCHEDULER_H
