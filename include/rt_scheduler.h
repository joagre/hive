#ifndef RT_SCHEDULER_H
#define RT_SCHEDULER_H

#include "rt_types.h"
#include <stdbool.h>

// Initialize scheduler
rt_status rt_scheduler_init(void);

// Cleanup scheduler
void rt_scheduler_cleanup(void);

// Run scheduler (blocks until all actors exit or shutdown requested)
void rt_scheduler_run(void);

// Request shutdown
void rt_scheduler_shutdown(void);

// Yield control back to scheduler (called by actors)
void rt_scheduler_yield(void);

// Check if shutdown was requested
bool rt_scheduler_should_stop(void);

// Get epoll file descriptor for event loop (for subsystems to register I/O)
int rt_scheduler_get_epoll_fd(void);

#endif // RT_SCHEDULER_H
