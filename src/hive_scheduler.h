#ifndef HIVE_SCHEDULER_H
#define HIVE_SCHEDULER_H

#include "hive_types.h"
#include <stdbool.h>

// Initialize scheduler
hive_status_t hive_scheduler_init(void);

// Cleanup scheduler
void hive_scheduler_cleanup(void);

// Run scheduler (blocks until all actors exit or shutdown requested)
void hive_scheduler_run(void);

// Run actors until all are blocked (for external event loop integration)
// Returns when no actor is READY (all are WAITING or DEAD)
hive_status_t hive_scheduler_run_until_blocked(void);

// Request shutdown
void hive_scheduler_shutdown(void);

// Yield control back to scheduler (called by actors)
void hive_scheduler_yield(void);

// Check if shutdown was requested
bool hive_scheduler_should_stop(void);

// =============================================================================
// Pool Wait Queue (for pool exhaustion blocking)
// =============================================================================
// When an actor with pool_block=true encounters pool exhaustion, it is added
// to the wait queue and yields. When pool space becomes available, the highest
// priority waiter is woken.

// Add current actor to pool wait queue and yield
// Called when pool allocation fails and pool_block is true
// Logs a rate-limited warning about blocking
void hive_scheduler_pool_wait(void);

// Wake highest priority waiter from pool wait queue (if any)
// Called when a pool entry is freed
// Waiters are woken in priority order (CRITICAL > HIGH > NORMAL > LOW)
// FIFO within same priority level
void hive_scheduler_pool_wake_one(void);

// Remove an actor from pool wait queue (if present)
// Called by hive_actor_kill() to clean up killed actors
void hive_scheduler_pool_wait_remove(hive_actor_id_t id);

#endif // HIVE_SCHEDULER_H
