#ifndef HIVE_RUNTIME_H
#define HIVE_RUNTIME_H

#include "hive_types.h"

// Initialize runtime (call once from main)
hive_status hive_init(void);

// Run scheduler (blocks until all actors exit or hive_shutdown called)
void hive_run(void);

// Request graceful shutdown
void hive_shutdown(void);

// Cleanup runtime
void hive_cleanup(void);

// Actor lifecycle API

// Spawn a new actor with default configuration
hive_status hive_spawn(actor_fn fn, void *arg, actor_id *out);

// Spawn with explicit configuration
hive_status hive_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg, actor_id *out);

// Terminate current actor
_Noreturn void hive_exit(void);

// Get current actor's ID
actor_id hive_self(void);

// Yield to scheduler
void hive_yield(void);

// Check if actor is alive
bool hive_actor_alive(actor_id id);

#endif // HIVE_RUNTIME_H
