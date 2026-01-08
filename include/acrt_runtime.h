#ifndef ACRT_RUNTIME_H
#define ACRT_RUNTIME_H

#include "acrt_types.h"

// Initialize runtime (call once from main)
acrt_status acrt_init(void);

// Run scheduler (blocks until all actors exit or acrt_shutdown called)
void acrt_run(void);

// Request graceful shutdown
void acrt_shutdown(void);

// Cleanup runtime
void acrt_cleanup(void);

// Actor lifecycle API

// Spawn a new actor with default configuration
acrt_status acrt_spawn(actor_fn fn, void *arg, actor_id *out);

// Spawn with explicit configuration
acrt_status acrt_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg, actor_id *out);

// Terminate current actor
_Noreturn void acrt_exit(void);

// Get current actor's ID
actor_id acrt_self(void);

// Yield to scheduler
void acrt_yield(void);

// Check if actor is alive
bool acrt_actor_alive(actor_id id);

#endif // ACRT_RUNTIME_H
