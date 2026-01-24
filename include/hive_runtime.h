#ifndef HIVE_RUNTIME_H
#define HIVE_RUNTIME_H

#include "hive_types.h"
#include "hive_select.h"

// Initialize runtime (call once from main)
hive_status_t hive_init(void);

// Run scheduler (blocks until all actors exit or hive_shutdown called)
void hive_run(void);

// Run actors until all are blocked (for external event loop integration, e.g.,
// Webots) Runs actors in priority order until no actor is READY (all are
// WAITING or DEAD). Use with hive_advance_time() for simulation integration.
// Returns: HIVE_OK on success
hive_status_t hive_run_until_blocked(void);

// Advance simulation time (microseconds) and fire due timers
// Calling this function enables simulation mode (timers use simulation time,
// not wall-clock). Use with hive_run_until_blocked() for simulation
// integration:
//   while (simulation_running) {
//       hive_advance_time(time_step_us);
//       hive_run_until_blocked();
//   }
void hive_advance_time(uint64_t delta_us);

// Request graceful shutdown
void hive_shutdown(void);

// Cleanup runtime
void hive_cleanup(void);

// Actor lifecycle API

// Spawn a new actor
// fn: Actor entry function
// init: Init function to transform init_args (NULL = skip, pass init_args directly)
// init_args: Arguments to init function (or direct args to actor if init is NULL)
// cfg: Actor configuration (NULL = use defaults)
// out: Output actor ID
//
// If cfg->auto_register is true and cfg->name is set, the actor will be
// automatically registered in the name registry. Spawn fails with
// HIVE_ERR_EXISTS if the name is already taken.
//
// The actor receives its own spawn info as siblings[0] with sibling_count=1.
// For supervised actors, siblings contains all sibling children.
hive_status_t hive_spawn(actor_fn_t fn, hive_actor_init_fn_t init,
                         void *init_args, const actor_config_t *cfg,
                         actor_id_t *out);

// Terminate current actor
_Noreturn void hive_exit(void);

// Get current actor's ID
actor_id_t hive_self(void);

// Yield to scheduler
void hive_yield(void);

// Check if actor is alive
bool hive_actor_alive(actor_id_t id);

// Kill an actor externally
// Terminates the target actor and notifies linked/monitoring actors.
// The target's exit reason will be HIVE_EXIT_KILLED.
// Cannot kill the currently running actor (use hive_exit instead).
// Returns HIVE_ERR_INVALID if target is self or invalid.
hive_status_t hive_kill(actor_id_t target);

// ============================================================================
// Name Registry API
// ============================================================================
// Actor naming. Actors can register themselves with a name, and other actors
// can look up the actor ID by name using whereis(). Names are automatically
// unregistered when the actor exits.

// Register the calling actor with a name (must be unique)
// The name string must remain valid for the lifetime of the registration
// (typically a string literal or static buffer).
// Returns HIVE_ERR_INVALID if name is NULL or already registered.
// Returns HIVE_ERR_NOMEM if registry is full.
hive_status_t hive_register(const char *name);

// Look up an actor ID by name
// Returns HIVE_ERR_INVALID if name is NULL or not found.
hive_status_t hive_whereis(const char *name, actor_id_t *out);

// Unregister a name (also happens automatically on actor exit)
// Only the owning actor can unregister its own name.
// Returns HIVE_ERR_INVALID if name is NULL, not found, or not owned by caller.
hive_status_t hive_unregister(const char *name);

#endif // HIVE_RUNTIME_H
