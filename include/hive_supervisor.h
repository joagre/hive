#ifndef HIVE_SUPERVISOR_H
#define HIVE_SUPERVISOR_H

#include "hive_types.h"
#include "hive_static_config.h"

// =============================================================================
// Supervisor for Hive
// =============================================================================
//
// A supervisor is an actor that monitors child actors and restarts them
// according to configurable policies.
//
// Key concepts:
// - Restart strategies: one_for_one, one_for_all, rest_for_one
// - Restart types: permanent, transient, temporary
// - Restart intensity: max restarts within time window

// -----------------------------------------------------------------------------
// Child Restart Types
// -----------------------------------------------------------------------------

typedef enum {
    HIVE_CHILD_PERMANENT, // Always restart, regardless of exit reason
    HIVE_CHILD_TRANSIENT, // Restart only on abnormal exit (crash)
    HIVE_CHILD_TEMPORARY  // Never restart
} hive_child_restart_t;

// -----------------------------------------------------------------------------
// Restart Strategies
// -----------------------------------------------------------------------------

typedef enum {
    // Restart only the failed child
    HIVE_STRATEGY_ONE_FOR_ONE,

    // Restart all children when one fails
    HIVE_STRATEGY_ONE_FOR_ALL,

    // Restart the failed child and all children started after it
    HIVE_STRATEGY_REST_FOR_ONE
} hive_restart_strategy_t;

// -----------------------------------------------------------------------------
// Child Specification
// -----------------------------------------------------------------------------

typedef struct {
    actor_fn_t start;          // Actor entry point
    hive_actor_init_fn_t init; // Init function (NULL = skip)
    void *init_args;           // Arguments to init function
    size_t init_args_size;     // Size to copy (0 = pass pointer directly)
    const char *name;   // Actor name (for registry AND supervisor tracking)
    bool auto_register; // Register in name registry
    hive_child_restart_t
        restart; // Restart policy (permanent, transient, temporary)
    actor_config_t
        actor_cfg; // Actor configuration (stack size, priority, etc.)
} hive_child_spec_t;

// -----------------------------------------------------------------------------
// Supervisor Configuration
// -----------------------------------------------------------------------------

typedef struct {
    hive_restart_strategy_t strategy; // How to handle child failures

    uint32_t max_restarts;      // Max restarts in period (0 = unlimited)
    uint32_t restart_period_ms; // Time window for max_restarts

    const hive_child_spec_t *children; // Array of child specifications
    size_t num_children;               // Number of children

    void (*on_shutdown)(void *ctx); // Called when supervisor shuts down
    void *shutdown_ctx;             // Context for shutdown callback
} hive_supervisor_config_t;

// Default configuration
#define HIVE_SUPERVISOR_CONFIG_DEFAULT                                  \
    {                                                                   \
        .strategy = HIVE_STRATEGY_ONE_FOR_ONE, .max_restarts = 3,       \
        .restart_period_ms = 5000, .children = NULL, .num_children = 0, \
        .on_shutdown = NULL, .shutdown_ctx = NULL                       \
    }

// -----------------------------------------------------------------------------
// Supervisor API
// -----------------------------------------------------------------------------

// Start a supervisor with the given configuration
// The supervisor is spawned as a new actor that monitors all children.
// Returns the supervisor's actor ID in out_supervisor.
//
// Errors:
//   HIVE_ERR_INVALID  - NULL config, too many children, invalid child spec
//   HIVE_ERR_NOMEM    - No supervisor slots available or spawn failed
hive_status_t hive_supervisor_start(const hive_supervisor_config_t *config,
                                    const actor_config_t *sup_actor_cfg,
                                    actor_id_t *out_supervisor);

// Request supervisor shutdown (async)
// The supervisor will stop all children and then exit.
// Use hive_link() or hive_monitor() to be notified when shutdown completes.
//
// Errors:
//   HIVE_ERR_INVALID  - Invalid supervisor ID
//   HIVE_ERR_NOMEM    - Failed to send shutdown message
hive_status_t hive_supervisor_stop(actor_id_t supervisor);

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------

// Convert restart strategy to string (for logging)
const char *hive_restart_strategy_str(hive_restart_strategy_t strategy);

// Convert child restart type to string (for logging)
const char *hive_child_restart_str(hive_child_restart_t restart);

#endif // HIVE_SUPERVISOR_H
