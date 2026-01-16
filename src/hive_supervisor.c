#include "hive_supervisor.h"
#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_log.h"
#include <string.h>

// =============================================================================
// Internal Types
// =============================================================================

// Message tags for supervisor control (max 27 bits = 0x07FFFFFF)
#define SUP_TAG_STOP 0x05550000 // Stop request

// Child runtime state
typedef struct {
    actor_id id;          // Current actor ID (0 if not running)
    uint32_t monitor_ref; // Monitor reference
    bool running;         // Is child currently running
} child_state;

// Restart timestamp for intensity tracking
typedef struct {
    uint64_t timestamp_us;
} restart_record;

// Supervisor instance state
typedef struct {
    bool in_use;
    actor_id supervisor_id;

    // Configuration (copied from user)
    hive_restart_strategy strategy;
    uint32_t max_restarts;
    uint32_t restart_period_ms;
    size_t num_children;
    void (*on_shutdown)(void *ctx);
    void *shutdown_ctx;

    // Child specs (copied from user)
    hive_child_spec children[HIVE_MAX_SUPERVISOR_CHILDREN];
    // Storage for copied child arguments
    uint8_t arg_storage[HIVE_MAX_SUPERVISOR_CHILDREN][HIVE_MAX_MESSAGE_SIZE];

    // Runtime state
    child_state child_states[HIVE_MAX_SUPERVISOR_CHILDREN];

    // Restart intensity tracking (ring buffer)
    restart_record restarts[HIVE_MAX_SUPERVISOR_CHILDREN];
    size_t restart_head;
    size_t restart_count;
} supervisor_state;

// =============================================================================
// Static Pool
// =============================================================================

static supervisor_state g_supervisors[HIVE_MAX_SUPERVISORS];

static supervisor_state *alloc_supervisor(void) {
    for (size_t i = 0; i < HIVE_MAX_SUPERVISORS; i++) {
        if (!g_supervisors[i].in_use) {
            memset(&g_supervisors[i], 0, sizeof(supervisor_state));
            g_supervisors[i].in_use = true;
            return &g_supervisors[i];
        }
    }
    return NULL;
}

static void free_supervisor(supervisor_state *sup) {
    if (sup) {
        sup->in_use = false;
    }
}

static supervisor_state *find_supervisor_by_id(actor_id id) {
    for (size_t i = 0; i < HIVE_MAX_SUPERVISORS; i++) {
        if (g_supervisors[i].in_use && g_supervisors[i].supervisor_id == id) {
            return &g_supervisors[i];
        }
    }
    return NULL;
}

// =============================================================================
// Time Utilities
// =============================================================================

#ifdef HIVE_PLATFORM_LINUX
#include <sys/time.h>
static uint64_t get_time_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000 + tv.tv_usec;
}
#else
// STM32: use timer tick (assumed to be available)
extern uint64_t hive_timer_now_us(void);
static uint64_t get_time_us(void) {
    return hive_timer_now_us();
}
#endif

// =============================================================================
// Restart Intensity Tracking
// =============================================================================

static void record_restart(supervisor_state *sup) {
    uint64_t now = get_time_us();

    // Add new restart record
    sup->restarts[sup->restart_head].timestamp_us = now;
    sup->restart_head = (sup->restart_head + 1) % HIVE_MAX_SUPERVISOR_CHILDREN;
    if (sup->restart_count < HIVE_MAX_SUPERVISOR_CHILDREN) {
        sup->restart_count++;
    }
}

static bool restart_intensity_exceeded(supervisor_state *sup) {
    if (sup->max_restarts == 0) {
        return false; // Unlimited restarts
    }

    uint64_t now = get_time_us();
    uint64_t window_us = (uint64_t)sup->restart_period_ms * 1000;

    // Count restarts within the window
    uint32_t count = 0;
    for (size_t i = 0; i < sup->restart_count; i++) {
        size_t idx =
            (sup->restart_head + HIVE_MAX_SUPERVISOR_CHILDREN - 1 - i) %
            HIVE_MAX_SUPERVISOR_CHILDREN;
        if (now - sup->restarts[idx].timestamp_us <= window_us) {
            count++;
        }
    }

    return count >= sup->max_restarts;
}

// =============================================================================
// Child Management
// =============================================================================

static hive_status spawn_child(supervisor_state *sup, size_t index) {
    hive_child_spec *spec = &sup->children[index];
    child_state *state = &sup->child_states[index];

    // Determine argument to pass
    void *arg = spec->arg;
    if (spec->arg_size > 0) {
        // Use copied argument from storage
        arg = sup->arg_storage[index];
    }

    // Spawn the child
    hive_status status =
        hive_spawn_ex(spec->fn, arg, &spec->actor_cfg, &state->id);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[SUP] Failed to spawn child \"%s\": %s", spec->id,
                       HIVE_ERR_STR(status));
        return status;
    }

    // Monitor the child
    status = hive_monitor(state->id, &state->monitor_ref);
    if (HIVE_FAILED(status)) {
        // Child spawned but monitor failed - child will run but we won't track it
        HIVE_LOG_ERROR("[SUP] Failed to monitor child \"%s\": %s", spec->id,
                       HIVE_ERR_STR(status));
        state->running = true;
        return status;
    }

    HIVE_LOG_DEBUG("[SUP] Child \"%s\" spawned (actor %u)", spec->id,
                   state->id);
    state->running = true;
    return HIVE_SUCCESS;
}

static void stop_child(supervisor_state *sup, size_t index) {
    child_state *state = &sup->child_states[index];

    if (state->running && state->id != ACTOR_ID_INVALID) {
        // Cancel monitor first (ignore errors - child may already be dead)
        hive_monitor_cancel(state->monitor_ref);

        // Kill the child actor
        hive_kill(state->id);

        state->running = false;
        state->id = ACTOR_ID_INVALID;
    }
}

static size_t find_child_by_actor(supervisor_state *sup, actor_id id) {
    for (size_t i = 0; i < sup->num_children; i++) {
        if (sup->child_states[i].id == id) {
            return i;
        }
    }
    return (size_t)-1;
}

// =============================================================================
// Restart Strategies
// =============================================================================

static bool should_restart_child(hive_child_restart restart,
                                 hive_exit_reason reason) {
    switch (restart) {
    case HIVE_CHILD_PERMANENT:
        return true;
    case HIVE_CHILD_TRANSIENT:
        return reason != HIVE_EXIT_NORMAL;
    case HIVE_CHILD_TEMPORARY:
        return false;
    default:
        return false;
    }
}

static hive_status restart_one_for_one(supervisor_state *sup,
                                       size_t failed_index,
                                       hive_exit_reason reason) {
    hive_child_spec *spec = &sup->children[failed_index];
    child_state *state = &sup->child_states[failed_index];

    state->running = false;
    state->id = ACTOR_ID_INVALID;

    if (!should_restart_child(spec->restart, reason)) {
        return HIVE_SUCCESS;
    }

    record_restart(sup);
    if (restart_intensity_exceeded(sup)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "restart intensity exceeded");
    }

    return spawn_child(sup, failed_index);
}

static hive_status restart_one_for_all(supervisor_state *sup,
                                       size_t failed_index,
                                       hive_exit_reason reason) {
    hive_child_spec *spec = &sup->children[failed_index];

    // Mark failed child as stopped
    sup->child_states[failed_index].running = false;
    sup->child_states[failed_index].id = ACTOR_ID_INVALID;

    if (!should_restart_child(spec->restart, reason)) {
        return HIVE_SUCCESS;
    }

    record_restart(sup);
    if (restart_intensity_exceeded(sup)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "restart intensity exceeded");
    }

    // Stop all other running children (they'll send exit messages)
    for (size_t i = 0; i < sup->num_children; i++) {
        if (i != failed_index) {
            stop_child(sup, i);
        }
    }

    // Restart all children
    for (size_t i = 0; i < sup->num_children; i++) {
        hive_status status = spawn_child(sup, i);
        if (HIVE_FAILED(status)) {
            return status;
        }
    }

    return HIVE_SUCCESS;
}

static hive_status restart_rest_for_one(supervisor_state *sup,
                                        size_t failed_index,
                                        hive_exit_reason reason) {
    hive_child_spec *spec = &sup->children[failed_index];

    // Mark failed child as stopped
    sup->child_states[failed_index].running = false;
    sup->child_states[failed_index].id = ACTOR_ID_INVALID;

    if (!should_restart_child(spec->restart, reason)) {
        return HIVE_SUCCESS;
    }

    record_restart(sup);
    if (restart_intensity_exceeded(sup)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "restart intensity exceeded");
    }

    // Stop children after the failed one
    for (size_t i = failed_index + 1; i < sup->num_children; i++) {
        stop_child(sup, i);
    }

    // Restart failed child and all after it
    for (size_t i = failed_index; i < sup->num_children; i++) {
        hive_status status = spawn_child(sup, i);
        if (HIVE_FAILED(status)) {
            return status;
        }
    }

    return HIVE_SUCCESS;
}

static hive_status handle_child_exit(supervisor_state *sup, actor_id child,
                                     hive_exit_reason reason) {
    size_t index = find_child_by_actor(sup, child);
    if (index == (size_t)-1) {
        // Unknown child - ignore (might be from previous restart cycle)
        return HIVE_SUCCESS;
    }

    HIVE_LOG_WARN("[SUP] Child \"%s\" exited (%s)", sup->children[index].id,
                  hive_exit_reason_str(reason));

    switch (sup->strategy) {
    case HIVE_STRATEGY_ONE_FOR_ONE:
        return restart_one_for_one(sup, index, reason);
    case HIVE_STRATEGY_ONE_FOR_ALL:
        return restart_one_for_all(sup, index, reason);
    case HIVE_STRATEGY_REST_FOR_ONE:
        return restart_rest_for_one(sup, index, reason);
    default:
        return HIVE_ERROR(HIVE_ERR_INVALID, "unknown restart strategy");
    }
}

// =============================================================================
// Supervisor Actor
// =============================================================================

static void supervisor_actor_fn(void *arg) {
    supervisor_state *sup = (supervisor_state *)arg;

    HIVE_LOG_INFO("[SUP] Starting with %zu children (strategy: %s)",
                  sup->num_children, hive_restart_strategy_str(sup->strategy));

    // Spawn all children
    for (size_t i = 0; i < sup->num_children; i++) {
        hive_status status = spawn_child(sup, i);
        if (HIVE_FAILED(status)) {
            // Failed to spawn initial child - shut down
            HIVE_LOG_ERROR("[SUP] Startup failed - shutting down");
            if (sup->on_shutdown) {
                sup->on_shutdown(sup->shutdown_ctx);
            }
            free_supervisor(sup);
            hive_exit();
        }
    }

    HIVE_LOG_INFO("[SUP] All %zu children started", sup->num_children);

    // Main loop - handle messages
    bool shutdown_requested = false;

    while (!shutdown_requested) {
        hive_message msg;
        hive_status status = hive_ipc_recv(&msg, -1);

        if (HIVE_FAILED(status)) {
            continue;
        }

        if (msg.class == HIVE_MSG_EXIT) {
            // Child died
            hive_exit_msg exit_info;
            hive_decode_exit(&msg, &exit_info);

            status = handle_child_exit(sup, exit_info.actor, exit_info.reason);
            if (HIVE_FAILED(status)) {
                // Restart intensity exceeded - shut down
                HIVE_LOG_ERROR("[SUP] Max restarts exceeded - shutting down");
                shutdown_requested = true;
            }
        } else if (msg.class == HIVE_MSG_NOTIFY && msg.tag == SUP_TAG_STOP) {
            // Stop request
            HIVE_LOG_INFO("[SUP] Stop requested");
            shutdown_requested = true;
        }
    }

    // Stop all children
    for (size_t i = 0; i < sup->num_children; i++) {
        stop_child(sup, i);
    }

    // Drain any remaining exit messages briefly
    hive_message msg;
    while (HIVE_SUCCEEDED(hive_ipc_recv(&msg, 10))) {
        // Discard
    }

    // Call shutdown callback
    if (sup->on_shutdown) {
        sup->on_shutdown(sup->shutdown_ctx);
    }

    free_supervisor(sup);
    hive_exit();
}

// =============================================================================
// Public API
// =============================================================================

hive_status hive_supervisor_start(const hive_supervisor_config *config,
                                  const actor_config *sup_actor_cfg,
                                  actor_id *out_supervisor) {
    if (!config || !out_supervisor) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL config or out_supervisor");
    }

    if (config->num_children > HIVE_MAX_SUPERVISOR_CHILDREN) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "too many children");
    }

    if (config->num_children > 0 && !config->children) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL children array");
    }

    // Validate child specs
    for (size_t i = 0; i < config->num_children; i++) {
        if (!config->children[i].fn) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "NULL child function");
        }
        if (config->children[i].arg_size > HIVE_MAX_MESSAGE_SIZE) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "child arg_size too large");
        }
    }

    // Allocate supervisor state
    supervisor_state *sup = alloc_supervisor();
    if (!sup) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "no supervisor slots available");
    }

    // Copy configuration
    sup->strategy = config->strategy;
    sup->max_restarts = config->max_restarts;
    sup->restart_period_ms = config->restart_period_ms;
    sup->num_children = config->num_children;
    sup->on_shutdown = config->on_shutdown;
    sup->shutdown_ctx = config->shutdown_ctx;

    // Copy child specs and arguments
    for (size_t i = 0; i < config->num_children; i++) {
        sup->children[i] = config->children[i];

        // Copy argument if needed
        if (config->children[i].arg_size > 0 && config->children[i].arg) {
            memcpy(sup->arg_storage[i], config->children[i].arg,
                   config->children[i].arg_size);
        }
    }

    // Use provided actor config or default
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    if (sup_actor_cfg) {
        cfg = *sup_actor_cfg;
    }
    if (!cfg.name) {
        cfg.name = "supervisor";
    }

    // Spawn supervisor actor
    hive_status status =
        hive_spawn_ex(supervisor_actor_fn, sup, &cfg, out_supervisor);
    if (HIVE_FAILED(status)) {
        free_supervisor(sup);
        return status;
    }

    sup->supervisor_id = *out_supervisor;
    return HIVE_SUCCESS;
}

hive_status hive_supervisor_stop(actor_id supervisor) {
    supervisor_state *sup = find_supervisor_by_id(supervisor);
    if (!sup) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid supervisor ID");
    }

    // Send stop message
    return hive_ipc_notify(supervisor, SUP_TAG_STOP, NULL, 0);
}

const char *hive_restart_strategy_str(hive_restart_strategy strategy) {
    switch (strategy) {
    case HIVE_STRATEGY_ONE_FOR_ONE:
        return "one_for_one";
    case HIVE_STRATEGY_ONE_FOR_ALL:
        return "one_for_all";
    case HIVE_STRATEGY_REST_FOR_ONE:
        return "rest_for_one";
    default:
        return "unknown";
    }
}

const char *hive_child_restart_str(hive_child_restart restart) {
    switch (restart) {
    case HIVE_CHILD_PERMANENT:
        return "permanent";
    case HIVE_CHILD_TRANSIENT:
        return "transient";
    case HIVE_CHILD_TEMPORARY:
        return "temporary";
    default:
        return "unknown";
    }
}
