#include "hive_supervisor.h"
#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_actor.h"
#include "hive_log.h"
#include <string.h>

// =============================================================================
// Internal Types
// =============================================================================

// Message tags for supervisor control (max 27 bits = 0x07FFFFFF)
#define SUP_TAG_STOP 0x05550000 // Stop request

// Child runtime state
typedef struct {
    actor_id_t id;       // Current actor_t ID (0 if not running)
    uint32_t monitor_id; // Monitor ID
    bool running;        // Is child currently running
} child_state_t;

// Restart timestamp for intensity tracking
typedef struct {
    uint64_t timestamp_us;
} restart_record_t;

// Supervisor instance state
typedef struct {
    bool in_use;
    actor_id_t supervisor_id;

    // Configuration (copied from user)
    hive_restart_strategy_t strategy;
    uint32_t max_restarts;
    uint32_t restart_period_ms;
    size_t num_children;
    void (*on_shutdown)(void *ctx);
    void *shutdown_ctx;

    // Child specs (copied from user)
    hive_child_spec_t children[HIVE_MAX_SUPERVISOR_CHILDREN];
    // Storage for copied child init_args
    uint8_t arg_storage[HIVE_MAX_SUPERVISOR_CHILDREN][HIVE_MAX_MESSAGE_SIZE];

    // Runtime state
    child_state_t child_states[HIVE_MAX_SUPERVISOR_CHILDREN];

    // Sibling info array (built during two-phase start)
    hive_spawn_info_t sibling_info[HIVE_MAX_SUPERVISOR_CHILDREN];

    // Restart intensity tracking (ring buffer)
    restart_record_t restarts[HIVE_MAX_SUPERVISOR_CHILDREN];
    size_t restart_head;
    size_t restart_count;
} supervisor_state_t;

// =============================================================================
// Static Pool
// =============================================================================

static supervisor_state_t s_supervisors[HIVE_MAX_SUPERVISORS];

static supervisor_state_t *alloc_supervisor(void) {
    for (size_t i = 0; i < HIVE_MAX_SUPERVISORS; i++) {
        if (!s_supervisors[i].in_use) {
            memset(&s_supervisors[i], 0, sizeof(supervisor_state_t));
            s_supervisors[i].in_use = true;
            return &s_supervisors[i];
        }
    }
    return NULL;
}

static void free_supervisor(supervisor_state_t *sup) {
    if (sup) {
        sup->in_use = false;
    }
}

static supervisor_state_t *find_supervisor_by_id(actor_id_t id) {
    for (size_t i = 0; i < HIVE_MAX_SUPERVISORS; i++) {
        if (s_supervisors[i].in_use && s_supervisors[i].supervisor_id == id) {
            return &s_supervisors[i];
        }
    }
    return NULL;
}

// =============================================================================
// Time Utilities
// =============================================================================

// hive_get_time() returns microseconds on all platforms
extern uint64_t hive_get_time(void);
static uint64_t get_time_us(void) {
    return hive_get_time();
}

// =============================================================================
// Restart Intensity Tracking
// =============================================================================

static void record_restart(supervisor_state_t *sup) {
    uint64_t now = get_time_us();

    // Add new restart record
    sup->restarts[sup->restart_head].timestamp_us = now;
    sup->restart_head = (sup->restart_head + 1) % HIVE_MAX_SUPERVISOR_CHILDREN;
    if (sup->restart_count < HIVE_MAX_SUPERVISOR_CHILDREN) {
        sup->restart_count++;
    }
}

static bool restart_intensity_exceeded(supervisor_state_t *sup) {
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

// Build sibling info array from current child states
static void build_sibling_info(supervisor_state_t *sup) {
    for (size_t i = 0; i < sup->num_children; i++) {
        sup->sibling_info[i].name = sup->children[i].name;
        sup->sibling_info[i].id = sup->child_states[i].id;
        sup->sibling_info[i].registered = sup->children[i].auto_register;
    }
}

// Update actor_t's startup info to point to supervisor's sibling array
static void set_child_siblings(supervisor_state_t *sup, size_t index) {
    actor_t *a = hive_actor_get(sup->child_states[index].id);
    if (a) {
        a->startup_siblings = sup->sibling_info;
        a->startup_sibling_count = sup->num_children;
    }
}

// Spawn a single child with sibling info
static hive_status_t spawn_child(supervisor_state_t *sup, size_t index) {
    hive_child_spec_t *spec = &sup->children[index];
    child_state_t *state = &sup->child_states[index];

    // Determine init_args to pass
    void *init_args = spec->init_args;
    if (spec->init_args_size > 0) {
        // Use copied argument from storage
        init_args = sup->arg_storage[index];
    }

    // Build actor_t config with name from child spec
    hive_actor_config_t cfg = spec->actor_cfg;
    cfg.name = spec->name;
    cfg.auto_register = spec->auto_register;

    // Spawn the child using new spawn API
    hive_status_t status =
        hive_spawn(spec->start, spec->init, init_args, &cfg, &state->id);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[SUP] Failed to spawn child \"%s\": %s", spec->name,
                       HIVE_ERR_STR(status));
        return status;
    }

    // Update sibling info for this child
    sup->sibling_info[index].name = spec->name;
    sup->sibling_info[index].id = state->id;
    sup->sibling_info[index].registered = spec->auto_register;

    // Set sibling info in the actor_t
    set_child_siblings(sup, index);

    // Monitor the child
    status = hive_monitor(state->id, &state->monitor_id);
    if (HIVE_FAILED(status)) {
        // Can't supervise a child we can't monitor - kill it and fail
        HIVE_LOG_ERROR("[SUP] Failed to monitor child \"%s\": %s", spec->name,
                       HIVE_ERR_STR(status));
        hive_actor_kill(state->id);
        state->id = HIVE_ACTOR_ID_INVALID;
        state->running = false;
        return status;
    }

    HIVE_LOG_DEBUG("[SUP] Child \"%s\" spawned (actor_t %u)", spec->name,
                   state->id);
    state->running = true;
    return HIVE_SUCCESS;
}

// Two-phase start: spawn all children, then set sibling info for all
static hive_status_t spawn_all_children_two_phase(supervisor_state_t *sup) {
    // Phase 1: Spawn all children
    for (size_t i = 0; i < sup->num_children; i++) {
        hive_child_spec_t *spec = &sup->children[i];
        child_state_t *state = &sup->child_states[i];

        // Determine init_args to pass
        void *init_args = spec->init_args;
        if (spec->init_args_size > 0) {
            init_args = sup->arg_storage[i];
        }

        // Build actor_t config
        hive_actor_config_t cfg = spec->actor_cfg;
        cfg.name = spec->name;
        cfg.auto_register = spec->auto_register;

        // Spawn the child
        hive_status_t status =
            hive_spawn(spec->start, spec->init, init_args, &cfg, &state->id);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[SUP] Failed to spawn child \"%s\": %s", spec->name,
                           HIVE_ERR_STR(status));
            // Rollback: kill all previously spawned children
            for (size_t j = 0; j < i; j++) {
                hive_actor_kill(sup->child_states[j].id);
                sup->child_states[j].id = HIVE_ACTOR_ID_INVALID;
                sup->child_states[j].running = false;
            }
            return status;
        }

        state->running = true;
    }

    // Phase 2: Build complete sibling info array
    build_sibling_info(sup);

    // Phase 3: Set sibling info for all children and add monitors
    for (size_t i = 0; i < sup->num_children; i++) {
        set_child_siblings(sup, i);

        // Monitor the child
        hive_status_t status = hive_monitor(sup->child_states[i].id,
                                            &sup->child_states[i].monitor_id);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[SUP] Failed to monitor child \"%s\": %s",
                           sup->children[i].name, HIVE_ERR_STR(status));
            // Rollback: cancel monitors and kill all children
            for (size_t j = 0; j < i; j++) {
                hive_demonitor(sup->child_states[j].monitor_id);
            }
            for (size_t j = 0; j < sup->num_children; j++) {
                hive_actor_kill(sup->child_states[j].id);
                sup->child_states[j].id = HIVE_ACTOR_ID_INVALID;
                sup->child_states[j].running = false;
            }
            return status;
        }

        HIVE_LOG_DEBUG("[SUP] Child \"%s\" spawned (actor_t %u)",
                       sup->children[i].name, sup->child_states[i].id);
    }

    return HIVE_SUCCESS;
}

static void stop_child(supervisor_state_t *sup, size_t index) {
    child_state_t *state = &sup->child_states[index];

    if (state->running && state->id != HIVE_ACTOR_ID_INVALID) {
        // Cancel monitor first (ignore errors - child may already be dead)
        hive_demonitor(state->monitor_id);

        // Kill the child actor_t
        hive_actor_kill(state->id);

        state->running = false;
        state->id = HIVE_ACTOR_ID_INVALID;
    }
}

static size_t find_child_by_actor(supervisor_state_t *sup, actor_id_t id) {
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

static bool should_restart_child(hive_child_restart_t restart,
                                 hive_exit_reason_t reason) {
    switch (restart) {
    case HIVE_CHILD_PERMANENT:
        return true;
    case HIVE_CHILD_TRANSIENT:
        return reason != HIVE_EXIT_REASON_NORMAL;
    case HIVE_CHILD_TEMPORARY:
        return false;
    default:
        return false;
    }
}

static hive_status_t restart_one_for_one(supervisor_state_t *sup,
                                         size_t failed_index,
                                         hive_exit_reason_t reason) {
    hive_child_spec_t *spec = &sup->children[failed_index];
    child_state_t *state = &sup->child_states[failed_index];

    state->running = false;
    state->id = HIVE_ACTOR_ID_INVALID;

    if (!should_restart_child(spec->restart, reason)) {
        return HIVE_SUCCESS;
    }

    record_restart(sup);
    if (restart_intensity_exceeded(sup)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "restart intensity exceeded");
    }

    return spawn_child(sup, failed_index);
}

static hive_status_t restart_one_for_all(supervisor_state_t *sup,
                                         size_t failed_index,
                                         hive_exit_reason_t reason) {
    hive_child_spec_t *spec = &sup->children[failed_index];

    // Mark failed child as stopped
    sup->child_states[failed_index].running = false;
    sup->child_states[failed_index].id = HIVE_ACTOR_ID_INVALID;

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
        hive_status_t status = spawn_child(sup, i);
        if (HIVE_FAILED(status)) {
            return status;
        }
    }

    return HIVE_SUCCESS;
}

static hive_status_t restart_rest_for_one(supervisor_state_t *sup,
                                          size_t failed_index,
                                          hive_exit_reason_t reason) {
    hive_child_spec_t *spec = &sup->children[failed_index];

    // Mark failed child as stopped
    sup->child_states[failed_index].running = false;
    sup->child_states[failed_index].id = HIVE_ACTOR_ID_INVALID;

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
        hive_status_t status = spawn_child(sup, i);
        if (HIVE_FAILED(status)) {
            return status;
        }
    }

    return HIVE_SUCCESS;
}

static hive_status_t handle_child_exit(supervisor_state_t *sup,
                                       actor_id_t child,
                                       hive_exit_reason_t reason) {
    size_t index = find_child_by_actor(sup, child);
    if (index == (size_t)-1) {
        // Unknown child - ignore (might be from previous restart cycle)
        return HIVE_SUCCESS;
    }

    HIVE_LOG_WARN("[SUP] Child \"%s\" exited (reason=%u)",
                  sup->children[index].name, (unsigned)reason);

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

static void supervisor_actor_fn(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    supervisor_state_t *sup = (supervisor_state_t *)args;

    HIVE_LOG_INFO("[SUP] Starting with %zu children (strategy: %s)",
                  sup->num_children, hive_restart_strategy_str(sup->strategy));

    // Spawn all children using two-phase start
    hive_status_t status = spawn_all_children_two_phase(sup);
    if (HIVE_FAILED(status)) {
        // Failed to spawn initial children - shut down
        HIVE_LOG_ERROR("[SUP] Startup failed - shutting down");
        if (sup->on_shutdown) {
            sup->on_shutdown(sup->shutdown_ctx);
        }
        free_supervisor(sup);
        hive_exit(HIVE_EXIT_REASON_NORMAL);
    }

    HIVE_LOG_INFO("[SUP] All %zu children started", sup->num_children);

    // Main loop - handle messages
    bool shutdown_requested = false;

    while (!shutdown_requested) {
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, -1);

        if (HIVE_FAILED(status)) {
            continue;
        }

        if (msg.class == HIVE_MSG_EXIT) {
            // Child died
            hive_exit_msg_t exit_info;
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
    hive_message_t msg;
    while (HIVE_SUCCEEDED(hive_ipc_recv(&msg, 10))) {
        // Discard
    }

    // Call shutdown callback
    if (sup->on_shutdown) {
        sup->on_shutdown(sup->shutdown_ctx);
    }

    free_supervisor(sup);
    hive_exit(HIVE_EXIT_REASON_NORMAL);
}

// =============================================================================
// Public API
// =============================================================================

hive_status_t hive_supervisor_start(const hive_supervisor_config_t *config,
                                    const hive_actor_config_t *sup_actor_cfg,
                                    actor_id_t *out_supervisor) {
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
        if (!config->children[i].start) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "NULL child function");
        }
        if (config->children[i].init_args_size > HIVE_MAX_MESSAGE_SIZE) {
            return HIVE_ERROR(HIVE_ERR_INVALID,
                              "child init_args_size too large");
        }
    }

    // Allocate supervisor state
    supervisor_state_t *sup = alloc_supervisor();
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

    // Copy child specs and init_args
    for (size_t i = 0; i < config->num_children; i++) {
        sup->children[i] = config->children[i];

        // Copy init_args if needed
        if (config->children[i].init_args_size > 0 &&
            config->children[i].init_args) {
            memcpy(sup->arg_storage[i], config->children[i].init_args,
                   config->children[i].init_args_size);
        }
    }

    // Use provided actor_t config or default
    hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    if (sup_actor_cfg) {
        cfg = *sup_actor_cfg;
    }
    if (!cfg.name) {
        cfg.name = "supervisor";
    }

    // Spawn supervisor actor_t using new spawn API
    hive_status_t status =
        hive_spawn(supervisor_actor_fn, NULL, sup, &cfg, out_supervisor);
    if (HIVE_FAILED(status)) {
        free_supervisor(sup);
        return status;
    }

    sup->supervisor_id = *out_supervisor;
    return HIVE_SUCCESS;
}

hive_status_t hive_supervisor_stop(actor_id_t supervisor) {
    supervisor_state_t *sup = find_supervisor_by_id(supervisor);
    if (!sup) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "invalid supervisor ID");
    }

    // Deliver stop message
    return hive_ipc_notify(supervisor, SUP_TAG_STOP, NULL, 0);
}

const char *hive_restart_strategy_str(hive_restart_strategy_t strategy) {
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

const char *hive_child_restart_str(hive_child_restart_t restart) {
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
