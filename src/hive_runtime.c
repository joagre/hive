#include "hive_runtime.h"
#include "hive_internal.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_static_config.h"
#include <stdlib.h>
#include <string.h>

// =============================================================================
// Name Registry
// =============================================================================

typedef struct {
    const char *name; // Points to user-provided string (must remain valid)
    actor_id_t actor;
} registry_entry_t;

static registry_entry_t s_registry[HIVE_MAX_REGISTERED_NAMES];
static size_t s_registry_count = 0;

hive_status_t hive_init(void) {
    // Initialize actor_t subsystem
    hive_status_t status = hive_actor_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Initialize scheduler
    status = hive_scheduler_init();
    if (HIVE_FAILED(status)) {
        hive_actor_cleanup();
        return status;
    }

    // Initialize IPC pools
    status = hive_ipc_init();
    if (HIVE_FAILED(status)) {
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }

    // Initialize link subsystem
    status = hive_link_init();
    if (HIVE_FAILED(status)) {
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }

#if HIVE_ENABLE_FILE
    // Initialize file I/O subsystem
    status = hive_file_init();
    if (HIVE_FAILED(status)) {
        hive_link_cleanup();
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }
#endif

#if HIVE_ENABLE_TCP
    // Initialize TCP subsystem
    status = hive_tcp_init();
    if (HIVE_FAILED(status)) {
#if HIVE_ENABLE_FILE
        hive_file_cleanup();
#endif
        hive_link_cleanup();
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }
#endif

    // Initialize timer subsystem
    status = hive_timer_init();
    if (HIVE_FAILED(status)) {
#if HIVE_ENABLE_TCP
        hive_tcp_cleanup();
#endif
#if HIVE_ENABLE_FILE
        hive_file_cleanup();
#endif
        hive_link_cleanup();
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }

    // Initialize bus subsystem
    status = hive_bus_init();
    if (HIVE_FAILED(status)) {
        hive_timer_cleanup();
#if HIVE_ENABLE_TCP
        hive_tcp_cleanup();
#endif
#if HIVE_ENABLE_FILE
        hive_file_cleanup();
#endif
        hive_link_cleanup();
        hive_scheduler_cleanup();
        hive_actor_cleanup();
        return status;
    }

    return HIVE_SUCCESS;
}

void hive_run(void) {
    hive_scheduler_run();
}

hive_status_t hive_run_until_blocked(void) {
    return hive_scheduler_run_until_blocked();
}

void hive_advance_time(uint64_t delta_us) {
    hive_timer_advance_time(delta_us);
}

void hive_shutdown(void) {
    hive_scheduler_shutdown();
}

void hive_cleanup(void) {
    hive_bus_cleanup();
    hive_timer_cleanup();
#if HIVE_ENABLE_TCP
    hive_tcp_cleanup();
#endif
#if HIVE_ENABLE_FILE
    hive_file_cleanup();
#endif
    hive_link_cleanup();
    hive_scheduler_cleanup();
    hive_actor_cleanup();
}

// Internal function to check if a name is already registered
static bool name_is_registered(const char *name) {
    for (size_t i = 0; i < s_registry_count; i++) {
        if (strcmp(s_registry[i].name, name) == 0) {
            return true;
        }
    }
    return false;
}

// Internal function to register an actor_t by ID (for auto_register)
static hive_status_t register_actor_by_id(const char *name, actor_id_t id) {
    if (s_registry_count >= HIVE_MAX_REGISTERED_NAMES) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Registry full");
    }

    s_registry[s_registry_count].name = name;
    s_registry[s_registry_count].actor = id;
    s_registry_count++;

    HIVE_LOG_DEBUG("Auto-registered actor_t %u as '%s'", id, name);
    return HIVE_SUCCESS;
}

hive_status_t hive_spawn(hive_actor_fn_t fn, hive_actor_init_fn_t init,
                         void *init_args, const hive_actor_config_t *cfg,
                         actor_id_t *out) {
    if (!fn) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL function pointer");
    }
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL output pointer");
    }

    // Use default config if none provided
    hive_actor_config_t default_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    const hive_actor_config_t *use_cfg = cfg ? cfg : &default_cfg;

    // Copy config field by field instead of struct copy to avoid alignment
    // issues (struct copy may use SIMD instructions requiring 16-byte
    // alignment)
    hive_actor_config_t actual_cfg;
    actual_cfg.stack_size = use_cfg->stack_size;
    actual_cfg.priority = use_cfg->priority;
    actual_cfg.name = use_cfg->name;
    actual_cfg.malloc_stack = use_cfg->malloc_stack;
    actual_cfg.auto_register = use_cfg->auto_register;
    actual_cfg.pool_block = use_cfg->pool_block;
    if (actual_cfg.stack_size == 0) {
        actual_cfg.stack_size = HIVE_DEFAULT_STACK_SIZE;
    }

    // Validate auto_register requirements
    if (actual_cfg.auto_register) {
        if (!actual_cfg.name) {
            return HIVE_ERROR(HIVE_ERR_INVALID,
                              "auto_register requires name to be set");
        }
        // Check if name is already registered
        if (name_is_registered(actual_cfg.name)) {
            return HIVE_ERROR(HIVE_ERR_EXISTS, "Name already registered");
        }
    }

    // Call init function if provided
    void *args = init_args;
    if (init) {
        args = init(init_args);
    }

    // Allocate actor_t first (with NULL siblings, we'll set them after)
    actor_t *a = hive_actor_alloc(fn, args, NULL, 0, &actual_cfg);
    if (!a) {
        return HIVE_ERROR(HIVE_ERR_NOMEM,
                          "Actor table or stack arena exhausted");
    }

    // Handle auto_register
    if (actual_cfg.auto_register) {
        hive_status_t reg_status = register_actor_by_id(actual_cfg.name, a->id);
        if (HIVE_FAILED(reg_status)) {
            // Registration failed, clean up actor_t
            hive_actor_free(a);
            return reg_status;
        }
    }

    // Set up self spawn info in the actor_t struct (stable storage)
    a->self_spawn_info.name = actual_cfg.name;
    a->self_spawn_info.id = a->id;
    a->self_spawn_info.registered = actual_cfg.auto_register;

    // For standalone spawn, point siblings to self_spawn_info with count=1
    a->startup_siblings = &a->self_spawn_info;
    a->startup_sibling_count = 1;

    *out = a->id;
    return HIVE_SUCCESS;
}

_Noreturn void hive_exit(void) {
    actor_t *current = hive_actor_current();
    if (current) {
        HIVE_LOG_DEBUG("Actor %u (%s) exiting", current->id,
                       current->name ? current->name : "unnamed");

        // Mark exit reason and actor_t state
        // Scheduler will clean up resources - don't free stack here!
        current->exit_reason = HIVE_EXIT_NORMAL;
        current->state = ACTOR_STATE_DEAD;
    }

    // Yield back to scheduler and never return
    hive_scheduler_yield();

    // Should never reach here
    HIVE_LOG_ERROR("hive_exit: returned from scheduler yield");
    abort();
}

_Noreturn void hive_exit_crash(void) {
    actor_t *current = hive_actor_current();
    if (current) {
        HIVE_LOG_ERROR("Actor %u (%s) returned without calling hive_exit()",
                       (unsigned)current->id,
                       current->name ? current->name : "unnamed");

        // Mark as crashed - linked/monitoring actors will be notified
        current->exit_reason = HIVE_EXIT_CRASH;
        current->state = ACTOR_STATE_DEAD;
    }

    // Yield back to scheduler and never return
    hive_scheduler_yield();

    // Should never reach here
    HIVE_LOG_ERROR("hive_exit_crash: returned from scheduler yield");
    abort();
}

actor_id_t hive_self(void) {
    actor_t *current = hive_actor_current();
    return current ? current->id : HIVE_ACTOR_ID_INVALID;
}

void hive_yield(void) {
    hive_scheduler_yield();
}

bool hive_actor_alive(actor_id_t id) {
    actor_t *a = hive_actor_get(id);
    return a != NULL && a->state != ACTOR_STATE_DEAD;
}

hive_status_t hive_actor_kill(actor_id_t target) {
    // Cannot kill self
    actor_t *current = hive_actor_current();
    if (current && current->id == target) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Cannot kill self (use hive_exit)");
    }

    // Get target actor_t
    actor_t *a = hive_actor_get(target);
    if (!a || a->state == ACTOR_STATE_DEAD) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid or dead actor_t");
    }

    HIVE_LOG_DEBUG("Killing actor_t %u (%s)", a->id,
                   a->name ? a->name : "unnamed");

    // Set exit reason before cleanup (so notifications report correct reason)
    a->exit_reason = HIVE_EXIT_KILLED;

    // Free actor_t resources and send death notifications
    hive_actor_free(a);

    return HIVE_SUCCESS;
}

// =============================================================================
// Name Registry Implementation
// =============================================================================

hive_status_t hive_register(const char *name) {
    if (!name) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL name");
    }

    actor_t *current = hive_actor_current();
    if (!current) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not called from actor_t context");
    }

    // Check for duplicate name
    for (size_t i = 0; i < s_registry_count; i++) {
        if (strcmp(s_registry[i].name, name) == 0) {
            return HIVE_ERROR(HIVE_ERR_INVALID, "Name already registered");
        }
    }

    // Check for space
    if (s_registry_count >= HIVE_MAX_REGISTERED_NAMES) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Registry full");
    }

    // Add entry
    s_registry[s_registry_count].name = name;
    s_registry[s_registry_count].actor = current->id;
    s_registry_count++;

    HIVE_LOG_DEBUG("Registered actor_t %u as '%s'", current->id, name);

    return HIVE_SUCCESS;
}

hive_status_t hive_whereis(const char *name, actor_id_t *out) {
    if (!name) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL name");
    }
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL output pointer");
    }

    for (size_t i = 0; i < s_registry_count; i++) {
        if (strcmp(s_registry[i].name, name) == 0) {
            *out = s_registry[i].actor;
            return HIVE_SUCCESS;
        }
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Name not found");
}

hive_status_t hive_unregister(const char *name) {
    if (!name) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL name");
    }

    actor_t *current = hive_actor_current();
    if (!current) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not called from actor_t context");
    }

    for (size_t i = 0; i < s_registry_count; i++) {
        if (strcmp(s_registry[i].name, name) == 0) {
            // Check ownership
            if (s_registry[i].actor != current->id) {
                return HIVE_ERROR(HIVE_ERR_INVALID,
                                  "Name not owned by calling actor_t");
            }

            HIVE_LOG_DEBUG("Unregistered '%s' (was actor_t %u)", name,
                           current->id);

            // Remove by swapping with last entry
            s_registry[i] = s_registry[s_registry_count - 1];
            s_registry_count--;
            return HIVE_SUCCESS;
        }
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Name not found");
}

// Called by hive_actor_free() to clean up registry entries for dying actor_t
void hive_registry_cleanup_actor(actor_id_t id) {
    // Remove all entries for this actor_t (scan backwards to allow removal)
    for (size_t i = s_registry_count; i > 0; i--) {
        if (s_registry[i - 1].actor == id) {
            HIVE_LOG_DEBUG("Auto-unregistered '%s' (actor_t %u exiting)",
                           s_registry[i - 1].name, id);
            // Remove by swapping with last entry
            s_registry[i - 1] = s_registry[s_registry_count - 1];
            s_registry_count--;
        }
    }
}

// =============================================================================
// Pool Exhaustion Behavior API
// =============================================================================

void hive_pool_set_block(hive_pool_block_t mode) {
    actor_t *current = hive_actor_current();
    if (!current) {
        return; // Not in actor context, ignore
    }

    switch (mode) {
    case HIVE_POOL_NO_BLOCK:
        current->pool_block = false;
        break;
    case HIVE_POOL_BLOCK:
        current->pool_block = true;
        break;
    case HIVE_POOL_DEFAULT:
        current->pool_block = current->pool_block_default;
        break;
    }
}

bool hive_pool_get_block(void) {
    actor_t *current = hive_actor_current();
    if (!current) {
        return false; // Not in actor context, default to non-blocking
    }
    return current->pool_block;
}
