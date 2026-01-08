#include "acrt_runtime.h"
#include "acrt_actor.h"
#include "acrt_scheduler.h"
#include "acrt_link.h"
#include "acrt_log.h"
#include "acrt_static_config.h"
#include <stdio.h>
#include <stdlib.h>

// Forward declarations for subsystems
extern acrt_status acrt_ipc_init(void);
extern acrt_status acrt_file_init(void);
extern void acrt_file_cleanup(void);
extern acrt_status acrt_net_init(void);
extern void acrt_net_cleanup(void);
extern acrt_status acrt_timer_init(void);
extern void acrt_timer_cleanup(void);
extern acrt_status acrt_bus_init(void);
extern void acrt_bus_cleanup(void);
extern acrt_status acrt_link_init(void);
extern void acrt_link_cleanup(void);

acrt_status acrt_init(void) {
    // Initialize actor subsystem
    acrt_status status = acrt_actor_init();
    if (ACRT_FAILED(status)) {
        return status;
    }

    // Initialize scheduler
    status = acrt_scheduler_init();
    if (ACRT_FAILED(status)) {
        acrt_actor_cleanup();
        return status;
    }

    // Initialize IPC pools
    status = acrt_ipc_init();
    if (ACRT_FAILED(status)) {
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    // Initialize link subsystem
    status = acrt_link_init();
    if (ACRT_FAILED(status)) {
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    // Initialize file I/O subsystem
    status = acrt_file_init();
    if (ACRT_FAILED(status)) {
        acrt_link_cleanup();
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    // Initialize network I/O subsystem
    status = acrt_net_init();
    if (ACRT_FAILED(status)) {
        acrt_file_cleanup();
        acrt_link_cleanup();
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    // Initialize timer subsystem
    status = acrt_timer_init();
    if (ACRT_FAILED(status)) {
        acrt_net_cleanup();
        acrt_file_cleanup();
        acrt_link_cleanup();
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    // Initialize bus subsystem
    status = acrt_bus_init();
    if (ACRT_FAILED(status)) {
        acrt_timer_cleanup();
        acrt_net_cleanup();
        acrt_file_cleanup();
        acrt_link_cleanup();
        acrt_scheduler_cleanup();
        acrt_actor_cleanup();
        return status;
    }

    return ACRT_SUCCESS;
}

void acrt_run(void) {
    acrt_scheduler_run();
}

void acrt_shutdown(void) {
    acrt_scheduler_shutdown();
}

void acrt_cleanup(void) {
    acrt_bus_cleanup();
    acrt_timer_cleanup();
    acrt_net_cleanup();
    acrt_file_cleanup();
    acrt_link_cleanup();
    acrt_scheduler_cleanup();
    acrt_actor_cleanup();
}

acrt_status acrt_spawn(actor_fn fn, void *arg, actor_id *out) {
    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = ACRT_DEFAULT_STACK_SIZE;
    return acrt_spawn_ex(fn, arg, &cfg, out);
}

acrt_status acrt_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg, actor_id *out) {
    if (!fn) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "NULL function pointer");
    }
    if (!out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "NULL output pointer");
    }

    // Copy config field by field instead of struct copy to avoid alignment issues
    // (struct copy may use SIMD instructions requiring 16-byte alignment)
    actor_config actual_cfg;
    actual_cfg.stack_size = cfg->stack_size;
    actual_cfg.priority = cfg->priority;
    actual_cfg.name = cfg->name;
    actual_cfg.malloc_stack = cfg->malloc_stack;
    if (actual_cfg.stack_size == 0) {
        actual_cfg.stack_size = ACRT_DEFAULT_STACK_SIZE;
    }

    actor *a = acrt_actor_alloc(fn, arg, &actual_cfg);
    if (!a) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Actor table or stack arena exhausted");
    }

    *out = a->id;
    return ACRT_SUCCESS;
}

_Noreturn void acrt_exit(void) {
    actor *current = acrt_actor_current();
    if (current) {
        ACRT_LOG_DEBUG("Actor %u (%s) exiting", current->id,
                     current->name ? current->name : "unnamed");

        // Mark exit reason and actor state
        // Scheduler will clean up resources - don't free stack here!
        current->exit_reason = ACRT_EXIT_NORMAL;
        current->state = ACTOR_STATE_DEAD;
    }

    // Yield back to scheduler and never return
    acrt_scheduler_yield();

    // Should never reach here
    ACRT_LOG_ERROR("acrt_exit: returned from scheduler yield");
    abort();
}

_Noreturn void acrt_exit_crash(void) {
    actor *current = acrt_actor_current();
    if (current) {
        ACRT_LOG_ERROR("Actor %u (%s) returned without calling acrt_exit()",
                     current->id, current->name ? current->name : "unnamed");

        // Mark as crashed - linked/monitoring actors will be notified
        current->exit_reason = ACRT_EXIT_CRASH;
        current->state = ACTOR_STATE_DEAD;
    }

    // Yield back to scheduler and never return
    acrt_scheduler_yield();

    // Should never reach here
    ACRT_LOG_ERROR("acrt_exit_crash: returned from scheduler yield");
    abort();
}

actor_id acrt_self(void) {
    actor *current = acrt_actor_current();
    return current ? current->id : ACTOR_ID_INVALID;
}

void acrt_yield(void) {
    acrt_scheduler_yield();
}

bool acrt_actor_alive(actor_id id) {
    actor *a = acrt_actor_get(id);
    return a != NULL && a->state != ACTOR_STATE_DEAD;
}
