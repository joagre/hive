#include "hive_runtime.h"
#include "hive_internal.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_static_config.h"
#include <stdlib.h>

hive_status hive_init(void) {
    // Initialize actor subsystem
    hive_status status = hive_actor_init();
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

#if HIVE_ENABLE_NET
    // Initialize network I/O subsystem
    status = hive_net_init();
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
#if HIVE_ENABLE_NET
        hive_net_cleanup();
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
#if HIVE_ENABLE_NET
        hive_net_cleanup();
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

hive_status hive_run_until_blocked(void) {
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
#if HIVE_ENABLE_NET
    hive_net_cleanup();
#endif
#if HIVE_ENABLE_FILE
    hive_file_cleanup();
#endif
    hive_link_cleanup();
    hive_scheduler_cleanup();
    hive_actor_cleanup();
}

hive_status hive_spawn(actor_fn fn, void *arg, actor_id *out) {
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = HIVE_DEFAULT_STACK_SIZE;
    return hive_spawn_ex(fn, arg, &cfg, out);
}

hive_status hive_spawn_ex(actor_fn fn, void *arg, const actor_config *cfg,
                          actor_id *out) {
    if (!fn) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL function pointer");
    }
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL output pointer");
    }

    // Copy config field by field instead of struct copy to avoid alignment
    // issues (struct copy may use SIMD instructions requiring 16-byte
    // alignment)
    actor_config actual_cfg;
    actual_cfg.stack_size = cfg->stack_size;
    actual_cfg.priority = cfg->priority;
    actual_cfg.name = cfg->name;
    actual_cfg.malloc_stack = cfg->malloc_stack;
    if (actual_cfg.stack_size == 0) {
        actual_cfg.stack_size = HIVE_DEFAULT_STACK_SIZE;
    }

    actor *a = hive_actor_alloc(fn, arg, &actual_cfg);
    if (!a) {
        return HIVE_ERROR(HIVE_ERR_NOMEM,
                          "Actor table or stack arena exhausted");
    }

    *out = a->id;
    return HIVE_SUCCESS;
}

_Noreturn void hive_exit(void) {
    actor *current = hive_actor_current();
    if (current) {
        HIVE_LOG_DEBUG("Actor %u (%s) exiting", current->id,
                       current->name ? current->name : "unnamed");

        // Mark exit reason and actor state
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
    actor *current = hive_actor_current();
    if (current) {
        HIVE_LOG_ERROR("Actor %u (%s) returned without calling hive_exit()",
                       current->id, current->name ? current->name : "unnamed");

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

actor_id hive_self(void) {
    actor *current = hive_actor_current();
    return current ? current->id : ACTOR_ID_INVALID;
}

void hive_yield(void) {
    hive_scheduler_yield();
}

bool hive_actor_alive(actor_id id) {
    actor *a = hive_actor_get(id);
    return a != NULL && a->state != ACTOR_STATE_DEAD;
}
