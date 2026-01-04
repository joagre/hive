#include "rt_actor.h"
#include "rt_static_config.h"
#include "rt_ipc.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Static actor storage
static actor g_actors[RT_MAX_ACTORS];

// Global actor table
static actor_table g_actor_table = {0};

// Current running actor
static actor *g_current_actor = NULL;

rt_status rt_actor_init(size_t max_actors) {
    if (max_actors == 0) {
        return RT_ERROR(RT_ERR_INVALID, "max_actors must be > 0");
    }

    if (max_actors > RT_MAX_ACTORS) {
        return RT_ERROR(RT_ERR_INVALID, "max_actors exceeds RT_MAX_ACTORS");
    }

    // Use static actor array (already zero-initialized)
    g_actor_table.actors = g_actors;
    g_actor_table.max_actors = max_actors;
    g_actor_table.num_actors = 0;
    g_actor_table.next_id = 1; // Start at 1, 0 is ACTOR_ID_INVALID

    return RT_SUCCESS;
}

void rt_actor_cleanup(void) {
    if (g_actor_table.actors) {
        // Free all actor stacks and mailboxes
        for (size_t i = 0; i < g_actor_table.max_actors; i++) {
            actor *a = &g_actor_table.actors[i];
            if (a->state != ACTOR_STATE_DEAD && a->stack) {
                free(a->stack);
                rt_ipc_mailbox_clear(&a->mbox);
            }
        }
        // Note: g_actor_table.actors points to static g_actors array, so no free() needed
        g_actor_table.actors = NULL;
    }
}

actor *rt_actor_get(actor_id id) {
    if (id == ACTOR_ID_INVALID) {
        return NULL;
    }

    for (size_t i = 0; i < g_actor_table.max_actors; i++) {
        actor *a = &g_actor_table.actors[i];
        if (a->id == id && a->state != ACTOR_STATE_DEAD) {
            return a;
        }
    }

    return NULL;
}

actor *rt_actor_alloc(actor_fn fn, void *arg, const actor_config *cfg) {
    if (g_actor_table.num_actors >= g_actor_table.max_actors) {
        return NULL;
    }

    // Find free slot
    actor *a = NULL;
    for (size_t i = 0; i < g_actor_table.max_actors; i++) {
        if (g_actor_table.actors[i].state == ACTOR_STATE_DEAD ||
            g_actor_table.actors[i].id == ACTOR_ID_INVALID) {
            a = &g_actor_table.actors[i];
            break;
        }
    }

    if (!a) {
        return NULL;
    }

    // Determine stack size
    size_t stack_size = cfg->stack_size > 0 ? cfg->stack_size : (64 * 1024); // Default 64KB

    // Allocate stack
    void *stack = malloc(stack_size);
    if (!stack) {
        return NULL;
    }

    // Initialize actor
    memset(a, 0, sizeof(actor));
    a->id = g_actor_table.next_id++;
    a->state = ACTOR_STATE_READY;
    a->priority = cfg->priority;
    a->name = cfg->name;
    a->stack = stack;
    a->stack_size = stack_size;

    // Initialize context
    rt_context_init(&a->ctx, stack, stack_size, fn, arg);

    g_actor_table.num_actors++;

    return a;
}

// External cleanup functions
extern void rt_bus_cleanup_actor(actor_id id);
extern void rt_link_cleanup_actor(actor_id id);

void rt_actor_free(actor *a) {
    if (!a) {
        return;
    }

    // Cleanup links/monitors and send death notifications
    rt_link_cleanup_actor(a->id);

    // Cleanup bus subscriptions
    rt_bus_cleanup_actor(a->id);

    // Free stack
    if (a->stack) {
        free(a->stack);
        a->stack = NULL;
    }

    // Free active message
    if (a->active_msg) {
        rt_ipc_free_active_msg(a->active_msg);
        a->active_msg = NULL;
    }

    // Free mailbox entries
    rt_ipc_mailbox_clear(&a->mbox);

    a->state = ACTOR_STATE_DEAD;
    g_actor_table.num_actors--;
}

actor *rt_actor_current(void) {
    return g_current_actor;
}

void rt_actor_set_current(actor *a) {
    g_current_actor = a;
}

// Get actor table (for scheduler)
actor_table *rt_actor_get_table(void) {
    return &g_actor_table;
}
