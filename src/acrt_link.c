#include "acrt_link.h"
#include "acrt_internal.h"
#include "acrt_static_config.h"
#include "acrt_pool.h"
#include "acrt_ipc.h"
#include "acrt_actor.h"
#include "acrt_scheduler.h"
#include "acrt_runtime.h"
#include "acrt_log.h"
#include <stdlib.h>
#include <string.h>

// Use shared SLIST_APPEND from acrt_internal.h


// Forward declarations
acrt_status acrt_link_init(void);
void acrt_link_cleanup(void);
void acrt_link_cleanup_actor(actor_id id);
static bool send_exit_notification(actor *recipient, actor_id dying_id, acrt_exit_reason reason);

// External function to get actor table
extern actor_table *acrt_actor_get_table(void);

// Static pools for links and monitors
static link_entry g_link_pool[ACRT_LINK_ENTRY_POOL_SIZE];
static bool g_link_used[ACRT_LINK_ENTRY_POOL_SIZE];
static acrt_pool g_link_pool_mgr;

static monitor_entry g_monitor_pool[ACRT_MONITOR_ENTRY_POOL_SIZE];
static bool g_monitor_used[ACRT_MONITOR_ENTRY_POOL_SIZE];
static acrt_pool g_monitor_pool_mgr;

// Global state
static struct {
    uint32_t next_monitor_id;
    bool     initialized;
} g_link_state = {0};

// Initialize link subsystem
acrt_status acrt_link_init(void) {
    ACRT_INIT_GUARD(g_link_state.initialized);

    // Initialize link and monitor pools
    acrt_pool_init(&g_link_pool_mgr, g_link_pool, g_link_used,
                 sizeof(link_entry), ACRT_LINK_ENTRY_POOL_SIZE);

    acrt_pool_init(&g_monitor_pool_mgr, g_monitor_pool, g_monitor_used,
                 sizeof(monitor_entry), ACRT_MONITOR_ENTRY_POOL_SIZE);

    g_link_state.next_monitor_id = 1;
    g_link_state.initialized = true;

    ACRT_LOG_DEBUG("Link subsystem initialized");
    return ACRT_SUCCESS;
}

// Cleanup link subsystem
void acrt_link_cleanup(void) {
    ACRT_CLEANUP_GUARD(g_link_state.initialized);

    g_link_state.initialized = false;
    ACRT_LOG_DEBUG("Link subsystem cleaned up");
}

// Helper: Check if actor already linked
static bool is_already_linked(actor *a, actor_id target_id) {
    for (link_entry *e = a->links; e != NULL; e = e->next) {
        if (e->target == target_id) {
            return true;
        }
    }
    return false;
}

// Create bidirectional link
acrt_status acrt_link(actor_id target_id) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Check for self-linking
    if (current->id == target_id) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Cannot link to self");
    }

    // Get target actor
    actor *target = acrt_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Check if already linked
    if (is_already_linked(current, target_id)) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Already linked to target");
    }

    // Allocate link entry for current -> target
    link_entry *current_link = acrt_pool_alloc(&g_link_pool_mgr);
    if (!current_link) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Link pool exhausted");
    }
    current_link->target = target_id;
    current_link->next = NULL;

    // Allocate link entry for target -> current
    link_entry *target_link = acrt_pool_alloc(&g_link_pool_mgr);
    if (!target_link) {
        acrt_pool_free(&g_link_pool_mgr, current_link);
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Link pool exhausted");
    }
    target_link->target = current->id;
    target_link->next = NULL;

    // Add to current actor's link list
    SLIST_APPEND(current->links, current_link);

    // Add to target actor's link list
    SLIST_APPEND(target->links, target_link);

    ACRT_LOG_DEBUG("Actor %u linked to actor %u", current->id, target_id);
    return ACRT_SUCCESS;
}

// Remove bidirectional link
acrt_status acrt_link_remove(actor_id target_id) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Remove from current actor's link list
    link_entry **prev = &current->links;
    link_entry *entry = current->links;
    bool found_current = false;

    while (entry) {
        if (entry->target == target_id) {
            *prev = entry->next;
            acrt_pool_free(&g_link_pool_mgr, entry);
            found_current = true;
            break;
        }
        prev = &entry->next;
        entry = entry->next;
    }

    if (!found_current) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Not linked to target");
    }

    // Remove from target actor's link list
    actor *target = acrt_actor_get(target_id);
    if (target && target->state != ACTOR_STATE_DEAD) {
        prev = &target->links;
        entry = target->links;

        while (entry) {
            if (entry->target == current->id) {
                *prev = entry->next;
                acrt_pool_free(&g_link_pool_mgr, entry);
                break;
            }
            prev = &entry->next;
            entry = entry->next;
        }
    }

    ACRT_LOG_DEBUG("Actor %u removed link to actor %u", current->id, target_id);
    return ACRT_SUCCESS;
}

// Create unidirectional monitor
acrt_status acrt_monitor(actor_id target_id, uint32_t *monitor_id) {
    if (!monitor_id) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid monitor_id pointer");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Check for self-monitoring
    if (current->id == target_id) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Cannot monitor self");
    }

    // Get target actor
    actor *target = acrt_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Allocate monitor entry from pool
    monitor_entry *entry = acrt_pool_alloc(&g_monitor_pool_mgr);
    if (!entry) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Monitor pool exhausted");
    }

    // Generate unique monitor ID
    entry->ref = g_link_state.next_monitor_id++;
    entry->target = target_id;
    entry->next = NULL;

    // Add to current actor's monitor list
    SLIST_APPEND(current->monitors, entry);

    *monitor_id = entry->ref;
    ACRT_LOG_DEBUG("Actor %u monitoring actor %u (ref=%u)", current->id, target_id, entry->ref);
    return ACRT_SUCCESS;
}

// Cancel unidirectional monitor
acrt_status acrt_monitor_cancel(uint32_t monitor_id) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Find and remove monitor entry
    monitor_entry **prev = &current->monitors;
    monitor_entry *entry = current->monitors;

    while (entry) {
        if (entry->ref == monitor_id) {
            *prev = entry->next;
            ACRT_LOG_DEBUG("Actor %u cancelled monitor (id=%u)", current->id, monitor_id);
            acrt_pool_free(&g_monitor_pool_mgr, entry);
            return ACRT_SUCCESS;
        }
        prev = &entry->next;
        entry = entry->next;
    }

    return ACRT_ERROR(ACRT_ERR_INVALID, "Monitor reference not found");
}

// Check if message is an exit notification
bool acrt_is_exit_msg(const acrt_message *msg) {
    if (!msg || !msg->data || msg->len < ACRT_MSG_HEADER_SIZE) {
        return false;
    }

    // Check message class
    acrt_msg_class class;
    acrt_msg_decode(msg, &class, NULL, NULL, NULL);

    return class == ACRT_MSG_EXIT;
}

// Decode exit message
acrt_status acrt_decode_exit(const acrt_message *msg, acrt_exit_msg *out) {
    if (!msg || !out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!acrt_is_exit_msg(msg)) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Not an exit message");
    }

    // Extract payload (after header)
    const void *payload;
    size_t payload_len;
    acrt_status status = acrt_msg_decode(msg, NULL, NULL, &payload, &payload_len);
    if (ACRT_FAILED(status)) {
        return status;
    }

    if (payload_len != sizeof(acrt_exit_msg)) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid exit message size");
    }

    memcpy(out, payload, sizeof(acrt_exit_msg));
    return ACRT_SUCCESS;
}

// Helper: Send exit notification to an actor
static bool send_exit_notification(actor *recipient, actor_id dying_id, acrt_exit_reason reason) {
    // Build exit message payload
    acrt_exit_msg exit_data = {
        .actor = dying_id,
        .reason = reason
    };

    // Send using acrt_ipc_notify_ex with ACRT_MSG_EXIT class
    // Sender is the dying actor so recipient knows who died
    acrt_status status = acrt_ipc_notify_ex(recipient->id, dying_id, ACRT_MSG_EXIT,
                                       ACRT_TAG_NONE, &exit_data, sizeof(exit_data));
    if (ACRT_FAILED(status)) {
        ACRT_LOG_ERROR("Failed to send exit notification: %s", status.msg);
        return false;
    }

    return true;
}

// Cleanup actor links/monitors and send death notifications
void acrt_link_cleanup_actor(actor_id dying_actor_id) {
    if (!g_link_state.initialized) {
        return;
    }

    // Get actor table and find the dying actor WITHOUT state check
    // (acrt_actor_get filters out DEAD actors, but we need to access it here)
    actor_table *table = acrt_actor_get_table();
    if (!table || !table->actors) {
        return;
    }

    actor *dying = NULL;
    for (size_t i = 0; i < table->max_actors; i++) {
        actor *a = &table->actors[i];
        if (a->id == dying_actor_id) {
            dying = a;
            break;
        }
    }

    if (!dying) {
        return;
    }

    ACRT_LOG_DEBUG("Cleaning up links/monitors for actor %u (reason=%d)",
                 dying_actor_id, dying->exit_reason);

    // Collect all actors that need notification
    // We'll process in two passes to avoid iterator invalidation

    // Pass 1: Send notifications for bidirectional links
    link_entry *link = dying->links;
    while (link) {
        actor *linked_actor = acrt_actor_get(link->target);
        if (linked_actor && linked_actor->state != ACTOR_STATE_DEAD) {
            // Send exit notification to linked actor
            if (send_exit_notification(linked_actor, dying_actor_id, dying->exit_reason)) {
                ACRT_LOG_TRACE("Sent link exit notification to actor %u", link->target);
            }

            // Remove reciprocal link from linked actor's list
            link_entry **prev = &linked_actor->links;
            link_entry *reciprocal = linked_actor->links;
            while (reciprocal) {
                if (reciprocal->target == dying_actor_id) {
                    *prev = reciprocal->next;
                    acrt_pool_free(&g_link_pool_mgr, reciprocal);
                    break;
                }
                prev = &reciprocal->next;
                reciprocal = reciprocal->next;
            }
        }

        link_entry *next_link = link->next;
        acrt_pool_free(&g_link_pool_mgr, link);
        link = next_link;
    }
    dying->links = NULL;

    // Pass 2: Send notifications for monitors (actors monitoring the dying actor)
    // We need to find all actors that are monitoring this one
    // This requires scanning all actors' monitor lists
    // (We already have table from above)
    {
        for (size_t i = 0; i < table->max_actors; i++) {
            actor *a = &table->actors[i];
            if (a->state == ACTOR_STATE_DEAD || a->id == ACTOR_ID_INVALID) {
                continue;
            }

            // Check if this actor is monitoring the dying actor
            monitor_entry **prev = &a->monitors;
            monitor_entry *mon = a->monitors;
            while (mon) {
                if (mon->target == dying_actor_id) {
                    // Send exit notification using helper
                    if (send_exit_notification(a, dying_actor_id, dying->exit_reason)) {
                        ACRT_LOG_TRACE("Sent monitor exit notification to actor %u (ref=%u)",
                                     a->id, mon->ref);
                    }

                    // Remove this monitor entry
                    monitor_entry *to_free = mon;
                    *prev = mon->next;
                    mon = mon->next;
                    acrt_pool_free(&g_monitor_pool_mgr, to_free);
                } else {
                    prev = &mon->next;
                    mon = mon->next;
                }
            }
        }
    }

    // Clean up any remaining monitors owned by dying actor
    monitor_entry *mon = dying->monitors;
    while (mon) {
        monitor_entry *next_mon = mon->next;
        acrt_pool_free(&g_monitor_pool_mgr, mon);
        mon = next_mon;
    }
    dying->monitors = NULL;
}
