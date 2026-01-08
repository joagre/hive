#include "rt_link.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include "rt_pool.h"
#include "rt_ipc.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_log.h"
#include <stdlib.h>
#include <string.h>

// Helper macro: Append entry to linked list
#define APPEND_TO_LIST(head, new_entry) \
    do { \
        if (head) { \
            __typeof__(head) _last = head; \
            while (_last->next) _last = _last->next; \
            _last->next = new_entry; \
        } else { \
            head = new_entry; \
        } \
    } while(0)


// Forward declarations
rt_status rt_link_init(void);
void rt_link_cleanup(void);
void rt_link_cleanup_actor(actor_id id);
static bool send_exit_notification(actor *recipient, actor_id dying_id, rt_exit_reason reason);

// External function to get actor table
extern actor_table *rt_actor_get_table(void);

// Static pools for links and monitors
static link_entry g_link_pool[RT_LINK_ENTRY_POOL_SIZE];
static bool g_link_used[RT_LINK_ENTRY_POOL_SIZE];
static rt_pool g_link_pool_mgr;

static monitor_entry g_monitor_pool[RT_MONITOR_ENTRY_POOL_SIZE];
static bool g_monitor_used[RT_MONITOR_ENTRY_POOL_SIZE];
static rt_pool g_monitor_pool_mgr;

// Global state
static struct {
    uint32_t next_monitor_ref;
    bool     initialized;
} g_link_state = {0};

// Initialize link subsystem
rt_status rt_link_init(void) {
    RT_INIT_GUARD(g_link_state.initialized);

    // Initialize link and monitor pools
    rt_pool_init(&g_link_pool_mgr, g_link_pool, g_link_used,
                 sizeof(link_entry), RT_LINK_ENTRY_POOL_SIZE);

    rt_pool_init(&g_monitor_pool_mgr, g_monitor_pool, g_monitor_used,
                 sizeof(monitor_entry), RT_MONITOR_ENTRY_POOL_SIZE);

    g_link_state.next_monitor_ref = 1;
    g_link_state.initialized = true;

    RT_LOG_DEBUG("Link subsystem initialized");
    return RT_SUCCESS;
}

// Cleanup link subsystem
void rt_link_cleanup(void) {
    RT_CLEANUP_GUARD(g_link_state.initialized);

    g_link_state.initialized = false;
    RT_LOG_DEBUG("Link subsystem cleaned up");
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
rt_status rt_link(actor_id target_id) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Check for self-linking
    if (current->id == target_id) {
        return RT_ERROR(RT_ERR_INVALID, "Cannot link to self");
    }

    // Get target actor
    actor *target = rt_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return RT_ERROR(RT_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Check if already linked
    if (is_already_linked(current, target_id)) {
        return RT_ERROR(RT_ERR_INVALID, "Already linked to target");
    }

    // Allocate link entry for current -> target
    link_entry *current_link = rt_pool_alloc(&g_link_pool_mgr);
    if (!current_link) {
        return RT_ERROR(RT_ERR_NOMEM, "Link pool exhausted");
    }
    current_link->target = target_id;
    current_link->next = NULL;

    // Allocate link entry for target -> current
    link_entry *target_link = rt_pool_alloc(&g_link_pool_mgr);
    if (!target_link) {
        rt_pool_free(&g_link_pool_mgr, current_link);
        return RT_ERROR(RT_ERR_NOMEM, "Link pool exhausted");
    }
    target_link->target = current->id;
    target_link->next = NULL;

    // Add to current actor's link list
    APPEND_TO_LIST(current->links, current_link);

    // Add to target actor's link list
    APPEND_TO_LIST(target->links, target_link);

    RT_LOG_DEBUG("Actor %u linked to actor %u", current->id, target_id);
    return RT_SUCCESS;
}

// Remove bidirectional link
rt_status rt_unlink(actor_id target_id) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Remove from current actor's link list
    link_entry **prev = &current->links;
    link_entry *entry = current->links;
    bool found_current = false;

    while (entry) {
        if (entry->target == target_id) {
            *prev = entry->next;
            rt_pool_free(&g_link_pool_mgr, entry);
            found_current = true;
            break;
        }
        prev = &entry->next;
        entry = entry->next;
    }

    if (!found_current) {
        return RT_ERROR(RT_ERR_INVALID, "Not linked to target");
    }

    // Remove from target actor's link list
    actor *target = rt_actor_get(target_id);
    if (target && target->state != ACTOR_STATE_DEAD) {
        prev = &target->links;
        entry = target->links;

        while (entry) {
            if (entry->target == current->id) {
                *prev = entry->next;
                rt_pool_free(&g_link_pool_mgr, entry);
                break;
            }
            prev = &entry->next;
            entry = entry->next;
        }
    }

    RT_LOG_DEBUG("Actor %u unlinked from actor %u", current->id, target_id);
    return RT_SUCCESS;
}

// Create unidirectional monitor
rt_status rt_monitor(actor_id target_id, uint32_t *monitor_ref) {
    if (!monitor_ref) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid monitor_ref pointer");
    }

    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Check for self-monitoring
    if (current->id == target_id) {
        return RT_ERROR(RT_ERR_INVALID, "Cannot monitor self");
    }

    // Get target actor
    actor *target = rt_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return RT_ERROR(RT_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Allocate monitor entry from pool
    monitor_entry *entry = rt_pool_alloc(&g_monitor_pool_mgr);
    if (!entry) {
        return RT_ERROR(RT_ERR_NOMEM, "Monitor pool exhausted");
    }

    // Generate unique monitor reference
    entry->ref = g_link_state.next_monitor_ref++;
    entry->target = target_id;
    entry->next = NULL;

    // Add to current actor's monitor list
    APPEND_TO_LIST(current->monitors, entry);

    *monitor_ref = entry->ref;
    RT_LOG_DEBUG("Actor %u monitoring actor %u (ref=%u)", current->id, target_id, entry->ref);
    return RT_SUCCESS;
}

// Remove unidirectional monitor
rt_status rt_demonitor(uint32_t monitor_ref) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    // Find and remove monitor entry
    monitor_entry **prev = &current->monitors;
    monitor_entry *entry = current->monitors;

    while (entry) {
        if (entry->ref == monitor_ref) {
            *prev = entry->next;
            RT_LOG_DEBUG("Actor %u stopped monitoring (ref=%u)", current->id, monitor_ref);
            rt_pool_free(&g_monitor_pool_mgr, entry);
            return RT_SUCCESS;
        }
        prev = &entry->next;
        entry = entry->next;
    }

    return RT_ERROR(RT_ERR_INVALID, "Monitor reference not found");
}

// Check if message is an exit notification
bool rt_is_exit_msg(const rt_message *msg) {
    if (!msg || !msg->data || msg->len < RT_MSG_HEADER_SIZE) {
        return false;
    }

    // Check message class
    rt_msg_class class;
    rt_msg_decode(msg, &class, NULL, NULL, NULL);

    return class == RT_MSG_SYSTEM;
}

// Decode exit message
rt_status rt_decode_exit(const rt_message *msg, rt_exit_msg *out) {
    if (!msg || !out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!rt_is_exit_msg(msg)) {
        return RT_ERROR(RT_ERR_INVALID, "Not an exit message");
    }

    // Extract payload (after header)
    const void *payload;
    size_t payload_len;
    rt_status status = rt_msg_decode(msg, NULL, NULL, &payload, &payload_len);
    if (RT_FAILED(status)) {
        return status;
    }

    if (payload_len != sizeof(rt_exit_msg)) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid exit message size");
    }

    memcpy(out, payload, sizeof(rt_exit_msg));
    return RT_SUCCESS;
}

// Helper: Send exit notification to an actor
static bool send_exit_notification(actor *recipient, actor_id dying_id, rt_exit_reason reason) {
    // Build exit message payload
    rt_exit_msg exit_data = {
        .actor = dying_id,
        .reason = reason
    };

    // Send using rt_ipc_send_ex with RT_MSG_SYSTEM class
    // Sender is the dying actor so recipient knows who died
    rt_status status = rt_ipc_send_ex(recipient->id, dying_id, RT_MSG_SYSTEM,
                                       RT_TAG_NONE, &exit_data, sizeof(exit_data));
    if (RT_FAILED(status)) {
        RT_LOG_ERROR("Failed to send exit notification: %s", status.msg);
        return false;
    }

    return true;
}

// Cleanup actor links/monitors and send death notifications
void rt_link_cleanup_actor(actor_id dying_actor_id) {
    if (!g_link_state.initialized) {
        return;
    }

    // Get actor table and find the dying actor WITHOUT state check
    // (rt_actor_get filters out DEAD actors, but we need to access it here)
    actor_table *table = rt_actor_get_table();
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

    RT_LOG_DEBUG("Cleaning up links/monitors for actor %u (reason=%d)",
                 dying_actor_id, dying->exit_reason);

    // Collect all actors that need notification
    // We'll process in two passes to avoid iterator invalidation

    // Pass 1: Send notifications for bidirectional links
    link_entry *link = dying->links;
    while (link) {
        actor *linked_actor = rt_actor_get(link->target);
        if (linked_actor && linked_actor->state != ACTOR_STATE_DEAD) {
            // Send exit notification to linked actor
            if (send_exit_notification(linked_actor, dying_actor_id, dying->exit_reason)) {
                RT_LOG_TRACE("Sent link exit notification to actor %u", link->target);
            }

            // Remove reciprocal link from linked actor's list
            link_entry **prev = &linked_actor->links;
            link_entry *reciprocal = linked_actor->links;
            while (reciprocal) {
                if (reciprocal->target == dying_actor_id) {
                    *prev = reciprocal->next;
                    rt_pool_free(&g_link_pool_mgr, reciprocal);
                    break;
                }
                prev = &reciprocal->next;
                reciprocal = reciprocal->next;
            }
        }

        link_entry *next_link = link->next;
        rt_pool_free(&g_link_pool_mgr, link);
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
                        RT_LOG_TRACE("Sent monitor exit notification to actor %u (ref=%u)",
                                     a->id, mon->ref);
                    }

                    // Remove this monitor entry
                    monitor_entry *to_free = mon;
                    *prev = mon->next;
                    mon = mon->next;
                    rt_pool_free(&g_monitor_pool_mgr, to_free);
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
        rt_pool_free(&g_monitor_pool_mgr, mon);
        mon = next_mon;
    }
    dying->monitors = NULL;
}
