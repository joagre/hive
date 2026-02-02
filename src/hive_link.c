#include "hive_link.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_ipc.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_runtime.h"
#include "hive_log.h"
#include <string.h>

// Use shared SLIST_APPEND from hive_internal.h

// Forward declarations for internal functions
void hive_link_cleanup_actor(actor_id_t id);
static bool deliver_exit_notification(actor_t *recipient, actor_id_t dying_id,
                                      hive_exit_reason_t reason,
                                      uint32_t monitor_id);

// External function to get actor_t table
extern actor_table_t *hive_actor_get_table(void);

// Static pools for links and monitors
static link_entry_t s_link_pool[HIVE_LINK_ENTRY_POOL_SIZE];
static bool s_link_used[HIVE_LINK_ENTRY_POOL_SIZE];
static hive_pool_t s_link_pool_mgr;

static monitor_entry_t s_monitor_pool[HIVE_MONITOR_ENTRY_POOL_SIZE];
static bool s_monitor_used[HIVE_MONITOR_ENTRY_POOL_SIZE];
static hive_pool_t s_monitor_pool_mgr;

// Global state
static struct {
    uint32_t next_monitor_id;
    bool initialized;
} s_link_state = {0};

// Initialize link subsystem
hive_status_t hive_link_init(void) {
    HIVE_INIT_GUARD(s_link_state.initialized);

    // Initialize link and monitor pools
    hive_pool_init(&s_link_pool_mgr, s_link_pool, s_link_used,
                   sizeof(link_entry_t), HIVE_LINK_ENTRY_POOL_SIZE);

    hive_pool_init(&s_monitor_pool_mgr, s_monitor_pool, s_monitor_used,
                   sizeof(monitor_entry_t), HIVE_MONITOR_ENTRY_POOL_SIZE);

    s_link_state.next_monitor_id = 1;
    s_link_state.initialized = true;

    HIVE_LOG_DEBUG("Link subsystem initialized");
    return HIVE_SUCCESS;
}

// Cleanup link subsystem
void hive_link_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_link_state.initialized);

    s_link_state.initialized = false;
    HIVE_LOG_DEBUG("Link subsystem cleaned up");
}

// Helper: Check if actor_t already linked
static bool is_already_linked(actor_t *a, actor_id_t target_id) {
    for (link_entry_t *e = a->links; e != NULL; e = e->next) {
        if (e->target == target_id) {
            return true;
        }
    }
    return false;
}

// Create bidirectional link
hive_status_t hive_link(actor_id_t target_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Check for self-linking
    if (current->id == target_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Cannot link to self");
    }

    // Get target actor_t
    actor_t *target = hive_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "Target actor_t is dead or invalid");
    }

    // Check if already linked
    if (is_already_linked(current, target_id)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Already linked to target");
    }

    // Allocate link entry for current -> target
    link_entry_t *current_link = hive_pool_alloc(&s_link_pool_mgr);
    if (!current_link) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Link pool exhausted");
    }
    current_link->target = target_id;
    current_link->next = NULL;

    // Allocate link entry for target -> current
    link_entry_t *target_link = hive_pool_alloc(&s_link_pool_mgr);
    if (!target_link) {
        hive_pool_free(&s_link_pool_mgr, current_link);
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Link pool exhausted");
    }
    target_link->target = current->id;
    target_link->next = NULL;

    // Add to current actor_t's link list
    SLIST_APPEND(current->links, current_link);

    // Add to target actor_t's link list
    SLIST_APPEND(target->links, target_link);

    HIVE_LOG_DEBUG("Actor %u linked to actor_t %u", current->id, target_id);
    return HIVE_SUCCESS;
}

// Remove bidirectional link
hive_status_t hive_unlink(actor_id_t target_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Remove from current actor_t's link list
    link_entry_t *found = NULL;
    SLIST_FIND_REMOVE(current->links, entry->target == target_id, found);
    if (!found) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not linked to target");
    }
    hive_pool_free(&s_link_pool_mgr, found);

    // Remove reciprocal link from target actor_t's list
    actor_t *target = hive_actor_get(target_id);
    if (target && target->state != ACTOR_STATE_DEAD) {
        link_entry_t *reciprocal = NULL;
        SLIST_FIND_REMOVE(target->links, entry->target == current->id,
                          reciprocal);
        if (reciprocal) {
            hive_pool_free(&s_link_pool_mgr, reciprocal);
        }
    }

    HIVE_LOG_DEBUG("Actor %u unlinked from actor_t %u", current->id, target_id);
    return HIVE_SUCCESS;
}

// Create unidirectional monitor
hive_status_t hive_monitor(actor_id_t target_id, uint32_t *monitor_id) {
    if (!monitor_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid monitor_id pointer");
    }

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Check for self-monitoring
    if (current->id == target_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Cannot monitor self");
    }

    // Get target actor_t
    actor_t *target = hive_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "Target actor_t is dead or invalid");
    }

    // Allocate monitor entry from pool
    monitor_entry_t *entry = hive_pool_alloc(&s_monitor_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Monitor pool exhausted");
    }

    // Generate unique monitor ID
    entry->ref = s_link_state.next_monitor_id++;
    entry->target = target_id;
    entry->next = NULL;

    // Add to current actor_t's monitor list
    SLIST_APPEND(current->monitors, entry);

    *monitor_id = entry->ref;
    HIVE_LOG_DEBUG("Actor %u monitoring actor_t %u (ref=%u)", current->id,
                   target_id, entry->ref);
    return HIVE_SUCCESS;
}

// Cancel unidirectional monitor
hive_status_t hive_demonitor(uint32_t monitor_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Find and remove monitor entry
    monitor_entry_t *found = NULL;
    SLIST_FIND_REMOVE(current->monitors, entry->ref == monitor_id, found);
    if (found) {
        HIVE_LOG_DEBUG("Actor %u demonitored (id=%u)", current->id, monitor_id);
        hive_pool_free(&s_monitor_pool_mgr, found);
        return HIVE_SUCCESS;
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Monitor ID not found");
}

// Check if message is an exit notification
bool hive_msg_is_exit(const hive_message_t *msg) {
    if (!msg) {
        return false;
    }
    return msg->class == HIVE_MSG_EXIT;
}

// Decode exit message
hive_status_t hive_decode_exit(const hive_message_t *msg,
                               hive_exit_msg_t *out) {
    if (!msg || !out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL msg or out pointer");
    }

    if (!hive_msg_is_exit(msg)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not an exit message");
    }

    // msg->data already points to payload, msg->len is payload length
    if (msg->len != sizeof(hive_exit_msg_t)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid exit message size");
    }

    memcpy(out, msg->data, sizeof(hive_exit_msg_t));
    return HIVE_SUCCESS;
}

// Helper: Deliver exit notification to an actor_t
// monitor_id: 0 for links, non-zero for monitors
static bool deliver_exit_notification(actor_t *recipient, actor_id_t dying_id,
                                      hive_exit_reason_t reason,
                                      uint32_t monitor_id) {
    // Build exit message payload
    hive_exit_msg_t exit_data = {
        .actor = dying_id, .reason = reason, .monitor_id = monitor_id};

    // Deliver using hive_ipc_notify_internal with HIVE_MSG_EXIT class
    // Sender is the dying actor_t so recipient knows who died
    hive_status_t status =
        hive_ipc_notify_internal(recipient->id, dying_id, HIVE_MSG_EXIT,
                                 HIVE_TAG_NONE, &exit_data, sizeof(exit_data));
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to deliver exit notification: %s", status.msg);
        return false;
    }

    return true;
}

// Cleanup actor_t links/monitors and deliver death notifications
void hive_link_cleanup_actor(actor_id_t dying_actor_id) {
    if (!s_link_state.initialized) {
        return;
    }

    // Get actor_t table and find the dying actor_t WITHOUT state check
    // (hive_actor_get filters out DEAD actors, but we need to access it here)
    actor_table_t *table = hive_actor_get_table();
    if (!table || !table->actors) {
        return;
    }

    actor_t *dying = NULL;
    for (size_t i = 0; i < table->max_actors; i++) {
        actor_t *a = &table->actors[i];
        if (a->id == dying_actor_id) {
            dying = a;
            break;
        }
    }

    if (!dying) {
        return;
    }

    HIVE_LOG_DEBUG("Cleaning up links/monitors for actor_t %u (reason=%d)",
                   dying_actor_id, dying->exit_reason);

    // Collect all actors that need notification
    // We'll process in two passes to avoid iterator invalidation

    // Pass 1: Deliver notifications for bidirectional links
    link_entry_t *link = dying->links;
    while (link) {
        actor_t *linked_actor = hive_actor_get(link->target);
        if (linked_actor && linked_actor->state != ACTOR_STATE_DEAD) {
            // Deliver exit notification to linked actor_t (monitor_id=0 for links)
            if (deliver_exit_notification(linked_actor, dying_actor_id,
                                          dying->exit_reason, 0)) {
                HIVE_LOG_TRACE("Delivered link exit notification to actor_t %u",
                               link->target);
            }

            // Remove reciprocal link from linked actor_t's list
            link_entry_t *reciprocal = NULL;
            SLIST_FIND_REMOVE(linked_actor->links,
                              entry->target == dying_actor_id, reciprocal);
            if (reciprocal) {
                hive_pool_free(&s_link_pool_mgr, reciprocal);
            }
        }

        link_entry_t *next_link = link->next;
        hive_pool_free(&s_link_pool_mgr, link);
        link = next_link;
    }
    dying->links = NULL;

    // Pass 2: Deliver notifications for monitors (actors monitoring the dying
    // actor_t) We need to find all actors that are monitoring this one This
    // requires scanning all actors' monitor lists (We already have table from
    // above)
    {
        for (size_t i = 0; i < table->max_actors; i++) {
            actor_t *a = &table->actors[i];
            if (a->state == ACTOR_STATE_DEAD ||
                a->id == HIVE_ACTOR_ID_INVALID) {
                continue;
            }

            // Check if this actor_t is monitoring the dying actor_t
            monitor_entry_t **prev = &a->monitors;
            monitor_entry_t *mon = a->monitors;
            while (mon) {
                if (mon->target == dying_actor_id) {
                    // Deliver exit notification with monitor ID
                    if (deliver_exit_notification(
                            a, dying_actor_id, dying->exit_reason, mon->ref)) {
                        HIVE_LOG_TRACE("Delivered monitor exit notification to "
                                       "actor_t %u (id=%u)",
                                       a->id, mon->ref);
                    }

                    // Remove this monitor entry
                    monitor_entry_t *to_free = mon;
                    *prev = mon->next;
                    mon = mon->next;
                    hive_pool_free(&s_monitor_pool_mgr, to_free);
                } else {
                    prev = &mon->next;
                    mon = mon->next;
                }
            }
        }
    }

    // Clean up any remaining monitors owned by dying actor_t
    monitor_entry_t *mon = dying->monitors;
    while (mon) {
        monitor_entry_t *next_mon = mon->next;
        hive_pool_free(&s_monitor_pool_mgr, mon);
        mon = next_mon;
    }
    dying->monitors = NULL;
}
