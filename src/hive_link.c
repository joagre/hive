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
void hive_link_cleanup_actor(actor_id id);
static bool send_exit_notification(actor *recipient, actor_id dying_id,
                                   hive_exit_reason reason,
                                   uint32_t monitor_id);

// External function to get actor table
extern actor_table *hive_actor_get_table(void);

// Static pools for links and monitors
static link_entry s_link_pool[HIVE_LINK_ENTRY_POOL_SIZE];
static bool s_link_used[HIVE_LINK_ENTRY_POOL_SIZE];
static hive_pool s_link_pool_mgr;

static monitor_entry s_monitor_pool[HIVE_MONITOR_ENTRY_POOL_SIZE];
static bool s_monitor_used[HIVE_MONITOR_ENTRY_POOL_SIZE];
static hive_pool s_monitor_pool_mgr;

// Global state
static struct {
    uint32_t next_monitor_id;
    bool initialized;
} s_link_state = {0};

// Initialize link subsystem
hive_status hive_link_init(void) {
    HIVE_INIT_GUARD(s_link_state.initialized);

    // Initialize link and monitor pools
    hive_pool_init(&s_link_pool_mgr, s_link_pool, s_link_used,
                   sizeof(link_entry), HIVE_LINK_ENTRY_POOL_SIZE);

    hive_pool_init(&s_monitor_pool_mgr, s_monitor_pool, s_monitor_used,
                   sizeof(monitor_entry), HIVE_MONITOR_ENTRY_POOL_SIZE);

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
hive_status hive_link(actor_id target_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Check for self-linking
    if (current->id == target_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Cannot link to self");
    }

    // Get target actor
    actor *target = hive_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Check if already linked
    if (is_already_linked(current, target_id)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Already linked to target");
    }

    // Allocate link entry for current -> target
    link_entry *current_link = hive_pool_alloc(&s_link_pool_mgr);
    if (!current_link) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Link pool exhausted");
    }
    current_link->target = target_id;
    current_link->next = NULL;

    // Allocate link entry for target -> current
    link_entry *target_link = hive_pool_alloc(&s_link_pool_mgr);
    if (!target_link) {
        hive_pool_free(&s_link_pool_mgr, current_link);
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Link pool exhausted");
    }
    target_link->target = current->id;
    target_link->next = NULL;

    // Add to current actor's link list
    SLIST_APPEND(current->links, current_link);

    // Add to target actor's link list
    SLIST_APPEND(target->links, target_link);

    HIVE_LOG_DEBUG("Actor %u linked to actor %u", current->id, target_id);
    return HIVE_SUCCESS;
}

// Remove bidirectional link
hive_status hive_link_remove(actor_id target_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Remove from current actor's link list
    link_entry *found = NULL;
    SLIST_FIND_REMOVE(current->links, entry->target == target_id, found);
    if (!found) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not linked to target");
    }
    hive_pool_free(&s_link_pool_mgr, found);

    // Remove reciprocal link from target actor's list
    actor *target = hive_actor_get(target_id);
    if (target && target->state != ACTOR_STATE_DEAD) {
        link_entry *reciprocal = NULL;
        SLIST_FIND_REMOVE(target->links, entry->target == current->id,
                          reciprocal);
        if (reciprocal) {
            hive_pool_free(&s_link_pool_mgr, reciprocal);
        }
    }

    HIVE_LOG_DEBUG("Actor %u removed link to actor %u", current->id, target_id);
    return HIVE_SUCCESS;
}

// Create unidirectional monitor
hive_status hive_monitor(actor_id target_id, uint32_t *monitor_id) {
    if (!monitor_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid monitor_id pointer");
    }

    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Check for self-monitoring
    if (current->id == target_id) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Cannot monitor self");
    }

    // Get target actor
    actor *target = hive_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Allocate monitor entry from pool
    monitor_entry *entry = hive_pool_alloc(&s_monitor_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Monitor pool exhausted");
    }

    // Generate unique monitor ID
    entry->ref = s_link_state.next_monitor_id++;
    entry->target = target_id;
    entry->next = NULL;

    // Add to current actor's monitor list
    SLIST_APPEND(current->monitors, entry);

    *monitor_id = entry->ref;
    HIVE_LOG_DEBUG("Actor %u monitoring actor %u (ref=%u)", current->id,
                   target_id, entry->ref);
    return HIVE_SUCCESS;
}

// Cancel unidirectional monitor
hive_status hive_monitor_cancel(uint32_t monitor_id) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    // Find and remove monitor entry
    monitor_entry *found = NULL;
    SLIST_FIND_REMOVE(current->monitors, entry->ref == monitor_id, found);
    if (found) {
        HIVE_LOG_DEBUG("Actor %u cancelled monitor (id=%u)", current->id,
                       monitor_id);
        hive_pool_free(&s_monitor_pool_mgr, found);
        return HIVE_SUCCESS;
    }

    return HIVE_ERROR(HIVE_ERR_INVALID, "Monitor ID not found");
}

// Check if message is an exit notification
bool hive_is_exit_msg(const hive_message *msg) {
    if (!msg) {
        return false;
    }
    return msg->class == HIVE_MSG_EXIT;
}

// Decode exit message
hive_status hive_decode_exit(const hive_message *msg, hive_exit_msg *out) {
    if (!msg || !out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL msg or out pointer");
    }

    if (!hive_is_exit_msg(msg)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Not an exit message");
    }

    // msg->data already points to payload, msg->len is payload length
    if (msg->len != sizeof(hive_exit_msg)) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid exit message size");
    }

    memcpy(out, msg->data, sizeof(hive_exit_msg));
    return HIVE_SUCCESS;
}

// Convert exit reason to string
const char *hive_exit_reason_str(hive_exit_reason reason) {
    switch (reason) {
    case HIVE_EXIT_NORMAL:
        return "NORMAL";
    case HIVE_EXIT_CRASH:
        return "CRASH";
    case HIVE_EXIT_CRASH_STACK:
        return "STACK_OVERFLOW";
    case HIVE_EXIT_KILLED:
        return "KILLED";
    default:
        return "UNKNOWN";
    }
}

// Helper: Send exit notification to an actor
// monitor_id: 0 for links, non-zero for monitors
static bool send_exit_notification(actor *recipient, actor_id dying_id,
                                   hive_exit_reason reason,
                                   uint32_t monitor_id) {
    // Build exit message payload
    hive_exit_msg exit_data = {
        .actor = dying_id, .reason = reason, .monitor_id = monitor_id};

    // Send using hive_ipc_notify_internal with HIVE_MSG_EXIT class
    // Sender is the dying actor so recipient knows who died
    hive_status status =
        hive_ipc_notify_internal(recipient->id, dying_id, HIVE_MSG_EXIT,
                                 HIVE_TAG_NONE, &exit_data, sizeof(exit_data));
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to send exit notification: %s", status.msg);
        return false;
    }

    return true;
}

// Cleanup actor links/monitors and send death notifications
void hive_link_cleanup_actor(actor_id dying_actor_id) {
    if (!s_link_state.initialized) {
        return;
    }

    // Get actor table and find the dying actor WITHOUT state check
    // (hive_actor_get filters out DEAD actors, but we need to access it here)
    actor_table *table = hive_actor_get_table();
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

    HIVE_LOG_DEBUG("Cleaning up links/monitors for actor %u (reason=%d)",
                   dying_actor_id, dying->exit_reason);

    // Collect all actors that need notification
    // We'll process in two passes to avoid iterator invalidation

    // Pass 1: Send notifications for bidirectional links
    link_entry *link = dying->links;
    while (link) {
        actor *linked_actor = hive_actor_get(link->target);
        if (linked_actor && linked_actor->state != ACTOR_STATE_DEAD) {
            // Send exit notification to linked actor (monitor_id=0 for links)
            if (send_exit_notification(linked_actor, dying_actor_id,
                                       dying->exit_reason, 0)) {
                HIVE_LOG_TRACE("Sent link exit notification to actor %u",
                               link->target);
            }

            // Remove reciprocal link from linked actor's list
            link_entry *reciprocal = NULL;
            SLIST_FIND_REMOVE(linked_actor->links,
                              entry->target == dying_actor_id, reciprocal);
            if (reciprocal) {
                hive_pool_free(&s_link_pool_mgr, reciprocal);
            }
        }

        link_entry *next_link = link->next;
        hive_pool_free(&s_link_pool_mgr, link);
        link = next_link;
    }
    dying->links = NULL;

    // Pass 2: Send notifications for monitors (actors monitoring the dying
    // actor) We need to find all actors that are monitoring this one This
    // requires scanning all actors' monitor lists (We already have table from
    // above)
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
                    // Send exit notification with monitor ID
                    if (send_exit_notification(a, dying_actor_id,
                                               dying->exit_reason, mon->ref)) {
                        HIVE_LOG_TRACE("Sent monitor exit notification to "
                                       "actor %u (id=%u)",
                                       a->id, mon->ref);
                    }

                    // Remove this monitor entry
                    monitor_entry *to_free = mon;
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

    // Clean up any remaining monitors owned by dying actor
    monitor_entry *mon = dying->monitors;
    while (mon) {
        monitor_entry *next_mon = mon->next;
        hive_pool_free(&s_monitor_pool_mgr, mon);
        mon = next_mon;
    }
    dying->monitors = NULL;
}
