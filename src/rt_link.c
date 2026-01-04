#include "rt_link.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_log.h"
#include <stdlib.h>
#include <string.h>

// Forward declarations
rt_status rt_link_init(void);
void rt_link_cleanup(void);
void rt_link_cleanup_actor(actor_id id);

// External function to get actor table
extern actor_table *rt_actor_get_table(void);

// Global state
static struct {
    uint32_t next_monitor_ref;
    bool     initialized;
} g_link_state = {0};

// Initialize link subsystem
rt_status rt_link_init(void) {
    if (g_link_state.initialized) {
        return RT_SUCCESS;
    }

    g_link_state.next_monitor_ref = 1;
    g_link_state.initialized = true;

    RT_LOG_DEBUG("Link subsystem initialized");
    return RT_SUCCESS;
}

// Cleanup link subsystem
void rt_link_cleanup(void) {
    if (!g_link_state.initialized) {
        return;
    }

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
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

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
    link_entry *current_link = malloc(sizeof(link_entry));
    if (!current_link) {
        return RT_ERROR(RT_ERR_NOMEM, "Failed to allocate link entry");
    }
    current_link->target = target_id;
    current_link->next = NULL;

    // Allocate link entry for target -> current
    link_entry *target_link = malloc(sizeof(link_entry));
    if (!target_link) {
        free(current_link);
        return RT_ERROR(RT_ERR_NOMEM, "Failed to allocate bidirectional link");
    }
    target_link->target = current->id;
    target_link->next = NULL;

    // Add to current actor's link list
    if (current->links) {
        link_entry *last = current->links;
        while (last->next) last = last->next;
        last->next = current_link;
    } else {
        current->links = current_link;
    }

    // Add to target actor's link list
    if (target->links) {
        link_entry *last = target->links;
        while (last->next) last = last->next;
        last->next = target_link;
    } else {
        target->links = target_link;
    }

    RT_LOG_DEBUG("Actor %u linked to actor %u", current->id, target_id);
    return RT_SUCCESS;
}

// Remove bidirectional link
rt_status rt_unlink(actor_id target_id) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    // Remove from current actor's link list
    link_entry **prev = &current->links;
    link_entry *entry = current->links;
    bool found_current = false;

    while (entry) {
        if (entry->target == target_id) {
            *prev = entry->next;
            free(entry);
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
                free(entry);
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

    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    // Check for self-monitoring
    if (current->id == target_id) {
        return RT_ERROR(RT_ERR_INVALID, "Cannot monitor self");
    }

    // Get target actor
    actor *target = rt_actor_get(target_id);
    if (!target || target->state == ACTOR_STATE_DEAD) {
        return RT_ERROR(RT_ERR_INVALID, "Target actor is dead or invalid");
    }

    // Allocate monitor entry
    monitor_entry *entry = malloc(sizeof(monitor_entry));
    if (!entry) {
        return RT_ERROR(RT_ERR_NOMEM, "Failed to allocate monitor entry");
    }

    // Generate unique monitor reference
    entry->ref = g_link_state.next_monitor_ref++;
    entry->target = target_id;
    entry->next = NULL;

    // Add to current actor's monitor list
    if (current->monitors) {
        monitor_entry *last = current->monitors;
        while (last->next) last = last->next;
        last->next = entry;
    } else {
        current->monitors = entry;
    }

    *monitor_ref = entry->ref;
    RT_LOG_DEBUG("Actor %u monitoring actor %u (ref=%u)", current->id, target_id, entry->ref);
    return RT_SUCCESS;
}

// Remove unidirectional monitor
rt_status rt_demonitor(uint32_t monitor_ref) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    // Find and remove monitor entry
    monitor_entry **prev = &current->monitors;
    monitor_entry *entry = current->monitors;

    while (entry) {
        if (entry->ref == monitor_ref) {
            *prev = entry->next;
            RT_LOG_DEBUG("Actor %u stopped monitoring (ref=%u)", current->id, monitor_ref);
            free(entry);
            return RT_SUCCESS;
        }
        prev = &entry->next;
        entry = entry->next;
    }

    return RT_ERROR(RT_ERR_INVALID, "Monitor reference not found");
}

// Check if message is an exit notification
bool rt_is_exit_msg(const rt_message *msg) {
    if (!msg) {
        return false;
    }

    return msg->sender == RT_SENDER_SYSTEM &&
           msg->len == sizeof(rt_exit_msg);
}

// Decode exit message
rt_status rt_decode_exit(const rt_message *msg, rt_exit_msg *out) {
    if (!msg || !out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!rt_is_exit_msg(msg)) {
        return RT_ERROR(RT_ERR_INVALID, "Not an exit message");
    }

    memcpy(out, msg->data, sizeof(rt_exit_msg));
    return RT_SUCCESS;
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
            // Pattern from rt_timer.c:348-380
            mailbox_entry *entry = malloc(sizeof(mailbox_entry));
            if (!entry) {
                RT_LOG_ERROR("Failed to send exit notification (OOM)");
                link = link->next;
                continue;
            }

            entry->sender = RT_SENDER_SYSTEM;
            entry->len = sizeof(rt_exit_msg);
            entry->data = malloc(sizeof(rt_exit_msg));
            if (!entry->data) {
                free(entry);
                RT_LOG_ERROR("Failed to allocate exit message data (OOM)");
                link = link->next;
                continue;
            }

            // Populate exit message
            rt_exit_msg *exit_data = (rt_exit_msg *)entry->data;
            exit_data->actor = dying_actor_id;
            exit_data->reason = dying->exit_reason;

            entry->borrow_ptr = NULL;
            entry->next = NULL;

            // Inject into linked actor's mailbox
            if (linked_actor->mbox.tail) {
                linked_actor->mbox.tail->next = entry;
            } else {
                linked_actor->mbox.head = entry;
            }
            linked_actor->mbox.tail = entry;
            linked_actor->mbox.count++;

            // Wake if blocked
            if (linked_actor->state == ACTOR_STATE_BLOCKED) {
                linked_actor->state = ACTOR_STATE_READY;
            }

            RT_LOG_TRACE("Sent link exit notification to actor %u", link->target);

            // Remove reciprocal link from linked actor's list
            link_entry **prev = &linked_actor->links;
            link_entry *reciprocal = linked_actor->links;
            while (reciprocal) {
                if (reciprocal->target == dying_actor_id) {
                    *prev = reciprocal->next;
                    free(reciprocal);
                    break;
                }
                prev = &reciprocal->next;
                reciprocal = reciprocal->next;
            }
        }

        link_entry *next_link = link->next;
        free(link);
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
                    // Send exit notification
                    mailbox_entry *entry = malloc(sizeof(mailbox_entry));
                    if (!entry) {
                        RT_LOG_ERROR("Failed to send monitor notification (OOM)");
                        prev = &mon->next;
                        mon = mon->next;
                        continue;
                    }

                    entry->sender = RT_SENDER_SYSTEM;
                    entry->len = sizeof(rt_exit_msg);
                    entry->data = malloc(sizeof(rt_exit_msg));
                    if (!entry->data) {
                        free(entry);
                        RT_LOG_ERROR("Failed to allocate monitor exit data (OOM)");
                        prev = &mon->next;
                        mon = mon->next;
                        continue;
                    }

                    // Populate exit message
                    rt_exit_msg *exit_data = (rt_exit_msg *)entry->data;
                    exit_data->actor = dying_actor_id;
                    exit_data->reason = dying->exit_reason;

                    entry->borrow_ptr = NULL;
                    entry->next = NULL;

                    // Inject into monitoring actor's mailbox
                    if (a->mbox.tail) {
                        a->mbox.tail->next = entry;
                    } else {
                        a->mbox.head = entry;
                    }
                    a->mbox.tail = entry;
                    a->mbox.count++;

                    // Wake if blocked
                    if (a->state == ACTOR_STATE_BLOCKED) {
                        a->state = ACTOR_STATE_READY;
                    }

                    RT_LOG_TRACE("Sent monitor exit notification to actor %u (ref=%u)",
                                 a->id, mon->ref);

                    // Remove this monitor entry
                    monitor_entry *to_free = mon;
                    *prev = mon->next;
                    mon = mon->next;
                    free(to_free);
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
        free(mon);
        mon = next_mon;
    }
    dying->monitors = NULL;
}
