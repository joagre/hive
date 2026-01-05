#include "rt_ipc.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include "rt_pool.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_timer.h"
#include "rt_log.h"
#include <stdlib.h>
#include <string.h>

// Static pools for IPC (mailbox entries and message data)
static mailbox_entry g_mailbox_pool[RT_MAILBOX_ENTRY_POOL_SIZE];
static bool g_mailbox_used[RT_MAILBOX_ENTRY_POOL_SIZE];
rt_pool g_mailbox_pool_mgr;  // Non-static so rt_link.c can access

// Message data pool - fixed size entries (type defined in rt_internal.h)

static message_data_entry g_message_pool[RT_MESSAGE_DATA_POOL_SIZE];
static bool g_message_used[RT_MESSAGE_DATA_POOL_SIZE];
rt_pool g_message_pool_mgr;  // Non-static so rt_link.c can access

// Forward declaration for init function
rt_status rt_ipc_init(void);

// Initialize IPC pools
rt_status rt_ipc_init(void) {
    rt_pool_init(&g_mailbox_pool_mgr, g_mailbox_pool, g_mailbox_used,
                 sizeof(mailbox_entry), RT_MAILBOX_ENTRY_POOL_SIZE);

    rt_pool_init(&g_message_pool_mgr, g_message_pool, g_message_used,
                 sizeof(message_data_entry), RT_MESSAGE_DATA_POOL_SIZE);

    return RT_SUCCESS;
}

rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode) {
    actor *sender = rt_actor_current();
    if (!sender) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    actor *receiver = rt_actor_get(to);
    if (!receiver) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid receiver actor ID");
    }

    // Validate message size
    if (len > RT_MAX_MESSAGE_SIZE) {
        return RT_ERROR(RT_ERR_INVALID, "Message exceeds RT_MAX_MESSAGE_SIZE");
    }

    // Allocate mailbox entry from pool
    mailbox_entry *entry = rt_pool_alloc(&g_mailbox_pool_mgr);
    if (!entry) {
        return RT_ERROR(RT_ERR_NOMEM, "Mailbox entry pool exhausted");
    }

    entry->sender = sender->id;
    entry->len = len;
    entry->next = NULL;

    if (mode == IPC_COPY) {
        // Copy mode: allocate and copy data from pool
        message_data_entry *msg_data = rt_pool_alloc(&g_message_pool_mgr);
        if (!msg_data) {
            rt_pool_free(&g_mailbox_pool_mgr, entry);
            return RT_ERROR(RT_ERR_NOMEM, "Message data pool exhausted");
        }
        memcpy(msg_data->data, data, len);
        entry->data = msg_data->data;
        entry->borrow_ptr = NULL;

        // Add to receiver's mailbox
        if (receiver->mbox.tail) {
            receiver->mbox.tail->next = entry;
        } else {
            receiver->mbox.head = entry;
        }
        receiver->mbox.tail = entry;
        receiver->mbox.count++;

        // If receiver is blocked waiting for message, wake it up
        if (receiver->state == ACTOR_STATE_BLOCKED) {
            RT_LOG_TRACE("IPC: Waking up blocked receiver %u", to);
            receiver->state = ACTOR_STATE_READY;
        }

        RT_LOG_TRACE("IPC: Message sent from %u to %u (COPY mode)", sender->id, to);
        return RT_SUCCESS;

    } else { // IPC_BORROW
        // Borrow mode: reference sender's data
        entry->data = NULL;
        entry->borrow_ptr = data;

        // Add to receiver's mailbox
        if (receiver->mbox.tail) {
            receiver->mbox.tail->next = entry;
        } else {
            receiver->mbox.head = entry;
        }
        receiver->mbox.tail = entry;
        receiver->mbox.count++;

        // If receiver is blocked waiting for message, wake it up
        if (receiver->state == ACTOR_STATE_BLOCKED) {
            receiver->state = ACTOR_STATE_READY;
        }

        // Block sender until receiver releases
        sender->waiting_for_release = true;
        sender->blocked_on_actor = to;
        sender->state = ACTOR_STATE_BLOCKED;

        // Yield to scheduler
        rt_scheduler_yield();

        // When we return here, the message has been released
        return RT_SUCCESS;
    }
}

rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms) {
    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    RT_LOG_TRACE("IPC recv: actor %u checking mailbox (count=%zu)", current->id, current->mbox.count);

    timer_id timeout_timer = TIMER_ID_INVALID;

    // Check if there's a message in the mailbox
    if (current->mbox.head == NULL) {
        if (timeout_ms == 0) {
            // Non-blocking
            return RT_ERROR(RT_ERR_WOULDBLOCK, "No messages available");
        } else if (timeout_ms > 0) {
            // Blocking with timeout - create a timer
            RT_LOG_TRACE("IPC recv: actor %u blocking with %d ms timeout", current->id, timeout_ms);
            rt_status status = rt_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
            if (RT_FAILED(status)) {
                return status;
            }

            // Block and wait for message (timer or real)
            current->state = ACTOR_STATE_BLOCKED;
            rt_scheduler_yield();

            // When we wake up, check mailbox
            RT_LOG_TRACE("IPC recv: actor %u woke up, mailbox count=%zu", current->id, current->mbox.count);
            if (current->mbox.head == NULL) {
                // Shouldn't happen, but handle gracefully
                if (timeout_timer != TIMER_ID_INVALID) {
                    rt_timer_cancel(timeout_timer);
                }
                return RT_ERROR(RT_ERR_WOULDBLOCK, "No messages available after wakeup");
            }
        } else {
            // Blocking forever (timeout_ms < 0)
            RT_LOG_TRACE("IPC recv: actor %u blocking, waiting for message", current->id);
            current->state = ACTOR_STATE_BLOCKED;
            rt_scheduler_yield();

            // When we wake up, check again
            RT_LOG_TRACE("IPC recv: actor %u woke up, mailbox count=%zu", current->id, current->mbox.count);
            if (current->mbox.head == NULL) {
                return RT_ERROR(RT_ERR_WOULDBLOCK, "No messages available after wakeup");
            }
        }
    }

    // At this point, mailbox has at least one message
    // If we created a timeout timer, check if first message is the timeout
    if (timeout_timer != TIMER_ID_INVALID) {
        mailbox_entry *entry = current->mbox.head;
        if (entry->sender == RT_SENDER_TIMER) {
            // Check if this is our timeout timer (not another timer)
            // For now, assume it is (could enhance with timer ID matching)
            RT_LOG_TRACE("IPC recv: actor %u timeout occurred", current->id);

            // Dequeue and discard the timer message
            current->mbox.head = entry->next;
            if (current->mbox.head == NULL) {
                current->mbox.tail = NULL;
            }
            current->mbox.count--;

            // Free the timer message entry
            if (entry->data) {
                message_data_entry *msg_data = DATA_TO_MSG_ENTRY(entry->data);
                rt_pool_free(&g_message_pool_mgr, msg_data);
            }
            rt_pool_free(&g_mailbox_pool_mgr, entry);

            return RT_ERROR(RT_ERR_TIMEOUT, "Receive timeout");
        } else {
            // Got a real message before timeout - cancel the timer
            RT_LOG_TRACE("IPC recv: actor %u got message before timeout, cancelling timer", current->id);
            rt_timer_cancel(timeout_timer);
        }
    }

    // Free previous active message if any (auto-release for BORROW)
    if (current->active_msg) {
        // If previous message was BORROW, unblock sender
        if (current->active_msg->borrow_ptr) {
            actor *sender = rt_actor_get(current->active_msg->sender);
            if (sender && sender->waiting_for_release && sender->blocked_on_actor == current->id) {
                sender->waiting_for_release = false;
                sender->blocked_on_actor = ACTOR_ID_INVALID;
                sender->state = ACTOR_STATE_READY;
            }
        }

        // Free COPY message data from pool
        if (current->active_msg->data) {
            message_data_entry *msg_data = DATA_TO_MSG_ENTRY(current->active_msg->data);
            rt_pool_free(&g_message_pool_mgr, msg_data);
        }
        rt_pool_free(&g_mailbox_pool_mgr, current->active_msg);
        current->active_msg = NULL;
    }

    // Dequeue message
    mailbox_entry *entry = current->mbox.head;
    current->mbox.head = entry->next;
    if (current->mbox.head == NULL) {
        current->mbox.tail = NULL;
    }
    current->mbox.count--;

    // Fill in message structure
    msg->sender = entry->sender;
    msg->len = entry->len;
    msg->data = entry->borrow_ptr ? entry->borrow_ptr : entry->data;

    // Store entry as active message for later cleanup
    current->active_msg = entry;

    return RT_SUCCESS;
}

void rt_ipc_release(const rt_message *msg) {
    if (!msg) {
        return;
    }

    actor *current = rt_actor_current();
    if (!current) {
        return;
    }

    // Find the sender if this was a borrowed message
    actor *sender = rt_actor_get(msg->sender);
    if (sender && sender->waiting_for_release && sender->blocked_on_actor == current->id) {
        // Unblock sender
        sender->waiting_for_release = false;
        sender->blocked_on_actor = ACTOR_ID_INVALID;
        sender->state = ACTOR_STATE_READY;
    }

    // Free the active message
    if (current->active_msg) {
        if (current->active_msg->data) {
            // Calculate message_data_entry pointer from data pointer
            message_data_entry *msg_data = DATA_TO_MSG_ENTRY(current->active_msg->data);
            rt_pool_free(&g_message_pool_mgr, msg_data);
        }
        rt_pool_free(&g_mailbox_pool_mgr, current->active_msg);
        current->active_msg = NULL;
    }
}

bool rt_ipc_pending(void) {
    actor *current = rt_actor_current();
    if (!current) {
        return false;
    }
    return current->mbox.head != NULL;
}

size_t rt_ipc_count(void) {
    actor *current = rt_actor_current();
    if (!current) {
        return 0;
    }
    return current->mbox.count;
}

// Clear all entries from a mailbox (called during actor cleanup)
void rt_ipc_mailbox_clear(mailbox *mbox) {
    if (!mbox) {
        return;
    }

    mailbox_entry *entry = mbox->head;
    while (entry) {
        mailbox_entry *next = entry->next;
        if (entry->data) {
            // Calculate message_data_entry pointer from data pointer
            message_data_entry *msg_data = DATA_TO_MSG_ENTRY(entry->data);
            rt_pool_free(&g_message_pool_mgr, msg_data);
        }
        rt_pool_free(&g_mailbox_pool_mgr, entry);
        entry = next;
    }
    mbox->head = NULL;
    mbox->tail = NULL;
    mbox->count = 0;
}

// Free an active message entry (called during actor cleanup)
void rt_ipc_free_active_msg(mailbox_entry *entry) {
    if (!entry) {
        return;
    }

    if (entry->data) {
        // Calculate message_data_entry pointer from data pointer
        message_data_entry *msg_data = DATA_TO_MSG_ENTRY(entry->data);
        rt_pool_free(&g_message_pool_mgr, msg_data);
    }
    rt_pool_free(&g_mailbox_pool_mgr, entry);
}
