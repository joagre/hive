#include "rt_ipc.h"
#include "rt_internal.h"
#include "rt_static_config.h"
#include "rt_pool.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_timer.h"
#include "rt_runtime.h"
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

// Borrow buffer pool - for IPC_SYNC to avoid UAF when sender dies
// Using pinned runtime buffers instead of sender's stack eliminates use-after-free
static message_data_entry g_sync_pool[RT_SYNC_BUFFER_POOL_SIZE];
static bool g_sync_used[RT_SYNC_BUFFER_POOL_SIZE];
static rt_pool g_sync_pool_mgr;

// Forward declaration for init function
rt_status rt_ipc_init(void);

// Initialize IPC pools
rt_status rt_ipc_init(void) {
    rt_pool_init(&g_mailbox_pool_mgr, g_mailbox_pool, g_mailbox_used,
                 sizeof(mailbox_entry), RT_MAILBOX_ENTRY_POOL_SIZE);

    rt_pool_init(&g_message_pool_mgr, g_message_pool, g_message_used,
                 sizeof(message_data_entry), RT_MESSAGE_DATA_POOL_SIZE);

    rt_pool_init(&g_sync_pool_mgr, g_sync_pool, g_sync_used,
                 sizeof(message_data_entry), RT_SYNC_BUFFER_POOL_SIZE);

    return RT_SUCCESS;
}

// Internal helper: Add mailbox entry to actor's mailbox and wake if blocked
void rt_mailbox_add_entry(actor *recipient, mailbox_entry *entry) {
    // Append to mailbox list
    if (recipient->mbox.tail) {
        recipient->mbox.tail->next = entry;
    } else {
        recipient->mbox.head = entry;
    }
    recipient->mbox.tail = entry;
    recipient->mbox.count++;

    // Wake actor if blocked
    if (recipient->state == ACTOR_STATE_BLOCKED) {
        recipient->state = ACTOR_STATE_READY;
    }
}

// Internal helper: Free a mailbox entry and its associated data buffers
void rt_ipc_free_entry(mailbox_entry *entry) {
    if (!entry) {
        return;
    }
    // Free sync buffer if SYNC mode
    if (entry->sync_ptr) {
        message_data_entry *sync_data = DATA_TO_MSG_ENTRY(entry->sync_ptr);
        rt_pool_free(&g_sync_pool_mgr, sync_data);
    }
    // Free ASYNC buffer if ASYNC mode
    if (entry->data) {
        message_data_entry *msg_data = DATA_TO_MSG_ENTRY(entry->data);
        rt_pool_free(&g_message_pool_mgr, msg_data);
    }
    rt_pool_free(&g_mailbox_pool_mgr, entry);
}

// Internal helper: Unblock a sender waiting for IPC_SYNC release
void rt_ipc_unblock_sender(actor_id sender_id, actor_id receiver_id) {
    actor *sender = rt_actor_get(sender_id);
    if (sender && sender->waiting_for_release && sender->blocked_on_actor == receiver_id) {
        sender->waiting_for_release = false;
        sender->blocked_on_actor = ACTOR_ID_INVALID;
        sender->state = ACTOR_STATE_READY;
    }
}

// Internal helper: Dequeue the head entry from an actor's mailbox
mailbox_entry *rt_ipc_dequeue_head(actor *a) {
    if (!a || !a->mbox.head) {
        return NULL;
    }
    mailbox_entry *entry = a->mbox.head;
    a->mbox.head = entry->next;
    if (a->mbox.head == NULL) {
        a->mbox.tail = NULL;
    }
    a->mbox.count--;
    entry->next = NULL;
    return entry;
}

// Internal helper: Check for timeout message and handle it
rt_status rt_mailbox_handle_timeout(actor *current, timer_id timeout_timer, const char *operation) {
    if (timeout_timer == TIMER_ID_INVALID) {
        return RT_SUCCESS;  // No timeout was set
    }

    // Check if first message is from OUR specific timeout timer
    if (current->mbox.head && current->mbox.head->sender == RT_SENDER_TIMER) {
        // Timer message - check if it's from our timeout timer (not a periodic or other timer)
        timer_id msg_timer_id = *(timer_id *)current->mbox.head->data;
        if (msg_timer_id == timeout_timer) {
            // This IS our timeout timer - dequeue, free, and return timeout error
            mailbox_entry *entry = rt_ipc_dequeue_head(current);
            rt_ipc_free_entry(entry);
            return RT_ERROR(RT_ERR_TIMEOUT, operation);
        }
    }

    // Not our timeout timer - another message arrived first (regular, different timer, etc.)
    // Cancel our timeout timer and return success
    rt_timer_cancel(timeout_timer);
    return RT_SUCCESS;
}

rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *sender = rt_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return RT_ERROR(RT_ERR_INVALID, "NULL data with non-zero length");
    }

    actor *receiver = rt_actor_get(to);
    if (!receiver) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid receiver actor ID");
    }

    // Validate message size
    if (len > RT_MAX_MESSAGE_SIZE) {
        return RT_ERROR(RT_ERR_INVALID, "Message exceeds RT_MAX_MESSAGE_SIZE");
    }

    // Prevent self-send with SYNC (guaranteed deadlock)
    if (mode == IPC_SYNC && to == sender->id) {
        return RT_ERROR(RT_ERR_INVALID, "Self-send with IPC_SYNC is forbidden (deadlock)");
    }

    // Allocate mailbox entry from pool
    mailbox_entry *entry = rt_pool_alloc(&g_mailbox_pool_mgr);
    if (!entry) {
        return RT_ERROR(RT_ERR_NOMEM, "Mailbox entry pool exhausted");
    }

    entry->sender = sender->id;
    entry->len = len;
    entry->next = NULL;

    if (mode == IPC_ASYNC) {
        // Copy mode: allocate and copy data from pool
        message_data_entry *msg_data = rt_pool_alloc(&g_message_pool_mgr);
        if (!msg_data) {
            rt_pool_free(&g_mailbox_pool_mgr, entry);
            return RT_ERROR(RT_ERR_NOMEM, "Message data pool exhausted");
        }
        memcpy(msg_data->data, data, len);
        entry->data = msg_data->data;
        entry->sync_ptr = NULL;

        // Add to receiver's mailbox and wake if blocked
        rt_mailbox_add_entry(receiver, entry);

        RT_LOG_TRACE("IPC: Message sent from %u to %u (ASYNC mode)", sender->id, to);
        return RT_SUCCESS;

    } else { // IPC_SYNC
        // Borrow mode: Copy to pinned runtime buffer (not sender's stack)
        // This prevents use-after-free if sender dies while receiver holds message
        message_data_entry *sync_buf = rt_pool_alloc(&g_sync_pool_mgr);
        if (!sync_buf) {
            rt_pool_free(&g_mailbox_pool_mgr, entry);
            return RT_ERROR(RT_ERR_NOMEM, "Borrow buffer pool exhausted");
        }
        memcpy(sync_buf->data, data, len);
        entry->data = NULL;
        entry->sync_ptr = sync_buf->data;

        // Add to receiver's mailbox and wake if blocked
        rt_mailbox_add_entry(receiver, entry);

        // Block sender until receiver releases
        sender->waiting_for_release = true;
        sender->blocked_on_actor = to;
        sender->io_status = RT_SUCCESS;  // Assume success, will be overridden if receiver dies
        sender->state = ACTOR_STATE_BLOCKED;

        // Yield to scheduler
        rt_scheduler_yield();

        // When we return here, the message has been released (or receiver died)
        // Return status set by either rt_ipc_release() (RT_SUCCESS) or receiver death (RT_ERR_CLOSED)
        return sender->io_status;
    }
}

rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms) {
    RT_REQUIRE_ACTOR_CONTEXT();
    actor *current = rt_actor_current();

    RT_LOG_TRACE("IPC recv: actor %u checking mailbox (count=%zu)", current->id, current->mbox.count);

    // Auto-release previous active message if any (must happen BEFORE blocking)
    // This ensures SYNC senders are unblocked even if this recv times out
    if (current->active_msg) {
        if (current->active_msg->sync_ptr) {
            rt_ipc_unblock_sender(current->active_msg->sender, current->id);
        }
        rt_ipc_free_entry(current->active_msg);
        current->active_msg = NULL;
    }

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
    // Check for timeout and handle it
    rt_status timeout_status = rt_mailbox_handle_timeout(current, timeout_timer, "Receive timeout");
    if (RT_FAILED(timeout_status)) {
        return timeout_status;
    }

    // Dequeue message
    mailbox_entry *entry = rt_ipc_dequeue_head(current);

    // Fill in message structure
    msg->sender = entry->sender;
    msg->len = entry->len;
    msg->data = entry->sync_ptr ? entry->sync_ptr : entry->data;

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

    // Unblock sender if this was a synced message
    rt_ipc_unblock_sender(msg->sender, current->id);

    // Free the active message
    if (current->active_msg) {
        rt_ipc_free_entry(current->active_msg);
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
        rt_ipc_free_entry(entry);
        entry = next;
    }
    mbox->head = NULL;
    mbox->tail = NULL;
    mbox->count = 0;
}

// Free an active message entry (called during actor cleanup)
// Note: This is now just a wrapper around rt_ipc_free_entry for API compatibility
void rt_ipc_free_active_msg(mailbox_entry *entry) {
    rt_ipc_free_entry(entry);
}
