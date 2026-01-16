#include "hive_ipc.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_timer.h"
#include "hive_runtime.h"
#include "hive_log.h"
#include <stdlib.h>
#include <string.h>

// Static pools for IPC (mailbox entries and message data)
static mailbox_entry g_mailbox_pool[HIVE_MAILBOX_ENTRY_POOL_SIZE];
static bool g_mailbox_used[HIVE_MAILBOX_ENTRY_POOL_SIZE];
hive_pool g_mailbox_pool_mgr;  // Non-static so hive_link.c can access

// Message data pool - fixed size entries (type defined in hive_internal.h)
static message_data_entry g_message_pool[HIVE_MESSAGE_DATA_POOL_SIZE];
static bool g_message_used[HIVE_MESSAGE_DATA_POOL_SIZE];
hive_pool g_message_pool_mgr;  // Non-static so hive_link.c can access

// Tag generator for request/reply correlation
static uint32_t g_next_tag = 1;

// -----------------------------------------------------------------------------
// Header Encoding/Decoding
// -----------------------------------------------------------------------------

static inline uint32_t encode_header(hive_msg_class class, uint32_t tag) {
    return ((uint32_t)class << 28) | (tag & 0x0FFFFFFF);
}

static inline void decode_header(uint32_t header, hive_msg_class *class, uint32_t *tag) {
    if (class) *class = (hive_msg_class)(header >> 28);
    if (tag) *tag = header & 0x0FFFFFFF;
}

static uint32_t generate_tag(void) {
    uint32_t tag = (g_next_tag++ & HIVE_TAG_VALUE_MASK) | HIVE_TAG_GEN_BIT;
    if ((g_next_tag & HIVE_TAG_VALUE_MASK) == 0) {
        g_next_tag = 1;  // Skip 0 on wrap
    }
    return tag;
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

hive_status hive_ipc_init(void) {
    hive_pool_init(&g_mailbox_pool_mgr, g_mailbox_pool, g_mailbox_used,
                 sizeof(mailbox_entry), HIVE_MAILBOX_ENTRY_POOL_SIZE);

    hive_pool_init(&g_message_pool_mgr, g_message_pool, g_message_used,
                 sizeof(message_data_entry), HIVE_MESSAGE_DATA_POOL_SIZE);

    return HIVE_SUCCESS;
}

// -----------------------------------------------------------------------------
// Internal Helpers
// -----------------------------------------------------------------------------

// Free message data back to the shared message pool
// This is the single point for freeing message pool entries (DRY principle)
void hive_msg_pool_free(void *data) {
    if (data) {
        message_data_entry *msg_data =
            (message_data_entry *)((char *)data - offsetof(message_data_entry, data));
        hive_pool_free(&g_message_pool_mgr, msg_data);
    }
}

// Free a mailbox entry and its associated data buffer
void hive_ipc_free_entry(mailbox_entry *entry) {
    if (!entry) {
        return;
    }
    hive_msg_pool_free(entry->data);
    hive_pool_free(&g_mailbox_pool_mgr, entry);
}

// Add mailbox entry to actor's mailbox (doubly-linked list) and wake if blocked
void hive_mailbox_add_entry(actor *recipient, mailbox_entry *entry) {
    entry->next = NULL;
    entry->prev = recipient->mailbox.tail;

    if (recipient->mailbox.tail) {
        recipient->mailbox.tail->next = entry;
    } else {
        recipient->mailbox.head = entry;
    }
    recipient->mailbox.tail = entry;
    recipient->mailbox.count++;

    // Wake actor if blocked and message matches its filter
    if (recipient->state == ACTOR_STATE_WAITING) {
        // Check if message matches receive filter
        bool matches = true;

        // Check sender filter
        if (recipient->recv_filter_sender != HIVE_SENDER_ANY) {
            if (entry->sender != recipient->recv_filter_sender) {
                matches = false;
            }
        }

        // Check class filter (need to decode header)
        if (matches && recipient->recv_filter_class != HIVE_MSG_ANY) {
            if (entry->len >= HIVE_MSG_HEADER_SIZE) {
                uint32_t header = *(uint32_t *)entry->data;
                hive_msg_class msg_class;
                decode_header(header, &msg_class, NULL);
                if (msg_class != recipient->recv_filter_class) {
                    matches = false;
                }
            } else {
                matches = false;
            }
        }

        // Check tag filter
        if (matches && recipient->recv_filter_tag != HIVE_TAG_ANY) {
            if (entry->len >= HIVE_MSG_HEADER_SIZE) {
                uint32_t header = *(uint32_t *)entry->data;
                uint32_t msg_tag;
                decode_header(header, NULL, &msg_tag);
                if (msg_tag != recipient->recv_filter_tag) {
                    matches = false;
                }
            } else {
                matches = false;
            }
        }

        if (matches) {
            recipient->state = ACTOR_STATE_READY;
        }
    }
}

// Unlink entry from mailbox (supports unlinking from middle)
static void mailbox_unlink(mailbox *mbox, mailbox_entry *entry) {
    if (entry->prev) {
        entry->prev->next = entry->next;
    } else {
        mbox->head = entry->next;
    }

    if (entry->next) {
        entry->next->prev = entry->prev;
    } else {
        mbox->tail = entry->prev;
    }

    entry->next = NULL;
    entry->prev = NULL;
    mbox->count--;
}

// Scan mailbox for matching message
static mailbox_entry *mailbox_find_match(mailbox *mbox, actor_id from,
                                          hive_msg_class class, uint32_t tag) {
    for (mailbox_entry *entry = mbox->head; entry; entry = entry->next) {
        // Check sender filter
        if (from != HIVE_SENDER_ANY && entry->sender != from) {
            continue;
        }

        // Need valid header to check class/tag
        if (entry->len < HIVE_MSG_HEADER_SIZE) {
            continue;  // Skip malformed messages
        }

        uint32_t header = *(uint32_t *)entry->data;
        hive_msg_class msg_class;
        uint32_t msg_tag;
        decode_header(header, &msg_class, &msg_tag);

        // Check class filter
        if (class != HIVE_MSG_ANY && msg_class != class) {
            continue;
        }

        // Check tag filter
        if (tag != HIVE_TAG_ANY && msg_tag != tag) {
            continue;
        }

        // Found a match
        return entry;
    }
    return NULL;
}

// Dequeue the head entry from an actor's mailbox
mailbox_entry *hive_ipc_dequeue_head(actor *a) {
    if (!a || !a->mailbox.head) {
        return NULL;
    }
    mailbox_entry *entry = a->mailbox.head;
    mailbox_unlink(&a->mailbox, entry);
    return entry;
}

// Check for timeout message and handle it
hive_status hive_mailbox_handle_timeout(actor *current, timer_id timeout_timer, const char *operation) {
    if (timeout_timer == TIMER_ID_INVALID) {
        return HIVE_SUCCESS;  // No timeout was set
    }

    // Check if first message is from OUR specific timeout timer
    if (current->mailbox.head && current->mailbox.head->len >= HIVE_MSG_HEADER_SIZE) {
        uint32_t header = *(uint32_t *)current->mailbox.head->data;
        hive_msg_class msg_class;
        uint32_t msg_tag;
        decode_header(header, &msg_class, &msg_tag);

        if (msg_class == HIVE_MSG_TIMER && msg_tag == timeout_timer) {
            // This IS our timeout timer - dequeue, free, and return timeout error
            mailbox_entry *entry = hive_ipc_dequeue_head(current);
            hive_ipc_free_entry(entry);
            return HIVE_ERROR(HIVE_ERR_TIMEOUT, operation);
        }
    }

    // Not our timeout timer - another message arrived first
    // Cancel our timeout timer and return success
    hive_timer_cancel(timeout_timer);
    return HIVE_SUCCESS;
}

// -----------------------------------------------------------------------------
// Core Send/Receive
// -----------------------------------------------------------------------------

// Internal notify with explicit sender, class and tag (used by timer, link, etc.)
hive_status hive_ipc_notify_internal(actor_id to, actor_id sender, hive_msg_class class,
                                   uint32_t tag, const void *data, size_t len) {
    actor *receiver = hive_actor_get(to);
    if (!receiver) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid receiver actor ID");
    }

    // Validate message size (payload + header)
    size_t total_len = len + HIVE_MSG_HEADER_SIZE;
    if (total_len > HIVE_MAX_MESSAGE_SIZE) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Message exceeds HIVE_MAX_MESSAGE_SIZE");
    }

    // Allocate mailbox entry from pool
    mailbox_entry *entry = hive_pool_alloc(&g_mailbox_pool_mgr);
    if (!entry) {
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Mailbox entry pool exhausted");
    }

    // Allocate message data from pool
    message_data_entry *msg_data = hive_pool_alloc(&g_message_pool_mgr);
    if (!msg_data) {
        hive_pool_free(&g_mailbox_pool_mgr, entry);
        return HIVE_ERROR(HIVE_ERR_NOMEM, "Message data pool exhausted");
    }

    // Build message: header + payload
    uint32_t header = encode_header(class, tag);
    memcpy(msg_data->data, &header, HIVE_MSG_HEADER_SIZE);
    if (data && len > 0) {
        memcpy(msg_data->data + HIVE_MSG_HEADER_SIZE, data, len);
    }

    entry->sender = sender;
    entry->len = total_len;
    entry->data = msg_data->data;
    entry->next = NULL;
    entry->prev = NULL;

    // Add to receiver's mailbox and wake if blocked
    hive_mailbox_add_entry(receiver, entry);

    HIVE_LOG_TRACE("IPC: Message sent from %u to %u (class=%d, tag=%u)", sender, to, class, tag);
    return HIVE_SUCCESS;
}

hive_status hive_ipc_notify(actor_id to, uint32_t tag, const void *data, size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *sender = hive_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    return hive_ipc_notify_internal(to, sender->id, HIVE_MSG_NOTIFY, tag, data, len);
}

hive_status hive_ipc_notify_ex(actor_id to, hive_msg_class class,
                               uint32_t tag, const void *data, size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *sender = hive_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    return hive_ipc_notify_internal(to, sender->id, class, tag, data, len);
}

hive_status hive_ipc_recv(hive_message *msg, int32_t timeout_ms) {
    // Use recv_match with wildcards for all filters
    return hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_TAG_ANY, msg, timeout_ms);
}

hive_status hive_ipc_recv_match(actor_id from, hive_msg_class class,
                            uint32_t tag, hive_message *msg, int32_t timeout_ms) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    HIVE_LOG_TRACE("IPC recv_match: actor %u (from=%u, class=%d, tag=%u)",
                 current->id, from, class, tag);

    // Auto-release previous active message if any
    if (current->active_msg) {
        hive_ipc_free_entry(current->active_msg);
        current->active_msg = NULL;
    }

    timer_id timeout_timer = TIMER_ID_INVALID;

    // Search mailbox for matching message
    mailbox_entry *entry = mailbox_find_match(&current->mailbox, from, class, tag);

    if (!entry) {
        if (timeout_ms == 0) {
            // Non-blocking
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No matching messages available");
        }

        // Set up filters for wake-on-match
        current->recv_filter_sender = from;
        current->recv_filter_class = class;
        current->recv_filter_tag = tag;

        if (timeout_ms > 0) {
            // Blocking with timeout - create a timer
            HIVE_LOG_TRACE("IPC recv_match: actor %u blocking with %d ms timeout", current->id, timeout_ms);
            hive_status status = hive_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
            if (HIVE_FAILED(status)) {
                return status;
            }
        }

        // Block and wait
        current->state = ACTOR_STATE_WAITING;
        hive_scheduler_yield();

        // Clear filters after waking
        current->recv_filter_sender = HIVE_SENDER_ANY;
        current->recv_filter_class = HIVE_MSG_ANY;
        current->recv_filter_tag = HIVE_TAG_ANY;

        // Check for timeout
        if (timeout_timer != TIMER_ID_INVALID) {
            hive_status timeout_status = hive_mailbox_handle_timeout(current, timeout_timer, "Receive timeout");
            if (HIVE_FAILED(timeout_status)) {
                return timeout_status;
            }
        }

        // Re-scan mailbox for match
        entry = mailbox_find_match(&current->mailbox, from, class, tag);
        if (!entry) {
            return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "No matching messages available after wakeup");
        }
    }

    // Found a match - unlink from mailbox
    mailbox_unlink(&current->mailbox, entry);

    // Pre-decode header and fill in message structure
    uint32_t header = *(uint32_t *)entry->data;
    hive_msg_class msg_class;
    uint32_t msg_tag;
    decode_header(header, &msg_class, &msg_tag);

    msg->sender = entry->sender;
    msg->class = msg_class;
    msg->tag = msg_tag;
    msg->len = entry->len - HIVE_MSG_HEADER_SIZE;  // Payload length only
    msg->data = (const uint8_t *)entry->data + HIVE_MSG_HEADER_SIZE;  // Skip header

    // Store entry as active message for later cleanup
    current->active_msg = entry;

    return HIVE_SUCCESS;
}

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

hive_status hive_ipc_request(actor_id to, const void *request, size_t req_len,
                      hive_message *reply, int32_t timeout_ms) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *sender = hive_actor_current();

    // Validate data pointer
    if (request == NULL && req_len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL request with non-zero length");
    }

    // Generate unique tag for this call
    uint32_t call_tag = generate_tag();

    // Send HIVE_MSG_REQUEST with generated tag
    hive_status status = hive_ipc_notify_internal(to, sender->id, HIVE_MSG_REQUEST, call_tag, request, req_len);
    if (HIVE_FAILED(status)) {
        return status;
    }

    // Wait for HIVE_MSG_REPLY with matching tag from the callee
    return hive_ipc_recv_match(to, HIVE_MSG_REPLY, call_tag, reply, timeout_ms);
}

hive_status hive_ipc_reply(const hive_message *request, const void *data, size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor *current = hive_actor_current();

    if (!request) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid request message");
    }

    // Verify this is a REQUEST message
    if (request->class != HIVE_MSG_REQUEST) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Can only reply to HIVE_MSG_REQUEST messages");
    }

    // Validate data pointer
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    // Send HIVE_MSG_REPLY with same tag back to caller
    return hive_ipc_notify_internal(request->sender, current->id, HIVE_MSG_REPLY, request->tag, data, len);
}

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

bool hive_msg_is_timer(const hive_message *msg) {
    if (!msg) {
        return false;
    }
    // Check message class
    return msg->class == HIVE_MSG_TIMER;
}

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

bool hive_ipc_pending(void) {
    actor *current = hive_actor_current();
    if (!current) {
        return false;
    }
    return current->mailbox.head != NULL;
}

size_t hive_ipc_count(void) {
    actor *current = hive_actor_current();
    if (!current) {
        return 0;
    }
    return current->mailbox.count;
}

// -----------------------------------------------------------------------------
// Cleanup Functions
// -----------------------------------------------------------------------------

// Clear all entries from a mailbox (called during actor cleanup)
void hive_ipc_mailbox_clear(mailbox *mbox) {
    if (!mbox) {
        return;
    }

    mailbox_entry *entry = mbox->head;
    while (entry) {
        mailbox_entry *next = entry->next;
        hive_ipc_free_entry(entry);
        entry = next;
    }
    mbox->head = NULL;
    mbox->tail = NULL;
    mbox->count = 0;
}

// Free an active message entry (called during actor cleanup)
void hive_ipc_free_active_msg(mailbox_entry *entry) {
    hive_ipc_free_entry(entry);
}
