#include "acrt_ipc.h"
#include "acrt_internal.h"
#include "acrt_static_config.h"
#include "acrt_pool.h"
#include "acrt_actor.h"
#include "acrt_scheduler.h"
#include "acrt_timer.h"
#include "acrt_runtime.h"
#include "acrt_log.h"
#include <stdlib.h>
#include <string.h>

// Static pools for IPC (mailbox entries and message data)
static mailbox_entry g_mailbox_pool[ACRT_MAILBOX_ENTRY_POOL_SIZE];
static bool g_mailbox_used[ACRT_MAILBOX_ENTRY_POOL_SIZE];
acrt_pool g_mailbox_pool_mgr;  // Non-static so acrt_link.c can access

// Message data pool - fixed size entries (type defined in acrt_internal.h)
static message_data_entry g_message_pool[ACRT_MESSAGE_DATA_POOL_SIZE];
static bool g_message_used[ACRT_MESSAGE_DATA_POOL_SIZE];
acrt_pool g_message_pool_mgr;  // Non-static so acrt_link.c can access

// Tag generator for request/reply correlation
static uint32_t g_next_tag = 1;

// Forward declaration for init function
acrt_status acrt_ipc_init(void);

// -----------------------------------------------------------------------------
// Header Encoding/Decoding
// -----------------------------------------------------------------------------

static inline uint32_t encode_header(acrt_msg_class class, uint32_t tag) {
    return ((uint32_t)class << 28) | (tag & 0x0FFFFFFF);
}

static inline void decode_header(uint32_t header, acrt_msg_class *class, uint32_t *tag) {
    if (class) *class = (acrt_msg_class)(header >> 28);
    if (tag) *tag = header & 0x0FFFFFFF;
}

static uint32_t generate_tag(void) {
    uint32_t tag = (g_next_tag++ & ACRT_TAG_VALUE_MASK) | ACRT_TAG_GEN_BIT;
    if ((g_next_tag & ACRT_TAG_VALUE_MASK) == 0) {
        g_next_tag = 1;  // Skip 0 on wrap
    }
    return tag;
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

acrt_status acrt_ipc_init(void) {
    acrt_pool_init(&g_mailbox_pool_mgr, g_mailbox_pool, g_mailbox_used,
                 sizeof(mailbox_entry), ACRT_MAILBOX_ENTRY_POOL_SIZE);

    acrt_pool_init(&g_message_pool_mgr, g_message_pool, g_message_used,
                 sizeof(message_data_entry), ACRT_MESSAGE_DATA_POOL_SIZE);

    return ACRT_SUCCESS;
}

// -----------------------------------------------------------------------------
// Internal Helpers
// -----------------------------------------------------------------------------

// Free message data back to the shared message pool
// This is the single point for freeing message pool entries (DRY principle)
void acrt_msg_pool_free(void *data) {
    if (data) {
        message_data_entry *msg_data = DATA_TO_MSG_ENTRY(data);
        acrt_pool_free(&g_message_pool_mgr, msg_data);
    }
}

// Free a mailbox entry and its associated data buffer
void acrt_ipc_free_entry(mailbox_entry *entry) {
    if (!entry) {
        return;
    }
    acrt_msg_pool_free(entry->data);
    acrt_pool_free(&g_mailbox_pool_mgr, entry);
}

// Add mailbox entry to actor's mailbox (doubly-linked list) and wake if blocked
void acrt_mailbox_add_entry(actor *recipient, mailbox_entry *entry) {
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
        if (recipient->recv_filter_sender != ACRT_SENDER_ANY) {
            if (entry->sender != recipient->recv_filter_sender) {
                matches = false;
            }
        }

        // Check class filter (need to decode header)
        if (matches && recipient->recv_filter_class != ACRT_MSG_ANY) {
            if (entry->len >= ACRT_MSG_HEADER_SIZE) {
                uint32_t header = *(uint32_t *)entry->data;
                acrt_msg_class msg_class;
                decode_header(header, &msg_class, NULL);
                if (msg_class != recipient->recv_filter_class) {
                    matches = false;
                }
            } else {
                matches = false;  // Invalid message
            }
        }

        // Check tag filter
        if (matches && recipient->recv_filter_tag != ACRT_TAG_ANY) {
            if (entry->len >= ACRT_MSG_HEADER_SIZE) {
                uint32_t header = *(uint32_t *)entry->data;
                uint32_t msg_tag;
                decode_header(header, NULL, &msg_tag);
                if (msg_tag != recipient->recv_filter_tag) {
                    matches = false;
                }
            } else {
                matches = false;  // Invalid message
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
                                          acrt_msg_class class, uint32_t tag) {
    for (mailbox_entry *entry = mbox->head; entry; entry = entry->next) {
        // Check sender filter
        if (from != ACRT_SENDER_ANY && entry->sender != from) {
            continue;
        }

        // Need valid header to check class/tag
        if (entry->len < ACRT_MSG_HEADER_SIZE) {
            continue;  // Skip malformed messages
        }

        uint32_t header = *(uint32_t *)entry->data;
        acrt_msg_class msg_class;
        uint32_t msg_tag;
        decode_header(header, &msg_class, &msg_tag);

        // Check class filter
        if (class != ACRT_MSG_ANY && msg_class != class) {
            continue;
        }

        // Check tag filter
        if (tag != ACRT_TAG_ANY && msg_tag != tag) {
            continue;
        }

        // Found a match
        return entry;
    }
    return NULL;
}

// Dequeue the head entry from an actor's mailbox
mailbox_entry *acrt_ipc_dequeue_head(actor *a) {
    if (!a || !a->mailbox.head) {
        return NULL;
    }
    mailbox_entry *entry = a->mailbox.head;
    mailbox_unlink(&a->mailbox, entry);
    return entry;
}

// Check for timeout message and handle it
acrt_status acrt_mailbox_handle_timeout(actor *current, timer_id timeout_timer, const char *operation) {
    if (timeout_timer == TIMER_ID_INVALID) {
        return ACRT_SUCCESS;  // No timeout was set
    }

    // Check if first message is from OUR specific timeout timer
    if (current->mailbox.head && current->mailbox.head->len >= ACRT_MSG_HEADER_SIZE) {
        uint32_t header = *(uint32_t *)current->mailbox.head->data;
        acrt_msg_class msg_class;
        uint32_t msg_tag;
        decode_header(header, &msg_class, &msg_tag);

        if (msg_class == ACRT_MSG_TIMER && msg_tag == timeout_timer) {
            // This IS our timeout timer - dequeue, free, and return timeout error
            mailbox_entry *entry = acrt_ipc_dequeue_head(current);
            acrt_ipc_free_entry(entry);
            return ACRT_ERROR(ACRT_ERR_TIMEOUT, operation);
        }
    }

    // Not our timeout timer - another message arrived first
    // Cancel our timeout timer and return success
    acrt_timer_cancel(timeout_timer);
    return ACRT_SUCCESS;
}

// -----------------------------------------------------------------------------
// Core Send/Receive
// -----------------------------------------------------------------------------

// Internal send with explicit class and tag (used by timer, link, etc.)
acrt_status acrt_ipc_notify_ex(actor_id to, actor_id sender, acrt_msg_class class,
                         uint32_t tag, const void *data, size_t len) {
    actor *receiver = acrt_actor_get(to);
    if (!receiver) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid receiver actor ID");
    }

    // Validate message size (payload + header)
    size_t total_len = len + ACRT_MSG_HEADER_SIZE;
    if (total_len > ACRT_MAX_MESSAGE_SIZE) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Message exceeds ACRT_MAX_MESSAGE_SIZE");
    }

    // Allocate mailbox entry from pool
    mailbox_entry *entry = acrt_pool_alloc(&g_mailbox_pool_mgr);
    if (!entry) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Mailbox entry pool exhausted");
    }

    // Allocate message data from pool
    message_data_entry *msg_data = acrt_pool_alloc(&g_message_pool_mgr);
    if (!msg_data) {
        acrt_pool_free(&g_mailbox_pool_mgr, entry);
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Message data pool exhausted");
    }

    // Build message: header + payload
    uint32_t header = encode_header(class, tag);
    memcpy(msg_data->data, &header, ACRT_MSG_HEADER_SIZE);
    if (data && len > 0) {
        memcpy(msg_data->data + ACRT_MSG_HEADER_SIZE, data, len);
    }

    entry->sender = sender;
    entry->len = total_len;
    entry->data = msg_data->data;
    entry->next = NULL;
    entry->prev = NULL;

    // Add to receiver's mailbox and wake if blocked
    acrt_mailbox_add_entry(receiver, entry);

    ACRT_LOG_TRACE("IPC: Message sent from %u to %u (class=%d, tag=%u)", sender, to, class, tag);
    return ACRT_SUCCESS;
}

acrt_status acrt_ipc_notify(actor_id to, const void *data, size_t len) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *sender = acrt_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "NULL data with non-zero length");
    }

    return acrt_ipc_notify_ex(to, sender->id, ACRT_MSG_ASYNC, ACRT_TAG_NONE, data, len);
}

acrt_status acrt_ipc_recv(acrt_message *msg, int32_t timeout_ms) {
    // Use recv_match with wildcards for all filters
    return acrt_ipc_recv_match(NULL, NULL, NULL, msg, timeout_ms);
}

acrt_status acrt_ipc_recv_match(const actor_id *from, const acrt_msg_class *class,
                            const uint32_t *tag, acrt_message *msg, int32_t timeout_ms) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Convert filter pointers to values (use wildcards if NULL)
    actor_id filter_from = from ? *from : ACRT_SENDER_ANY;
    acrt_msg_class filter_class = class ? *class : ACRT_MSG_ANY;
    uint32_t filter_tag = tag ? *tag : ACRT_TAG_ANY;

    ACRT_LOG_TRACE("IPC recv_match: actor %u (from=%u, class=%d, tag=%u)",
                 current->id, filter_from, filter_class, filter_tag);

    // Auto-release previous active message if any
    if (current->active_msg) {
        acrt_ipc_free_entry(current->active_msg);
        current->active_msg = NULL;
    }

    timer_id timeout_timer = TIMER_ID_INVALID;

    // Search mailbox for matching message
    mailbox_entry *entry = mailbox_find_match(&current->mailbox, filter_from, filter_class, filter_tag);

    if (!entry) {
        if (timeout_ms == 0) {
            // Non-blocking
            return ACRT_ERROR(ACRT_ERR_WOULDBLOCK, "No matching messages available");
        }

        // Set up filters for wake-on-match
        current->recv_filter_sender = filter_from;
        current->recv_filter_class = filter_class;
        current->recv_filter_tag = filter_tag;

        if (timeout_ms > 0) {
            // Blocking with timeout - create a timer
            ACRT_LOG_TRACE("IPC recv_match: actor %u blocking with %d ms timeout", current->id, timeout_ms);
            acrt_status status = acrt_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
            if (ACRT_FAILED(status)) {
                return status;
            }
        }

        // Block and wait
        current->state = ACTOR_STATE_WAITING;
        acrt_scheduler_yield();

        // Clear filters after waking
        current->recv_filter_sender = ACRT_SENDER_ANY;
        current->recv_filter_class = ACRT_MSG_ANY;
        current->recv_filter_tag = ACRT_TAG_ANY;

        // Check for timeout
        if (timeout_timer != TIMER_ID_INVALID) {
            acrt_status timeout_status = acrt_mailbox_handle_timeout(current, timeout_timer, "Receive timeout");
            if (ACRT_FAILED(timeout_status)) {
                return timeout_status;
            }
        }

        // Re-scan mailbox for match
        entry = mailbox_find_match(&current->mailbox, filter_from, filter_class, filter_tag);
        if (!entry) {
            return ACRT_ERROR(ACRT_ERR_WOULDBLOCK, "No matching messages available after wakeup");
        }
    }

    // Found a match - unlink from mailbox
    mailbox_unlink(&current->mailbox, entry);

    // Fill in message structure
    msg->sender = entry->sender;
    msg->len = entry->len;
    msg->data = entry->data;

    // Store entry as active message for later cleanup
    current->active_msg = entry;

    return ACRT_SUCCESS;
}

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

acrt_status acrt_ipc_request(actor_id to, const void *request, size_t req_len,
                      acrt_message *reply, int32_t timeout_ms) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *sender = acrt_actor_current();

    // Validate data pointer
    if (request == NULL && req_len > 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "NULL request with non-zero length");
    }

    // Generate unique tag for this call
    uint32_t call_tag = generate_tag();

    // Send ACRT_MSG_REQUEST with generated tag
    acrt_status status = acrt_ipc_notify_ex(to, sender->id, ACRT_MSG_REQUEST, call_tag, request, req_len);
    if (ACRT_FAILED(status)) {
        return status;
    }

    // Wait for ACRT_MSG_REPLY with matching tag from the callee
    acrt_msg_class reply_class = ACRT_MSG_REPLY;
    return acrt_ipc_recv_match(&to, &reply_class, &call_tag, reply, timeout_ms);
}

acrt_status acrt_ipc_reply(const acrt_message *request, const void *data, size_t len) {
    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    if (!request || !request->data || request->len < ACRT_MSG_HEADER_SIZE) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid request message");
    }

    // Extract tag from request header
    uint32_t header = *(uint32_t *)request->data;
    acrt_msg_class req_class;
    uint32_t req_tag;
    decode_header(header, &req_class, &req_tag);

    // Verify this is a REQUEST message
    if (req_class != ACRT_MSG_REQUEST) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Can only reply to ACRT_MSG_REQUEST messages");
    }

    // Validate data pointer
    if (data == NULL && len > 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "NULL data with non-zero length");
    }

    // Send ACRT_MSG_REPLY with same tag back to caller
    return acrt_ipc_notify_ex(request->sender, current->id, ACRT_MSG_REPLY, req_tag, data, len);
}

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

acrt_status acrt_msg_decode(const acrt_message *msg, acrt_msg_class *class,
                        uint32_t *tag, const void **payload, size_t *payload_len) {
    if (!msg || !msg->data || msg->len < ACRT_MSG_HEADER_SIZE) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid message");
    }

    uint32_t header = *(uint32_t *)msg->data;
    decode_header(header, class, tag);

    if (payload) {
        *payload = (const uint8_t *)msg->data + ACRT_MSG_HEADER_SIZE;
    }
    if (payload_len) {
        *payload_len = msg->len - ACRT_MSG_HEADER_SIZE;
    }

    return ACRT_SUCCESS;
}

bool acrt_msg_is_timer(const acrt_message *msg) {
    if (!msg || !msg->data || msg->len < ACRT_MSG_HEADER_SIZE) {
        return false;
    }

    uint32_t header = *(uint32_t *)msg->data;
    acrt_msg_class class;
    decode_header(header, &class, NULL);

    return class == ACRT_MSG_TIMER;
}

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

bool acrt_ipc_pending(void) {
    actor *current = acrt_actor_current();
    if (!current) {
        return false;
    }
    return current->mailbox.head != NULL;
}

size_t acrt_ipc_count(void) {
    actor *current = acrt_actor_current();
    if (!current) {
        return 0;
    }
    return current->mailbox.count;
}

// -----------------------------------------------------------------------------
// Cleanup Functions
// -----------------------------------------------------------------------------

// Clear all entries from a mailbox (called during actor cleanup)
void acrt_ipc_mailbox_clear(mailbox *mbox) {
    if (!mbox) {
        return;
    }

    mailbox_entry *entry = mbox->head;
    while (entry) {
        mailbox_entry *next = entry->next;
        acrt_ipc_free_entry(entry);
        entry = next;
    }
    mbox->head = NULL;
    mbox->tail = NULL;
    mbox->count = 0;
}

// Free an active message entry (called during actor cleanup)
void acrt_ipc_free_active_msg(mailbox_entry *entry) {
    acrt_ipc_free_entry(entry);
}
