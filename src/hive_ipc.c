#include "hive_ipc.h"
#include "hive_internal.h"
#include "hive_static_config.h"
#include "hive_pool.h"
#include "hive_actor.h"
#include "hive_scheduler.h"
#include "hive_timer.h"
#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_log.h"
#include "hive_select.h"
#include <string.h>

// Compile-time checks
_Static_assert(HIVE_MAX_MESSAGE_SIZE > HIVE_MSG_HEADER_SIZE,
               "HIVE_MAX_MESSAGE_SIZE must exceed HIVE_MSG_HEADER_SIZE");
_Static_assert(HIVE_RESERVED_SYSTEM_ENTRIES < HIVE_MAILBOX_ENTRY_POOL_SIZE,
               "HIVE_RESERVED_SYSTEM_ENTRIES must be less than pool size");

// Static pools for IPC (mailbox_t entries and message data)
static mailbox_entry_t s_mailbox_pool[HIVE_MAILBOX_ENTRY_POOL_SIZE];
static bool s_mailbox_used[HIVE_MAILBOX_ENTRY_POOL_SIZE];
hive_pool_t g_mailbox_pool_mgr; // Non-static so hive_link.c can access

// Message data pool - fixed size entries (type defined in hive_internal.h)
static message_data_entry_t s_message_pool[HIVE_MESSAGE_DATA_POOL_SIZE];
static bool s_message_used[HIVE_MESSAGE_DATA_POOL_SIZE];
hive_pool_t g_message_pool_mgr; // Non-static so hive_link.c can access

// Tag generator for request/reply correlation
static uint32_t s_next_tag = 1;

// -----------------------------------------------------------------------------
// Header Encoding/Decoding
// -----------------------------------------------------------------------------
// Header layout: [class:4][tag:28] [id:16] = 6 bytes
// First 4 bytes: class and tag (for request/reply correlation)
// Last 2 bytes: id (for message type dispatch)

static inline void encode_header(uint8_t *buf, hive_msg_class_t class,
                                 uint16_t id, uint32_t tag) {
    uint32_t word0 = ((uint32_t) class << 28) | (tag & 0x0FFFFFFF);
    memcpy(buf, &word0, 4);
    memcpy(buf + 4, &id, 2);
}

static inline void decode_header(const uint8_t *buf, hive_msg_class_t *class,
                                 uint16_t *id, uint32_t *tag) {
    uint32_t word0;
    memcpy(&word0, buf, 4);
    if (class != NULL)
        *class = (hive_msg_class_t)(word0 >> 28);
    if (tag != NULL)
        *tag = word0 & 0x0FFFFFFF;
    if (id != NULL) {
        memcpy(id, buf + 4, 2);
    }
}

static uint32_t generate_tag(void) {
    uint32_t tag = s_next_tag++ & HIVE_TAG_VALUE_MASK;
    if (tag == 0) {
        tag = s_next_tag++ & HIVE_TAG_VALUE_MASK; // Skip 0
    }
    return tag;
}

// -----------------------------------------------------------------------------
// Initialization
// -----------------------------------------------------------------------------

hive_status_t hive_ipc_init(void) {
    hive_pool_init(&g_mailbox_pool_mgr, s_mailbox_pool, s_mailbox_used,
                   sizeof(mailbox_entry_t), HIVE_MAILBOX_ENTRY_POOL_SIZE);

    hive_pool_init(&g_message_pool_mgr, s_message_pool, s_message_used,
                   sizeof(message_data_entry_t), HIVE_MESSAGE_DATA_POOL_SIZE);

    return HIVE_SUCCESS;
}

// -----------------------------------------------------------------------------
// Internal Helpers
// -----------------------------------------------------------------------------

// Free message data back to the shared message pool
// This is the single point for freeing message pool entries (DRY principle)
void hive_msg_pool_free(void *data) {
    if (data) {
        message_data_entry_t *msg_data =
            (message_data_entry_t *)((char *)data -
                                     offsetof(message_data_entry_t, data));
        hive_pool_free(&g_message_pool_mgr, msg_data);
        // Wake any actor waiting for pool space
        hive_scheduler_pool_wake_one();
    }
}

// Free a mailbox_t entry and its associated data buffer
void hive_ipc_free_entry(mailbox_entry_t *entry) {
    if (!entry) {
        return;
    }
    hive_msg_pool_free(entry->data); // This wakes one waiter
    hive_pool_free(&g_mailbox_pool_mgr, entry);
    // Wake another waiter for the mailbox entry pool
    hive_scheduler_pool_wake_one();
}

// Check if a mailbox_t entry matches a single filter
static bool entry_matches_filter(mailbox_entry_t *entry,
                                 const hive_recv_filter_t *filter) {
    // Check sender filter (0 = wildcard)
    if (filter->sender != HIVE_SENDER_ANY && entry->sender != filter->sender) {
        return false;
    }

    // Need valid header to check class/id/tag
    if (entry->len < HIVE_MSG_HEADER_SIZE) {
        return false;
    }

    hive_msg_class_t msg_class;
    uint16_t msg_id;
    uint32_t msg_tag;
    decode_header(entry->data, &msg_class, &msg_id, &msg_tag);

    // Check class filter (0 = wildcard)
    if (filter->class != HIVE_MSG_ANY && msg_class != filter->class) {
        return false;
    }

    // Check id filter (0 = wildcard)
    if (filter->id != HIVE_ID_ANY && msg_id != filter->id) {
        return false;
    }

    // Check tag filter (0 = wildcard)
    if (filter->tag != HIVE_TAG_ANY && msg_tag != filter->tag) {
        return false;
    }

    return true;
}

// Add mailbox_t entry to actor_t's mailbox_t (doubly-linked list) and wake if blocked
void hive_mailbox_add_entry(actor_t *recipient, mailbox_entry_t *entry) {
    entry->next = NULL;
    entry->prev = recipient->mailbox.tail;

    if (recipient->mailbox.tail) {
        recipient->mailbox.tail->next = entry;
    } else {
        recipient->mailbox.head = entry;
    }
    recipient->mailbox.tail = entry;
    recipient->mailbox.count++;

    // Wake actor_t if blocked
    if (recipient->state == ACTOR_STATE_WAITING) {
        bool should_wake = false;

        // Check hive_select sources first (if active)
        if (recipient->select_sources) {
            for (size_t i = 0; i < recipient->select_source_count; i++) {
                if (recipient->select_sources[i].type == HIVE_SEL_IPC &&
                    entry_matches_filter(entry,
                                         &recipient->select_sources[i].ipc)) {
                    should_wake = true;
                    break;
                }
            }
            // Also wake on TIMER messages (could be timeout timer)
            if (!should_wake && entry->len >= HIVE_MSG_HEADER_SIZE) {
                hive_msg_class_t msg_class;
                decode_header(entry->data, &msg_class, NULL, NULL);
                if (msg_class == HIVE_MSG_TIMER) {
                    should_wake = true;
                }
            }
        } else {
            // No filter active - wake on any message
            should_wake = true;
        }

        if (should_wake) {
            recipient->state = ACTOR_STATE_READY;
        }
    }
}

// Unlink entry from mailbox_t (supports unlinking from middle)
static void mailbox_unlink(mailbox_t *mbox, mailbox_entry_t *entry) {
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

// Scan mailbox_t for message matching any of the filters
// Returns the matching entry and sets *matched_index to which filter matched
static mailbox_entry_t *
mailbox_find_match_any(mailbox_t *mbox, const hive_recv_filter_t *filters,
                       size_t num_filters, size_t *matched_index) {
    for (mailbox_entry_t *entry = mbox->head; entry; entry = entry->next) {
        for (size_t i = 0; i < num_filters; i++) {
            if (entry_matches_filter(entry, &filters[i])) {
                if (matched_index) {
                    *matched_index = i;
                }
                return entry;
            }
        }
    }
    return NULL;
}

// Dequeue the head entry from an actor_t's mailbox_t
mailbox_entry_t *hive_ipc_dequeue_head(actor_t *a) {
    if (!a || !a->mailbox.head) {
        return NULL;
    }
    mailbox_entry_t *entry = a->mailbox.head;
    mailbox_unlink(&a->mailbox, entry);
    return entry;
}

// Check for timeout message and handle it
hive_status_t hive_mailbox_handle_timeout(actor_t *current,
                                          hive_timer_id_t timeout_timer,
                                          const char *operation) {
    if (timeout_timer == HIVE_TIMER_ID_INVALID) {
        return HIVE_SUCCESS; // No timeout was set
    }

    // Check if first message is from OUR specific timeout timer
    if (current->mailbox.head &&
        current->mailbox.head->len >= HIVE_MSG_HEADER_SIZE) {
        hive_msg_class_t msg_class;
        uint32_t msg_tag;
        decode_header(current->mailbox.head->data, &msg_class, NULL, &msg_tag);

        if (msg_class == HIVE_MSG_TIMER && msg_tag == timeout_timer) {
            // This IS our timeout timer - dequeue, free, and return timeout
            // error
            mailbox_entry_t *entry = hive_ipc_dequeue_head(current);
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
// Core Notify/Receive
// -----------------------------------------------------------------------------

// Internal notify with explicit sender, class, id and tag (used by timer, link,
// etc.)
hive_status_t hive_ipc_notify_internal(hive_actor_id_t to,
                                       hive_actor_id_t sender,
                                       hive_msg_class_t class, uint16_t id,
                                       uint32_t tag, const void *data,
                                       size_t len) {
    actor_t *receiver = hive_actor_get(to);
    if (!receiver) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid receiver actor_t ID");
    }

    // Validate message size (payload + header)
    size_t total_len = len + HIVE_MSG_HEADER_SIZE;
    if (total_len > HIVE_MAX_MESSAGE_SIZE) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "Message exceeds HIVE_MAX_MESSAGE_SIZE");
    }

    // Check if current actor should block on pool exhaustion
    actor_t *current = hive_actor_current();
    bool should_block = current && current->pool_block;

    // System messages (TIMER, EXIT) can use reserved pool entries
    bool is_system_msg = (class == HIVE_MSG_TIMER || class == HIVE_MSG_EXIT);

    mailbox_entry_t *entry = NULL;
    message_data_entry_t *msg_data = NULL;

    // Allocation loop - retries if blocking is enabled
    while (true) {
        // Allocate mailbox_t entry from pool (respecting reserved entries)
        entry = hive_pool_alloc_reserved(
            &g_mailbox_pool_mgr, HIVE_RESERVED_SYSTEM_ENTRIES, is_system_msg);
        if (!entry) {
            if (should_block && !is_system_msg) {
                hive_scheduler_pool_wait();
                continue; // Retry after waking
            }
            return HIVE_ERROR(HIVE_ERR_NOMEM, "Mailbox entry pool exhausted");
        }

        // Allocate message data from pool (respecting reserved entries)
        msg_data = hive_pool_alloc_reserved(
            &g_message_pool_mgr, HIVE_RESERVED_SYSTEM_ENTRIES, is_system_msg);
        if (!msg_data) {
            hive_pool_free(&g_mailbox_pool_mgr, entry);
            if (should_block && !is_system_msg) {
                hive_scheduler_pool_wait();
                continue; // Retry after waking
            }
            return HIVE_ERROR(HIVE_ERR_NOMEM, "Message data pool exhausted");
        }

        // Both allocations succeeded
        break;
    }

    // Build message: header + payload
    encode_header(msg_data->data, class, id, tag);
    if (data && len > 0) {
        memcpy(msg_data->data + HIVE_MSG_HEADER_SIZE, data, len);
    }

    entry->sender = sender;
    entry->len = total_len;
    entry->data = msg_data->data;
    entry->next = NULL;
    entry->prev = NULL;

    // Add to receiver's mailbox_t and wake if blocked
    hive_mailbox_add_entry(receiver, entry);

    HIVE_LOG_TRACE("IPC: Message sent from %u to %u (class=%d, id=%u, tag=%u)",
                   sender, to, class, id, tag);
    return HIVE_SUCCESS;
}

hive_status_t hive_ipc_notify(hive_actor_id_t to, uint16_t id, const void *data,
                              size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *sender = hive_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    return hive_ipc_notify_internal(to, sender->id, HIVE_MSG_NOTIFY, id,
                                    HIVE_TAG_NONE, data, len);
}

hive_status_t hive_ipc_notify_ex(hive_actor_id_t to, hive_msg_class_t class,
                                 uint16_t id, const void *data, size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *sender = hive_actor_current();

    // Validate data pointer - NULL with len > 0 would cause memcpy crash
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    return hive_ipc_notify_internal(to, sender->id, class, id, HIVE_TAG_NONE,
                                    data, len);
}

hive_status_t hive_ipc_recv(hive_message_t *msg, int32_t timeout_ms) {
    // Wrapper around hive_select with wildcard IPC filter
    hive_select_source_t source = {.type = HIVE_SEL_IPC,
                                   .ipc = {.class = HIVE_MSG_ANY}};
    hive_select_result_t result;
    hive_status_t s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) {
        *msg = result.ipc;
    }
    return s;
}

hive_status_t hive_ipc_recv_match(hive_actor_id_t from, hive_msg_class_t class,
                                  uint16_t id, hive_message_t *msg,
                                  int32_t timeout_ms) {
    // Wrapper around hive_select with single IPC filter
    hive_select_source_t source = {
        .type = HIVE_SEL_IPC,
        .ipc = {.sender = from, .class = class, .id = id}};
    hive_select_result_t result;
    hive_status_t s = hive_select(&source, 1, &result, timeout_ms);
    if (HIVE_SUCCEEDED(s)) {
        *msg = result.ipc;
    }
    return s;
}

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

hive_status_t hive_ipc_request(hive_actor_id_t to, uint16_t id,
                               const void *request, size_t req_len,
                               hive_message_t *reply, int32_t timeout_ms) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    // Validate data pointer
    if (request == NULL && req_len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "NULL request with non-zero length");
    }

    // Set up temporary monitor to detect if target dies during request
    uint32_t monitor_id;
    hive_status_t status = hive_monitor(to, &monitor_id);
    if (HIVE_FAILED(status)) {
        // Target doesn't exist or is already dead
        return HIVE_ERROR(HIVE_ERR_CLOSED, "Target actor_t not found");
    }

    // Generate unique tag for this call
    uint32_t call_tag = generate_tag();

    // Dispatch HIVE_MSG_REQUEST with user id and generated tag
    status = hive_ipc_notify_internal(to, current->id, HIVE_MSG_REQUEST, id,
                                      call_tag, request, req_len);
    if (HIVE_FAILED(status)) {
        hive_demonitor(monitor_id);
        return status;
    }

    // Wait for REPLY or EXIT from target using hive_select
    // Match by tag for correlation (id doesn't matter for reply matching)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_IPC,
         .ipc = {.sender = to, .class = HIVE_MSG_REPLY, .tag = call_tag}},
        {.type = HIVE_SEL_IPC, .ipc = {.sender = to, .class = HIVE_MSG_EXIT}},
    };

    hive_select_result_t result;
    status = hive_select(sources, 2, &result, timeout_ms);
    hive_demonitor(monitor_id);

    if (HIVE_FAILED(status)) {
        return status;
    }

    if (result.index == 1) {
        // EXIT - target died
        return HIVE_ERROR(HIVE_ERR_CLOSED, "Target actor_t died");
    }

    // REPLY received - return to caller
    *reply = result.ipc;
    return HIVE_SUCCESS;
}

hive_status_t hive_ipc_reply(const hive_message_t *request, const void *data,
                             size_t len) {
    HIVE_REQUIRE_ACTOR_CONTEXT();
    actor_t *current = hive_actor_current();

    if (!request) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Invalid request message");
    }

    // Verify this is a REQUEST message
    if (request->class != HIVE_MSG_REQUEST) {
        return HIVE_ERROR(HIVE_ERR_INVALID,
                          "Can only reply to HIVE_MSG_REQUEST messages");
    }

    // Validate data pointer
    if (data == NULL && len > 0) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL data with non-zero length");
    }

    // Dispatch HIVE_MSG_REPLY with same id and tag back to caller
    return hive_ipc_notify_internal(request->sender, current->id,
                                    HIVE_MSG_REPLY, request->id, request->tag,
                                    data, len);
}

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

bool hive_msg_is_timer(const hive_message_t *msg) {
    if (!msg) {
        return false;
    }
    return msg->class == HIVE_MSG_TIMER;
}

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

bool hive_ipc_pending(void) {
    actor_t *current = hive_actor_current();
    if (!current) {
        return false;
    }
    return current->mailbox.head != NULL;
}

size_t hive_ipc_count(void) {
    actor_t *current = hive_actor_current();
    if (!current) {
        return 0;
    }
    return current->mailbox.count;
}

// -----------------------------------------------------------------------------
// Named IPC
// -----------------------------------------------------------------------------

hive_status_t hive_ipc_named_notify(const char *name, uint16_t id,
                                    const void *data, size_t len) {
    hive_actor_id_t target;
    hive_status_t status = hive_whereis(name, &target);
    if (HIVE_FAILED(status)) {
        return status;
    }
    return hive_ipc_notify(target, id, data, len);
}

hive_status_t hive_ipc_named_request(const char *name, uint16_t id,
                                     const void *request, size_t req_len,
                                     hive_message_t *reply,
                                     int32_t timeout_ms) {
    hive_actor_id_t target;
    hive_status_t status = hive_whereis(name, &target);
    if (HIVE_FAILED(status)) {
        return status;
    }
    return hive_ipc_request(target, id, request, req_len, reply, timeout_ms);
}

// -----------------------------------------------------------------------------
// Cleanup Functions
// -----------------------------------------------------------------------------

// Clear all entries from a mailbox_t (called during actor_t cleanup)
void hive_ipc_mailbox_clear(mailbox_t *mbox) {
    if (!mbox) {
        return;
    }

    mailbox_entry_t *entry = mbox->head;
    while (entry) {
        mailbox_entry_t *next = entry->next;
        hive_ipc_free_entry(entry);
        entry = next;
    }
    mbox->head = NULL;
    mbox->tail = NULL;
    mbox->count = 0;
}

// Free an active message entry (called during actor_t cleanup)
void hive_ipc_free_active_msg(mailbox_entry_t *entry) {
    hive_ipc_free_entry(entry);
}

// -----------------------------------------------------------------------------
// hive_select helpers
// -----------------------------------------------------------------------------

// Scan mailbox_t for message matching any of the filters (non-blocking)
mailbox_entry_t *hive_ipc_scan_mailbox(const hive_recv_filter_t *filters,
                                       size_t num_filters,
                                       size_t *matched_index) {
    actor_t *current = hive_actor_current();
    if (!current || !filters || num_filters == 0) {
        return NULL;
    }
    return mailbox_find_match_any(&current->mailbox, filters, num_filters,
                                  matched_index);
}

// Consume (unlink) a mailbox_t entry and decode into hive_message_t
void hive_ipc_consume_entry(mailbox_entry_t *entry, hive_message_t *msg) {
    actor_t *current = hive_actor_current();
    if (!current || !entry || !msg) {
        return;
    }

    // Auto-release previous active message if any
    if (current->active_msg) {
        hive_ipc_free_entry(current->active_msg);
        current->active_msg = NULL;
    }

    // Unlink from mailbox_t
    mailbox_unlink(&current->mailbox, entry);

    // Decode header and fill in message structure
    hive_msg_class_t msg_class;
    uint16_t msg_id;
    uint32_t msg_tag;
    decode_header(entry->data, &msg_class, &msg_id, &msg_tag);

    msg->sender = entry->sender;
    msg->class = msg_class;
    msg->id = msg_id;
    msg->tag = msg_tag;
    msg->len = entry->len - HIVE_MSG_HEADER_SIZE;
    msg->data = (const uint8_t *)entry->data + HIVE_MSG_HEADER_SIZE;

    // Store entry as active message for later cleanup
    current->active_msg = entry;
}
