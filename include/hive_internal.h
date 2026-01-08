#ifndef HIVE_INTERNAL_H
#define HIVE_INTERNAL_H

#include "hive_static_config.h"
#include "hive_types.h"
#include "hive_timer.h"
#include "hive_actor.h"
#include "hive_io_source.h"
#include <stddef.h>
#include <stdbool.h>

// Internal shared types and macros for runtime implementation
// This header is NOT part of the public API

// Internal tag constants (not exposed in public API)
#define HIVE_TAG_GEN_BIT     0x08000000  // Bit 27: distinguishes generated tags
#define HIVE_TAG_VALUE_MASK  0x07FFFFFF  // Lower 27 bits: tag value

// Message data entry type (shared by IPC, bus, link, timer subsystems)
typedef struct {
    uint8_t data[HIVE_MAX_MESSAGE_SIZE];
} message_data_entry;

// Macro to extract message_data_entry pointer from data pointer
// Used when freeing message data back to the pool
#define DATA_TO_MSG_ENTRY(data_ptr) \
    ((message_data_entry*)((char*)(data_ptr) - offsetof(message_data_entry, data)))

// -----------------------------------------------------------------------------
// Shared Linked List Macros
// -----------------------------------------------------------------------------

// Remove entry from singly-linked list using pointer-to-pointer pattern
// Usage: SLIST_REMOVE(head, entry_to_remove)
#define SLIST_REMOVE(head, entry_to_remove) \
    do { \
        __typeof__(head) *_prev = &(head); \
        while (*_prev && *_prev != (entry_to_remove)) { \
            _prev = &(*_prev)->next; \
        } \
        if (*_prev) { \
            *_prev = (*_prev)->next; \
        } \
    } while(0)

// Append entry to singly-linked list
// Usage: SLIST_APPEND(head, new_entry)
#define SLIST_APPEND(head, new_entry) \
    do { \
        (new_entry)->next = NULL; \
        if (head) { \
            __typeof__(head) _last = head; \
            while (_last->next) _last = _last->next; \
            _last->next = (new_entry); \
        } else { \
            (head) = (new_entry); \
        } \
    } while(0)

// Find and remove entry matching condition from singly-linked list
// Returns the removed entry or NULL if not found
// Usage: entry = SLIST_FIND_REMOVE(head, entry->field == value)
#define SLIST_FIND_REMOVE(head, condition, removed_entry) \
    do { \
        __typeof__(head) *_prev = &(head); \
        __typeof__(head) _curr = (head); \
        (removed_entry) = NULL; \
        while (_curr) { \
            __typeof__(head) entry = _curr; /* for use in condition */ \
            if (condition) { \
                *_prev = _curr->next; \
                (removed_entry) = _curr; \
                break; \
            } \
            _prev = &_curr->next; \
            _curr = _curr->next; \
        } \
    } while(0)

// Initialization guard macro - early return if already initialized
// Used by: All subsystem init functions
#define HIVE_INIT_GUARD(initialized_flag) \
    do { \
        if (initialized_flag) { \
            return HIVE_SUCCESS; \
        } \
    } while(0)

// Cleanup guard macro - early return if not initialized
// Used by: All subsystem cleanup functions
#define HIVE_CLEANUP_GUARD(initialized_flag) \
    do { \
        if (!(initialized_flag)) { \
            return; \
        } \
    } while(0)

// Actor context requirement macro - returns error if not in actor context
// Used by: IPC, link, bus, timer, network APIs
#define HIVE_REQUIRE_ACTOR_CONTEXT() \
    do { \
        if (!hive_actor_current()) { \
            return HIVE_ERROR(HIVE_ERR_INVALID, "Not called from actor context"); \
        } \
    } while(0)

// Internal helper functions (implemented in hive_ipc.c)

// Free message data back to the shared message pool
// Handles NULL safely. Used by: IPC, bus, link subsystems
void hive_msg_pool_free(void *data);

// Add mailbox entry to actor's mailbox and wake if blocked
// Used by: timer, link subsystems (via hive_ipc_notify_ex)
void hive_mailbox_add_entry(actor *recipient, mailbox_entry *entry);

// Check for timeout message in mailbox and dequeue if present
// Returns HIVE_ERR_TIMEOUT if timeout occurred, otherwise cancels timer and returns HIVE_SUCCESS
// Used by: IPC recv, network I/O, bus
hive_status hive_mailbox_handle_timeout(actor *current, timer_id timeout_timer, const char *operation);

// Free a mailbox entry and its associated data buffers
// Used by: IPC, mailbox clear, actor cleanup
void hive_ipc_free_entry(mailbox_entry *entry);

// Dequeue the head entry from an actor's mailbox
// Returns NULL if mailbox is empty
// Used by: IPC recv, timeout handling, bus
mailbox_entry *hive_ipc_dequeue_head(actor *a);

// Actor crash handler (called when actor returns without hive_exit)
// Sets HIVE_EXIT_CRASH and yields to scheduler - never returns
_Noreturn void hive_exit_crash(void);

// Event loop handlers (called by scheduler when I/O sources become ready)

// Handle timer event (timerfd ready)
void hive_timer_handle_event(io_source *source);

// Handle network event (socket ready)
void hive_net_handle_event(io_source *source);

// Clear mailbox entries (used during actor cleanup)
void hive_ipc_mailbox_clear(mailbox *mailbox);

// Free active message entry (used during actor cleanup)
void hive_ipc_free_active_msg(mailbox_entry *entry);

// Send with explicit class and tag (used internally by timer, link, etc.)
hive_status hive_ipc_notify_ex(actor_id to, actor_id sender, hive_msg_class class,
                               uint32_t tag, const void *data, size_t len);

#endif // HIVE_INTERNAL_H
