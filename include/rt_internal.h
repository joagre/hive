#ifndef RT_INTERNAL_H
#define RT_INTERNAL_H

#include "rt_static_config.h"
#include "rt_types.h"
#include "rt_timer.h"
#include "rt_actor.h"
#include "rt_io_source.h"
#include <stddef.h>
#include <stdbool.h>

// Internal shared types and macros for runtime implementation
// This header is NOT part of the public API

// Message data entry type (shared by IPC, bus, link, timer subsystems)
typedef struct {
    uint8_t data[RT_MAX_MESSAGE_SIZE];
} message_data_entry;

// Macro to extract message_data_entry pointer from data pointer
// Used when freeing message data back to the pool
#define DATA_TO_MSG_ENTRY(data_ptr) \
    ((message_data_entry*)((char*)(data_ptr) - offsetof(message_data_entry, data)))

// Initialization guard macro - early return if already initialized
// Used by: All subsystem init functions
#define RT_INIT_GUARD(initialized_flag) \
    do { \
        if (initialized_flag) { \
            return RT_SUCCESS; \
        } \
    } while(0)

// Cleanup guard macro - early return if not initialized
// Used by: All subsystem cleanup functions
#define RT_CLEANUP_GUARD(initialized_flag) \
    do { \
        if (!(initialized_flag)) { \
            return; \
        } \
    } while(0)

// Internal helper functions (implemented in rt_ipc.c)

// Add mailbox entry to actor's mailbox and wake if blocked
// Used by: IPC, link, timer subsystems
void rt_mailbox_add_entry(actor *recipient, mailbox_entry *entry);

// Check for timeout message in mailbox and dequeue if present
// Returns RT_ERR_TIMEOUT if timeout occurred, otherwise cancels timer and returns RT_SUCCESS
// Used by: IPC recv, network I/O
rt_status rt_mailbox_handle_timeout(actor *current, timer_id timeout_timer, const char *operation);

// Event loop handlers (called by scheduler when I/O sources become ready)

// Handle timer event (timerfd ready)
void rt_timer_handle_event(io_source *source);

// Handle network event (socket ready)
void rt_net_handle_event(io_source *source);

#endif // RT_INTERNAL_H
