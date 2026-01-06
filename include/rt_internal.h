#ifndef RT_INTERNAL_H
#define RT_INTERNAL_H

#include "rt_static_config.h"
#include "rt_types.h"
#include "rt_timer.h"
#include "rt_actor.h"
#include "rt_spsc.h"
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

// Initialize SPSC queue pair (request + completion) with error handling
// Used by: Timer, file I/O, network I/O subsystems
#define RT_INIT_SPSC_QUEUES(req_queue, req_buf, req_type, comp_queue, comp_buf, comp_type) \
    do { \
        rt_status _status = rt_spsc_init(&(req_queue), (req_buf), \
                                          sizeof(req_type), RT_COMPLETION_QUEUE_SIZE); \
        if (RT_FAILED(_status)) { \
            return _status; \
        } \
        _status = rt_spsc_init(&(comp_queue), (comp_buf), \
                               sizeof(comp_type), RT_COMPLETION_QUEUE_SIZE); \
        if (RT_FAILED(_status)) { \
            rt_spsc_destroy(&(req_queue)); \
            return _status; \
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

// Push to SPSC queue with blocking retry (yields until successful)
// Used by: Timer, file I/O, network I/O, bus subsystems
void rt_spsc_push_blocking(rt_spsc_queue *queue, const void *entry);

#endif // RT_INTERNAL_H
