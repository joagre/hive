#ifndef ACRT_TYPES_H
#define ACRT_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handles
typedef uint32_t actor_id;

#define ACTOR_ID_INVALID  ((actor_id)0)

// Wildcard sender for filtering (use with acrt_ipc_recv_match)
#define ACRT_SENDER_ANY     ((actor_id)0xFFFFFFFF)

// Message header size (prepended to all messages)
#define ACRT_MSG_HEADER_SIZE 4

// Message classes (4 bits, stored in header bits 31-28)
typedef enum {
    ACRT_MSG_ASYNC   = 0,  // Async message (sender doesn't wait)
    ACRT_MSG_REQUEST = 1,  // Request expecting reply
    ACRT_MSG_REPLY   = 2,  // Reply to request
    ACRT_MSG_TIMER   = 3,  // Timer tick
    ACRT_MSG_EXIT    = 4,  // Exit notification (actor died)
    ACRT_MSG_ANY     = 15, // Wildcard for filtering
} acrt_msg_class;

// Tag constants
#define ACRT_TAG_NONE        0           // No tag
#define ACRT_TAG_ANY         0x0FFFFFFF  // Wildcard for filtering

// Timeout constants for blocking operations
#define ACRT_TIMEOUT_INFINITE    ((int32_t)-1)  // Block forever
#define ACRT_TIMEOUT_NONBLOCKING ((int32_t)0)   // Return immediately

// Priority levels (lower value = higher priority)
typedef enum {
    ACRT_PRIORITY_CRITICAL = 0,
    ACRT_PRIORITY_HIGH     = 1,
    ACRT_PRIORITY_NORMAL   = 2,
    ACRT_PRIORITY_LOW      = 3,
    ACRT_PRIORITY_COUNT    = 4
} acrt_priority_level;

// Error codes
typedef enum {
    ACRT_OK = 0,
    ACRT_ERR_NOMEM,
    ACRT_ERR_INVALID,
    ACRT_ERR_TIMEOUT,
    ACRT_ERR_CLOSED,
    ACRT_ERR_WOULDBLOCK,
    ACRT_ERR_IO,
} acrt_error_code;

// Status with optional message
typedef struct {
    acrt_error_code code;
    const char    *msg;   // string literal or NULL, never heap-allocated
} acrt_status;

// Convenience macros
#define ACRT_SUCCESS ((acrt_status){ACRT_OK, NULL})
#define ACRT_FAILED(s) ((s).code != ACRT_OK)
#define ACRT_ERROR(c, m) ((acrt_status){(c), (m)})

// Actor entry point
typedef void (*actor_fn)(void *arg);

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    acrt_priority_level priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
} actor_config;

// Default configuration
#define ACRT_ACTOR_CONFIG_DEFAULT { \
    .stack_size = 0, \
    .priority = ACRT_PRIORITY_NORMAL, \
    .name = NULL, \
    .malloc_stack = false \
}

// Message structure
typedef struct {
    actor_id    sender;
    size_t      len;
    const void *data;   // Valid until next acrt_ipc_recv() or acrt_ipc_recv_match()
} acrt_message;

// Exit reason codes
typedef enum {
    ACRT_EXIT_NORMAL,       // Actor called acrt_exit()
    ACRT_EXIT_CRASH,        // Actor function returned without calling acrt_exit()
    ACRT_EXIT_CRASH_STACK,  // Stack overflow detected
    ACRT_EXIT_KILLED,       // Actor was killed externally (reserved for future use)
} acrt_exit_reason;

#endif // ACRT_TYPES_H
