#ifndef HIVE_TYPES_H
#define HIVE_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handles
typedef uint32_t actor_id;

#define ACTOR_ID_INVALID  ((actor_id)0)

// Wildcard sender for filtering (use with hive_ipc_recv_match)
#define HIVE_SENDER_ANY     ((actor_id)0xFFFFFFFF)

// Message header size (prepended to all messages)
#define HIVE_MSG_HEADER_SIZE 4

// Message classes (4 bits, stored in header bits 31-28)
typedef enum {
    HIVE_MSG_NOTIFY   = 0,  // Fire-and-forget notification
    HIVE_MSG_REQUEST = 1,  // Request expecting reply
    HIVE_MSG_REPLY   = 2,  // Reply to request
    HIVE_MSG_TIMER   = 3,  // Timer tick
    HIVE_MSG_EXIT    = 4,  // Exit notification (actor died)
    HIVE_MSG_ANY     = 15, // Wildcard for filtering
} hive_msg_class;

// Tag constants
#define HIVE_TAG_NONE        0           // No tag
#define HIVE_TAG_ANY         0x0FFFFFFF  // Wildcard for filtering

// Timeout constants for blocking operations
#define HIVE_TIMEOUT_INFINITE    ((int32_t)-1)  // Block forever
#define HIVE_TIMEOUT_NONBLOCKING ((int32_t)0)   // Return immediately

// Priority levels (lower value = higher priority)
typedef enum {
    HIVE_PRIORITY_CRITICAL = 0,
    HIVE_PRIORITY_HIGH     = 1,
    HIVE_PRIORITY_NORMAL   = 2,
    HIVE_PRIORITY_LOW      = 3,
    HIVE_PRIORITY_COUNT    = 4
} hive_priority_level;

// Error codes
typedef enum {
    HIVE_OK = 0,
    HIVE_ERR_NOMEM,
    HIVE_ERR_INVALID,
    HIVE_ERR_TIMEOUT,
    HIVE_ERR_CLOSED,
    HIVE_ERR_WOULDBLOCK,
    HIVE_ERR_IO,
} hive_error_code;

// Status with optional message
typedef struct {
    hive_error_code code;
    const char    *msg;   // string literal or NULL, never heap-allocated
} hive_status;

// Convenience macros
#define HIVE_SUCCESS ((hive_status){HIVE_OK, NULL})
#define HIVE_SUCCEEDED(s) ((s).code == HIVE_OK)
#define HIVE_FAILED(s) ((s).code != HIVE_OK)
#define HIVE_ERROR(c, m) ((hive_status){(c), (m)})
#define HIVE_ERR_STR(s) ((s).msg ? (s).msg : "unknown error")

// Actor entry point
typedef void (*actor_fn)(void *arg);

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    hive_priority_level priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
} actor_config;

// Default configuration
#define HIVE_ACTOR_CONFIG_DEFAULT { \
    .stack_size = 0, \
    .priority = HIVE_PRIORITY_NORMAL, \
    .name = NULL, \
    .malloc_stack = false \
}

// Message structure
typedef struct {
    actor_id       sender;       // Sender actor ID
    hive_msg_class class;        // Message class
    uint32_t       tag;          // Message tag
    size_t         len;          // Payload length (excludes 4-byte header)
    const void    *data;         // Payload pointer (past header), valid until next recv
} hive_message;

// Exit reason codes
typedef enum {
    HIVE_EXIT_NORMAL,       // Actor called hive_exit()
    HIVE_EXIT_CRASH,        // Actor function returned without calling hive_exit()
    HIVE_EXIT_CRASH_STACK,  // Stack overflow detected
    HIVE_EXIT_KILLED,       // Actor was killed externally (reserved for future use)
} hive_exit_reason;

#endif // HIVE_TYPES_H
