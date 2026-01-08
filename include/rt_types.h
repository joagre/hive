#ifndef RT_TYPES_H
#define RT_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handles
typedef uint32_t actor_id;

#define ACTOR_ID_INVALID  ((actor_id)0)

// Wildcard sender for filtering (use with rt_ipc_recv_match)
#define RT_SENDER_ANY     ((actor_id)0xFFFFFFFF)

// Message header size (prepended to all messages)
#define RT_MSG_HEADER_SIZE 4

// Message classes (4 bits, stored in header bits 31-28)
typedef enum {
    RT_MSG_CAST   = 0,  // Fire-and-forget message
    RT_MSG_CALL   = 1,  // Request expecting reply
    RT_MSG_REPLY  = 2,  // Response to call
    RT_MSG_TIMER  = 3,  // Timer tick
    RT_MSG_SYSTEM = 4,  // System message (exit notifications, etc.)
    RT_MSG_ANY    = 15, // Wildcard for filtering (use with rt_ipc_recv_match)
} rt_msg_class;

// Tag constants
#define RT_TAG_NONE        0           // No tag
#define RT_TAG_ANY         0x0FFFFFFF  // Wildcard for filtering
#define RT_TAG_GEN_BIT     0x08000000  // Bit 27: distinguishes generated tags
#define RT_TAG_VALUE_MASK  0x07FFFFFF  // Lower 27 bits: tag value

// Priority levels (lower value = higher priority)
typedef enum {
    RT_PRIO_CRITICAL = 0,
    RT_PRIO_HIGH     = 1,
    RT_PRIO_NORMAL   = 2,
    RT_PRIO_LOW      = 3,
    RT_PRIO_COUNT    = 4
} rt_priority;

// Error codes
typedef enum {
    RT_OK = 0,
    RT_ERR_NOMEM,
    RT_ERR_INVALID,
    RT_ERR_TIMEOUT,
    RT_ERR_CLOSED,
    RT_ERR_WOULDBLOCK,
    RT_ERR_IO,
} rt_status_code;

// Status with optional message
typedef struct {
    rt_status_code code;
    const char    *msg;   // string literal or NULL, never heap-allocated
} rt_status;

// Convenience macros
#define RT_SUCCESS ((rt_status){RT_OK, NULL})
#define RT_FAILED(s) ((s).code != RT_OK)
#define RT_ERROR(c, m) ((rt_status){(c), (m)})

// Actor entry point
typedef void (*actor_fn)(void *arg);

// Actor configuration
typedef struct {
    size_t      stack_size;   // bytes, 0 = default
    rt_priority priority;
    const char *name;         // for debugging, may be NULL
    bool        malloc_stack; // false = use static arena (default), true = malloc
} actor_config;

// Default configuration
#define RT_ACTOR_CONFIG_DEFAULT { \
    .stack_size = 0, \
    .priority = RT_PRIO_NORMAL, \
    .name = NULL, \
    .malloc_stack = false \
}

// Message structure
typedef struct {
    actor_id    sender;
    size_t      len;
    const void *data;   // Valid until next rt_ipc_recv() or rt_ipc_recv_match()
} rt_message;

// Exit reason codes
typedef enum {
    RT_EXIT_NORMAL,       // Actor called rt_exit()
    RT_EXIT_CRASH,        // Actor function returned without calling rt_exit()
    RT_EXIT_CRASH_STACK,  // Stack overflow detected
    RT_EXIT_KILLED,       // Actor was killed externally (reserved for future use)
} rt_exit_reason;

#endif // RT_TYPES_H
