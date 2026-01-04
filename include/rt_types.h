#ifndef RT_TYPES_H
#define RT_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handles
typedef uint32_t actor_id;

#define ACTOR_ID_INVALID  ((actor_id)0)

// Special sender IDs
#define RT_SENDER_TIMER   ((actor_id)0xFFFFFFFF)
#define RT_SENDER_SYSTEM  ((actor_id)0xFFFFFFFE)

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
} actor_config;

// Default configuration
#define RT_ACTOR_CONFIG_DEFAULT { \
    .stack_size = 0, \
    .priority = RT_PRIO_NORMAL, \
    .name = NULL \
}

// Message structure
typedef struct {
    actor_id    sender;
    size_t      len;
    const void *data;   // valid until rt_ipc_release() or next rt_ipc_recv()
} rt_message;

// IPC send mode
typedef enum {
    IPC_COPY,    // memcpy payload to receiver's mailbox
    IPC_BORROW,  // zero-copy, sender blocks until receiver consumes
} rt_ipc_mode;

// Exit reason codes
typedef enum {
    RT_EXIT_NORMAL,    // Actor called rt_exit()
    RT_EXIT_CRASH,     // Actor function returned without calling rt_exit()
    RT_EXIT_KILLED,    // Actor was killed externally (reserved for future use)
} rt_exit_reason;

#endif // RT_TYPES_H
