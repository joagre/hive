#ifndef HIVE_TYPES_H
#define HIVE_TYPES_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Opaque handles
typedef uint32_t hive_actor_id_t;
typedef uint32_t hive_bus_id_t;
typedef uint32_t hive_timer_id_t;
typedef uint8_t hive_hal_event_id_t;

#define HIVE_ACTOR_ID_INVALID ((hive_actor_id_t)0)
#define HIVE_HAL_EVENT_INVALID 0xFF
#define HIVE_HAL_EVENT_MAX 32

// Wildcard constants for filtering (all wildcards = 0 for designated initializer)
#define HIVE_SENDER_ANY ((hive_actor_id_t)0)
#define HIVE_ID_ANY 0
#define HIVE_TAG_ANY 0

// Explicit "no value" constants (same as wildcards, different semantic intent)
#define HIVE_ID_NONE 0
#define HIVE_TAG_NONE 0

// Message header size (prepended to all messages)
// Layout: [class:4][tag:28] [id:16] = 6 bytes
#define HIVE_MSG_HEADER_SIZE 6

// Message classes (4 bits, stored in header bits 31-28)
// HIVE_MSG_ANY = 0 enables wildcard via C designated initializer defaults
typedef enum {
    HIVE_MSG_ANY = 0,     // Wildcard for filtering
    HIVE_MSG_NOTIFY = 1,  // Fire-and-forget notification
    HIVE_MSG_REQUEST = 2, // Request expecting reply
    HIVE_MSG_REPLY = 3,   // Reply to request
    HIVE_MSG_TIMER = 4,   // Timer tick
    HIVE_MSG_EXIT = 5,    // Exit notification (actor died)
} hive_msg_class_t;

// Timeout constants for blocking operations
#define HIVE_TIMEOUT_INFINITE ((int32_t) - 1) // Block forever
#define HIVE_TIMEOUT_NONBLOCKING ((int32_t)0) // Return immediately

// Priority levels (lower value = higher priority)
typedef enum {
    HIVE_PRIORITY_CRITICAL = 0,
    HIVE_PRIORITY_HIGH = 1,
    HIVE_PRIORITY_NORMAL = 2,
    HIVE_PRIORITY_LOW = 3,
    HIVE_PRIORITY_COUNT = 4
} hive_priority_level_t;

// Error codes
typedef enum {
    HIVE_OK = 0,
    HIVE_ERR_NOMEM,
    HIVE_ERR_INVALID,
    HIVE_ERR_TIMEOUT,
    HIVE_ERR_CLOSED,
    HIVE_ERR_WOULDBLOCK,
    HIVE_ERR_INPROGRESS, // Async operation in progress (e.g., connect)
    HIVE_ERR_IO,
    HIVE_ERR_EXISTS,    // Name already registered (for auto_register failure)
    HIVE_ERR_TRUNCATED, // Data truncated to fit buffer (bus read)
    HIVE_ERR_NOT_FOUND, // Name not registered in name registry
} hive_error_code_t;

// Status with optional message
typedef struct {
    hive_error_code_t code;
    const char *msg; // string literal or NULL, never heap-allocated
} hive_status_t;

// Convenience macros
#define HIVE_SUCCESS ((hive_status_t){HIVE_OK, NULL})
#define HIVE_SUCCEEDED(s) ((s).code == HIVE_OK)
#define HIVE_FAILED(s) ((s).code != HIVE_OK)
#define HIVE_ERROR(c, m) ((hive_status_t){(c), (m)})
#define HIVE_ERR_STR(s) ((s).msg ? (s).msg : "unknown error")

// Forward declaration for spawn info
typedef struct hive_spawn_info hive_spawn_info_t;

// Actor entry point (receives args, sibling info array, and count)
typedef void (*hive_actor_fn_t)(void *args, const hive_spawn_info_t *siblings,
                                size_t sibling_count);

// Actor init function: transforms init_args before actor runs
// Called in spawner context. Return value becomes args to actor function.
// Returning NULL is valid (actor receives NULL args).
typedef void *(*hive_actor_init_fn_t)(void *init_args);

// Info about a spawned actor (passed to actor function)
struct hive_spawn_info {
    const char *name;   // Actor name (NULL if unnamed)
    hive_actor_id_t id; // Actor ID
    bool registered;    // Whether registered in name registry
};

// Actor configuration
typedef struct {
    size_t stack_size; // bytes, 0 = default
    hive_priority_level_t priority;
    const char *name;   // for debugging AND registry (if auto_register)
    bool malloc_stack;  // false = use static arena (default), true = malloc
    bool auto_register; // Register name in registry (requires name != NULL)
    bool pool_block;    // false = return NOMEM (default), true = block on pool
    hive_actor_id_t reuse_actor_id; // 0 = allocate new, non-zero = reuse slot
} hive_actor_config_t;

// Default configuration
#define HIVE_ACTOR_CONFIG_DEFAULT                                           \
    {                                                                       \
        .stack_size = 0, .priority = HIVE_PRIORITY_NORMAL, .name = NULL,    \
        .malloc_stack = false, .auto_register = false, .pool_block = false, \
        .reuse_actor_id = 0                                                 \
    }

// Message structure
typedef struct {
    hive_actor_id_t sender; // Sender actor ID
    hive_msg_class_t class; // Message class
    uint16_t id;            // Message type (user-provided for dispatch)
    uint32_t tag;           // Correlation tag (internal for request/reply)
    size_t len;             // Payload length (excludes 6-byte header)
    const void *data; // Payload pointer (past header), valid until next recv
} hive_message_t;

// Filter for selective receive (used by hive_select)
// All wildcards = 0, so unspecified fields in designated initializers match any.
// Example: {.sender = actor, .class = HIVE_MSG_NOTIFY} matches any id/tag from actor.
typedef struct {
    hive_actor_id_t sender; // HIVE_SENDER_ANY (0) for any sender
    hive_msg_class_t class; // HIVE_MSG_ANY (0) for any class
    uint16_t id;            // HIVE_ID_ANY (0) for any id
    uint32_t tag;           // HIVE_TAG_ANY (0) for any tag
} hive_recv_filter_t;

// Exit reason codes
// Reserved values use high range (0xFFFC-0xFFFF), leaving 0-0xFFFB for app-defined
#define HIVE_EXIT_REASON_NORMAL 0xFFFF // Normal termination
#define HIVE_EXIT_REASON_KILLED \
    0xFFFE                            // Killed externally via hive_actor_kill()
#define HIVE_EXIT_REASON_CRASH 0xFFFD // Abnormal termination (app-signaled)
#define HIVE_EXIT_REASON_STACK_OVERFLOW \
    0xFFFC // Reserved for future MPU detection
typedef uint16_t hive_exit_reason_t;

// Pool exhaustion blocking mode
typedef enum {
    HIVE_POOL_NO_BLOCK, // Force non-blocking (return NOMEM on exhaustion)
    HIVE_POOL_BLOCK,    // Force blocking (yield until pool available)
    HIVE_POOL_DEFAULT   // Restore spawn default
} hive_pool_block_t;

// -----------------------------------------------------------------------------
// Select Types (for hive_select unified event API)
// -----------------------------------------------------------------------------

// Select source types
typedef enum {
    HIVE_SEL_IPC,       // Wait for IPC message matching filter
    HIVE_SEL_BUS,       // Wait for data on bus
    HIVE_SEL_HAL_EVENT, // Wait for HAL event (from ISR)
} hive_select_type_t;

// Select source (tagged union)
typedef struct {
    hive_select_type_t type;
    union {
        hive_recv_filter_t ipc; // For HIVE_SEL_IPC: message filter
        hive_bus_id_t bus;      // For HIVE_SEL_BUS: bus ID
        uint8_t event;          // For HIVE_SEL_HAL_EVENT: event ID
    };
} hive_select_source_t;

// Select result
typedef struct {
    size_t index;            // Which source triggered (0-based index in array)
    hive_select_type_t type; // Type of triggered source (convenience copy)
    union {
        hive_message_t ipc; // For HIVE_SEL_IPC: the received message
        struct {
            void *data; // For HIVE_SEL_BUS: pointer to bus data
            size_t len; // Length of bus data
        } bus;
    };
} hive_select_result_t;

// -----------------------------------------------------------------------------
// Sibling Helper Function
// -----------------------------------------------------------------------------

// Find a sibling actor by name in the spawn info array
// Returns the hive_actor_id_t if found, or HIVE_ACTOR_ID_INVALID if not found
hive_actor_id_t hive_find_sibling(const hive_spawn_info_t *siblings,
                                  size_t count, const char *name);

#endif // HIVE_TYPES_H
