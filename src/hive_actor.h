#ifndef HIVE_ACTOR_H
#define HIVE_ACTOR_H

#include "hive_types.h"
#include "hive_context.h"

// Actor states
typedef enum {
    ACTOR_STATE_DEAD = 0, // Terminated (must be 0 for calloc initialization)
    ACTOR_STATE_READY,    // Ready to run
    ACTOR_STATE_RUNNING,  // Currently executing
    ACTOR_STATE_WAITING,  // Waiting for I/O (IPC, timer, TCP, etc.)
} actor_state_t;

// Mailbox entry (linked list)
typedef struct mailbox_entry_t {
    hive_actor_id_t sender;
    size_t len;
    void *data;
    struct mailbox_entry_t *next;
    struct mailbox_entry_t *prev; // For unlinking in selective receive
} mailbox_entry_t;

// Mailbox
typedef struct {
    mailbox_entry_t *head;
    mailbox_entry_t *tail;
    size_t count;
} mailbox_t;

// Link entry (bidirectional relationship)
typedef struct link_entry_t {
    hive_actor_id_t target;
    struct link_entry_t *next;
} link_entry_t;

// Monitor entry (unidirectional relationship)
typedef struct monitor_entry_t {
    uint32_t ref;
    hive_actor_id_t target;
    struct monitor_entry_t *next;
} monitor_entry_t;

// Actor control block
typedef struct {
    hive_actor_id_t id;
    actor_state_t state;
    hive_priority_level_t priority;
    const char *name;

    // Context and stack
    hive_context_t ctx;
    void *stack;
    size_t stack_size;
    bool stack_is_malloced; // true if malloc'd, false if from pool

    // Startup info (used by context_entry to call actor function)
    void *startup_args;                        // Arguments from init or direct
    const hive_spawn_info_t *startup_siblings; // Sibling info array
    size_t startup_sibling_count;              // Number of siblings
    hive_spawn_info_t
        self_spawn_info; // This actor's own spawn info (for standalone spawns)

    // Mailbox
    mailbox_t mailbox;

    // Active message (for proper cleanup)
    mailbox_entry_t *active_msg;

    // For selective receive: filter array to match against (multi-pattern)
    const hive_recv_filter_t *recv_filters; // NULL = no filter active
    size_t recv_filter_count;               // Number of filters in array

    // For hive_select: multi-source wait (IPC + bus)
    const hive_select_source_t *select_sources; // NULL = not in select
    size_t select_source_count;                 // Number of sources in array

    // For I/O completion results
    hive_status_t io_status;
    int io_result_fd;       // For file_open
    size_t io_result_bytes; // For file read/write

    // Links and monitors
    link_entry_t *links;            // Bidirectional links to other actors
    monitor_entry_t *monitors;      // Actors we are monitoring (unidirectional)
    hive_exit_reason_t exit_reason; // Why this actor exited

    // Pool exhaustion behavior
    bool pool_block_default; // From hive_actor_config_t at spawn time
    bool pool_block; // Effective value now (can be overridden at runtime)
} actor_t;

// Actor table - global storage for all actors
typedef struct {
    actor_t *actors;         // Array of actors
    size_t max_actors;       // Maximum number of actors
    size_t num_actors;       // Current number of live actors
    hive_actor_id_t next_id; // Next actor ID to assign
} actor_table_t;

// Initialize actor subsystem
hive_status_t hive_actor_init(void);

// Cleanup actor subsystem
void hive_actor_cleanup(void);

// Get actor by ID
actor_t *hive_actor_get(hive_actor_id_t id);

// Allocate a new actor
// fn: actor function
// args: arguments to pass to actor (from init function or direct)
// siblings: sibling info array (stack-allocated by spawner, copied to actor's stack)
// sibling_count: number of siblings
// cfg: actor configuration
actor_t *hive_actor_alloc(hive_actor_fn_t fn, void *args,
                          const hive_spawn_info_t *siblings,
                          size_t sibling_count, const hive_actor_config_t *cfg);

// Free an actor
void hive_actor_free(actor_t *a);

// Get current actor (must be called from within an actor)
actor_t *hive_actor_current(void);

// Set current actor (used by scheduler)
void hive_actor_set_current(actor_t *a);

// Stack profiling functions are declared in hive_runtime.h (public API):
//   size_t hive_actor_stack_size(hive_actor_id_t id);
//   size_t hive_actor_stack_usage(hive_actor_id_t id);
//   void hive_actor_stack_usage_all(hive_stack_usage_callback_t cb);

#endif // HIVE_ACTOR_H
