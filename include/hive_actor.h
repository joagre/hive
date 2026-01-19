#ifndef HIVE_ACTOR_H
#define HIVE_ACTOR_H

#include "hive_types.h"
#include "hive_context.h"

// Actor states
typedef enum {
    ACTOR_STATE_DEAD = 0, // Terminated (must be 0 for calloc initialization)
    ACTOR_STATE_READY,    // Ready to run
    ACTOR_STATE_RUNNING,  // Currently executing
    ACTOR_STATE_WAITING,  // Waiting for I/O (IPC, timer, network, etc.)
} actor_state;

// Mailbox entry (linked list)
typedef struct mailbox_entry {
    actor_id sender;
    size_t len;
    void *data;
    struct mailbox_entry *next;
    struct mailbox_entry *prev; // For unlinking in selective receive
} mailbox_entry;

// Mailbox
typedef struct {
    mailbox_entry *head;
    mailbox_entry *tail;
    size_t count;
} mailbox;

// Link entry (bidirectional relationship)
typedef struct link_entry {
    actor_id target;
    struct link_entry *next;
} link_entry;

// Monitor entry (unidirectional relationship)
typedef struct monitor_entry {
    uint32_t ref;
    actor_id target;
    struct monitor_entry *next;
} monitor_entry;

// Actor control block
typedef struct {
    actor_id id;
    actor_state state;
    hive_priority_level priority;
    const char *name;

    // Context and stack
    hive_context ctx;
    void *stack;
    size_t stack_size;
    bool stack_is_malloced; // true if malloc'd, false if from pool

    // Startup info (used by context_entry to call actor function)
    void *startup_args;                      // Arguments from init or direct
    const hive_spawn_info *startup_siblings; // Sibling info array
    size_t startup_sibling_count;            // Number of siblings
    hive_spawn_info
        self_spawn_info; // This actor's own spawn info (for standalone spawns)

    // Mailbox
    mailbox mailbox;

    // Active message (for proper cleanup)
    mailbox_entry *active_msg;

    // For selective receive: filter array to match against (multi-pattern)
    const hive_recv_filter *recv_filters; // NULL = no filter active
    size_t recv_filter_count;             // Number of filters in array

    // For hive_select: multi-source wait (IPC + bus)
    const hive_select_source *select_sources; // NULL = not in select
    size_t select_source_count;               // Number of sources in array

    // For I/O completion results
    hive_status io_status;
    int io_result_fd;       // For file_open
    size_t io_result_bytes; // For file read/write

    // Links and monitors
    link_entry *links;            // Bidirectional links to other actors
    monitor_entry *monitors;      // Actors we are monitoring (unidirectional)
    hive_exit_reason exit_reason; // Why this actor exited
} actor;

// Actor table - global storage for all actors
typedef struct {
    actor *actors;     // Array of actors
    size_t max_actors; // Maximum number of actors
    size_t num_actors; // Current number of live actors
    actor_id next_id;  // Next actor ID to assign
} actor_table;

// Initialize actor subsystem
hive_status hive_actor_init(void);

// Cleanup actor subsystem
void hive_actor_cleanup(void);

// Get actor by ID
actor *hive_actor_get(actor_id id);

// Allocate a new actor
// fn: actor function
// args: arguments to pass to actor (from init function or direct)
// siblings: sibling info array (stack-allocated by spawner, copied to actor's stack)
// sibling_count: number of siblings
// cfg: actor configuration
actor *hive_actor_alloc(actor_fn fn, void *args,
                        const hive_spawn_info *siblings, size_t sibling_count,
                        const actor_config *cfg);

// Free an actor
void hive_actor_free(actor *a);

// Get current actor (must be called from within an actor)
actor *hive_actor_current(void);

// Set current actor (used by scheduler)
void hive_actor_set_current(actor *a);

// Stack watermarking functions (enabled when HIVE_STACK_WATERMARK=1)
// Returns bytes used if watermarking enabled, or stack_size if disabled
size_t hive_actor_stack_usage(actor_id id);

// Callback for hive_actor_stack_usage_all()
typedef void (*stack_usage_callback)(actor_id id, const char *name,
                                     size_t stack_size, size_t used);

// Iterate all live actors and report their stack usage via callback
void hive_actor_stack_usage_all(stack_usage_callback cb);

#endif // HIVE_ACTOR_H
