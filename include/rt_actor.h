#ifndef RT_ACTOR_H
#define RT_ACTOR_H

#include "rt_types.h"
#include "rt_context.h"

// Actor states
typedef enum {
    ACTOR_STATE_DEAD = 0,   // Terminated (must be 0 for calloc initialization)
    ACTOR_STATE_READY,      // Ready to run
    ACTOR_STATE_RUNNING,    // Currently executing
    ACTOR_STATE_WAITING,    // Waiting for I/O (IPC, timer, network, etc.)
} actor_state;

// Mailbox entry (linked list)
typedef struct mailbox_entry {
    actor_id                sender;
    size_t                  len;
    void                   *data;
    struct mailbox_entry   *next;
    struct mailbox_entry   *prev;  // For unlinking in selective receive
} mailbox_entry;

// Mailbox
typedef struct {
    mailbox_entry *head;
    mailbox_entry *tail;
    size_t         count;
} mailbox;

// Link entry (bidirectional relationship)
typedef struct link_entry {
    actor_id            target;
    struct link_entry  *next;
} link_entry;

// Monitor entry (unidirectional relationship)
typedef struct monitor_entry {
    uint32_t               ref;
    actor_id               target;
    struct monitor_entry  *next;
} monitor_entry;

// Actor control block
typedef struct {
    actor_id       id;
    actor_state    state;
    rt_priority    priority;
    const char    *name;

    // Context and stack
    rt_context     ctx;
    void          *stack;
    size_t         stack_size;
    bool           stack_is_malloced; // true if malloc'd, false if from pool

    // Mailbox
    mailbox        mailbox;

    // Active message (for proper cleanup)
    mailbox_entry *active_msg;

    // For selective receive: filters to match against
    actor_id       recv_filter_from;
    rt_msg_class   recv_filter_class;
    uint32_t       recv_filter_tag;

    // For I/O completion results
    rt_status      io_status;
    int            io_result_fd;      // For file_open
    size_t         io_result_nbytes;  // For file read/write

    // Links and monitors
    link_entry    *links;        // Bidirectional links to other actors
    monitor_entry *monitors;     // Actors we are monitoring (unidirectional)
    rt_exit_reason exit_reason;  // Why this actor exited
} actor;

// Actor table - global storage for all actors
typedef struct {
    actor  *actors;        // Array of actors
    size_t  max_actors;    // Maximum number of actors
    size_t  num_actors;    // Current number of live actors
    actor_id next_id;      // Next actor ID to assign
} actor_table;

// Initialize actor subsystem
rt_status rt_actor_init(void);

// Cleanup actor subsystem
void rt_actor_cleanup(void);

// Get actor by ID
actor *rt_actor_get(actor_id id);

// Allocate a new actor
actor *rt_actor_alloc(actor_fn fn, void *arg, const actor_config *cfg);

// Free an actor
void rt_actor_free(actor *a);

// Get current actor (must be called from within an actor)
actor *rt_actor_current(void);

// Set current actor (used by scheduler)
void rt_actor_set_current(actor *a);

#endif // RT_ACTOR_H
