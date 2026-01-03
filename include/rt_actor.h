#ifndef RT_ACTOR_H
#define RT_ACTOR_H

#include "rt_types.h"
#include "rt_context.h"

// Actor states
typedef enum {
    ACTOR_STATE_DEAD = 0,   // Terminated (must be 0 for calloc initialization)
    ACTOR_STATE_READY,      // Ready to run
    ACTOR_STATE_RUNNING,    // Currently executing
    ACTOR_STATE_BLOCKED,    // Blocked on IPC receive
} actor_state;

// Mailbox entry (linked list)
typedef struct mailbox_entry {
    actor_id                sender;
    size_t                  len;
    void                   *data;       // NULL for borrowed messages
    const void             *borrow_ptr; // Non-NULL for borrowed messages
    struct mailbox_entry   *next;
} mailbox_entry;

// Mailbox
typedef struct {
    mailbox_entry *head;
    mailbox_entry *tail;
    size_t         count;
} mailbox;

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

    // Mailbox
    mailbox        mbox;

    // For blocked IPC_BORROW sends
    bool           waiting_for_release;
    actor_id       blocked_on_actor;

    // For I/O completion results
    rt_status      io_status;
    int            io_result_fd;      // For file_open
    size_t         io_result_nbytes;  // For file read/write
} actor;

// Actor table - global storage for all actors
typedef struct {
    actor  *actors;        // Array of actors
    size_t  max_actors;    // Maximum number of actors
    size_t  num_actors;    // Current number of live actors
    actor_id next_id;      // Next actor ID to assign
} actor_table;

// Initialize actor subsystem
rt_status rt_actor_init(size_t max_actors);

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
