#include "rt_actor.h"
#include "rt_static_config.h"
#include "rt_ipc.h"
#include "rt_log.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// Stack arena allocator for actor stacks
typedef struct arena_block {
    size_t size;              // Size of this free block (excluding header)
    struct arena_block *next; // Next free block in list
} arena_block;

typedef struct {
    uint8_t *base;
    size_t   total_size;
    arena_block *free_list;
} stack_arena;

// Stack alignment for x86-64 ABI
#define STACK_ALIGNMENT 16
#define MIN_BLOCK_SIZE 64

// Stack overflow detection
#define STACK_GUARD_PATTERN 0xDEADBEEFCAFEBABEULL
#define STACK_GUARD_SIZE 8  // sizeof(uint64_t)

// Static arena storage (16-byte aligned)
static uint8_t g_stack_arena_memory[RT_STACK_ARENA_SIZE] __attribute__((aligned(16)));
static stack_arena g_stack_arena = {0};

// Static actor storage
static actor g_actors[RT_MAX_ACTORS];

// Global actor table
static actor_table g_actor_table = {0};

// Current running actor
static actor *g_current_actor = NULL;

// Initialize stack arena
static void arena_init(void) {
    g_stack_arena.base = g_stack_arena_memory;
    g_stack_arena.total_size = RT_STACK_ARENA_SIZE;

    // Initialize with one large free block
    arena_block *block = (arena_block *)g_stack_arena.base;
    block->size = RT_STACK_ARENA_SIZE - sizeof(arena_block);
    block->next = NULL;
    g_stack_arena.free_list = block;
}

// Allocate from arena with 16-byte alignment
static void *arena_alloc(size_t size) {
    // Round size to alignment
    size = (size + STACK_ALIGNMENT - 1) & ~(STACK_ALIGNMENT - 1);

    // Search free list for first-fit
    arena_block **prev_ptr = &g_stack_arena.free_list;
    arena_block *curr = g_stack_arena.free_list;

    while (curr != NULL) {
        if (curr->size >= size) {
            // Found suitable block
            size_t remaining = curr->size - size;

            // Check if we should split the block
            if (remaining >= sizeof(arena_block) + MIN_BLOCK_SIZE) {
                // Split: allocate from beginning, create new free block
                arena_block *new_block = (arena_block *)((uint8_t *)curr + sizeof(arena_block) + size);
                new_block->size = remaining - sizeof(arena_block);
                new_block->next = curr->next;
                *prev_ptr = new_block;

                curr->size = size;
            } else {
                // Don't split, use entire block
                *prev_ptr = curr->next;
            }

            // Return usable space (after header)
            return (uint8_t *)curr + sizeof(arena_block);
        }

        prev_ptr = &curr->next;
        curr = curr->next;
    }

    return NULL; // No suitable block found
}

// Free to arena with coalescing
static void arena_free(void *ptr) {
    if (!ptr) {
        return;
    }

    // Get block header
    arena_block *block = (arena_block *)((uint8_t *)ptr - sizeof(arena_block));

    // Insert into free list (maintain address-sorted order) and coalesce
    arena_block **prev_ptr = &g_stack_arena.free_list;
    arena_block *curr = g_stack_arena.free_list;
    arena_block *prev_block = NULL;

    // Find insertion point
    while (curr != NULL && curr < block) {
        prev_block = curr;
        prev_ptr = &curr->next;
        curr = curr->next;
    }

    // Insert block
    block->next = curr;
    *prev_ptr = block;

    // Coalesce with previous block if adjacent
    if (prev_block != NULL) {
        uint8_t *prev_end = (uint8_t *)prev_block + sizeof(arena_block) + prev_block->size;
        if (prev_end == (uint8_t *)block) {
            // Merge with previous
            prev_block->size += sizeof(arena_block) + block->size;
            prev_block->next = block->next;
            block = prev_block;
        }
    }

    // Coalesce with next block if adjacent
    if (block->next != NULL) {
        uint8_t *block_end = (uint8_t *)block + sizeof(arena_block) + block->size;
        if (block_end == (uint8_t *)block->next) {
            // Merge with next
            arena_block *next = block->next;
            block->size += sizeof(arena_block) + next->size;
            block->next = next->next;
        }
    }
}

rt_status rt_actor_init(void) {
    // Initialize stack arena
    arena_init();

    // Use static actor array (already zero-initialized by C)
    g_actor_table.actors = g_actors;
    g_actor_table.max_actors = RT_MAX_ACTORS;
    g_actor_table.num_actors = 0;
    g_actor_table.next_id = 1; // Start at 1, 0 is ACTOR_ID_INVALID

    return RT_SUCCESS;
}

void rt_actor_cleanup(void) {
    if (g_actor_table.actors) {
        // Free all actor stacks and mailboxes
        for (size_t i = 0; i < g_actor_table.max_actors; i++) {
            actor *a = &g_actor_table.actors[i];
            if (a->state != ACTOR_STATE_DEAD && a->stack) {
                if (a->stack_is_malloced) {
                    free(a->stack);
                } else {
                    arena_free(a->stack);
                }
                rt_ipc_mailbox_clear(&a->mbox);
            }
        }
        // Note: g_actor_table.actors points to static g_actors array, so no free() needed
        g_actor_table.actors = NULL;
    }
}

actor *rt_actor_get(actor_id id) {
    if (id == ACTOR_ID_INVALID) {
        return NULL;
    }

    for (size_t i = 0; i < g_actor_table.max_actors; i++) {
        actor *a = &g_actor_table.actors[i];
        if (a->id == id && a->state != ACTOR_STATE_DEAD) {
            return a;
        }
    }

    return NULL;
}

actor *rt_actor_alloc(actor_fn fn, void *arg, const actor_config *cfg) {
    if (g_actor_table.num_actors >= g_actor_table.max_actors) {
        return NULL;
    }

    // Find free slot
    actor *a = NULL;
    for (size_t i = 0; i < g_actor_table.max_actors; i++) {
        if (g_actor_table.actors[i].state == ACTOR_STATE_DEAD ||
            g_actor_table.actors[i].id == ACTOR_ID_INVALID) {
            a = &g_actor_table.actors[i];
            break;
        }
    }

    if (!a) {
        return NULL;
    }

    // Determine stack size
    size_t stack_size = cfg->stack_size > 0 ? cfg->stack_size : RT_DEFAULT_STACK_SIZE;

    // Allocate stack (arena or malloc based on config)
    void *stack;
    bool is_malloced;

    if (cfg->malloc_stack) {
        // Explicitly requested malloc
        stack = malloc(stack_size);
        is_malloced = true;
    } else {
        // Use arena allocator (no fallback)
        stack = arena_alloc(stack_size);
        is_malloced = false;
    }

    if (!stack) {
        // Allocation failed (either malloc or arena exhausted)
        return NULL;
    }

    // Initialize stack guard patterns for overflow detection
    // Layout: [GUARD_LOW][usable stack][GUARD_HIGH]
    uint64_t *guard_low = (uint64_t *)stack;
    uint64_t *guard_high = (uint64_t *)((uint8_t *)stack + stack_size - STACK_GUARD_SIZE);
    *guard_low = STACK_GUARD_PATTERN;
    *guard_high = STACK_GUARD_PATTERN;

    // Initialize actor
    memset(a, 0, sizeof(actor));
    a->id = g_actor_table.next_id++;
    a->state = ACTOR_STATE_READY;
    a->priority = cfg->priority;
    a->name = cfg->name;
    a->stack = stack;
    a->stack_size = stack_size;
    a->stack_is_malloced = is_malloced;  // Track allocation method

    // Initialize context with usable stack area (excluding guards)
    void *usable_stack = (uint8_t *)stack + STACK_GUARD_SIZE;
    size_t usable_size = stack_size - (2 * STACK_GUARD_SIZE);
    rt_context_init(&a->ctx, usable_stack, usable_size, fn, arg);

    g_actor_table.num_actors++;

    return a;
}

// External cleanup functions
extern void rt_bus_cleanup_actor(actor_id id);
extern void rt_link_cleanup_actor(actor_id id);

void rt_actor_free(actor *a) {
    if (!a) {
        return;
    }

    // Cleanup links/monitors and send death notifications
    // This happens even on stack overflow - the guard pattern detection means
    // the overflow is localized to the stack, and actor metadata is intact
    rt_link_cleanup_actor(a->id);

    // Cleanup bus subscriptions
    rt_bus_cleanup_actor(a->id);

    // Free stack
    if (a->stack) {
        if (a->stack_is_malloced) {
            free(a->stack);
        } else {
            arena_free(a->stack);
        }
        a->stack = NULL;
    }

    // Free active message
    // If actor dies with an active SYNC message, unblock the sender
    if (a->active_msg) {
        if (a->active_msg->sync_ptr != NULL) {
            // This is a SYNC message - unblock the sender
            actor *sender = rt_actor_get(a->active_msg->sender);
            if (sender && sender->waiting_for_release && sender->blocked_on_actor == a->id) {
                // Receiver died - unblock sender with RT_ERR_CLOSED
                sender->waiting_for_release = false;
                sender->blocked_on_actor = ACTOR_ID_INVALID;
                sender->io_status = RT_ERROR(RT_ERR_CLOSED, "Receiver died without releasing");
                sender->state = ACTOR_STATE_READY;
            }
        }
        rt_ipc_free_active_msg(a->active_msg);
        a->active_msg = NULL;
    }

    // Free mailbox entries
    rt_ipc_mailbox_clear(&a->mbox);

    a->state = ACTOR_STATE_DEAD;
    g_actor_table.num_actors--;
}

actor *rt_actor_current(void) {
    return g_current_actor;
}

void rt_actor_set_current(actor *a) {
    g_current_actor = a;
}

// Get actor table (for scheduler)
actor_table *rt_actor_get_table(void) {
    return &g_actor_table;
}
