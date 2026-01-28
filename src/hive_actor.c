#include "hive_actor.h"
#include "hive_static_config.h"
#include "hive_internal.h"
#include "hive_scheduler.h"
#include "hive_log.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

// Stack arena allocator for actor_t stacks
typedef struct arena_block_t {
    size_t size;                // Size of this free block (excluding header)
    struct arena_block_t *next; // Next free block in list
} arena_block_t;

typedef struct {
    uint8_t *base;
    size_t total_size;
    arena_block_t *free_list;
} stack_arena_t;

// Stack alignment for x86-64 ABI
#define STACK_ALIGNMENT 16
#define MIN_BLOCK_SIZE 64

// Static arena storage (16-byte aligned)
static uint8_t s_stack_arena_memory[HIVE_STACK_ARENA_SIZE]
    __attribute__((aligned(16)));
static stack_arena_t s_stack_arena = {0};

// Static actor_t storage
static actor_t s_actors[HIVE_MAX_ACTORS];

// Static actor_t table
static actor_table_t s_actor_table = {0};

// Current running actor_t
static actor_t *s_current_actor = NULL;

// Initialize stack arena
static void arena_init(void) {
    s_stack_arena.base = s_stack_arena_memory;
    s_stack_arena.total_size = HIVE_STACK_ARENA_SIZE;

    // Initialize with one large free block
    arena_block_t *block = (arena_block_t *)s_stack_arena.base;
    block->size = HIVE_STACK_ARENA_SIZE - sizeof(arena_block_t);
    block->next = NULL;
    s_stack_arena.free_list = block;
}

// Allocate from arena with 16-byte alignment
static void *arena_alloc(size_t size) {
    // Round size to alignment
    size = (size + STACK_ALIGNMENT - 1) & ~(STACK_ALIGNMENT - 1);

    // Search free list for first-fit
    arena_block_t **prev_ptr = &s_stack_arena.free_list;
    arena_block_t *curr = s_stack_arena.free_list;

    while (curr != NULL) {
        if (curr->size >= size) {
            // Found suitable block
            size_t remaining = curr->size - size;

            // Check if we should split the block
            if (remaining >= sizeof(arena_block_t) + MIN_BLOCK_SIZE) {
                // Split: allocate from beginning, create new free block
                arena_block_t *new_block =
                    (arena_block_t *)((uint8_t *)curr + sizeof(arena_block_t) +
                                      size);
                new_block->size = remaining - sizeof(arena_block_t);
                new_block->next = curr->next;
                *prev_ptr = new_block;

                curr->size = size;
            } else {
                // Don't split, use entire block
                *prev_ptr = curr->next;
            }

            // Return usable space (after header)
            return (uint8_t *)curr + sizeof(arena_block_t);
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
    arena_block_t *block =
        (arena_block_t *)((uint8_t *)ptr - sizeof(arena_block_t));

    // Insert into free list (maintain address-sorted order) and coalesce
    arena_block_t **prev_ptr = &s_stack_arena.free_list;
    arena_block_t *curr = s_stack_arena.free_list;
    arena_block_t *prev_block = NULL;

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
        uint8_t *prev_end =
            (uint8_t *)prev_block + sizeof(arena_block_t) + prev_block->size;
        if (prev_end == (uint8_t *)block) {
            // Merge with previous
            prev_block->size += sizeof(arena_block_t) + block->size;
            prev_block->next = block->next;
            block = prev_block;
        }
    }

    // Coalesce with next block if adjacent
    if (block->next != NULL) {
        uint8_t *block_end =
            (uint8_t *)block + sizeof(arena_block_t) + block->size;
        if (block_end == (uint8_t *)block->next) {
            // Merge with next
            arena_block_t *next = block->next;
            block->size += sizeof(arena_block_t) + next->size;
            block->next = next->next;
        }
    }
}

hive_status_t hive_actor_init(void) {
    // Initialize stack arena
    arena_init();

    // Use static actor_t array (already zero-initialized by C)
    s_actor_table.actors = s_actors;
    s_actor_table.max_actors = HIVE_MAX_ACTORS;
    s_actor_table.num_actors = 0;
    s_actor_table.next_id = 1; // Start at 1, 0 is HIVE_ACTOR_ID_INVALID

    return HIVE_SUCCESS;
}

void hive_actor_cleanup(void) {
    if (s_actor_table.actors) {
        // Free all actor_t stacks and mailboxes
        for (size_t i = 0; i < s_actor_table.max_actors; i++) {
            actor_t *a = &s_actor_table.actors[i];
            if (a->state != ACTOR_STATE_DEAD && a->stack) {
                if (a->stack_is_malloced) {
                    free(a->stack);
                } else {
                    arena_free(a->stack);
                }
                hive_ipc_mailbox_clear(&a->mailbox);
            }
        }
        // Note: s_actor_table.actors points to static s_actors array, so no
        // free() needed
        s_actor_table.actors = NULL;
    }
}

actor_t *hive_actor_get(actor_id_t id) {
    if (id == HIVE_ACTOR_ID_INVALID) {
        return NULL;
    }

    for (size_t i = 0; i < s_actor_table.max_actors; i++) {
        actor_t *a = &s_actor_table.actors[i];
        if (a->id == id && a->state != ACTOR_STATE_DEAD) {
            return a;
        }
    }

    return NULL;
}

actor_t *hive_actor_alloc(hive_actor_fn_t fn, void *args,
                          const hive_spawn_info_t *siblings,
                          size_t sibling_count,
                          const hive_actor_config_t *cfg) {
    if (s_actor_table.num_actors >= s_actor_table.max_actors) {
        return NULL;
    }

    // Find free slot
    actor_t *a = NULL;
    for (size_t i = 0; i < s_actor_table.max_actors; i++) {
        if (s_actor_table.actors[i].state == ACTOR_STATE_DEAD ||
            s_actor_table.actors[i].id == HIVE_ACTOR_ID_INVALID) {
            a = &s_actor_table.actors[i];
            break;
        }
    }

    if (!a) {
        return NULL;
    }

    // Determine stack size
    size_t stack_size =
        cfg->stack_size > 0 ? cfg->stack_size : HIVE_DEFAULT_STACK_SIZE;

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

    // Initialize actor_t
    memset(a, 0, sizeof(actor_t));
    a->id = s_actor_table.next_id++;
    a->state = ACTOR_STATE_READY;
    a->priority = cfg->priority;
    a->name = cfg->name;
    a->pool_block_default = cfg->pool_block;
    a->pool_block = cfg->pool_block;
    a->stack = stack;
    a->stack_size = stack_size;
    a->stack_is_malloced = is_malloced; // Track allocation method

#if HIVE_STACK_WATERMARK
    // Fill stack with watermark pattern for usage measurement
    // Stack grows downward, so fill entire stack with pattern
    {
        uint32_t *p = (uint32_t *)stack;
        size_t count = stack_size / sizeof(uint32_t);
        for (size_t i = 0; i < count; i++) {
            p[i] = HIVE_STACK_WATERMARK_PATTERN;
        }
    }
#endif

    // Store startup info for context_entry to use
    a->startup_args = args;
    a->startup_siblings = siblings;
    a->startup_sibling_count = sibling_count;

    // Initialize receive filters (NULL = no active filter)
    a->recv_filters = NULL;
    a->recv_filter_count = 0;

    // Initialize context with actor_t function
    // Startup info (args, siblings, count) is stored in actor_t struct
    // Cast to match hive_context_init signature (const void* vs const hive_spawn_info_t*)
    hive_context_init(&a->ctx, stack, stack_size,
                      (void (*)(void *, const void *, size_t))fn);

    s_actor_table.num_actors++;

    return a;
}

// External cleanup functions
extern void hive_bus_cleanup_actor(actor_id_t id);
extern void hive_link_cleanup_actor(actor_id_t id);
extern void hive_registry_cleanup_actor(actor_id_t id);

void hive_actor_free(actor_t *a) {
    if (!a) {
        return;
    }

    // Cleanup links/monitors and send death notifications
    hive_link_cleanup_actor(a->id);

    // Cleanup bus subscriptions
    hive_bus_cleanup_actor(a->id);

    // Cleanup registry entries
    hive_registry_cleanup_actor(a->id);

    // Remove from pool wait queue (if waiting on pool exhaustion)
    hive_scheduler_pool_wait_remove(a->id);

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
    if (a->active_msg) {
        hive_ipc_free_active_msg(a->active_msg);
        a->active_msg = NULL;
    }

    // Free mailbox_t entries
    hive_ipc_mailbox_clear(&a->mailbox);

    a->state = ACTOR_STATE_DEAD;
    s_actor_table.num_actors--;
}

actor_t *hive_actor_current(void) {
    return s_current_actor;
}

void hive_actor_set_current(actor_t *a) {
    s_current_actor = a;
}

// Get actor_t table (for scheduler)
actor_table_t *hive_actor_get_table(void) {
    return &s_actor_table;
}

// Find a sibling actor_t by name in the spawn info array
actor_id_t hive_find_sibling(const hive_spawn_info_t *siblings, size_t count,
                             const char *name) {
    if (!siblings || !name) {
        return HIVE_ACTOR_ID_INVALID;
    }

    for (size_t i = 0; i < count; i++) {
        if (siblings[i].name && strcmp(siblings[i].name, name) == 0) {
            return siblings[i].id;
        }
    }

    return HIVE_ACTOR_ID_INVALID;
}

// Stack watermarking functions
size_t hive_actor_stack_usage(actor_id_t id) {
    actor_t *a = hive_actor_get(id);
    if (!a || !a->stack) {
        return 0;
    }

#if HIVE_STACK_WATERMARK
    // Stack grows downward from top (stack + stack_size) toward base (stack)
    // Count how many words at the base still have the watermark pattern
    uint32_t *p = (uint32_t *)a->stack;
    size_t total_words = a->stack_size / sizeof(uint32_t);
    size_t clean_words = 0;

    for (size_t i = 0; i < total_words; i++) {
        if (p[i] == HIVE_STACK_WATERMARK_PATTERN) {
            clean_words++;
        } else {
            break; // First non-pattern word found
        }
    }

    // Used bytes = total bytes - clean bytes
    size_t clean_bytes = clean_words * sizeof(uint32_t);
    return a->stack_size - clean_bytes;
#else
    // Watermarking disabled, return full stack size as "used"
    return a->stack_size;
#endif
}

void hive_actor_stack_usage_all(stack_usage_callback_t cb) {
    if (!cb) {
        return;
    }

    for (size_t i = 0; i < s_actor_table.max_actors; i++) {
        actor_t *a = &s_actor_table.actors[i];
        if (a->state != ACTOR_STATE_DEAD && a->id != HIVE_ACTOR_ID_INVALID) {
            size_t used = hive_actor_stack_usage(a->id);
            cb(a->id, a->name, a->stack_size, used);
        }
    }
}
