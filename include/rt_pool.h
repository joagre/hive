#ifndef RT_POOL_H
#define RT_POOL_H

#include <stddef.h>
#include <stdbool.h>

// Simple fixed-size object pool allocator
// Used for static allocation of mailbox entries, link entries, etc.

typedef struct rt_pool {
    void   *entries;       // Pointer to static array of entries
    bool   *used;          // Bitmap of which entries are in use
    size_t  entry_size;    // Size of each entry in bytes
    size_t  capacity;      // Total number of entries
    size_t  allocated;     // Number of currently allocated entries
} rt_pool;

// Initialize a pool with a static array
// entries: pointer to static array (e.g., static mailbox_entry pool[256])
// used: pointer to static bool array (e.g., static bool used[256])
// entry_size: sizeof(entry_type)
// capacity: number of entries in the array
void rt_pool_init(rt_pool *pool, void *entries, bool *used,
                  size_t entry_size, size_t capacity);

// Allocate an entry from the pool
// Returns NULL if pool is exhausted
void* rt_pool_alloc(rt_pool *pool);

// Free an entry back to the pool
// entry must have been allocated from this pool
void rt_pool_free(rt_pool *pool, void *entry);

// Get number of free entries remaining
size_t rt_pool_available(const rt_pool *pool);

// Check if pool is empty
bool rt_pool_is_empty(const rt_pool *pool);

#endif // RT_POOL_H
