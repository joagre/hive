#ifndef HIVE_POOL_H
#define HIVE_POOL_H

#include <stddef.h>
#include <stdbool.h>

// Simple fixed-size object pool allocator
// Used for static allocation of mailbox entries, link entries, etc.

typedef struct hive_pool_t {
    void *entries;     // Pointer to static array of entries
    bool *used;        // Bitmap of which entries are in use
    size_t entry_size; // Size of each entry in bytes
    size_t capacity;   // Total number of entries
    size_t allocated;  // Number of currently allocated entries
} hive_pool_t;

// Initialize a pool with a static array
// entries: pointer to static array (e.g., static mailbox_entry pool[256])
// used: pointer to static bool array (e.g., static bool used[256])
// entry_size: sizeof(entry_type)
// capacity: number of entries in the array
void hive_pool_init(hive_pool_t *pool, void *entries, bool *used,
                    size_t entry_size, size_t capacity);

// Allocate an entry from the pool
// Returns NULL if pool is exhausted
void *hive_pool_alloc(hive_pool_t *pool);

// Free an entry back to the pool
// entry must have been allocated from this pool
void hive_pool_free(hive_pool_t *pool, void *entry);

#endif // HIVE_POOL_H
