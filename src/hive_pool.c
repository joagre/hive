#include "hive_pool.h"
#include <string.h>

void hive_pool_init(hive_pool *pool, void *entries, bool *used,
                  size_t entry_size, size_t capacity) {
    pool->entries = entries;
    pool->used = used;
    pool->entry_size = entry_size;
    pool->capacity = capacity;
    pool->allocated = 0;

    // Mark all entries as free
    memset(used, 0, capacity * sizeof(bool));
}

void* hive_pool_alloc(hive_pool *pool) {
    // Find first free entry
    for (size_t i = 0; i < pool->capacity; i++) {
        if (!pool->used[i]) {
            pool->used[i] = true;
            pool->allocated++;

            // Return pointer to this entry
            return (char*)pool->entries + (i * pool->entry_size);
        }
    }

    // Pool exhausted
    return NULL;
}

void hive_pool_free(hive_pool *pool, void *entry) {
    if (!entry) {
        return;
    }

    // Calculate index from pointer
    size_t offset = (char*)entry - (char*)pool->entries;
    size_t index = offset / pool->entry_size;

    // Validate index
    if (index >= pool->capacity) {
        return;  // Invalid entry
    }

    // Free the entry
    if (pool->used[index]) {
        pool->used[index] = false;
        pool->allocated--;
    }
}
