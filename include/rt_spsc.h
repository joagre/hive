#ifndef RT_SPSC_H
#define RT_SPSC_H

#include "rt_types.h"
#include <stdatomic.h>
#include <stdbool.h>

// Lock-free SPSC (Single Producer Single Consumer) queue
// Used for I/O completion queues between worker threads and scheduler
typedef struct {
    void           *buffer;       // Ring buffer for entries
    size_t          entry_size;   // Size of each entry in bytes
    size_t          capacity;     // Must be power of 2
    atomic_size_t   head;         // Written by producer
    atomic_size_t   tail;         // Written by consumer
} rt_spsc_queue;

// Initialize SPSC queue with pre-allocated buffer
// capacity must be a power of 2
// buffer must be at least entry_size * capacity bytes
rt_status rt_spsc_init(rt_spsc_queue *q, void *buffer, size_t entry_size, size_t capacity);

// Destroy SPSC queue and free resources
void rt_spsc_destroy(rt_spsc_queue *q);

// Producer operations (called by I/O thread)
// Returns true if entry was pushed, false if queue is full
bool rt_spsc_push(rt_spsc_queue *q, const void *entry);

// Consumer operations (called by scheduler)
// Returns true if entry was popped, false if queue is empty
bool rt_spsc_pop(rt_spsc_queue *q, void *entry_out);

// Peek at next entry without consuming it
// Returns true if entry exists, false if queue is empty
bool rt_spsc_peek(rt_spsc_queue *q, void *entry_out);

#endif // RT_SPSC_H
