#include "rt_spsc.h"
#include <stdlib.h>
#include <string.h>

// Check if a number is power of 2
static bool is_power_of_2(size_t n) {
    return n > 0 && (n & (n - 1)) == 0;
}

rt_status rt_spsc_init(rt_spsc_queue *q, void *buffer, size_t entry_size, size_t capacity) {
    if (!q) {
        return RT_ERROR(RT_ERR_INVALID, "Queue pointer is NULL");
    }

    if (!buffer) {
        return RT_ERROR(RT_ERR_INVALID, "Buffer pointer is NULL");
    }

    if (entry_size == 0) {
        return RT_ERROR(RT_ERR_INVALID, "Entry size must be > 0");
    }

    if (!is_power_of_2(capacity)) {
        return RT_ERROR(RT_ERR_INVALID, "Capacity must be power of 2");
    }

    // Use provided static buffer
    q->buffer = buffer;
    q->entry_size = entry_size;
    q->capacity = capacity;
    atomic_init(&q->head, 0);
    atomic_init(&q->tail, 0);

    return RT_SUCCESS;
}

void rt_spsc_destroy(rt_spsc_queue *q) {
    if (q) {
        // Note: buffer points to static memory, no free needed
        q->buffer = NULL;
    }
}

bool rt_spsc_push(rt_spsc_queue *q, const void *entry) {
    if (!q || !entry) {
        return false;
    }

    // Load current positions
    size_t head = atomic_load_explicit(&q->head, memory_order_relaxed);
    size_t tail = atomic_load_explicit(&q->tail, memory_order_acquire);

    // Check if queue is full
    // Queue is full when head + 1 == tail (mod capacity)
    size_t next_head = (head + 1) & (q->capacity - 1);
    if (next_head == tail) {
        return false; // Queue full
    }

    // Copy entry to buffer
    void *slot = (char *)q->buffer + (head * q->entry_size);
    memcpy(slot, entry, q->entry_size);

    // Update head (release semantics ensures entry is visible before head update)
    atomic_store_explicit(&q->head, next_head, memory_order_release);

    return true;
}

bool rt_spsc_pop(rt_spsc_queue *q, void *entry_out) {
    if (!q || !entry_out) {
        return false;
    }

    // Load current positions
    size_t tail = atomic_load_explicit(&q->tail, memory_order_relaxed);
    size_t head = atomic_load_explicit(&q->head, memory_order_acquire);

    // Check if queue is empty
    if (tail == head) {
        return false; // Queue empty
    }

    // Copy entry from buffer
    void *slot = (char *)q->buffer + (tail * q->entry_size);
    memcpy(entry_out, slot, q->entry_size);

    // Update tail (release semantics for consistency)
    size_t next_tail = (tail + 1) & (q->capacity - 1);
    atomic_store_explicit(&q->tail, next_tail, memory_order_release);

    return true;
}

bool rt_spsc_peek(rt_spsc_queue *q, void *entry_out) {
    if (!q || !entry_out) {
        return false;
    }

    // Load current positions
    size_t tail = atomic_load_explicit(&q->tail, memory_order_relaxed);
    size_t head = atomic_load_explicit(&q->head, memory_order_acquire);

    // Check if queue is empty
    if (tail == head) {
        return false; // Queue empty
    }

    // Copy entry from buffer without updating tail
    void *slot = (char *)q->buffer + (tail * q->entry_size);
    memcpy(entry_out, slot, q->entry_size);

    return true;
}
