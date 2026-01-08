#ifndef ACRT_BUS_H
#define ACRT_BUS_H

#include "acrt_types.h"
#include <stdint.h>
#include <stddef.h>

// Bus ID type
typedef uint32_t bus_id;

#define BUS_ID_INVALID ((bus_id)0)

// Bus configuration
typedef struct {
    uint8_t  max_subscribers;     // max concurrent subscribers (1..ACRT_MAX_BUS_SUBSCRIBERS)
    uint8_t  consume_after_reads; // remove entry after N reads, 0 = keep until aged out
    uint32_t max_age_ms;          // expire entries after ms, 0 = no expiry
    size_t   max_entries;         // ring buffer capacity
    size_t   max_entry_size;      // max payload bytes per entry
} acrt_bus_config;

// Default bus configuration
#define ACRT_BUS_CONFIG_DEFAULT { \
    .max_subscribers = 32, \
    .consume_after_reads = 0, \
    .max_age_ms = 0, \
    .max_entries = 16, \
    .max_entry_size = 256 \
}

// Bus operations

// Create bus
acrt_status acrt_bus_create(const acrt_bus_config *cfg, bus_id *out);

// Destroy bus (fails if subscribers exist)
acrt_status acrt_bus_destroy(bus_id bus);

// Publish data
acrt_status acrt_bus_publish(bus_id bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
acrt_status acrt_bus_subscribe(bus_id bus);
acrt_status acrt_bus_unsubscribe(bus_id bus);

// Read entry (non-blocking)
// Returns ACRT_ERR_WOULDBLOCK if no data available
acrt_status acrt_bus_read(bus_id bus, void *buf, size_t max_len, size_t *bytes_read);

// Read with blocking
acrt_status acrt_bus_read_wait(bus_id bus, void *buf, size_t max_len,
                               size_t *bytes_read, int32_t timeout_ms);

// Query bus state
size_t acrt_bus_entry_count(bus_id bus);

#endif // ACRT_BUS_H
