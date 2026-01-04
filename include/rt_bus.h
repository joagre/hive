#ifndef RT_BUS_H
#define RT_BUS_H

#include "rt_types.h"
#include <stdint.h>
#include <stddef.h>

// Bus ID type
typedef uint32_t bus_id;

#define BUS_ID_INVALID ((bus_id)0)

// Bus configuration
typedef struct {
    uint8_t  max_readers;     // consume after N reads, 0 = unlimited
    uint32_t max_age_ms;      // expire entries after ms, 0 = no expiry
    size_t   max_entries;     // ring buffer capacity
    size_t   max_entry_size;  // max payload bytes per entry
} rt_bus_config;

// Default bus configuration
#define RT_BUS_CONFIG_DEFAULT { \
    .max_readers = 0, \
    .max_age_ms = 0, \
    .max_entries = 16, \
    .max_entry_size = 256 \
}

// Bus operations

// Create bus
rt_status rt_bus_create(const rt_bus_config *cfg, bus_id *out);

// Destroy bus (fails if subscribers exist)
rt_status rt_bus_destroy(bus_id bus);

// Publish data
rt_status rt_bus_publish(bus_id bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
rt_status rt_bus_subscribe(bus_id bus);
rt_status rt_bus_unsubscribe(bus_id bus);

// Read entry (non-blocking)
// Returns RT_ERR_WOULDBLOCK if no data available
rt_status rt_bus_read(bus_id bus, void *buf, size_t max_len, size_t *actual_len);

// Read with blocking
rt_status rt_bus_read_wait(bus_id bus, void *buf, size_t max_len,
                           size_t *actual_len, int32_t timeout_ms);

// Query bus state
size_t rt_bus_entry_count(bus_id bus);

#endif // RT_BUS_H
