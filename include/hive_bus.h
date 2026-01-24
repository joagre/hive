#ifndef HIVE_BUS_H
#define HIVE_BUS_H

#include "hive_types.h"
#include <stdint.h>
#include <stddef.h>

// bus_id_t typedef is now in hive_types.h

#define BUS_ID_INVALID ((bus_id_t)0)

// Bus configuration
typedef struct {
    uint8_t max_subscribers;     // max concurrent subscribers
                                 // (1..HIVE_MAX_BUS_SUBSCRIBERS)
    uint8_t consume_after_reads; // remove entry after N reads, 0 = keep until
                                 // aged out
    uint32_t max_age_ms;         // expire entries after ms, 0 = no expiry
    size_t max_entries;          // ring buffer capacity
    size_t max_entry_size;       // max payload bytes per entry
} hive_bus_config_t;

// Default bus configuration
#define HIVE_BUS_CONFIG_DEFAULT                                           \
    {                                                                     \
        .max_subscribers = 32, .consume_after_reads = 0, .max_age_ms = 0, \
        .max_entries = 16, .max_entry_size = 256                          \
    }

// Bus operations

// Create bus
hive_status_t hive_bus_create(const hive_bus_config_t *cfg, bus_id_t *out);

// Destroy bus (fails if subscribers exist)
hive_status_t hive_bus_destroy(bus_id_t bus);

// Publish data
hive_status_t hive_bus_publish(bus_id_t bus, const void *data, size_t len);

// Subscribe/unsubscribe current actor
hive_status_t hive_bus_subscribe(bus_id_t bus);
hive_status_t hive_bus_unsubscribe(bus_id_t bus);

// Read entry (non-blocking)
// Returns HIVE_ERR_WOULDBLOCK if no data available
hive_status_t hive_bus_read(bus_id_t bus, void *buf, size_t max_len,
                            size_t *bytes_read);

// Read with blocking
hive_status_t hive_bus_read_wait(bus_id_t bus, void *buf, size_t max_len,
                                 size_t *bytes_read, int32_t timeout_ms);

// Query bus state
size_t hive_bus_entry_count(bus_id_t bus);

#endif // HIVE_BUS_H
