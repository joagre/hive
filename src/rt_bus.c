#include "rt_bus.h"
#include "rt_actor.h"
#include "rt_scheduler.h"
#include "rt_runtime.h"
#include "rt_log.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

// Forward declarations
rt_status rt_bus_init(void);
void rt_bus_cleanup(void);
void rt_bus_cleanup_actor(actor_id id);

// Bus entry in ring buffer
typedef struct {
    void     *data;           // Payload
    size_t    len;            // Payload length
    uint64_t  timestamp_ms;   // When entry was published
    uint8_t   read_count;     // How many actors have read this
    bool      valid;          // Is this entry valid?
    uint32_t  readers_mask;   // Bitmask of which subscribers have read (max 32 subscribers)
} bus_entry;

// Subscriber info
typedef struct {
    actor_id id;
    size_t   next_read_idx;   // Next entry index to read
    bool     active;
} bus_subscriber;

#define MAX_SUBSCRIBERS 32

// Bus structure
typedef struct {
    bus_id           id;
    rt_bus_config    config;
    bus_entry       *entries;         // Ring buffer
    size_t           head;             // Write position
    size_t           tail;             // Oldest entry position
    size_t           count;            // Number of valid entries
    bus_subscriber   subscribers[MAX_SUBSCRIBERS];
    size_t           num_subscribers;
    bool             active;
} bus_t;

#define MAX_BUSES 32

// Bus table
static struct {
    bus_t     buses[MAX_BUSES];
    bus_id    next_id;
    bool      initialized;
} g_bus_table = {0};

// Get current time in milliseconds
static uint64_t get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

// Find bus by ID
static bus_t *find_bus(bus_id id) {
    if (id == BUS_ID_INVALID) {
        return NULL;
    }

    for (size_t i = 0; i < MAX_BUSES; i++) {
        if (g_bus_table.buses[i].active && g_bus_table.buses[i].id == id) {
            return &g_bus_table.buses[i];
        }
    }

    return NULL;
}

// Find subscriber index in bus
static int find_subscriber(bus_t *bus, actor_id id) {
    for (size_t i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (bus->subscribers[i].active && bus->subscribers[i].id == id) {
            return (int)i;
        }
    }
    return -1;
}

// Expire old entries based on max_age_ms
static void expire_old_entries(bus_t *bus) {
    if (bus->config.max_age_ms == 0) {
        return;  // No time-based expiry
    }

    uint64_t now = get_time_ms();

    while (bus->count > 0) {
        bus_entry *entry = &bus->entries[bus->tail];
        if (!entry->valid) {
            break;
        }

        uint64_t age = now - entry->timestamp_ms;
        if (age < bus->config.max_age_ms) {
            break;  // This entry and all newer ones are still fresh
        }

        // Expire this entry
        free(entry->data);
        entry->valid = false;
        bus->tail = (bus->tail + 1) % bus->config.max_entries;
        bus->count--;
    }
}

// Initialize bus subsystem
rt_status rt_bus_init(void) {
    if (g_bus_table.initialized) {
        return RT_SUCCESS;
    }

    memset(&g_bus_table, 0, sizeof(g_bus_table));
    g_bus_table.next_id = 1;
    g_bus_table.initialized = true;

    return RT_SUCCESS;
}

// Cleanup bus subsystem
void rt_bus_cleanup(void) {
    if (!g_bus_table.initialized) {
        return;
    }

    // Destroy all buses
    for (size_t i = 0; i < MAX_BUSES; i++) {
        bus_t *bus = &g_bus_table.buses[i];
        if (bus->active) {
            // Free all entry data
            for (size_t j = 0; j < bus->config.max_entries; j++) {
                if (bus->entries[j].valid && bus->entries[j].data) {
                    free(bus->entries[j].data);
                }
            }
            free(bus->entries);
            bus->active = false;
        }
    }

    g_bus_table.initialized = false;
}

// Cleanup actor bus subscriptions (called when actor dies)
void rt_bus_cleanup_actor(actor_id id) {
    if (!g_bus_table.initialized) {
        return;
    }

    for (size_t i = 0; i < MAX_BUSES; i++) {
        bus_t *bus = &g_bus_table.buses[i];
        if (!bus->active) {
            continue;
        }

        // Remove actor from subscribers
        for (size_t j = 0; j < MAX_SUBSCRIBERS; j++) {
            if (bus->subscribers[j].active && bus->subscribers[j].id == id) {
                bus->subscribers[j].active = false;
                bus->num_subscribers--;
                RT_LOG_DEBUG("Actor %u unsubscribed from bus %u (cleanup)", id, bus->id);
            }
        }
    }
}

// Create bus
rt_status rt_bus_create(const rt_bus_config *cfg, bus_id *out) {
    if (!cfg || !out) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_bus_table.initialized) {
        return RT_ERROR(RT_ERR_INVALID, "Bus subsystem not initialized");
    }

    if (cfg->max_entries == 0 || cfg->max_entry_size == 0) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid bus configuration");
    }

    // Find free slot
    bus_t *bus = NULL;
    for (size_t i = 0; i < MAX_BUSES; i++) {
        if (!g_bus_table.buses[i].active) {
            bus = &g_bus_table.buses[i];
            break;
        }
    }

    if (!bus) {
        return RT_ERROR(RT_ERR_NOMEM, "Bus table full");
    }

    // Allocate ring buffer
    bus_entry *entries = calloc(cfg->max_entries, sizeof(bus_entry));
    if (!entries) {
        return RT_ERROR(RT_ERR_NOMEM, "Failed to allocate bus entries");
    }

    // Initialize bus
    memset(bus, 0, sizeof(bus_t));
    bus->id = g_bus_table.next_id++;
    bus->config = *cfg;
    bus->entries = entries;
    bus->head = 0;
    bus->tail = 0;
    bus->count = 0;
    bus->num_subscribers = 0;
    bus->active = true;

    *out = bus->id;
    RT_LOG_DEBUG("Created bus %u (max_entries=%zu, max_entry_size=%zu)",
                 bus->id, cfg->max_entries, cfg->max_entry_size);

    return RT_SUCCESS;
}

// Destroy bus
rt_status rt_bus_destroy(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return RT_ERROR(RT_ERR_INVALID, "Bus not found");
    }

    if (bus->num_subscribers > 0) {
        return RT_ERROR(RT_ERR_INVALID, "Cannot destroy bus with active subscribers");
    }

    // Free all entry data
    for (size_t i = 0; i < bus->config.max_entries; i++) {
        if (bus->entries[i].valid && bus->entries[i].data) {
            free(bus->entries[i].data);
        }
    }

    free(bus->entries);
    bus->active = false;

    RT_LOG_DEBUG("Destroyed bus %u", id);
    return RT_SUCCESS;
}

// Publish data
rt_status rt_bus_publish(bus_id id, const void *data, size_t len) {
    if (!data || len == 0) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid data");
    }

    bus_t *bus = find_bus(id);
    if (!bus) {
        return RT_ERROR(RT_ERR_INVALID, "Bus not found");
    }

    if (len > bus->config.max_entry_size) {
        return RT_ERROR(RT_ERR_INVALID, "Data exceeds max entry size");
    }

    // Expire old entries
    expire_old_entries(bus);

    // If buffer is full, evict oldest entry
    if (bus->count >= bus->config.max_entries) {
        bus_entry *oldest = &bus->entries[bus->tail];
        if (oldest->valid && oldest->data) {
            free(oldest->data);
        }
        oldest->valid = false;
        bus->tail = (bus->tail + 1) % bus->config.max_entries;
        bus->count--;
    }

    // Allocate and copy data
    void *entry_data = malloc(len);
    if (!entry_data) {
        return RT_ERROR(RT_ERR_NOMEM, "Failed to allocate entry data");
    }
    memcpy(entry_data, data, len);

    // Add new entry
    bus_entry *entry = &bus->entries[bus->head];
    entry->data = entry_data;
    entry->len = len;
    entry->timestamp_ms = get_time_ms();
    entry->read_count = 0;
    entry->readers_mask = 0;
    entry->valid = true;

    bus->head = (bus->head + 1) % bus->config.max_entries;
    bus->count++;

    RT_LOG_TRACE("Published %zu bytes to bus %u (count=%zu)", len, id, bus->count);

    return RT_SUCCESS;
}

// Subscribe current actor
rt_status rt_bus_subscribe(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return RT_ERROR(RT_ERR_INVALID, "Bus not found");
    }

    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    // Check if already subscribed
    if (find_subscriber(bus, current->id) >= 0) {
        return RT_ERROR(RT_ERR_INVALID, "Already subscribed");
    }

    // Find free subscriber slot
    bus_subscriber *sub = NULL;
    for (size_t i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!bus->subscribers[i].active) {
            sub = &bus->subscribers[i];
            break;
        }
    }

    if (!sub) {
        return RT_ERROR(RT_ERR_NOMEM, "Subscriber table full");
    }

    // Initialize subscriber
    sub->id = current->id;
    sub->next_read_idx = bus->head;  // Start reading from newest entry
    sub->active = true;
    bus->num_subscribers++;

    RT_LOG_DEBUG("Actor %u subscribed to bus %u", current->id, id);

    return RT_SUCCESS;
}

// Unsubscribe current actor
rt_status rt_bus_unsubscribe(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return RT_ERROR(RT_ERR_INVALID, "Bus not found");
    }

    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    int sub_idx = find_subscriber(bus, current->id);
    if (sub_idx < 0) {
        return RT_ERROR(RT_ERR_INVALID, "Not subscribed");
    }

    bus->subscribers[sub_idx].active = false;
    bus->num_subscribers--;

    RT_LOG_DEBUG("Actor %u unsubscribed from bus %u", current->id, id);

    return RT_SUCCESS;
}

// Read entry (non-blocking)
rt_status rt_bus_read(bus_id id, void *buf, size_t max_len, size_t *actual_len) {
    if (!buf || !actual_len) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    bus_t *bus = find_bus(id);
    if (!bus) {
        return RT_ERROR(RT_ERR_INVALID, "Bus not found");
    }

    actor *current = rt_actor_current();
    if (!current) {
        return RT_ERROR(RT_ERR_INVALID, "Not called from actor context");
    }

    int sub_idx = find_subscriber(bus, current->id);
    if (sub_idx < 0) {
        return RT_ERROR(RT_ERR_INVALID, "Not subscribed");
    }

    bus_subscriber *sub = &bus->subscribers[sub_idx];

    // Expire old entries
    expire_old_entries(bus);

    // Find next unread entry
    size_t idx = sub->next_read_idx;
    bus_entry *entry = NULL;

    // Search for next valid unread entry
    for (size_t i = 0; i < bus->count; i++) {
        size_t check_idx = (bus->tail + i) % bus->config.max_entries;
        bus_entry *e = &bus->entries[check_idx];

        if (!e->valid) {
            continue;
        }

        // Check if this subscriber has already read this entry
        if (e->readers_mask & (1u << sub_idx)) {
            continue;  // Already read
        }

        entry = e;
        idx = check_idx;
        break;
    }

    if (!entry) {
        return RT_ERROR(RT_ERR_WOULDBLOCK, "No data available");
    }

    // Copy data
    size_t copy_len = entry->len < max_len ? entry->len : max_len;
    memcpy(buf, entry->data, copy_len);
    *actual_len = entry->len;

    // Mark as read by this subscriber
    entry->readers_mask |= (1u << sub_idx);
    entry->read_count++;

    // Update subscriber's next read position
    sub->next_read_idx = (idx + 1) % bus->config.max_entries;

    RT_LOG_TRACE("Actor %u read %zu bytes from bus %u", current->id, copy_len, id);

    // Check if entry should be removed (max_readers)
    if (bus->config.max_readers > 0 && entry->read_count >= bus->config.max_readers) {
        free(entry->data);
        entry->valid = false;
        entry->data = NULL;

        // Advance tail if this was the tail entry
        if (idx == bus->tail) {
            while (bus->count > 0 && !bus->entries[bus->tail].valid) {
                bus->tail = (bus->tail + 1) % bus->config.max_entries;
                bus->count--;
            }
        }

        RT_LOG_TRACE("Bus %u entry consumed by %u readers", id, entry->read_count);
    }

    return RT_SUCCESS;
}

// Read with blocking
rt_status rt_bus_read_wait(bus_id id, void *buf, size_t max_len,
                           size_t *actual_len, int32_t timeout_ms) {
    if (!buf || !actual_len) {
        return RT_ERROR(RT_ERR_INVALID, "Invalid arguments");
    }

    // Try non-blocking read first
    rt_status status = rt_bus_read(id, buf, max_len, actual_len);
    if (!RT_FAILED(status)) {
        return status;
    }

    if (status.code != RT_ERR_WOULDBLOCK) {
        return status;  // Real error, not just no data
    }

    // No data available
    if (timeout_ms == 0) {
        return status;  // Non-blocking
    }

    // Poll until data is available
    // Use cooperative yielding - actors remain runnable
    // TODO: Timeout support
    (void)timeout_ms;

    while (true) {
        rt_yield();

        status = rt_bus_read(id, buf, max_len, actual_len);
        if (!RT_FAILED(status)) {
            return status;
        }

        if (status.code != RT_ERR_WOULDBLOCK) {
            return status;
        }
    }
}

// Query bus state
size_t rt_bus_entry_count(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return 0;
    }

    return bus->count;
}
