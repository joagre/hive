#include "acrt_bus.h"
#include "acrt_internal.h"
#include "acrt_static_config.h"
#include "acrt_pool.h"
#include "acrt_actor.h"
#include "acrt_scheduler.h"
#include "acrt_runtime.h"
#include "acrt_log.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

// External IPC pools (defined in acrt_ipc.c)
extern acrt_pool g_message_pool_mgr;

// Forward declarations
acrt_status acrt_bus_init(void);
void acrt_bus_cleanup(void);
void acrt_bus_cleanup_actor(actor_id id);

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
    bool     blocked;         // Is actor blocked waiting for data?
} bus_subscriber;

// Bus structure
typedef struct {
    bus_id           id;
    acrt_bus_config    config;
    bus_entry       *entries;         // Ring buffer (dynamically allocated)
    size_t           head;             // Write position
    size_t           tail;             // Oldest entry position
    size_t           count;            // Number of valid entries
    bus_subscriber  *subscribers;     // Dynamically allocated array
    size_t           num_subscribers;
    bool             active;
} bus_t;

// Static bus storage
static bus_t g_buses[ACRT_MAX_BUSES];
static bus_entry g_bus_entries[ACRT_MAX_BUSES][ACRT_MAX_BUS_ENTRIES];
static bus_subscriber g_bus_subscribers[ACRT_MAX_BUSES][ACRT_MAX_BUS_SUBSCRIBERS];

// Bus table
static struct {
    bus_t    *buses;        // Points to static g_buses array
    size_t    max_buses;    // Maximum number of buses
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

    for (size_t i = 0; i < g_bus_table.max_buses; i++) {
        if (g_bus_table.buses[i].active && g_bus_table.buses[i].id == id) {
            return &g_bus_table.buses[i];
        }
    }

    return NULL;
}

// Find subscriber index in bus
static int find_subscriber(bus_t *bus, actor_id id) {
    for (size_t i = 0; i < bus->config.max_subscribers; i++) {
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
        acrt_msg_pool_free(entry->data);
        entry->valid = false;
        bus->tail = (bus->tail + 1) % bus->config.max_entries;
        bus->count--;
    }
}

// Initialize bus subsystem
acrt_status acrt_bus_init(void) {
    ACRT_INIT_GUARD(g_bus_table.initialized);

    // Use static bus array (already zero-initialized)
    g_bus_table.buses = g_buses;
    g_bus_table.max_buses = ACRT_MAX_BUSES;
    g_bus_table.next_id = 1;
    g_bus_table.initialized = true;

    return ACRT_SUCCESS;
}

// Cleanup bus subsystem
void acrt_bus_cleanup(void) {
    if (!g_bus_table.initialized) {
        return;
    }

    // Destroy all buses
    for (size_t i = 0; i < g_bus_table.max_buses; i++) {
        bus_t *bus = &g_bus_table.buses[i];
        if (bus->active) {
            // Free all entry data from pool
            for (size_t j = 0; j < bus->config.max_entries; j++) {
                if (bus->entries[j].valid) {
                    acrt_msg_pool_free(bus->entries[j].data);
                }
            }
            // Note: bus->entries and bus->subscribers point to static arrays, no free needed
            bus->active = false;
        }
    }

    // Note: g_bus_table.buses points to static g_buses array, no free needed
    g_bus_table.buses = NULL;
    g_bus_table.max_buses = 0;
    g_bus_table.initialized = false;
}

// Cleanup actor bus subscriptions (called when actor dies)
void acrt_bus_cleanup_actor(actor_id id) {
    if (!g_bus_table.initialized) {
        return;
    }

    for (size_t i = 0; i < g_bus_table.max_buses; i++) {
        bus_t *bus = &g_bus_table.buses[i];
        if (!bus->active) {
            continue;
        }

        // Remove actor from subscribers
        for (size_t j = 0; j < bus->config.max_subscribers; j++) {
            if (bus->subscribers[j].active && bus->subscribers[j].id == id) {
                bus->subscribers[j].active = false;
                bus->num_subscribers--;
                ACRT_LOG_DEBUG("Actor %u unsubscribed from bus %u (cleanup)", id, bus->id);
            }
        }
    }
}

// Create bus
acrt_status acrt_bus_create(const acrt_bus_config *cfg, bus_id *out) {
    if (!cfg || !out) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    if (!g_bus_table.initialized) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus subsystem not initialized");
    }

    if (cfg->max_entries == 0 || cfg->max_entry_size == 0 || cfg->max_subscribers == 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid bus configuration");
    }

    // Validate against compile-time limits
    if (cfg->max_entries > ACRT_MAX_BUS_ENTRIES) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "max_entries exceeds ACRT_MAX_BUS_ENTRIES");
    }

    if (cfg->max_subscribers > ACRT_MAX_BUS_SUBSCRIBERS) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "max_subscribers exceeds ACRT_MAX_BUS_SUBSCRIBERS");
    }

    if (cfg->max_entry_size > ACRT_MAX_MESSAGE_SIZE) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "max_entry_size exceeds ACRT_MAX_MESSAGE_SIZE");
    }

    // Find free slot
    bus_t *bus = NULL;
    size_t bus_idx = 0;
    for (size_t i = 0; i < g_bus_table.max_buses; i++) {
        if (!g_bus_table.buses[i].active) {
            bus = &g_bus_table.buses[i];
            bus_idx = i;
            break;
        }
    }

    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Bus table full");
    }

    // Initialize bus using static arrays
    memset(bus, 0, sizeof(bus_t));
    bus->id = g_bus_table.next_id++;
    bus->config = *cfg;
    bus->entries = g_bus_entries[bus_idx];
    bus->subscribers = g_bus_subscribers[bus_idx];
    bus->head = 0;
    bus->tail = 0;
    bus->count = 0;
    bus->num_subscribers = 0;
    bus->active = true;

    *out = bus->id;
    ACRT_LOG_DEBUG("Created bus %u (max_entries=%zu, max_entry_size=%zu, max_subscribers=%zu)",
                 bus->id, cfg->max_entries, cfg->max_entry_size, cfg->max_subscribers);

    return ACRT_SUCCESS;
}

// Destroy bus
acrt_status acrt_bus_destroy(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    if (bus->num_subscribers > 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Cannot destroy bus with active subscribers");
    }

    // Free all entry data from pool
    for (size_t i = 0; i < bus->config.max_entries; i++) {
        if (bus->entries[i].valid) {
            acrt_msg_pool_free(bus->entries[i].data);
        }
    }

    // Note: bus->entries and bus->subscribers point to static arrays, no free needed
    bus->active = false;

    ACRT_LOG_DEBUG("Destroyed bus %u", id);
    return ACRT_SUCCESS;
}

// Publish data
acrt_status acrt_bus_publish(bus_id id, const void *data, size_t len) {
    if (!data || len == 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid data");
    }

    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    if (len > bus->config.max_entry_size) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Data exceeds max entry size");
    }

    // Expire old entries
    expire_old_entries(bus);

    // Validate message size against pool limit
    if (len > ACRT_MAX_MESSAGE_SIZE) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Message exceeds ACRT_MAX_MESSAGE_SIZE");
    }

    // If buffer is full, evict oldest entry
    if (bus->count >= bus->config.max_entries) {
        bus_entry *oldest = &bus->entries[bus->tail];
        if (oldest->valid) {
            acrt_msg_pool_free(oldest->data);
        }
        oldest->valid = false;
        bus->tail = (bus->tail + 1) % bus->config.max_entries;
        bus->count--;
    }

    // Allocate from message pool and copy data
    message_data_entry *msg_data = acrt_pool_alloc(&g_message_pool_mgr);
    if (!msg_data) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Message pool exhausted");
    }
    memcpy(msg_data->data, data, len);
    void *entry_data = msg_data->data;

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

    ACRT_LOG_TRACE("Published %zu bytes to bus %u (count=%zu)", len, id, bus->count);

    // Wake up any blocked subscribers
    for (size_t i = 0; i < bus->config.max_subscribers; i++) {
        bus_subscriber *sub = &bus->subscribers[i];
        if (sub->active && sub->blocked) {
            actor *a = acrt_actor_get(sub->id);
            if (a && a->state == ACTOR_STATE_WAITING) {
                a->state = ACTOR_STATE_READY;
                ACRT_LOG_TRACE("Woke blocked subscriber %u on bus %u", sub->id, id);
            }
        }
    }

    return ACRT_SUCCESS;
}

// Subscribe current actor
acrt_status acrt_bus_subscribe(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    // Check if already subscribed
    if (find_subscriber(bus, current->id) >= 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Already subscribed");
    }

    // Find free subscriber slot
    bus_subscriber *sub = NULL;
    for (size_t i = 0; i < bus->config.max_subscribers; i++) {
        if (!bus->subscribers[i].active) {
            sub = &bus->subscribers[i];
            break;
        }
    }

    if (!sub) {
        return ACRT_ERROR(ACRT_ERR_NOMEM, "Subscriber table full");
    }

    // Initialize subscriber
    sub->id = current->id;
    sub->next_read_idx = bus->head;  // Start reading from newest entry
    sub->active = true;
    sub->blocked = false;
    bus->num_subscribers++;

    ACRT_LOG_DEBUG("Actor %u subscribed to bus %u", current->id, id);

    return ACRT_SUCCESS;
}

// Unsubscribe current actor
acrt_status acrt_bus_unsubscribe(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    int sub_idx = find_subscriber(bus, current->id);
    if (sub_idx < 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Not subscribed");
    }

    bus->subscribers[sub_idx].active = false;
    bus->num_subscribers--;

    ACRT_LOG_DEBUG("Actor %u unsubscribed from bus %u", current->id, id);

    return ACRT_SUCCESS;
}

// Read entry (non-blocking)
acrt_status acrt_bus_read(bus_id id, void *buf, size_t max_len, size_t *actual_len) {
    if (!buf || !actual_len) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    int sub_idx = find_subscriber(bus, current->id);
    if (sub_idx < 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Not subscribed");
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
        return ACRT_ERROR(ACRT_ERR_WOULDBLOCK, "No data available");
    }

    // Copy data (truncate to buffer size to prevent overflow)
    size_t copy_len = entry->len < max_len ? entry->len : max_len;
    memcpy(buf, entry->data, copy_len);
    *actual_len = copy_len;  // Return actual bytes copied, not original length

    // Mark as read by this subscriber
    entry->readers_mask |= (1u << sub_idx);
    entry->read_count++;

    // Update subscriber's next read position
    sub->next_read_idx = (idx + 1) % bus->config.max_entries;

    ACRT_LOG_TRACE("Actor %u read %zu bytes from bus %u", current->id, copy_len, id);

    // Check if entry should be removed (max_readers)
    if (bus->config.consume_after_reads > 0 && entry->read_count >= bus->config.consume_after_reads) {
        acrt_msg_pool_free(entry->data);
        entry->valid = false;
        entry->data = NULL;

        // Advance tail if this was the tail entry
        if (idx == bus->tail) {
            while (bus->count > 0 && !bus->entries[bus->tail].valid) {
                bus->tail = (bus->tail + 1) % bus->config.max_entries;
                bus->count--;
            }
        }

        ACRT_LOG_TRACE("Bus %u entry consumed by %u readers", id, entry->read_count);
    }

    return ACRT_SUCCESS;
}

// Read with blocking
acrt_status acrt_bus_read_wait(bus_id id, void *buf, size_t max_len,
                           size_t *actual_len, int32_t timeout_ms) {
    if (!buf || !actual_len) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Invalid arguments");
    }

    bus_t *bus = find_bus(id);
    if (!bus) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Bus not found");
    }

    ACRT_REQUIRE_ACTOR_CONTEXT();
    actor *current = acrt_actor_current();

    int sub_idx = find_subscriber(bus, current->id);
    if (sub_idx < 0) {
        return ACRT_ERROR(ACRT_ERR_INVALID, "Not subscribed");
    }

    bus_subscriber *sub = &bus->subscribers[sub_idx];

    // Try non-blocking read first
    acrt_status status = acrt_bus_read(id, buf, max_len, actual_len);
    if (!ACRT_FAILED(status)) {
        return status;
    }

    if (status.code != ACRT_ERR_WOULDBLOCK) {
        return status;  // Real error, not just no data
    }

    // No data available
    if (timeout_ms == 0) {
        return status;  // Non-blocking
    }

    // Block until data is available or timeout expires
    // Mark subscriber as blocked so acrt_bus_publish can wake us
    sub->blocked = true;
    current->state = ACTOR_STATE_WAITING;

    // Create timeout timer if needed
    timer_id timeout_timer = TIMER_ID_INVALID;
    if (timeout_ms > 0) {
        acrt_status timer_status = acrt_timer_after((uint32_t)timeout_ms * 1000, &timeout_timer);
        if (ACRT_FAILED(timer_status)) {
            sub->blocked = false;
            current->state = ACTOR_STATE_READY;
            return timer_status;
        }
    }

    // Yield to scheduler - will resume when woken by publish or timeout
    acrt_scheduler_yield();

    // Woken up - clear blocked flag
    sub->blocked = false;

    // Check for timeout
    if (timeout_timer != TIMER_ID_INVALID) {
        acrt_status timeout_status = acrt_mailbox_handle_timeout(current, timeout_timer, "Bus read timeout");
        if (ACRT_FAILED(timeout_status)) {
            return timeout_status;
        }
    }

    // Try to read again
    return acrt_bus_read(id, buf, max_len, actual_len);
}

// Query bus state
size_t acrt_bus_entry_count(bus_id id) {
    bus_t *bus = find_bus(id);
    if (!bus) {
        return 0;
    }

    return bus->count;
}
