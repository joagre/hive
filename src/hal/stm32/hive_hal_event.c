// Hardware Abstraction Layer - STM32 Event Implementation
//
// Event system uses software timer wheel + WFI for idle.
// HAL events use a bitmask for ISR-to-actor signaling.

#include "hal/hive_hal_event.h"
#include "hive_io_source.h"

// Process pending software timers (from hive_hal_timer.c)
extern void hive_timer_process_pending(void);

// -----------------------------------------------------------------------------
// HAL Event Signaling - Bitmask implementation
// -----------------------------------------------------------------------------

// Event flags (set by ISRs, checked by actors)
// volatile ensures compiler doesn't optimize away reads
static volatile uint32_t s_event_flags = 0;

// Tracks which event IDs are allocated
static uint32_t s_event_allocated = 0;

hive_status_t hive_hal_event_init(void) {
    // No initialization needed - software timers are managed by timer subsystem
    return HIVE_SUCCESS;
}

void hive_hal_event_cleanup(void) {
    // No cleanup needed
}

void hive_hal_event_poll(void) {
    // Process any pending timer events
    hive_timer_process_pending();
}

void hive_hal_event_wait(int timeout_ms) {
    // Process pending timers first
    hive_timer_process_pending();

    // If timeout allows waiting, use WFI to sleep until interrupt
    if (timeout_ms != 0) {
        __asm__ volatile("wfi");
    }
}

hive_status_t hive_hal_event_register(int fd, uint32_t events,
                                      io_source_t *source) {
    // STM32 doesn't use file descriptors for timers - software timer wheel
    // This is a no-op but returns success for compatibility
    (void)fd;
    (void)events;
    (void)source;
    return HIVE_SUCCESS;
}

void hive_hal_event_unregister(int fd) {
    // No-op on STM32
    (void)fd;
}

// -----------------------------------------------------------------------------
// HAL Event Signaling Functions
// -----------------------------------------------------------------------------

hive_hal_event_id_t hive_hal_event_create(void) {
    // Find first unallocated slot
    for (uint8_t i = 0; i < HIVE_HAL_EVENT_MAX; i++) {
        if (!(s_event_allocated & (1U << i))) {
            s_event_allocated |= (1U << i);
            return i;
        }
    }
    return HIVE_HAL_EVENT_INVALID;
}

void hive_hal_event_destroy(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        s_event_allocated &= ~(1U << id);
        s_event_flags &= ~(1U << id);
    }
}

void hive_hal_event_signal(hive_hal_event_id_t id) {
    // ISR-safe: single atomic store on ARM Cortex-M
    // The OR operation compiles to LDR, ORR, STR but the final STR is atomic
    // and we only ever SET bits from ISR context (never clear)
    if (id < HIVE_HAL_EVENT_MAX) {
        s_event_flags |= (1U << id);
    }
}

bool hive_hal_event_is_set(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        return (s_event_flags & (1U << id)) != 0;
    }
    return false;
}

void hive_hal_event_clear(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        // Disable interrupts to make clear atomic
        // (prevents ISR from setting bit during read-modify-write)
        __asm__ volatile("cpsid i" ::: "memory");
        s_event_flags &= ~(1U << id);
        __asm__ volatile("cpsie i" ::: "memory");
    }
}
