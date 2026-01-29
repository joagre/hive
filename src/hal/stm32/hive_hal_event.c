// Hardware Abstraction Layer - STM32 Event Implementation
//
// Event system uses software timer wheel + WFI for idle.

#include "hal/hive_hal_event.h"
#include "hive_io_source.h"

// Process pending software timers (from hive_hal_timer.c)
extern void hive_timer_process_pending(void);

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
