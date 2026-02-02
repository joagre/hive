// Unified Timer Implementation
//
// Platform-independent wrapper that handles:
// - Subsystem initialization guards
// - Argument validation
// - hive_sleep() implementation (timer + selective receive)
//
// Actual timer operations are delegated to platform-specific HAL
// implementations:
// - Linux: timerfd + epoll (in hive_hal_timer.c)
// - STM32: Software timer wheel (in hive_hal_timer.c)

#include "hive_timer.h"
#include "hive_internal.h"
#include "hive_actor.h"
#include "hive_ipc.h"
#include "hal/hive_hal_timer.h"

// Timer subsystem state
static struct {
    bool initialized;
} s_timer = {0};

hive_status_t hive_timer_init(void) {
    HIVE_INIT_GUARD(s_timer.initialized);

    hive_status_t status = hive_hal_timer_init();
    if (HIVE_FAILED(status)) {
        return status;
    }

    s_timer.initialized = true;
    return HIVE_SUCCESS;
}

void hive_timer_cleanup(void) {
    HIVE_CLEANUP_GUARD(s_timer.initialized);

    hive_hal_timer_cleanup();
    s_timer.initialized = false;
}

hive_status_t hive_timer_after(uint32_t delay_us, hive_timer_id_t *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(s_timer.initialized, "Timer");
    HIVE_REQUIRE_ACTOR_CONTEXT();

    actor_t *current = hive_actor_current();
    return hive_hal_timer_create(delay_us, false, current->id, out);
}

hive_status_t hive_timer_every(uint32_t interval_us, hive_timer_id_t *out) {
    if (!out) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "NULL out pointer");
    }

    HIVE_REQUIRE_INIT(s_timer.initialized, "Timer");
    HIVE_REQUIRE_ACTOR_CONTEXT();

    actor_t *current = hive_actor_current();
    return hive_hal_timer_create(interval_us, true, current->id, out);
}

hive_status_t hive_timer_cancel(hive_timer_id_t id) {
    HIVE_REQUIRE_INIT(s_timer.initialized, "Timer");

    return hive_hal_timer_cancel(id);
}

hive_status_t hive_sleep(uint32_t delay_us) {
    // Create one-shot timer
    hive_timer_id_t timer;
    hive_status_t s = hive_timer_after(delay_us, &timer);
    if (HIVE_FAILED(s)) {
        return s;
    }

    // Wait specifically for THIS timer message
    // Other messages remain in mailbox_t (selective receive)
    hive_message_t msg;
    return hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg,
                               -1);
}

uint64_t hive_get_time(void) {
    return hive_hal_timer_get_time();
}

void hive_timer_advance_time(uint64_t delta_us) {
    if (!s_timer.initialized) {
        return;
    }

    hive_hal_timer_advance_time(delta_us);
}
