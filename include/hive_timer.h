#ifndef HIVE_TIMER_H
#define HIVE_TIMER_H

#include "hive_types.h"
#include <stdint.h>

// timer_id_t is defined in hive_types.h

#define HIVE_TIMER_ID_INVALID ((timer_id_t)0)

// Timer operations
// All timers are owned by the calling actor and are automatically cancelled
// when the actor dies

// One-shot: wake current actor after delay
// Timer message: class=HIVE_MSG_TIMER, tag=timer_id_t, no payload
// Use hive_msg_is_timer() to check, msg.tag for timer_id_t
hive_status_t hive_timer_after(uint32_t delay_us, timer_id_t *out);

// Periodic: wake current actor every interval
// Timer message: class=HIVE_MSG_TIMER, tag=timer_id_t, no payload
// Use hive_msg_is_timer() to check, msg.tag for timer_id_t
hive_status_t hive_timer_every(uint32_t interval_us, timer_id_t *out);

// Cancel timer
hive_status_t hive_timer_cancel(timer_id_t id);

// Sleep for specified duration (microseconds)
// Uses selective receive - other messages remain in mailbox
hive_status_t hive_sleep(uint32_t delay_us);

// Get current time in microseconds
// Returns monotonic time suitable for measuring elapsed durations.
// In simulation mode, returns simulated time.
uint64_t hive_get_time(void);

#endif // HIVE_TIMER_H
