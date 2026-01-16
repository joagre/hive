#ifndef HIVE_TIMER_H
#define HIVE_TIMER_H

#include "hive_types.h"
#include <stdint.h>

// Timer ID type
typedef uint32_t timer_id;

#define TIMER_ID_INVALID ((timer_id)0)

// Timer operations
// All timers are owned by the calling actor and are automatically cancelled
// when the actor dies

// One-shot: wake current actor after delay
// Timer message: class=HIVE_MSG_TIMER, tag=timer_id, no payload
// Use hive_msg_is_timer() to check, msg.tag for timer_id
hive_status hive_timer_after(uint32_t delay_us, timer_id *out);

// Periodic: wake current actor every interval
// Timer message: class=HIVE_MSG_TIMER, tag=timer_id, no payload
// Use hive_msg_is_timer() to check, msg.tag for timer_id
hive_status hive_timer_every(uint32_t interval_us, timer_id *out);

// Cancel timer
hive_status hive_timer_cancel(timer_id id);

// Sleep for specified duration (microseconds)
// Uses selective receive - other messages remain in mailbox
hive_status hive_sleep(uint32_t delay_us);

// Get current time in microseconds
// Returns monotonic time suitable for measuring elapsed durations.
// In simulation mode, returns simulated time.
uint64_t hive_get_time(void);

#endif // HIVE_TIMER_H
