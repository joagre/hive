#ifndef RT_TIMER_H
#define RT_TIMER_H

#include "rt_types.h"
#include <stdint.h>

// Timer ID type
typedef uint32_t timer_id;

#define TIMER_ID_INVALID ((timer_id)0)

// Timer operations
// All timers are owned by the calling actor and are automatically cancelled when the actor dies

// One-shot: wake current actor after delay
// Timer message will be delivered via rt_ipc_recv() with sender == RT_SENDER_TIMER
rt_status rt_timer_after(uint32_t delay_us, timer_id *out);

// Periodic: wake current actor every interval
// Timer messages will be delivered via rt_ipc_recv() with sender == RT_SENDER_TIMER
rt_status rt_timer_every(uint32_t interval_us, timer_id *out);

// Cancel timer
rt_status rt_timer_cancel(timer_id id);

// Check if message is from timer
bool rt_timer_is_tick(const rt_message *msg);

#endif // RT_TIMER_H
