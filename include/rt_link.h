#ifndef RT_LINK_H
#define RT_LINK_H

#include "rt_types.h"

// Exit message structure (exit reason is defined in rt_types.h)
typedef struct {
    actor_id       actor;    // ID of the actor that died
    rt_exit_reason reason;   // Why the actor exited
} rt_exit_msg;

// Bidirectional linking - both actors notified when either dies
rt_status rt_link(actor_id target);
rt_status rt_link_remove(actor_id target);

// Unidirectional monitoring - only monitor notified when target dies
rt_status rt_monitor(actor_id target, uint32_t *monitor_id);
rt_status rt_monitor_cancel(uint32_t monitor_id);

// Exit message helpers
bool rt_is_exit_msg(const rt_message *msg);
rt_status rt_decode_exit(const rt_message *msg, rt_exit_msg *out);

#endif // RT_LINK_H
