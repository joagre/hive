#ifndef ACRT_LINK_H
#define ACRT_LINK_H

#include "acrt_types.h"

// Exit message structure (exit reason is defined in acrt_types.h)
typedef struct {
    actor_id       actor;    // ID of the actor that died
    acrt_exit_reason reason;   // Why the actor exited
} acrt_exit_msg;

// Bidirectional linking - both actors notified when either dies
acrt_status acrt_link(actor_id target);
acrt_status acrt_link_remove(actor_id target);

// Unidirectional monitoring - only monitor notified when target dies
acrt_status acrt_monitor(actor_id target, uint32_t *out);
acrt_status acrt_monitor_cancel(uint32_t id);

// Exit message helpers
bool acrt_is_exit_msg(const acrt_message *msg);
acrt_status acrt_decode_exit(const acrt_message *msg, acrt_exit_msg *out);

// Convert exit reason to string (for logging/debugging)
const char *acrt_exit_reason_str(acrt_exit_reason reason);

#endif // ACRT_LINK_H
