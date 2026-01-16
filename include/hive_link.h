#ifndef HIVE_LINK_H
#define HIVE_LINK_H

#include "hive_types.h"

// Exit message structure (exit reason is defined in hive_types.h)
typedef struct {
    actor_id actor;          // ID of the actor that died
    hive_exit_reason reason; // Why the actor exited
} hive_exit_msg;

// Bidirectional linking - both actors notified when either dies
hive_status hive_link(actor_id target);
hive_status hive_link_remove(actor_id target);

// Unidirectional monitoring - only monitor notified when target dies
hive_status hive_monitor(actor_id target, uint32_t *out);
hive_status hive_monitor_cancel(uint32_t id);

// Exit message helpers
bool hive_is_exit_msg(const hive_message *msg);
hive_status hive_decode_exit(const hive_message *msg, hive_exit_msg *out);

// Convert exit reason to string (for logging/debugging)
const char *hive_exit_reason_str(hive_exit_reason reason);

#endif // HIVE_LINK_H
