#ifndef HIVE_LINK_H
#define HIVE_LINK_H

#include "hive_types.h"

// Exit message structure (exit reason is defined in hive_types.h)
typedef struct {
    actor_id_t actor;          // ID of the actor that died
    hive_exit_reason_t reason; // Why the actor exited
    uint32_t monitor_id;       // 0 = from link, non-zero = from monitor
} hive_exit_msg_t;

// Bidirectional linking - both actors notified when either dies
hive_status_t hive_link(actor_id_t target);
hive_status_t hive_unlink(actor_id_t target);

// Unidirectional monitoring - only monitor notified when target dies
hive_status_t hive_monitor(actor_id_t target, uint32_t *out);
hive_status_t hive_demonitor(uint32_t id);

// Exit message helpers
bool hive_msg_is_exit(const hive_message_t *msg);
hive_status_t hive_decode_exit(const hive_message_t *msg, hive_exit_msg_t *out);

#endif // HIVE_LINK_H
