#ifndef RT_IPC_H
#define RT_IPC_H

#include "rt_types.h"
#include "rt_actor.h"

// Send message to actor
// Blocks if mode == IPC_BORROW (until receiver consumes)
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);

// Receive message
// timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded
// Note: For this minimal version, we only support blocking (timeout_ms < 0) and non-blocking (timeout_ms == 0)
rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms);

// Release borrowed message (must call after consuming IPC_BORROW message)
void rt_ipc_release(const rt_message *msg);

// Query mailbox state
bool rt_ipc_pending(void);
size_t rt_ipc_count(void);

// Clear mailbox entries (used during actor cleanup)
void rt_ipc_mailbox_clear(mailbox *mbox);

// Free active message entry (used during actor cleanup)
void rt_ipc_free_active_msg(mailbox_entry *entry);

#endif // RT_IPC_H
