#ifndef RT_IPC_H
#define RT_IPC_H

#include "rt_types.h"
#include "rt_actor.h"

// Send message to actor
//
// IPC_ASYNC: Payload copied to receiver's mailbox, sender continues immediately
//   - Safe default for most use cases
//   - Suitable for small messages and general communication
//
// IPC_SYNC: Payload copied to sync buffer pool, sender blocks until receiver releases
//   - WARNING: REQUIRES CAREFUL USE - see spec.md "IPC_SYNC Safety Considerations"
//   - Actor context only (not I/O threads or completion handlers)
//   - Data copied to pinned runtime buffer (NOT sender's stack, eliminates UAF)
//   - Sender blocks and cannot process other messages
//   - Risk of deadlock with circular/nested synchronous sends
//   - Use for: Flow control, backpressure, trusted cooperating actors
//
// Returns RT_ERR_NOMEM if IPC pools exhausted (see spec.md for handling)
rt_status rt_ipc_send(actor_id to, const void *data, size_t len, rt_ipc_mode mode);

// Receive message
// timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded
rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms);

// Release sync message (must call after consuming IPC_SYNC message)
void rt_ipc_release(const rt_message *msg);

// Query mailbox state
bool rt_ipc_pending(void);
size_t rt_ipc_count(void);

// Clear mailbox entries (used during actor cleanup)
void rt_ipc_mailbox_clear(mailbox *mbox);

// Free active message entry (used during actor cleanup)
void rt_ipc_free_active_msg(mailbox_entry *entry);

#endif // RT_IPC_H
