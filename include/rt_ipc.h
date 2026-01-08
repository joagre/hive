#ifndef RT_IPC_H
#define RT_IPC_H

#include "rt_types.h"
#include "rt_actor.h"

// -----------------------------------------------------------------------------
// Core Send/Receive
// -----------------------------------------------------------------------------

// Send a fire-and-forget message (RT_MSG_CAST)
// Payload is copied to receiver's mailbox, sender continues immediately.
// Returns RT_ERR_NOMEM if IPC pools exhausted.
rt_status rt_ipc_send(actor_id to, const void *data, size_t len);

// Receive any message (FIFO order)
// timeout_ms == 0:  non-blocking, returns RT_ERR_WOULDBLOCK if empty
// timeout_ms < 0:   block forever
// timeout_ms > 0:   block up to timeout, returns RT_ERR_TIMEOUT if exceeded
rt_status rt_ipc_recv(rt_message *msg, int32_t timeout_ms);

// Receive message matching filters (selective receive)
// Pass NULL for any filter parameter to accept any value.
// Pass RT_SENDER_ANY, RT_MSG_ANY, or RT_TAG_ANY to match any.
// Scans mailbox for first matching message (O(n) worst case).
rt_status rt_ipc_recv_match(const actor_id *from, const rt_msg_class *class,
                            const uint32_t *tag, rt_message *msg, int32_t timeout_ms);

// -----------------------------------------------------------------------------
// RPC Pattern (Request/Reply)
// -----------------------------------------------------------------------------

// Send request and wait for reply (blocking RPC)
// Sends RT_MSG_CALL with generated tag, blocks until RT_MSG_REPLY received.
// The reply message is returned in 'reply'.
rt_status rt_ipc_call(actor_id to, const void *request, size_t req_len,
                      rt_message *reply, int32_t timeout_ms);

// Reply to a call message
// Extracts tag from request header and sends RT_MSG_REPLY.
// 'request' must be a RT_MSG_CALL message from the current rt_ipc_recv().
rt_status rt_ipc_reply(const rt_message *request, const void *data, size_t len);

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

// Decode message header fields
// Extracts class, tag, and payload from raw message data.
// Pass NULL for any field you don't need.
rt_status rt_msg_decode(const rt_message *msg, rt_msg_class *class,
                        uint32_t *tag, const void **payload, size_t *payload_len);

// Check if message is a timer tick
bool rt_msg_is_timer(const rt_message *msg);

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

// Query mailbox state
bool rt_ipc_pending(void);
size_t rt_ipc_count(void);

// -----------------------------------------------------------------------------
// Internal (used by runtime)
// -----------------------------------------------------------------------------

// Clear mailbox entries (used during actor cleanup)
void rt_ipc_mailbox_clear(mailbox *mbox);

// Free active message entry (used during actor cleanup)
void rt_ipc_free_active_msg(mailbox_entry *entry);

// Send with explicit class and tag (used internally by timer, link, etc.)
rt_status rt_ipc_send_ex(actor_id to, actor_id sender, rt_msg_class class,
                         uint32_t tag, const void *data, size_t len);

#endif // RT_IPC_H
