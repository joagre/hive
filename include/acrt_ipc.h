#ifndef ACRT_IPC_H
#define ACRT_IPC_H

#include "acrt_types.h"
#include "acrt_actor.h"

// -----------------------------------------------------------------------------
// Core Send/Receive
// -----------------------------------------------------------------------------

// Send an async notification (ACRT_MSG_ASYNC)
// Payload is copied to receiver's mailbox, sender continues immediately.
// Returns ACRT_ERR_NOMEM if IPC pools exhausted.
acrt_status acrt_ipc_notify(actor_id to, const void *data, size_t len);

// Receive any message (FIFO order)
// timeout_ms: ACRT_TIMEOUT_NONBLOCKING (0) returns ACRT_ERR_WOULDBLOCK if empty
//             ACRT_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns ACRT_ERR_TIMEOUT if exceeded
acrt_status acrt_ipc_recv(acrt_message *msg, int32_t timeout_ms);

// Receive message matching filters (selective receive)
// Pass NULL for any filter parameter to accept any value.
// Pass ACRT_SENDER_ANY, ACRT_MSG_ANY, or ACRT_TAG_ANY to match any.
// Scans mailbox for first matching message (O(n) worst case).
acrt_status acrt_ipc_recv_match(const actor_id *from, const acrt_msg_class *class,
                            const uint32_t *tag, acrt_message *msg, int32_t timeout_ms);

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

// Send request and wait for reply (blocking)
// Sends ACRT_MSG_REQUEST with generated tag, blocks until ACRT_MSG_REPLY received.
// The reply message is returned in 'reply'.
acrt_status acrt_ipc_request(actor_id to, const void *request, size_t req_len,
                         acrt_message *reply, int32_t timeout_ms);

// Reply to a request message
// Extracts tag from request header and sends ACRT_MSG_REPLY.
// 'request' must be a ACRT_MSG_REQUEST message from the current acrt_ipc_recv().
acrt_status acrt_ipc_reply(const acrt_message *request, const void *data, size_t len);

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

// Decode message header fields
// Extracts class, tag, and payload from raw message data.
// Pass NULL for any field you don't need.
acrt_status acrt_msg_decode(const acrt_message *msg, acrt_msg_class *class,
                        uint32_t *tag, const void **payload, size_t *payload_len);

// Check if message is a timer tick
bool acrt_msg_is_timer(const acrt_message *msg);

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

// Query mailbox state
bool acrt_ipc_pending(void);
size_t acrt_ipc_count(void);

#endif // ACRT_IPC_H
