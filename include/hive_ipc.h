#ifndef HIVE_IPC_H
#define HIVE_IPC_H

#include "hive_types.h"
#include "hive_actor.h"

// -----------------------------------------------------------------------------
// Core Send/Receive
// -----------------------------------------------------------------------------

// Send an async notification (HIVE_MSG_NOTIFY)
// Tag identifies the notification type, enabling selective receive.
// Payload is copied to receiver's mailbox, sender continues immediately.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status hive_ipc_notify(actor_id to, uint32_t tag, const void *data,
                            size_t len);

// Send a message with explicit class
// Like hive_ipc_notify, but allows specifying message class.
// Useful for implementing custom protocols beyond NOTIFY.
// The sender is automatically set to the current actor.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status hive_ipc_notify_ex(actor_id to, hive_msg_class class, uint32_t tag,
                               const void *data, size_t len);

// Receive any message (FIFO order)
// timeout_ms: HIVE_TIMEOUT_NONBLOCKING (0) returns HIVE_ERR_WOULDBLOCK if empty
//             HIVE_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns HIVE_ERR_TIMEOUT if
//             exceeded
hive_status hive_ipc_recv(hive_message *msg, int32_t timeout_ms);

// Receive message matching filters (selective receive)
// Pass HIVE_SENDER_ANY, HIVE_MSG_ANY, or HIVE_TAG_ANY to match any.
// Scans mailbox for first matching message (O(n) worst case).
hive_status hive_ipc_recv_match(actor_id from, hive_msg_class class,
                                uint32_t tag, hive_message *msg,
                                int32_t timeout_ms);

// Receive message matching ANY of the provided filters (multi-pattern receive)
// Waits until a message matches one of the filters in the array.
// If matched_index is non-NULL, stores which filter matched (0-based).
// Useful for waiting on multiple conditions (e.g., REPLY or EXIT, timer A or B).
hive_status hive_ipc_recv_match_any(const hive_recv_filter *filters,
                                    size_t num_filters, hive_message *msg,
                                    int32_t timeout_ms, size_t *matched_index);

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

// Send request and wait for reply (blocking)
// Sends HIVE_MSG_REQUEST with generated tag, blocks until HIVE_MSG_REPLY
// received. The reply message is returned in 'reply'.
hive_status hive_ipc_request(actor_id to, const void *request, size_t req_len,
                             hive_message *reply, int32_t timeout_ms);

// Reply to a request message
// Extracts tag from request header and sends HIVE_MSG_REPLY.
// 'request' must be a HIVE_MSG_REQUEST message from the current
// hive_ipc_recv().
hive_status hive_ipc_reply(const hive_message *request, const void *data,
                           size_t len);

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

// Check if message is a timer tick
bool hive_msg_is_timer(const hive_message *msg);

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

// Query mailbox state
bool hive_ipc_pending(void);
size_t hive_ipc_count(void);

#endif // HIVE_IPC_H
