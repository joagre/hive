#ifndef HIVE_IPC_H
#define HIVE_IPC_H

#include "hive_types.h"

// -----------------------------------------------------------------------------
// Core Notify/Receive
// -----------------------------------------------------------------------------

// Notify asynchronously (HIVE_MSG_NOTIFY)
// id identifies the notification type, enabling selective receive by message type.
// Payload is copied to receiver's mailbox, caller continues immediately.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status_t hive_ipc_notify(hive_actor_id_t to, uint16_t id, const void *data,
                              size_t len);

// Notify with explicit class
// Like hive_ipc_notify, but allows specifying message class.
// Useful for implementing custom protocols beyond NOTIFY.
// The sender is automatically set to the current actor.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status_t hive_ipc_notify_ex(hive_actor_id_t to, hive_msg_class_t class,
                                 uint16_t id, const void *data, size_t len);

// Receive any message (FIFO order)
// timeout_ms: HIVE_TIMEOUT_NONBLOCKING (0) returns HIVE_ERR_WOULDBLOCK if empty
//             HIVE_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns HIVE_ERR_TIMEOUT if
//             exceeded
hive_status_t hive_ipc_recv(hive_message_t *msg, int32_t timeout_ms);

// Receive message matching filters (selective receive)
// Pass HIVE_SENDER_ANY, HIVE_MSG_ANY, HIVE_ID_ANY to match any.
// Scans mailbox for first matching message (O(n) worst case).
hive_status_t hive_ipc_recv_match(hive_actor_id_t from, hive_msg_class_t class,
                                  uint16_t id, hive_message_t *msg,
                                  int32_t timeout_ms);

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

// Request and wait for reply (blocking)
// id identifies the request type for selective receive on the receiver side.
// Dispatches HIVE_MSG_REQUEST with id and auto-generated tag for correlation.
// Blocks until HIVE_MSG_REPLY received. The reply message is returned in 'reply'.
hive_status_t hive_ipc_request(hive_actor_id_t to, uint16_t id,
                               const void *request, size_t req_len,
                               hive_message_t *reply, int32_t timeout_ms);

// Reply to a request message
// Extracts id and tag from request header and dispatches HIVE_MSG_REPLY.
// 'request' must be a HIVE_MSG_REQUEST message from the current
// hive_ipc_recv().
hive_status_t hive_ipc_reply(const hive_message_t *request, const void *data,
                             size_t len);

// -----------------------------------------------------------------------------
// Message Inspection
// -----------------------------------------------------------------------------

// Check if message is a timer tick
bool hive_msg_is_timer(const hive_message_t *msg);

// -----------------------------------------------------------------------------
// Named IPC (uses name registry under the hood)
// -----------------------------------------------------------------------------

// Notify actor by name
// Resolves name via hive_whereis(), then notifies.
// Returns HIVE_ERR_NOT_FOUND if name not registered.
hive_status_t hive_ipc_named_notify(const char *name, uint16_t id,
                                    const void *data, size_t len);

// Request actor by name and wait for reply
// Resolves name via hive_whereis(), then requests.
// Returns HIVE_ERR_NOT_FOUND if name not registered.
hive_status_t hive_ipc_named_request(const char *name, uint16_t id,
                                     const void *request, size_t req_len,
                                     hive_message_t *reply, int32_t timeout_ms);

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

// Query mailbox state
bool hive_ipc_pending(void);
size_t hive_ipc_count(void);

#endif // HIVE_IPC_H
