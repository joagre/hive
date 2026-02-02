#ifndef HIVE_IPC_H
#define HIVE_IPC_H

#include "hive_types.h"

// -----------------------------------------------------------------------------
// Core Notify/Receive
// -----------------------------------------------------------------------------

// Notify asynchronously (HIVE_MSG_NOTIFY)
// Tag identifies the notification type, enabling selective receive.
// Payload is copied to receiver's mailbox, caller continues immediately.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status_t hive_ipc_notify(hive_actor_id_t to, uint32_t tag,
                              const void *data, size_t len);

// Notify with explicit class
// Like hive_ipc_notify, but allows specifying message class.
// Useful for implementing custom protocols beyond NOTIFY.
// The sender is automatically set to the current actor.
// Returns HIVE_ERR_NOMEM if IPC pools exhausted.
hive_status_t hive_ipc_notify_ex(hive_actor_id_t to, hive_msg_class_t class,
                                 uint32_t tag, const void *data, size_t len);

// Receive any message (FIFO order)
// timeout_ms: HIVE_TIMEOUT_NONBLOCKING (0) returns HIVE_ERR_WOULDBLOCK if empty
//             HIVE_TIMEOUT_INFINITE (-1) blocks forever
//             positive value blocks up to timeout, returns HIVE_ERR_TIMEOUT if
//             exceeded
hive_status_t hive_ipc_recv(hive_message_t *msg, int32_t timeout_ms);

// Receive message matching filters (selective receive)
// Pass HIVE_SENDER_ANY, HIVE_MSG_ANY, or HIVE_TAG_ANY to match any.
// Scans mailbox for first matching message (O(n) worst case).
hive_status_t hive_ipc_recv_match(hive_actor_id_t from, hive_msg_class_t class,
                                  uint32_t tag, hive_message_t *msg,
                                  int32_t timeout_ms);

// Receive message matching ANY of the provided filters (multi-pattern receive)
// Waits until a message matches one of the filters in the array.
// If matched_index is non-NULL, stores which filter matched (0-based).
// Useful for waiting on multiple conditions (e.g., REPLY or EXIT, timer A or B).
hive_status_t hive_ipc_recv_matches(const hive_recv_filter_t *filters,
                                    size_t num_filters, hive_message_t *msg,
                                    int32_t timeout_ms, size_t *matched_index);

// -----------------------------------------------------------------------------
// Request/Reply Pattern
// -----------------------------------------------------------------------------

// Request and wait for reply (blocking)
// Dispatches HIVE_MSG_REQUEST with generated tag, blocks until HIVE_MSG_REPLY
// received. The reply message is returned in 'reply'.
hive_status_t hive_ipc_request(hive_actor_id_t to, const void *request,
                               size_t req_len, hive_message_t *reply,
                               int32_t timeout_ms);

// Reply to a request message
// Extracts tag from request header and dispatches HIVE_MSG_REPLY.
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
hive_status_t hive_ipc_named_notify(const char *name, uint32_t tag,
                                    const void *data, size_t len);

// Request actor by name and wait for reply
// Resolves name via hive_whereis(), then requests.
// Returns HIVE_ERR_NOT_FOUND if name not registered.
hive_status_t hive_ipc_named_request(const char *name, const void *request,
                                     size_t req_len, hive_message_t *reply,
                                     int32_t timeout_ms);

// -----------------------------------------------------------------------------
// Query Functions
// -----------------------------------------------------------------------------

// Query mailbox state
bool hive_ipc_pending(void);
size_t hive_ipc_count(void);

#endif // HIVE_IPC_H
