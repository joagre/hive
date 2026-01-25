#ifndef HIVE_SELECT_H
#define HIVE_SELECT_H

#include "hive_types.h"

// -----------------------------------------------------------------------------
// hive_select - Unified Event Waiting API
// -----------------------------------------------------------------------------
//
// hive_select() blocks until data is available from any of the specified
// sources (IPC messages or bus entries). It is the single primitive that
// underlies all blocking receive operations.
//
// Priority: Sources are checked in strict array order. When multiple sources
// are ready simultaneously, the first one in the array wins. There is no
// type-based priority - bus and IPC sources are treated equally.
//
// Usage:
//   enum { SEL_STATE, SEL_TIMER, SEL_CMD };
//   hive_select_source_t sources[] = {
//       [SEL_STATE] = {HIVE_SEL_BUS, .bus = state_bus},
//       [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer_id}},
//       [SEL_CMD]   = {HIVE_SEL_IPC, .ipc = {HIVE_SENDER_ANY, HIVE_MSG_NOTIFY, CMD_TAG}},
//   };
//   hive_select_result_t result;
//   hive_select(sources, 3, &result, -1);
//
//   switch (result.index) {
//       case SEL_STATE: process_state(result.bus.data, result.bus.len); break;
//       case SEL_TIMER: handle_timer(); break;
//       case SEL_CMD:   handle_command(&result.ipc); break;
//   }
//
// Returns:
//   HIVE_OK - data available from one source (check result.index)
//   HIVE_ERR_TIMEOUT - timeout expired, no data available
//   HIVE_ERR_WOULDBLOCK - timeout_ms=0 and no data immediately available
//   HIVE_ERR_INVALID - invalid arguments (NULL pointers, unsubscribed bus)
//
// Data lifetime:
//   result.ipc - valid until next hive_select() or hive_ipc_recv*() call
//   result.bus.data - valid until next hive_select() or hive_bus_read*() call

hive_status_t hive_select(const hive_select_source_t *sources,
                          size_t num_sources, hive_select_result_t *result,
                          int32_t timeout_ms);

#endif // HIVE_SELECT_H
