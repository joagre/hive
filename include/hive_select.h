#ifndef HIVE_SELECT_H
#define HIVE_SELECT_H

#include "hive_types.h"

// -----------------------------------------------------------------------------
// hive_select - Unified Event Waiting API
// -----------------------------------------------------------------------------
//
// hive_select() blocks until data is available from any of the specified
// sources (IPC messages, bus entries, or HAL events). It is the single
// primitive that underlies all blocking receive operations.
//
// Source types:
//   HIVE_SEL_IPC       - Wait for IPC message matching filter
//   HIVE_SEL_BUS       - Wait for bus data
//   HIVE_SEL_HAL_EVENT - Wait for hardware interrupt (ISR-safe)
//
// Priority: Sources are checked in strict array order. When multiple sources
// are ready simultaneously, the first one in the array wins. There is no
// type-based priority - all source types are treated equally.
//
// Usage:
//   enum { SEL_STATE, SEL_TIMER, SEL_UART };
//   hive_select_source_t sources[] = {
//       [SEL_STATE] = {.type = HIVE_SEL_BUS, .bus = state_bus},
//       [SEL_TIMER] = {.type = HIVE_SEL_IPC,
//                      .ipc = {.class = HIVE_MSG_TIMER, .tag = timer_id}},
//       [SEL_UART]  = {.type = HIVE_SEL_HAL_EVENT, .event = uart_rx_event},
//   };
//   hive_select_result_t result;
//   hive_select(sources, 3, &result, -1);
//
//   switch (result.index) {
//   case SEL_STATE:
//       process_state(result.bus.data, result.bus.len);
//       break;
//   case SEL_TIMER:
//       handle_timer();
//       break;
//   case SEL_UART:
//       hal_uart_read(buf, len);  // HAL event has no data, call HAL to get it
//       break;
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
//   HAL events have no data payload - just a signal that the event occurred

hive_status_t hive_select(const hive_select_source_t *sources,
                          size_t num_sources, hive_select_result_t *result,
                          int32_t timeout_ms);

// -----------------------------------------------------------------------------
// hive_event_wait - Convenience wrapper for single HAL event
// -----------------------------------------------------------------------------
//
// Blocks until the specified HAL event is signaled.
// This is a thin wrapper around hive_select() for the common case of
// waiting on a single hardware event.
//
// Usage:
//   hive_hal_event_id_t rx_event = hal_uart_get_rx_event();
//   while (true) {
//       hive_event_wait(rx_event, -1);  // Block until event
//       hal_uart_read(buf, len);        // Process the data
//   }
//
// Returns:
//   HIVE_OK - event was signaled (and cleared)
//   HIVE_ERR_TIMEOUT - timeout expired
//   HIVE_ERR_WOULDBLOCK - timeout_ms=0 and event not set

hive_status_t hive_event_wait(hive_hal_event_id_t event, int32_t timeout_ms);

#endif // HIVE_SELECT_H
