# Known Issues

## Bus Example Deadlock (Post Event Loop Migration)

**Status:** Open
**Severity:** Medium
**Affects:** `examples/bus.c` only
**Since:** Event loop migration (commit TBD)

### Description

The bus example (`./build/bus`) hangs indefinitely when all actors are blocked waiting for I/O:
- Publisher blocks in `rt_ipc_recv()` waiting for timer ticks
- Subscribers block in `rt_bus_read_wait()` waiting for bus messages

The periodic timer is created successfully and should fire every 200ms to wake the publisher, but the scheduler appears to stop dispatching timer events.

### Symptoms

```
$ timeout 2 ./build/bus
=== Actor Runtime Bus Example ===

Created sensor bus (ID: 1)

Spawned actors: publisher=3, subscriber_a=1, subscriber_b=2

INFO  Scheduler started
Publisher actor started (ID: 3)
Publisher: Created periodic timer (200ms)
Subscriber A actor started (ID: 1)
Subscriber A: Subscribed to sensor bus
Subscriber B actor started (ID: 2)
Subscriber B: Subscribed to sensor bus

[hangs indefinitely - no timer ticks delivered]
```

### Working Examples

All other examples work correctly:
- ✅ `./build/timer` - Timers work (one-shot and periodic)
- ✅ `./build/fileio` - Synchronous file I/O works
- ✅ `./build/echo` - Network I/O with epoll works
- ✅ `./build/pingpong` - IPC messaging works
- ✅ `./build/link_demo` - Actor linking works
- ✅ `./build/supervisor` - Supervision works

### Investigation Notes

1. Actors spawn and start successfully
2. Timer is created and registered with epoll successfully
3. All 3 actors transition to BLOCKED state correctly
4. Scheduler loop appears to stop executing after actors block
5. No epoll events are being dispatched (timer events should fire every 200ms)

### Hypothesis

Likely related to scheduler context switching when all actors are blocked. The issue may be:
- Context switch from actor→scheduler not properly restoring scheduler execution
- Or a subtle interaction between bus blocking operations and the event loop

This appears to be specific to the combination of:
- Multiple blocking actors (bus read + IPC recv)
- Timer-driven wakeup pattern
- Bus subscription state

### Workaround

None available. Avoid using the bus example until this is resolved.

### Priority

Medium - Does not affect core functionality. Bus subsystem works in other contexts,
and all other I/O operations (timers, network, file, IPC) work correctly with the
event loop.

### Next Steps

1. Investigate with gdb to trace scheduler execution after actors block
2. Create minimal reproduction case (2 actors: one blocking on bus, one on timer)
3. Add detailed scheduler state logging
4. Check if this is a pre-existing issue or specific to event loop migration
