# Actor Runtime Specification

This directory contains the complete design specification for the Hive actor runtime.

## Document Structure

| Document | Description |
|----------|-------------|
| [design.md](design.md) | System design, architecture, and guarantees |
| [api.md](api.md) | Complete API reference |
| [internals.md](internals.md) | Implementation details and platform abstraction |

---

## Table of Contents

### Design & Architecture ([design.md](design.md))

**Introduction**
- [Overview](design.md#overview)
- [Target Platforms](design.md#target-platforms)
- [Guarantees and Non-Guarantees](design.md#guarantees-and-non-guarantees)
- [Failure Modes Summary](design.md#failure-modes-summary)
- [Design Trade-offs and Sharp Edges](design.md#design-trade-offs-and-sharp-edges)

**System Architecture**
- [Architecture](design.md#architecture)
- [Scheduling](design.md#scheduling)
- [Thread Safety](design.md#thread-safety)

**Memory & Resources**
- [Memory Model](design.md#memory-model)
- [Architectural Limits](design.md#architectural-limits)

**Error Handling**
- [Error Handling](design.md#error-handling)

### API Reference ([api.md](api.md))

- [Core Types](api.md#core-types)
- [Actor API](api.md#actor-api)
- [IPC API](api.md#ipc-api)
- [Bus API](api.md#bus-api)
- [Unified Event Waiting API](api.md#unified-event-waiting-api)
- [Timer API](api.md#timer-api)
- [Supervisor API](api.md#supervisor-api)
- [TCP API](api.md#tcp-api)
- [File API](api.md#file-api)
- [Logging API](api.md#logging-api)

### Implementation Details ([internals.md](internals.md))

- [Memory Allocation Architecture](internals.md#memory-allocation-architecture)
- [Actor Death Handling](internals.md#actor-death-handling)
- [Scheduler Main Loop](internals.md#scheduler-main-loop)
- [Event Loop Architecture](internals.md#event-loop-architecture)
- [Platform Abstraction](internals.md#platform-abstraction)
- [Stack Overflow](internals.md#stack-overflow)
- [Future Extensions](internals.md#future-extensions)
