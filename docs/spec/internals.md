# Implementation Details

This document covers implementation internals, platform abstraction, and potential future extensions.

---

## Memory Allocation Architecture

The runtime uses **compile-time configuration** for bounded, predictable memory allocation.

### Compile-Time Configuration (`hive_static_config.h`)

All resource limits are defined at compile-time and require recompilation to change:

```c
// Feature toggles (set to 0 to disable)
#define HIVE_ENABLE_TCP 1                     // TCP subsystem
#define HIVE_ENABLE_FILE 1                    // File I/O subsystem
#define HIVE_ENABLE_SD 0                      // SD card support (STM32 only, requires FatFS)

// Resource limits
#define HIVE_MAX_ACTORS 64                    // Maximum concurrent actors
#define HIVE_STACK_ARENA_SIZE (1*1024*1024)   // Stack arena size (1 MB)
#define HIVE_MAX_BUSES 32                     // Maximum concurrent buses
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 256      // Mailbox entry pool
#define HIVE_MESSAGE_DATA_POOL_SIZE 256       // Message data pool
#define HIVE_MAX_MESSAGE_SIZE 256             // Maximum message size
#define HIVE_LINK_ENTRY_POOL_SIZE 128         // Link entry pool
#define HIVE_MONITOR_ENTRY_POOL_SIZE 128      // Monitor entry pool
#define HIVE_TIMER_ENTRY_POOL_SIZE 64         // Timer entry pool
#define HIVE_DEFAULT_STACK_SIZE 65536         // Default actor stack size

// Stack watermarking (for measuring actual stack usage)
#define HIVE_STACK_WATERMARK 0                // 0=disabled, 1=enabled
#define HIVE_STACK_WATERMARK_PATTERN 0xDEADBEEF  // Pattern for watermark

// Supervisor limits
#define HIVE_MAX_SUPERVISOR_CHILDREN 16       // Max children per supervisor
#define HIVE_MAX_SUPERVISORS 8                // Max concurrent supervisors

// SD card limits (STM32 only, when HIVE_ENABLE_SD=1)
#define HIVE_MAX_SD_FILES 4                   // Max concurrent open files on SD card
```

Feature toggles can also be set via Makefile: `make ENABLE_TCP=0 ENABLE_FILE=0` or `make ENABLE_SD=1`.

All runtime structures are **statically allocated** based on these limits. Actor stacks use a static arena allocator by default (configurable via `actor_config_t.malloc_stack` for malloc). This ensures:
- Bounded memory footprint (calculable at link time)
- Zero heap allocation in runtime operations (see [Heap Usage Policy](design.md#heap-usage-policy))
- O(1) pool allocation for hot paths (scheduling, IPC); O(n) bounded arena allocation for cold paths (spawn/exit)
- Suitable for embedded/MCU deployment

### Memory Sizing Guide

Use these formulae to calculate memory requirements for your application.

**Fixed overhead**
```
Runtime base          ~12 KB   (scheduler, pools metadata, context data)
Actor table           HIVE_MAX_ACTORS x 72 bytes
Bus table             HIVE_MAX_BUSES x 64 bytes
Supervisor table      HIVE_MAX_SUPERVISORS x (80 + HIVE_MAX_SUPERVISOR_CHILDREN x 32) bytes
Name registry         HIVE_MAX_REGISTERED_NAMES x 40 bytes
```

**Pool memory**
```
Mailbox entries       HIVE_MAILBOX_ENTRY_POOL_SIZE x 24 bytes
Message data          HIVE_MESSAGE_DATA_POOL_SIZE x (HIVE_MAX_MESSAGE_SIZE + 8) bytes
Link entries          HIVE_LINK_ENTRY_POOL_SIZE x 16 bytes
Monitor entries       HIVE_MONITOR_ENTRY_POOL_SIZE x 20 bytes
Timer entries         HIVE_TIMER_ENTRY_POOL_SIZE x 24 bytes
```

**Stack memory**
```
Stack arena           HIVE_STACK_ARENA_SIZE (default 1 MB)
  - or -
Per-actor stacks      Sum of all actor stack_size values (if malloc_stack=true)
```

**Sizing formulae**

| Parameter | Formula | Notes |
|-----------|---------|-------|
| `HIVE_MAX_ACTORS` | Peak concurrent actors | Include supervisor(s) |
| `HIVE_MAX_BUSES` | Number of buses created | Usually 1 per data stream |
| `HIVE_MAILBOX_ENTRY_POOL_SIZE` | 1.5x peak pending messages | Shared across all actors |
| `HIVE_MESSAGE_DATA_POOL_SIZE` | 1.5x peak pending (IPC + bus entries) | Includes bus data |
| `HIVE_TIMER_ENTRY_POOL_SIZE` | Active timers x 2 | Include margin for overlapping timers |
| `HIVE_STACK_ARENA_SIZE` | Sum of actor stacks + 20% | Fragmentation overhead |

**Example: Flight controller (pilot example)**
```c
// 10-11 actors (9 children + supervisor + optional comms/tlog), 7 buses
#define HIVE_MAX_ACTORS 13
#define HIVE_MAX_BUSES 8
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 32    // Low IPC usage
#define HIVE_MESSAGE_DATA_POOL_SIZE 64     // 7 buses x 4 entries each + margin
#define HIVE_TIMER_ENTRY_POOL_SIZE 10      // Few timers
#define HIVE_MAX_SUPERVISORS 1             // Single supervisor
#define HIVE_MAX_SUPERVISOR_CHILDREN 12    // 9-11 children + margin
#define HIVE_STACK_ARENA_SIZE (64*1024)    // 64 KB (STM32 constrained)
// Total RAM: ~90 KB (fits in STM32F405's 192 KB)
```

**Example: Server application (many actors, heavy IPC)**
```c
// 50+ actors, high message throughput
#define HIVE_MAX_ACTORS 64
#define HIVE_MAX_BUSES 16
#define HIVE_MAILBOX_ENTRY_POOL_SIZE 512   // Heavy IPC traffic
#define HIVE_MESSAGE_DATA_POOL_SIZE 512    // Many pending messages
#define HIVE_TIMER_ENTRY_POOL_SIZE 128     // Many timers
#define HIVE_MAX_SUPERVISORS 8             // Supervision hierarchy
#define HIVE_MAX_SUPERVISOR_CHILDREN 16    // Workers per supervisor
#define HIVE_STACK_ARENA_SIZE (2*1024*1024) // 2 MB (Linux desktop)
// Total RAM: ~2.5 MB
```

### Runtime API

```c
// Initialize runtime (call once from main)
hive_status_t hive_init(void);

// Run scheduler (blocks until all actors exit or hive_shutdown called)
void hive_run(void);

// Request graceful shutdown
void hive_shutdown(void);

// Cleanup runtime resources (call after hive_run completes)
void hive_cleanup(void);
```

## Actor Death Handling

When an actor dies (via `hive_exit(reason)`, return from actor function, or external kill):

**Note** - Stack overflow results in undefined behavior since there is no runtime detection. The cleanup below assumes normal actor death. If stack overflow corrupts runtime metadata, the system may crash before cleanup completes.

**Normal death cleanup**

1. **Mailbox cleared** - All pending messages are discarded.

2. **Links notified** - All linked actors receive an exit message (class=`HIVE_MSG_EXIT`, sender=dying actor).

3. **Monitors notified** - All monitoring actors receive an exit message (class=`HIVE_MSG_EXIT`).

4. **Bus subscriptions removed** - Actor is unsubscribed from all buses.

5. **Timers cancelled** - All timers owned by the actor are cancelled.

6. **Resources freed** - Stack and actor table entry are released.

### Exit Notification Ordering

**When exit notifications are sent**

Exit notifications (steps 2-3 above) are enqueued in recipient mailboxes **during death processing**, following standard FIFO mailbox semantics.

**Ordering guarantees**

1. **Tail-append semantics**
   - Exit notifications are enqueued at the **tail** of recipient mailboxes at the moment death processing occurs
   - Any messages already in that mailbox before that point remain ahead of the exit notification
   - Example: If B's mailbox contains [M1, M2] when A's death is processed, B receives: M1 -> M2 -> EXIT(A)

2. **No global ordering guarantee**
   - The runtime does NOT guarantee ordering relative to concurrent events (timers, TCP)
   - If death processing and other enqueues occur in the same scheduling phase, ordering depends on processing order
   - Example: If A dies and a timer fires for B in the same phase, the order of EXIT(A) vs timer tick in B's mailbox depends on event dispatch order

3. **Messages in dying actor's mailbox**
   - Dying actor's mailbox is **cleared** (step 1)
   - All pending messages are **discarded**
   - Senders are **not** notified of message loss

4. **Multiple recipients**
   - Each recipient's exit notification is enqueued independently
   - No ordering guarantee across different recipients
   - Example: A linked to B and C, dies. B and C both receive EXIT(A), but no guarantee which processes it first.

**Consequences**

```c
// Dying actor sends messages before death
void actor_A(void *arg) {
    hive_ipc_notify(B, HIVE_TAG_NONE, &msg1, sizeof(msg1));  // Enqueued in B's mailbox
    hive_ipc_notify(C, HIVE_TAG_NONE, &msg2, sizeof(msg2));  // Enqueued in C's mailbox
    return;  // Exit notifications sent to links/monitors
}
// Linked actor B will receive: msg1, then EXIT(A) (class=HIVE_MSG_EXIT)
// Actor C will receive: msg2 (no exit notification, not linked)

// Messages sent TO dying actor are lost
void actor_B(void *arg) {
    hive_ipc_notify(A, HIVE_TAG_NONE, &msg, sizeof(msg));  // Enqueued in A's mailbox
}
void actor_A(void *arg) {
    // ... does some work ...
    return;  // Mailbox cleared, msg from B is discarded
}
```

**Design rationale**
- FIFO enqueuing = predictable, testable behavior
- Messages before death delivered before exit = "happens-before" relationship
- Mailbox clearing = simple cleanup, no orphaned messages
- No sender notification = simpler implementation, sender must handle recipient death via links/monitors

**Design choice** - Exit notifications follow standard FIFO mailbox ordering, enqueued at tail when death is processed. This provides simpler, more predictable semantics.

## Scheduler Main Loop

Pseudocode for the scheduler:

```
procedure hive_run():
    epoll_fd = epoll_create1(0)

    while not shutdown_requested and actors_alive > 0:
        # 1. Pick highest-priority runnable actor
        actor = pick_next_runnable()

        if actor exists:
            # 2. Context switch to actor
            current_actor = actor
            context_switch(scheduler_ctx, actor.ctx)
            # Returns here when actor yields/blocks

        else:
            # 3. No runnable actors, wait for I/O events
            events = epoll_wait(epoll_fd, timeout=10ms)  # Bounded timeout for defensive wakeup

            # 4. Dispatch I/O events
            for event in events:
                source = event.data.ptr
                if source.type == TIMER:
                    read(timerfd, &expirations, 8)  # Clear level-triggered, get count
                    send_timer_tick(source.owner)   # One tick regardless of count
                    wake_actor(source.owner)
                elif source.type == TCP:
                    perform_io_operation(source)  # recv/send partial, connect checks SO_ERROR
                    wake_actor(source.owner)
```

### Simulation Time Integration

For integration with external event loops (e.g., Webots simulator), use `hive_advance_time()` and `hive_run_until_blocked()`:

```
procedure hive_advance_time(delta_us):
    # Enable simulation time mode on first call
    if not sim_mode:
        sim_mode = true

    # Advance simulation time
    sim_time_us += delta_us

    # Fire all timers that are now due
    for timer in active_timers:
        while timer.expiry_us <= sim_time_us:
            send_timer_message(timer.owner, timer.id)
            if timer.periodic:
                timer.expiry_us += timer.interval_us
            else:
                remove_timer(timer)
                break

procedure hive_run_until_blocked():
    # Run actors until all are blocked (WAITING) or dead
    while not shutdown_requested and num_actors > 0:
        # Poll for I/O events (non-blocking)
        events = epoll_wait(epoll_fd, timeout=0)
        dispatch_io_events(events)

        # Find next ready actor (priority-based round-robin)
        actor = find_next_runnable()
        if actor is None:
            break  # All actors blocked or dead

        context_switch(scheduler_ctx, actor.ctx)

    return HIVE_OK
```

**Use case** - Simulation integration where an external loop controls time:
```c
// All actors use timers (same code as production)
void sensor_actor(void *arg) {
    timer_id_t timer;
    hive_timer_every(TIME_STEP_MS * 1000, &timer);  // Timer-driven
    while (1) {
        hive_message_t msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);
        read_sensors();
        publish_to_bus();
    }
}

// Main loop advances simulation time
while (wb_robot_step(TIME_STEP_MS) != -1) {
    hive_advance_time(TIME_STEP_MS * 1000);  // Fire due timers
    hive_run_until_blocked();                 // Run until all blocked
}
```

**Benefits of simulation time mode**
- Same actor code runs in simulation and production
- Deterministic, reproducible behavior
- Timer granularity matches simulation time step
- No wall-clock dependencies in actors

### Simulation Invariant

**`hive_run_until_blocked()` REQUIRES actors to eventually block.** If any actor runs without blocking, the function loops forever.

**The contract**
- Every actor MUST either block (recv, select, bus_read_wait) or exit within finite computation
- Actors MUST NOT spin-poll without yielding
- This is a correctness requirement, not a suggestion

**Correct pattern**
```c
void sensor_actor(void *arg) {
    timer_id_t timer;
    hive_timer_every(TIME_STEP_MS * 1000, &timer);
    while (1) {
        hive_message_t msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);  // BLOCKS
        read_sensors();
        publish_to_bus();
    }
}
```

**INCORRECT pattern (will hang simulation)**
```c
void bad_actor(void *arg) {
    while (1) {
        // NO BLOCKING CALL - hive_run_until_blocked() loops forever
        do_work();
    }
}
```

**Why this matters**
- Simulation requires lockstep coordination: advance time, run until blocked, repeat
- Actors that don't block break this contract
- In production (`hive_run()`), non-blocking actors would starve others, but the system keeps running
- In simulation (`hive_run_until_blocked()`), the system hangs

**Diagnosis** - If simulation appears to hang, check that all actors have a blocking call in their main loop. Common fixes:
- Add timer-driven loop (most actors should use this)
- Add bus_read_wait for bus consumers
- Add recv/recv_match for IPC consumers

## Event Loop Architecture

When all actors are blocked on I/O, the scheduler waits efficiently for I/O events using platform-specific event notification mechanisms.

### Implementation

**Linux (epoll)**
```c
// Scheduler event loop
int epoll_fd = epoll_create1(0);

// Timer creation - add timerfd to epoll
int tfd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
timerfd_settime(tfd, 0, &its, NULL);
struct epoll_event ev = {.events = EPOLLIN, .data.ptr = timer_source};
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, tfd, &ev);

// TCP - add socket to epoll when would block
int sock = socket(AF_INET, SOCK_STREAM, 0);
fcntl(sock, F_SETFL, O_NONBLOCK);
struct epoll_event ev = {.events = EPOLLIN, .data.ptr = net_source};
epoll_ctl(epoll_fd, EPOLL_CTL_ADD, sock, &ev);

// Scheduler waits when no actors are ready
if (no_runnable_actors) {
    struct epoll_event events[64];
    // Bounded timeout provides defensive wakeup (guards against lost events)
    int n = epoll_wait(epoll_fd, events, 64, 10);  // 10ms timeout
    if (n < 0 && errno == EINTR) continue;  // Signal interrupted, retry
    for (int i = 0; i < n; i++) {
        io_source *source = events[i].data.ptr;
        if (source->type == TIMER) {
            uint64_t expirations;
            read(source->fd, &expirations, sizeof(expirations));  // Clear timerfd
            // expirations may be > 1 for periodic timers (coalesced)
            // We send ONE tick message regardless of count
        }
        dispatch_io_event(source);  // Handle timer tick or TCP
    }
}
```

**STM32 (bare metal with WFI)**
```c
// Interrupt handlers set flags
volatile uint32_t pending_events = 0;
#define EVENT_TIMER   (1 << 0)
#define EVENT_TCP (1 << 1)

void SysTick_Handler(void) {
    pending_events |= EVENT_TIMER;
}

void ETH_IRQHandler(void) {
    pending_events |= EVENT_TCP;
}

// Scheduler waits when no actors are ready
if (no_runnable_actors) {
    __disable_irq();
    if (pending_events == 0) {
        __WFI();  // Wait For Interrupt - CPU sleeps until interrupt
    }
    __enable_irq();

    // Process pending events
    if (pending_events & EVENT_TIMER) {
        pending_events &= ~EVENT_TIMER;
        handle_timer_event();
    }
    if (pending_events & EVENT_TCP) {
        pending_events &= ~EVENT_TCP;
        handle_tcp_event();
    }
}
```

### Semantics

- **Single-threaded event loop**: All I/O is multiplexed in the scheduler thread
- **Non-blocking I/O registration**: Timers create timerfds, TCP operations register sockets with epoll
- **Event dispatching**: epoll_wait returns when any I/O source becomes ready
- **Immediate handling**: Timer ticks and TCP are processed immediately when ready

### Event Loop Guarantees

**Event processing order**
- epoll returns events in **kernel order** (arrival-dependent, not guaranteed FIFO)
- All returned events are processed before next epoll_wait
- No I/O events are lost

**epoll_wait return handling**
- Returns > 0: Events ready, process them
- Returns 0: Timeout expired (10ms), no events - scheduler retries (defensive wakeup)
- Returns -1 with `errno=EINTR`: Signal interrupted syscall - scheduler retries
- Note: Runtime APIs are not reentrant - signal handlers must not call runtime APIs

**Lost event prevention**
- Timerfds are level-triggered (epoll reports ready until `read()` clears the event)
- Scheduler **must** read timerfd to clear level-triggered state (otherwise epoll spins)
- Sockets are level-triggered by default (readable until data consumed)
- epoll guarantees: if I/O is ready, epoll_wait will return it

**File I/O stalling**
- File operations (read, write, fsync) are **synchronous** and stall the entire scheduler
- See [Scheduler-Stalling Calls](design.md#scheduler-stalling-calls) section for detailed semantics and consequences
- On embedded systems: FATFS/littlefs operations are fast (< 1ms typical)
- On Linux dev: Acceptable for development workloads

### Advantages

- **Standard pattern**: Used by Node.js, Tokio, libuv, asyncio
- **Single-threaded**: No synchronization, no race conditions
- **Efficient**: Kernel/hardware wakes scheduler only when I/O is ready
- **Portable**: epoll (Linux), kqueue (BSD/macOS), WFI + interrupts (bare metal)
- **Low overhead**: No context switches between threads, no queue copies

## Platform Abstraction

The runtime uses a Hardware Abstraction Layer (HAL) to isolate platform-specific code. Porters implement HAL functions without needing to understand scheduler or timer internals.

### HAL Architecture

| Component | Linux (dev) | STM32 bare metal (prod) |
|-----------|-------------|-------------------------|
| Context switch | x86-64 asm | ARM Cortex-M asm |
| Event notification | epoll | WFI + interrupt flags |
| Timer | timerfd + epoll | Software timer wheel (SysTick/TIM) |
| TCP | Non-blocking BSD sockets + epoll | Stubs (future lwIP support) |
| File | Synchronous POSIX | Flash-backed virtual files + optional SD card via FatFS |

### HAL Headers

```
include/hal/
  hive_hal_time.h      - Time + critical sections (3 functions)
  hive_hal_event.h     - Event loop primitives (5 functions)
  hive_hal_timer.h     - Timer operations (6 functions)
  hive_hal_context.h   - Context switching (1 function + struct)
  hive_hal_file.h      - File I/O (8 functions, optional)
  hive_hal_tcp.h       - TCP (10 functions, optional)
  hive_mount.h         - Mount table types (backend enum, function declarations)
```

### HAL Functions

| Category | Functions | Required |
|----------|-----------|----------|
| Time | `get_time_us`, `critical_enter`, `critical_exit` | Yes |
| Event | `init`, `cleanup`, `poll`, `wait`, `register`, `unregister` | Yes |
| Timer | `init`, `cleanup`, `create`, `cancel`, `get_time`, `advance_time` | Yes |
| Context | `init` (C), `switch` (asm) | Yes |
| File | `init`, `cleanup`, `open`, `close`, `read`, `pread`, `write`, `pwrite`, `sync` | Optional |
| TCP | `init`, `cleanup`, `socket`, `bind`, `listen`, `accept`, `connect`, `connect_check`, `close`, `recv`, `send` | Optional |

**Minimum port** - ~16 C functions + 1 assembly function + 1 struct definition

### Platform-Specific Source Files

Platform implementations live in `src/hal/<platform>/`:

| Component | Linux | STM32 |
|-----------|-------|-------|
| Time HAL | `hal/linux/hive_hal_time.c` | `hal/stm32/hive_hal_time.c` |
| Event HAL | `hal/linux/hive_hal_event.c` | `hal/stm32/hive_hal_event.c` |
| Timer HAL | `hal/linux/hive_hal_timer.c` | `hal/stm32/hive_hal_timer.c` |
| File HAL | `hal/linux/hive_hal_file.c` | `hal/stm32/hive_hal_file.c` |
| TCP HAL | `hal/linux/hive_hal_tcp.c` | `hal/stm32/hive_hal_tcp.c` |
| Context init | `hal/linux/hive_hal_context.c` | `hal/stm32/hive_hal_context.c` |
| Context switch | `hal/linux/hive_context_x86_64.S` | `hal/stm32/hive_context_arm_cm.S` |
| Context struct | `hal/linux/hive_hal_context_defs.h` | `hal/stm32/hive_hal_context_defs.h` |
| Mount table | `hal/linux/hive_mounts.c` | `hal/stm32/hive_mounts.c` (or board-specific) |
| SD card driver | N/A | `hal/stm32/spi_sd.c` (conditional, `HIVE_ENABLE_SD`) |

Platform-independent wrappers call HAL functions:
- `hive_scheduler.c` - Unified scheduler (calls HAL event functions)
- `hive_timer.c` - Timer wrapper (calls HAL timer functions)
- `hive_file.c` - File I/O wrapper (calls HAL file functions)
- `hive_tcp.c` - TCP wrapper (calls HAL TCP functions)

### Porting Guide

Templates for new ports are in `src/hal/template/`:
- `hive_hal_time.c` - Time and critical section functions
- `hive_hal_event.c` - Event loop primitives
- `hive_hal_timer.c` - Timer HAL implementation
- `hive_hal_file.c` - File I/O (optional)
- `hive_hal_tcp.c` - TCP (optional)
- `hive_hal_context_defs.h` - Context struct template
- `hive_hal_context.c` - Context init template
- `hive_context.S` - Assembly template
- `README.md` - Complete porting guide

### Building for Different Platforms

```bash
# Linux (default)
make                           # Build for x86-64 Linux
make PLATFORM=linux            # Explicit

# STM32 (requires ARM cross-compiler)
make PLATFORM=stm32 CC=arm-none-eabi-gcc

# Feature toggles (disable TCP and file I/O)
make ENABLE_TCP=0 ENABLE_FILE=0

# STM32 defaults to ENABLE_TCP=0 ENABLE_FILE=1 ENABLE_SD=0
```

Platform selection sets `HIVE_PLATFORM_LINUX` or `HIVE_PLATFORM_STM32` preprocessor defines.

## Stack Overflow

### No Runtime Detection

The runtime does **not** perform stack overflow detection. Stack overflow results in **undefined behavior**.

**Rationale** - Pattern-based guard detection (checking magic values on context switch) was removed because:
1. Detection occurred too late - after memory corruption had already happened
2. Severe overflows corrupt memory beyond the guard, causing crashes before detection
3. The mechanism gave a false sense of security while not providing reliable protection
4. Proper stack sizing is the only reliable solution

### Behavior on Overflow

**Undefined behavior.** Possible outcomes:
- **Segfault** (most likely on Linux)
- **Corruption of adjacent actor stacks** (arena allocator places stacks contiguously)
- **Corruption of runtime state** (if overflow is severe)
- **Silent data corruption** (worst case - no immediate crash)

**Links/monitors are NOT notified.** The `HIVE_EXIT_REASON_CRASH_STACK` exit reason is reserved for future use; the runtime does not currently detect stack overflow.

### Required Mitigation

Stack overflow prevention is the **application's responsibility**:

1. **Size stacks conservatively** - Use 2-3x safety margin over measured worst-case
2. **Profile stack usage** - Measure actual usage under worst-case conditions
3. **Use static analysis** - Tools like `gcc -fstack-usage` report per-function stack use
4. **Test thoroughly** - Include stress tests with deep call stacks
5. **Use AddressSanitizer** - `-fsanitize=address` catches stack issues during development

### System-Level Protection

For production systems:
- **Watchdog timer** - Detect hung system, trigger reboot/failsafe
- **Actor monitoring** - Use links/monitors to detect failures, restart actors as needed
- **Memory isolation** - Hardware MPU (ARM Cortex-M) can provide hardware-guaranteed protection

### Future Extensions (Stack)

Hardware-based protection may be added in future versions:
- **Linux** - `mprotect()` guard pages (immediate SIGSEGV on overflow)
- **ARM Cortex-M** - MPU guard regions (immediate HardFault on overflow)

These would provide immediate, hardware-guaranteed detection with zero runtime overhead.

---

## Future Extensions

This section documents potential improvements that have been considered but are not currently implemented. These are valid enhancements that preserve Hive's philosophy (single-threaded, cooperative, bounded memory) while addressing known limitations.

### Guaranteed EXIT/Monitor Delivery

**Current behavior** - EXIT and monitor notifications use the same message pool as normal IPC. Under pool exhaustion, supervisors may miss actor death notifications.

**Potential improvement** - Reserve a dedicated EXIT slot per actor (adds ~8 bytes per actor) to guarantee delivery regardless of pool state.

**Rationale for deferral** - Pool exhaustion is a systemic error that should trigger shutdown. In well-sized systems, this is not a practical concern. The complexity of maintaining a separate delivery mechanism is not justified for embedded systems with correctly sized pools.

### Bounded I/O Dispatch

**Current behavior** - `epoll_wait()` is only called when the run queue is empty. If an actor never yields, I/O events can be starved indefinitely.

**Potential improvement** - Poll I/O every N actor dispatches (compile-time configurable). This bounds worst-case I/O latency to N x max-actor-run-time.

**Rationale for deferral** - Well-behaved actors yield regularly via `recv`, `select`, or explicit `yield`. Flight-critical code follows this pattern. Adding periodic polling adds complexity and may interfere with deterministic scheduling in time-critical applications.

### Priority Fairness Mode

**Current behavior** - Strict priority scheduling. Higher-priority actors always preempt lower-priority actors. A misbehaving CRITICAL actor can starve all others.

**Potential improvement** - Opt-in fairness mode (compile-time flag) that enforces:
- Per-priority quota (must schedule lower priority after N runs)
- Priority aging for waiting actors
- Per-actor time budget with forced yield

**Rationale for deferral** - Strict priority is intentional for deterministic real-time behavior. Applications requiring fairness can implement it at the actor level. Adding scheduler-level fairness significantly complicates timing analysis.

### Selective Receive Save Queue

**Current behavior** - Each selective receive scans the mailbox from the head. Repeated receives with the same filter rescan previously-checked messages.

**Potential improvement** - Track scan position per filter pattern (Erlang's "save queue" optimization). Only scan new messages on repeated receives.

**Rationale for deferral** - For embedded systems with small mailboxes (< 20 messages), the O(n) scan is acceptable. The optimization adds state tracking complexity. If this becomes a bottleneck, a `scan_start` pointer could be added to the mailbox struct.

### Bus Retention on Subscribe

**Current behavior** - Late subscribers see no data until the next publish. After supervisor restart, downstream actors wait for the next publish cycle.

**Potential improvement** - Optional "retain latest" flag for buses with `max_entries=1`. New subscribers immediately receive the most recent value.

**Rationale for deferral** - For high-frequency buses (e.g., sensor data at 250Hz), the wait is negligible (< 4ms). The current behavior is predictable and avoids complexity around stale data semantics.
