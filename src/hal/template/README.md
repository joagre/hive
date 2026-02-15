# HAL Porting Guide

This directory contains templates for porting the Hive actor runtime to a new platform.

## Overview

The Hardware Abstraction Layer (HAL) isolates platform-specific code so that porters can implement a small set of functions without understanding the runtime's internal scheduler and timer mechanics.

### What Porters Implement

| Category | Files | Functions | Required |
|----------|-------|-----------|----------|
| Time | `hive_hal_time.c` | 3 functions | Yes |
| Event Loop | `hive_hal_event.c` | 6 functions | Yes |
| HAL Event Signaling | `hive_hal_event.c` | 5 functions | Yes |
| Timer | `hive_hal_timer.c` | 6 functions | Yes |
| Context | `hive_hal_context.c` | 1 function | Yes |
| Context | `hive_context.S` | 1 function | Yes |
| Context | `hive_hal_context_defs.h` | 1 struct | Yes |
| File I/O | `hive_hal_file.c` | 9 functions | Optional |
| TCP | `hive_hal_tcp.c` | 11 functions | Optional |

**Minimum port** - ~21 functions + 1 assembly function + 1 struct definition

## Quick Start

1. Create your platform directory:
   ```
   mkdir src/hal/<platform>
   ```

2. Copy templates:
   ```
   cp src/hal/template/hive_hal_time.c src/hal/<platform>/
   cp src/hal/template/hive_hal_event.c src/hal/<platform>/
   cp src/hal/template/hive_hal_timer.c src/hal/<platform>/
   cp src/hal/template/hive_hal_context_defs.h src/hal/<platform>/
   cp src/hal/template/hive_hal_context.c src/hal/<platform>/
   cp src/hal/template/hive_context.S src/hal/<platform>/
   ```

3. For optional features, also copy:
   ```
   cp src/hal/template/hive_hal_file.c src/hal/<platform>/  # If HIVE_ENABLE_FILE=1
   cp src/hal/template/hive_hal_tcp.c src/hal/<platform>/   # If HIVE_ENABLE_TCP=1
   ```

4. Implement the functions (see sections below)

5. Update the Makefile to select your platform files

## Required Functions

### Time Functions (3 functions) - `hive_hal_time.c`

```c
// Get monotonic time in microseconds
uint64_t hive_hal_get_time_us(void);

// Enter/exit critical sections
uint32_t hive_hal_critical_enter(void);
void hive_hal_critical_exit(uint32_t state);
```

**Implementation notes**
- `get_time_us`: Must be monotonic, microsecond resolution preferred
- Critical sections: Disable interrupts, support nesting

### Event Loop Functions (6 functions) - `hive_hal_event.c`

```c
hive_status_t hive_hal_event_init(void);
void hive_hal_event_cleanup(void);
void hive_hal_event_poll(void);
void hive_hal_event_wait(int timeout_ms);
hive_status_t hive_hal_event_register(int fd, uint32_t events, io_source *source);
void hive_hal_event_unregister(int fd);
```

**Implementation notes**
- `event_poll`: Check and process ready I/O events without blocking
- `event_wait`: Sleep until event or timeout; process events on wake
- `event_register/unregister`: Add/remove file descriptors from watch list

For platforms without file descriptors (bare metal), register/unregister can be no-ops if timers use a software wheel.

### HAL Event Signaling Functions (5 functions) - `hive_hal_event.c`

These functions enable ISR-to-actor communication via `hive_select()`:

```c
hive_hal_event_id_t hive_hal_event_create(void);
void hive_hal_event_destroy(hive_hal_event_id_t id);
void hive_hal_event_signal(hive_hal_event_id_t id);  // ISR-safe!
bool hive_hal_event_is_set(hive_hal_event_id_t id);
void hive_hal_event_clear(hive_hal_event_id_t id);
```

**Implementation notes**
- `event_create`: Allocate event ID from pool (max 32 events, uses bitmask)
- `event_destroy`: Return event ID to pool
- `event_signal`: **Must be ISR-safe** - set flag using atomic operation
- `event_is_set`: Check if event flag is set (volatile read)
- `event_clear`: Clear event flag (must be atomic on platforms with ISRs)

**ISR safety requirements for `event_signal()`:**
- **ARM Cortex-M**: Single 32-bit store to `volatile uint32_t` is atomic
- **x86-64/Linux**: Use `atomic_fetch_or()` or equivalent
- **Other**: Ensure single-instruction flag set, no read-modify-write

**ISR safety requirements for `event_clear()`:**
- **ARM Cortex-M**: Disable interrupts around read-modify-write (`CPSID I` / `CPSIE I`)
- **x86-64/Linux**: Use `atomic_fetch_and()` or equivalent
- Clear is called from actor context, not ISR context

**Example (ARM Cortex-M):**
```c
static volatile uint32_t s_event_flags = 0;
static uint32_t s_event_allocated = 0;

void hive_hal_event_signal(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        s_event_flags |= (1U << id);  // Single store = atomic on ARM
    }
}

void hive_hal_event_clear(hive_hal_event_id_t id) {
    if (id < HIVE_HAL_EVENT_MAX) {
        __disable_irq();
        s_event_flags &= ~(1U << id);
        __enable_irq();
    }
}
```

### Timer Functions (6 functions) - `hive_hal_timer.c`

```c
hive_status_t hive_hal_timer_init(void);
void hive_hal_timer_cleanup(void);
hive_status_t hive_hal_timer_create(uint32_t interval_us, bool periodic,
                                  hive_actor_id_t owner, hive_timer_id_t *out);
hive_status_t hive_hal_timer_cancel(hive_timer_id_t id);
uint64_t hive_hal_timer_get_time(void);
void hive_hal_timer_advance_time(uint64_t delta_us);
```

**Implementation notes**
- `timer_create`: Create a one-shot or periodic timer that delivers `HIVE_MSG_TIMER` to owner
- `timer_cancel`: Cancel an active timer by ID
- `timer_get_time`: Return current time in microseconds (for simulation mode)
- `timer_advance_time`: Advance simulation time and fire due timers (for testing/simulation)

**Platform approaches**
- **Linux**: Use `timerfd` + epoll for real-time mode; software timers for simulation mode
- **STM32**: Software timer wheel driven by SysTick interrupt (`hive_timer_tick_isr()`)
- **Simulation**: Store expiry times, fire timers when `advance_time` is called

See `include/hal/hive_hal_timer.h` for the full API specification.

### Context Functions (1 C function + 1 assembly function)

```c
// In hive_hal_context.c
void hive_context_init(hive_context_t *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t));

// In hive_context.S (assembly)
void hive_context_switch(hive_context_t *from, hive_context_t *to);
```

Plus the context struct in `hive_hal_context_defs.h`:

```c
typedef struct {
    void *sp;           // Stack pointer
    // ... callee-saved registers for your architecture
} hive_context_t;
```

**Implementation notes**
- `context_init`: Set up stack so `context_switch` starts execution at `fn`
- `context_switch`: Save/restore callee-saved registers and stack pointer
- See your architecture's ABI for which registers are callee-saved

## Optional Functions

### File I/O (9 functions, HIVE_ENABLE_FILE=1) - `hive_hal_file.c`

```c
hive_status_t hive_hal_file_init(void);
void hive_hal_file_cleanup(void);
hive_status_t hive_hal_file_open(const char *path, int flags, int mode, int *out);
hive_status_t hive_hal_file_close(int fd);
hive_status_t hive_hal_file_read(int fd, void *buf, size_t len, size_t *bytes_read);
hive_status_t hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read);
hive_status_t hive_hal_file_write(int fd, const void *buf, size_t len, size_t *bytes_written);
hive_status_t hive_hal_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written);
hive_status_t hive_hal_file_sync(int fd);
```

**Implementation notes**
- All file operations should be synchronous (blocking) from the HAL caller's perspective
- STM32: Only virtual paths work (`/log`, `/config`, `/sd`) - arbitrary paths rejected
- `/sd` requires SD card hardware and `HIVE_ENABLE_SD=1` build flag
- SD card writes use DMA internally for speed but this is below the HAL interface
- Mount availability checking is handled by the separate mount module (`hive_mount.h`)

### TCP (11 functions, HIVE_ENABLE_TCP=1) - `hive_hal_tcp.c`

```c
hive_status_t hive_hal_tcp_init(void);
void hive_hal_tcp_cleanup(void);
hive_status_t hive_hal_tcp_socket(int *out);
hive_status_t hive_hal_tcp_bind(int fd, uint16_t port);
hive_status_t hive_hal_tcp_listen(int fd, int backlog);
hive_status_t hive_hal_tcp_accept(int listen_fd, int *out);
hive_status_t hive_hal_tcp_connect(int fd, const char *ip, uint16_t port);
hive_status_t hive_hal_tcp_connect_check(int fd);
hive_status_t hive_hal_tcp_close(int fd);
hive_status_t hive_hal_tcp_recv(int fd, void *buf, size_t len, size_t *bytes_read);
hive_status_t hive_hal_tcp_send(int fd, const void *buf, size_t len, size_t *bytes_written);
```

**Implementation notes**
- All TCP operations must be **non-blocking**
- Return `HIVE_ERR_WOULDBLOCK` when operation would block
- Return `HIVE_ERR_INPROGRESS` for async connect in progress

## Error Codes

Use these error codes in your HAL implementation:

| Code | Meaning |
|------|---------|
| `HIVE_OK` | Success |
| `HIVE_ERR_NOMEM` | Out of memory / pool exhausted |
| `HIVE_ERR_INVALID` | Invalid argument |
| `HIVE_ERR_TIMEOUT` | Operation timed out |
| `HIVE_ERR_WOULDBLOCK` | Non-blocking operation would block |
| `HIVE_ERR_INPROGRESS` | Async operation in progress |
| `HIVE_ERR_IO` | I/O error |

## Platform Examples

### Linux

See `src/hal/linux/`:
- `hive_hal_time.c` - Time (clock_gettime), critical sections (no-op)
- `hive_hal_event.c` - Event loop (epoll) + HAL event signaling (C11 atomics)
- `hive_hal_timer.c` - Timer HAL (timerfd + simulation mode)
- `hive_hal_file.c` - File I/O (POSIX)
- `hive_hal_tcp.c` - TCP (BSD sockets)
- `hive_hal_context.c` - x86-64 context init
- `hive_context_x86_64.S` - x86-64 context switch
- `hive_hal_context_defs.h` - x86-64 context struct

### STM32 (ARM Cortex-M)

See `src/hal/stm32/`:
- `hive_hal_time.c` - Time (tick counter), critical sections (PRIMASK)
- `hive_hal_event.c` - Event loop (WFI) + HAL event signaling (atomic stores + CPSID/CPSIE)
- `hive_hal_timer.c` - Timer HAL (software timer wheel)
- `hive_hal_file.c` - Flash-based virtual files + SD card via FatFS
- `hive_hal_tcp.c` - TCP stubs (future lwIP)
- `hive_hal_context.c` - ARM context init
- `hive_context_arm_cm.S` - ARM context switch
- `hive_hal_context_defs.h` - ARM context struct (with FPU support)

## Testing Your Port

1. Build the library:
   ```
   make PLATFORM=<yourplatform>
   ```

2. Run basic tests:
   - Actor spawn/exit
   - Context switching (multiple actors)
   - Timer firing
   - IPC messaging

3. Run the full test suite (if your platform supports it)

## Common Pitfalls

1. **Stack alignment**: Most ABIs require 8 or 16-byte stack alignment
2. **Interrupt safety**: Critical sections must disable all relevant interrupts
3. **Callee-saved registers**: Missing a register causes random crashes
4. **Return address**: Context switch must "return" to the correct address
5. **Stack direction**: Stacks grow DOWN on most architectures

## Getting Help

- Check existing implementations in `src/hal/linux/` and `src/hal/stm32/`
- Consult your architecture's ABI documentation
- Review docs/spec/ for runtime requirements
