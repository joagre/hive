# HAL Porting Guide

This directory contains templates for porting the Hive actor runtime to a new platform.

## Overview

The Hardware Abstraction Layer (HAL) isolates platform-specific code so that porters can implement a small set of functions without understanding the runtime's internal scheduler and timer mechanics.

### What Porters Implement

| Category | Files | Functions | Required |
|----------|-------|-----------|----------|
| Core HAL | `hive_hal_<platform>.c` | 7 functions | Yes |
| Context | `hive_hal_context_<platform>.c` | 1 function | Yes |
| Context | `hive_context_<arch>.S` | 1 function | Yes |
| Context | `hive_hal_context_defs.h` | 1 struct | Yes |
| File I/O | (in hal or separate) | 8 functions | Optional |
| Network | (in hal or separate) | 10 functions | Optional |

**Minimum port**: ~9 functions + 1 assembly function + 1 struct definition

## Quick Start

1. Create your platform directory:
   ```
   mkdir src/hal/<platform>
   ```

2. Copy templates:
   ```
   cp src/hal/template/hive_hal_template.c src/hal/<platform>/hive_hal_<platform>.c
   cp src/hal/template/hive_hal_context_defs.h src/hal/<platform>/
   cp src/hal/template/hive_hal_context_template.c src/hal/<platform>/hive_hal_context_<platform>.c
   cp src/hal/template/hive_context_template.S src/hal/<platform>/hive_context_<arch>.S
   ```

3. Implement the functions (see sections below)

4. Update the Makefile to select your platform files

## Required Functions

### Time Functions (2 functions)

```c
// Get monotonic time in microseconds
uint64_t hive_hal_get_time_us(void);

// Enter/exit critical sections
uint32_t hive_hal_critical_enter(void);
void hive_hal_critical_exit(uint32_t state);
```

**Implementation notes:**
- `get_time_us`: Must be monotonic, microsecond resolution preferred
- Critical sections: Disable interrupts, support nesting

### Event Functions (5 functions)

```c
hive_status hive_hal_event_init(void);
void hive_hal_event_cleanup(void);
void hive_hal_event_poll(void);
void hive_hal_event_wait(int timeout_ms);
hive_status hive_hal_event_register(int fd, uint32_t events, io_source *source);
void hive_hal_event_unregister(int fd);
```

**Implementation notes:**
- `event_poll`: Check and process ready I/O events without blocking
- `event_wait`: Sleep until event or timeout; process events on wake
- `event_register/unregister`: Add/remove file descriptors from watch list

For platforms without file descriptors (bare metal), register/unregister can be no-ops if timers use a software wheel.

### Context Functions (1 C function + 1 assembly function)

```c
// In hive_hal_context_<platform>.c
void hive_context_init(hive_context *ctx, void *stack, size_t stack_size,
                       void (*fn)(void *, const void *, size_t));

// In hive_context_<arch>.S (assembly)
void hive_context_switch(hive_context *from, hive_context *to);
```

Plus the context struct in `hive_hal_context_defs.h`:

```c
typedef struct {
    void *sp;           // Stack pointer
    // ... callee-saved registers for your architecture
} hive_context;
```

**Implementation notes:**
- `context_init`: Set up stack so `context_switch` starts execution at `fn`
- `context_switch`: Save/restore callee-saved registers and stack pointer
- See your architecture's ABI for which registers are callee-saved

## Optional Functions

### File I/O (8 functions, HIVE_ENABLE_FILE=1)

```c
hive_status hive_hal_file_init(void);
void hive_hal_file_cleanup(void);
hive_status hive_hal_file_open(const char *path, int flags, int mode, int *fd_out);
hive_status hive_hal_file_close(int fd);
hive_status hive_hal_file_read(int fd, void *buf, size_t len, size_t *bytes_read);
hive_status hive_hal_file_pread(int fd, void *buf, size_t len, size_t offset, size_t *bytes_read);
hive_status hive_hal_file_write(int fd, const void *buf, size_t len, size_t *bytes_written);
hive_status hive_hal_file_pwrite(int fd, const void *buf, size_t len, size_t offset, size_t *bytes_written);
hive_status hive_hal_file_sync(int fd);
```

### Network I/O (10 functions, HIVE_ENABLE_NET=1)

```c
hive_status hive_hal_net_init(void);
void hive_hal_net_cleanup(void);
hive_status hive_hal_net_socket(int *fd_out);
hive_status hive_hal_net_bind(int fd, uint16_t port);
hive_status hive_hal_net_listen(int fd, int backlog);
hive_status hive_hal_net_accept(int listen_fd, int *conn_fd_out);
hive_status hive_hal_net_connect(int fd, const char *ip, uint16_t port);
hive_status hive_hal_net_connect_check(int fd);
hive_status hive_hal_net_close(int fd);
hive_status hive_hal_net_recv(int fd, void *buf, size_t len, size_t *received);
hive_status hive_hal_net_send(int fd, const void *buf, size_t len, size_t *sent);
```

**Implementation notes:**
- All network operations must be **non-blocking**
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
- `hive_hal_linux.c` - Time, events (epoll), file, network
- `hive_hal_context_linux.c` - x86-64 context init
- `hive_context_x86_64.S` - x86-64 context switch
- `hive_hal_context_defs.h` - x86-64 context struct

### STM32 (ARM Cortex-M)

See `src/hal/stm32/`:
- `hive_hal_stm32.c` - Time (SysTick), events (WFI), network stubs
- `hive_hal_file_stm32.c` - Flash-based virtual file system
- `hive_hal_context_stm32.c` - ARM context init
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
- Review the SPEC.md for runtime requirements
