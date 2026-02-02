# STM32F4 Flash Logger Design

## Overview

Flight data logging for post-mortem analysis on STM32 platforms (e.g., Crazyflie 2.1+).
Designed to be non-intrusive to flight-critical control loops.

### Goals

1. **Non-intrusive** - Zero impact on flight control timing
2. **Crash-safe** - Recoverable logs after power loss or crash
3. **Simple** - Minimal code complexity in safety-critical path
4. **Efficient** - Maximize logged data within constraints

### Non-Goals

- Real-time log streaming (use UART debug for that)
- Filesystem abstraction (raw flash access)
- Log rotation across flights (erase before each flight)

---

## Hardware Constraints

### STM32F401CCU6 Resources

| Resource | Total | Used by Pilot | Available for Logger |
|----------|-------|---------------|---------------------|
| Flash | 256 KB | ~60 KB (firmware) | ~64-128 KB |
| RAM | 64 KB | ~45 KB (actors, stacks, pools) | ~8-16 KB |

### Flash Sector Layout (STM32F401)

| Sector | Address | Size | Usage |
|--------|---------|------|-------|
| 0 | 0x0800_0000 | 16 KB | Firmware |
| 1 | 0x0800_4000 | 16 KB | Firmware |
| 2 | 0x0800_8000 | 16 KB | Firmware |
| 3 | 0x0800_C000 | 16 KB | Firmware |
| 4 | 0x0801_0000 | 64 KB | Firmware |
| 5 | 0x0802_0000 | 128 KB | **LOG REGION** |

**Decision** - Use Sector 5 (128 KB) for logging. This provides ~2 minutes of
dense logging at 1 KB/s, or ~10+ minutes at typical rates.

### Flash Programming Constraints

- **Erase time** - 1-4 seconds per 128KB sector (blocking!)
- **Program granularity** - Byte/half-word/word (we'll use word = 32-bit)
- **Program time** - ~16 us per word typical
- **Read stall** - CPU stalls on flash read while BSY=1 (same bank)
- **Endurance** - 10K erase cycles (not a concern for flight logging)

### Critical Constraint: Flash Read Stall

When programming flash, the CPU cannot fetch instructions from the same
flash bank. STM32F401 has a single flash bank, so:

**All code executing during flash write must reside in RAM.**

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      FLIGHT-CRITICAL PATH                       │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐           │
│  │ Sensor   │ │ Altitude │ │ Attitude │ │  Motor   │           │
│  │  Actor   │ │  Actor   │ │  Actor   │ │  Actor   │           │
│  └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘           │
│       │            │            │            │                  │
│       └────────────┴─────┬──────┴────────────┘                  │
│                          │                                      │
│                          ▼                                      │
│              ┌───────────────────────┐                          │
│              │   log_push() - O(1)   │  Lock-free, never blocks │
│              └───────────┬───────────┘                          │
│                          │                                      │
└──────────────────────────┼──────────────────────────────────────┘
                           │
                           ▼
              ┌───────────────────────┐
              │    RAM Ring Buffer    │  8-16 KB static allocation
              │  (Binary log records) │
              └───────────┬───────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 ▼                 │
        │    ┌───────────────────────┐      │
        │    │    Logger Actor       │      │  LOW priority
        │    │  (Drains ring buffer) │      │  Yields frequently
        │    └───────────┬───────────┘      │
        │                │                  │
        │                ▼                  │
        │    ┌───────────────────────┐      │
        │    │   Staging Buffer      │      │  256 bytes
        │    │  (Accumulates block)  │      │
        │    └───────────┬───────────┘      │
        │                │                  │
        │                ▼                  │
        │    ┌───────────────────────┐      │
        │    │  flash_write_block()  │      │  Runs from RAM
        │    │   (IRQs masked)       │      │  ~1-2 ms max
        │    └───────────┬───────────┘      │
        │                │                  │
        └────────────────┼──────────────────┘
                         │
                         ▼
              ┌───────────────────────┐
              │   Flash Log Region    │  Sector 5: 128 KB
              │    (Sequential)       │
              └───────────────────────┘
```

---

## Data Structures

### Log Region Header (64 bytes, at start of Sector 5)

```c
typedef struct {
    uint32_t magic;           // 0x464C4F47 ("FLOG")
    uint16_t version;         // Format version (1)
    uint16_t header_size;     // Size of this header (64)
    uint32_t region_size;     // Total log region size (128KB)
    uint32_t data_offset;     // Offset to first data block (64)
    uint32_t boot_id;         // Incremented each boot
    uint32_t erase_timestamp; // When region was erased
    uint8_t  erased_ok;       // 1 = region ready for writes
    uint8_t  reserved[27];    // Pad to 64 bytes
} log_region_header_t;
```

### Log Block Header (16 bytes)

```c
typedef struct {
    uint32_t magic;           // 0x424C4B48 ("BLKH")
    uint16_t block_seq;       // Block sequence number (monotonic)
    uint16_t payload_len;     // Bytes of payload following header
    uint32_t timestamp_us;    // Microseconds since boot
    uint32_t dropped_total;   // Total records dropped so far
} log_block_header_t;
```

### Log Block Footer (4 bytes)

```c
typedef struct {
    uint32_t crc32;           // CRC32 of header + payload
} log_block_footer_t;
```

### Log Record (variable size, packed in payload)

```c
typedef struct {
    uint8_t  record_type;     // Record type (see below)
    uint8_t  hive_actor_id_t;        // Source actor (0 = system)
    uint16_t payload_len;     // Length of record payload
    uint32_t timestamp_us;    // Record timestamp
    // uint8_t payload[];     // Record-specific data follows
} log_record_header_t;
```

### Record Types

| Type | Name | Payload | Description |
|------|------|---------|-------------|
| 0x01 | STATE | 48B | Full state estimate (pos, vel, attitude) |
| 0x02 | SENSOR | 36B | Raw sensor readings |
| 0x03 | MOTOR | 16B | Motor commands |
| 0x04 | EVENT | 4B+ | Event code + optional data |
| 0x05 | WAYPOINT | 20B | Waypoint reached/target |
| 0x10 | BOOT | 16B | Boot info, config hash |
| 0x11 | ARM | 8B | Arm timestamp, flight profile |
| 0x12 | DISARM | 8B | Disarm timestamp, reason |
| 0xFE | STATUS | 12B | Logger status, dropped count |
| 0xFF | MARKER | 4B | Sync marker, block boundary |

### Block Layout

```
┌──────────────────────────────────────────────────┐
│ Block Header (16 bytes)                          │
├──────────────────────────────────────────────────┤
│ Record 1: [type][actor][len][ts][payload...]     │
│ Record 2: [type][actor][len][ts][payload...]     │
│ Record 3: [type][actor][len][ts][payload...]     │
│ ... (packed, no padding)                         │
│ (unused space filled with 0xFF)                  │
├──────────────────────────────────────────────────┤
│ Block Footer (4 bytes) - CRC32                   │
└──────────────────────────────────────────────────┘
```

**Block size** - 256 bytes (240 payload + 16 header + 4 footer = 260, round to 256)

Adjusted: 256 - 16 - 4 = 236 bytes payload per block.

---

## RAM Ring Buffer

### Design

- **Size** - 8192 bytes (8 KB) - fits ~80-200 records
- **Type** - Single-producer-multi-consumer not needed; SPSC sufficient
- **Lock-free** - Yes, for O(1) push from flight actors
- **Overflow policy** - Drop newest, increment counter

### Implementation

```c
typedef struct {
    uint8_t  data[LOG_RING_SIZE];   // 8192 bytes
    volatile uint32_t head;          // Write position (producers)
    volatile uint32_t tail;          // Read position (logger actor)
    volatile uint32_t dropped;       // Dropped record count
    volatile uint32_t high_water;    // Max fill level seen
} log_ring_t;
```

### Push Operation (Flight Actors)

```c
// Called from flight-critical actors - MUST be O(1), no blocking
bool log_push(uint8_t type, uint8_t actor, const void *data, uint16_t len);
```

1. Calculate total size: `sizeof(log_record_header_t) + len`
2. Check available space: `(head - tail) < (size - total)`
3. If no space: increment `dropped`, return false
4. Copy header + payload to ring at `head`
5. Advance `head` (atomic store)
6. Update `high_water` if needed

### Pop Operation (Logger Actor)

```c
// Called from logger actor only - can take time
bool log_pop(log_record_header_t *hdr, void *payload, uint16_t max_len);
```

1. Check if data available: `head != tail`
2. Copy header from ring at `tail`
3. Validate and copy payload
4. Advance `tail` (atomic store)

---

## Logger Actor

### Priority

**LOW (3)** - Lowest priority, never preempts flight control.

### State Machine

```
    ┌─────────┐
    │  IDLE   │◄────────────────────────────┐
    └────┬────┘                             │
         │ ring has data                    │
         ▼                                  │
    ┌─────────┐                             │
    │ DRAIN   │ Pop records from ring       │
    └────┬────┘ into staging buffer         │
         │ staging full or timeout          │
         ▼                                  │
    ┌─────────┐                             │
    │ WRITE   │ Write block to flash        │
    └────┬────┘ (single block, then yield)  │
         │                                  │
         └──────────────────────────────────┘
```

### Timing Budget

- **DRAIN phase** - Process up to 10 records per cycle, then yield
- **WRITE phase** - Write one 256B block, then yield
- **Yield frequency** - Every 1-2 ms max between yields

### Pseudo-code

```
logger_actor():
    while true:
        // Drain ring buffer into staging
        while staging_space_available() and ring_has_data():
            record = ring_pop()
            staging_append(record)
            if records_processed > 10:
                yield()  // Don't hog CPU

        // Write block if staging has enough data (or timeout)
        if staging_ready_to_commit():
            block = staging_finalize()  // Add header, CRC
            flash_write_block(block)    // ~1-2ms, IRQs masked briefly
            staging_reset()
            yield()
        else:
            sleep(10ms)  // Nothing to do, wait
```

---

## Flash Writer

### Requirements

1. **Execute from RAM** - Cannot fetch from flash during write
2. **Minimal IRQ-off time** - Target < 2ms per block
3. **No HAL calls** - Direct register access only
4. **Error handling** - Clear error flags, retry logic

### RAM Function Placement

```c
__attribute__((section(".ramfunc")))
static void flash_program_words(uint32_t addr, const uint32_t *data, uint32_t count);
```

Linker script addition:
```ld
.ramfunc : {
    . = ALIGN(4);
    _sramfunc = .;
    *(.ramfunc .ramfunc.*)
    . = ALIGN(4);
    _eramfunc = .;
} > RAM AT > FLASH

_siramfunc = LOADADDR(.ramfunc);
```

Startup code copies `.ramfunc` from flash to RAM.

### Write Sequence

```c
bool flash_write_block(uint32_t offset, const void *data, uint32_t len) {
    uint32_t addr = LOG_REGION_BASE + offset;

    // 1. Unlock flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    // 2. Clear error flags
    FLASH->SR = FLASH_SR_ERRORS;

    // 3. Program words (from RAM function, IRQs masked)
    __disable_irq();
    flash_program_words(addr, data, len / 4);  // Runs from RAM
    __enable_irq();

    // 4. Lock flash
    FLASH->CR |= FLASH_CR_LOCK;

    // 5. Verify (optional, adds time)
    return memcmp((void*)addr, data, len) == 0;
}
```

### IRQ-Off Timing Analysis

- Word program time: ~16 us
- 256 bytes = 64 words
- Total: 64 x 16 us = **~1 ms** with IRQs disabled

This is acceptable. Control loop at 500 Hz has 2ms period, so 1ms IRQ-off
occasionally is tolerable (will delay one control cycle slightly).

---

## Pre-Flight Erase

### When to Erase

**During arming sequence**, before motors are enabled:

1. Main actor sends ARM command
2. Logger actor receives ARM notification
3. Logger erases Sector 5 (takes 1-4 seconds)
4. Logger writes region header with `erased_ok = 1`
5. Logger signals ready
6. Main actor proceeds with flight

### Erase Safety

- **Never erase during flight** - Takes seconds, would crash
- **erased_ok flag** - Only write if flag is set
- **Boot check** - On boot, check if region is valid; if not, mark as needs-erase

### Erase Code

```c
void flash_erase_log_region(void) {
    // Unlock
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    // Erase Sector 5
    FLASH->CR = FLASH_CR_SER | (5 << FLASH_CR_SNB_Pos) | FLASH_CR_STRT;

    // Wait for completion (this blocks for 1-4 seconds!)
    while (FLASH->SR & FLASH_SR_BSY);

    // Lock
    FLASH->CR |= FLASH_CR_LOCK;
}
```

---

## Post-Flight Dump

### Trigger

After landing and disarm, dump can be triggered by:
- Button press (if available)
- UART command
- Automatic on USB connect

### Dump Protocol (UART)

Simple text-based protocol over existing USART1:

```
> LOG DUMP
< LOG START boot_id=42 blocks=347 bytes=88832
< BLOCK 0 seq=0 ts=0 len=236 crc=0xABCD1234
< [base64 encoded block data]
< BLOCK 1 seq=1 ts=10234 len=236 crc=0xDEADBEEF
< [base64 encoded block data]
...
< LOG END blocks=347 errors=0
```

### PC-Side Tool

Python script to:
1. Capture dump from serial
2. Decode base64 blocks
3. Verify CRCs
4. Parse records into structured data
5. Export to CSV/JSON for analysis

---

## API Summary

### Producer API (Flight Actors)

```c
// Initialize logger (call from main before hive_run)
void log_init(void);

// Push a log record - O(1), never blocks, safe from any actor
bool log_push(uint8_t type, uint8_t actor, const void *data, uint16_t len);

// Convenience macros
#define LOG_STATE(state_ptr)    log_push(LOG_TYPE_STATE, hive_self(), state_ptr, sizeof(state_t))
#define LOG_EVENT(code)         log_push(LOG_TYPE_EVENT, hive_self(), &(uint32_t){code}, 4)
```

### Logger Actor API

```c
// Spawn logger actor (call from main)
void logger_actor_spawn(void);

// Logger receives these notifications:
// - NOTIFY_ARM: Erase flash, prepare for logging
// - NOTIFY_DISARM: Flush remaining data, finalize
```

### Dump API

```c
// Dump log region over UART (blocking, call after disarm)
void log_dump_uart(void);
```

---

## Memory Budget

| Component | Size | Notes |
|-----------|------|-------|
| Ring buffer | 8,192 B | Static, in .bss |
| Staging buffer | 256 B | Static, in .bss |
| Logger actor stack | 1,024 B | Minimal processing |
| RAM functions | ~256 B | .ramfunc section |
| **Total RAM** | **~9.7 KB** | |
| Flash log region | 131,072 B | Sector 5 |
| Log code | ~2-3 KB | In firmware |

---

## Implementation Phases

### Phase 1: Core Infrastructure
- [ ] Ring buffer implementation
- [ ] Log record format
- [ ] `log_push()` API
- [ ] Unit tests on Linux (mock flash)

### Phase 2: Flash Writer
- [ ] RAM function setup in linker script
- [ ] Flash erase routine
- [ ] Flash write routine (256B blocks)
- [ ] Block format with CRC

### Phase 3: Logger Actor
- [ ] Actor implementation
- [ ] Integration with main actor (ARM/DISARM)
- [ ] Drain and write state machine

### Phase 4: Integration
- [ ] Add logging calls to pilot actors
- [ ] Test on hardware
- [ ] Verify timing impact

### Phase 5: Dump & Analysis
- [ ] UART dump command
- [ ] Python decode tool
- [ ] CSV export

---

## Open Questions

1. **Block size** - 256B chosen for ~1ms write time. Should we go smaller (128B)
   for lower latency, or larger (512B) for efficiency?

2. **Ring buffer size** - 8KB allows ~100-200 records. Is this enough buffer
   for worst-case flash write delays?

3. **What to log** - Which actors log what? Every state update? Every N updates?
   Need to balance detail vs. flash space.

4. **Timestamp source** - Use `HAL_GetTick()` (ms) or implement us timer?
   Flight events may need us resolution.

5. **Compression** - Simple RLE or delta encoding could 2-3x the log capacity.
   Worth the complexity?
