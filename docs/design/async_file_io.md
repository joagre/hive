# Async File I/O Design

Non-blocking file I/O for the Hive actor runtime, targeting SD card writes
on STM32 that currently block the scheduler.

## Problem

Hive is single-threaded and cooperative. When an actor calls
`hive_file_write()` on an SD card path, the write goes synchronously through
FatFS -> SPI. SD cards have unpredictable write latency (50-250ms+ spikes
from internal garbage collection). During this time, no other actor can run -
including flight-critical control loops that need the CPU every 4ms.

This caused a crash during flight test 9: the logger and hive log system
generated hundreds of SD card writes per second, an SD latency spike blocked
the scheduler for 108ms, the motor deadman timeout fired, and the drone
flipped at 0.35m altitude.

### Current write paths

**Flash files** (`/log`, `/config`) - Already non-blocking for common case:
```
hive_file_write() -> ring_push() [8KB RAM buffer, microseconds]
                  -> staging_commit() [256B flash write, ~1-2ms, only when full]
```

The flash ring buffer exists because of flash-specific constraints: sector
erase is slow, programming must run from RAM with interrupts disabled. The
ring buffer decouples write frequency from flash programming frequency. This
is the right design for flash but is not a general I/O pattern.

**SD card files** (`/sd/*`) - Fully synchronous, no buffering:
```
hive_file_write() -> f_write() [FatFS] -> disk_write() [SPI] -> hardware
                     entire call blocks scheduler for duration of SPI transfer
```

**Linux files** - Synchronous POSIX I/O, but the OS kernel buffers writes in
page cache. Rarely blocks in practice. No changes needed.

## Design

DMA + yield in the SPI SD driver (STM32 only). Make SPI transfers
non-blocking by yielding to the scheduler during DMA.

No ring buffer needed for SD:
- On Linux, the OS page cache already buffers writes
- On STM32, DMA + yield makes the SPI transfer itself non-blocking
- FatFS has its own internal sector cache for small writes
- Adding a ring buffer would be buffering on top of buffering

### Where the yield goes

The call chain is:
```
hive_file_write() -> f_write() [FatFS] -> disk_write() [diskio.c]
    -> spi_sd_write_blocks() [spi_sd.c] -> spi_send(buf, 512) [blocking]
                                         -> wait_ready()       [blocking]
```

The blocking happens inside `spi_sd.c`, not `diskio.c`. The SD protocol
requires sending commands, data tokens, CRC bytes, and polling busy status -
all of which live in `spi_sd.c`. The DMA replaces `spi_send(buf, 512)` and
`spi_recv(buf, 512)` calls, and the yield replaces the `wait_ready()` busy
loop. `diskio.c` and FatFS are unchanged.

### DMA + yield in spi_sd

**Modified SPI SD driver:**
```c
// src/hal/stm32/spi_sd.c - inside spi_sd_write_blocks()

// Instead of byte-by-byte: spi_send(buf, 512)
// Start DMA transfer (returns immediately)
spi_ll_dma_start(buf, 512, SPI_LL_DIR_TX);

// Yield to scheduler - other actors run while DMA transfers data
hive_event_wait(s_dma_event, SD_TIMEOUT_MS);

// DMA complete - check result
if (!spi_ll_dma_ok()) {
    spi_ll_cs_high();
    return -1;
}
```

**How it works:**
1. `spi_sd_write_blocks()` sends the SD command and data token (byte-by-byte,
   ~10 bytes - fast)
2. `spi_ll_dma_start()` starts DMA for the 512-byte sector data
3. DMA runs autonomously on the SPI1 peripheral (no CPU involvement)
4. `hive_event_wait()` yields the current actor to the scheduler
5. The scheduler runs other actors (sensor, estimator, altitude, etc.)
6. When DMA completes, the SPI1 RX DMA ISR calls `hive_hal_event_signal()`
7. The scheduler wakes the yielded actor
8. `spi_sd_write_blocks()` sends CRC, checks response, and waits for card
   ready (also yielding)

**Key insight:** FatFS and diskio.c don't change. The yield is invisible to
them - `spi_sd_write_blocks()` still returns 0 or -1. The scheduler ran
flight-critical actors during the DMA transfer.

### DMA hardware (Crazyflie 2.1+)

SD card uses SPI1 (shared with Flow Deck):
- PA5 = SCK, PA6 = MISO, PA7 = MOSI, PC12 = CS

SPI is full-duplex, so both TX and RX DMA streams must run for every
transfer. For writes, TX sends actual data and RX captures discarded bytes.
For reads, TX sends 0xFF and RX captures data.

**DMA2 stream allocation on STM32F405:**
| Stream | Channel | Usage |
|--------|---------|-------|
| DMA2 Stream 0 | Ch 3 | SPI1_RX (SD card) |
| DMA2 Stream 2 | Ch 5 | USART6_RX (syslink - already in use) |
| DMA2 Stream 3 | Ch 3 | SPI1_TX (SD card) |

No conflicts. Stream 2 is syslink, streams 0 and 3 are free.

**ISR integration:**
```c
// SPI1 RX DMA complete ISR (DMA2 Stream 0)
// RX completes after TX, so this signals "transfer done"
void DMA2_Stream0_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0;            // Clear flag
        hive_hal_event_signal(s_sd_dma_event_id);   // Wake actor
    }
}
```

`hive_hal_event_signal()` is already ISR-safe (single 32-bit store on ARM
Cortex-M). This uses the existing HAL event mechanism - no new primitives.

**Reads use the same pattern.** DMA + yield for reads too. This makes log
download via radio non-blocking as well.

### SPI bus locking during DMA

On the Crazyflie 2.1+, the SD card and Flow Deck (PMW3901) share SPI1.
Today this works because cooperative scheduling means only one actor
touches SPI1 at a time - no yield occurs during an SPI transfer. DMA +
yield breaks this: the logger starts a DMA transfer on SPI1, yields, the
sensor actor runs at CRITICAL priority, and tries to read the flow deck
via SPI1. Both devices selected, both driving MISO - corrupted data.

Fix: add `spi_ll_lock()` / `spi_ll_unlock()` to the spi_ll interface. A
simple boolean flag, sufficient for a single-threaded cooperative scheduler.

```c
// In spi_ll.h
void spi_ll_lock(void);    // Acquire SPI bus (yields if busy)
void spi_ll_unlock(void);  // Release SPI bus
```

The SD DMA path holds the lock during the transfer (~195us for 512 bytes
at 21MHz). The flow deck driver checks the lock before each SPI access and
yields if busy. The sensor read gets delayed by at most ~195us, negligible
for a 4ms sensor loop.

During the busy-wait phase, CS management is important. The current code
holds SD CS low for the entire wait_ready() loop. With timer-based polling,
we deassert CS between polls so the flow deck can use SPI1 freely. SD cards
track busy state internally - reasserting CS and clocking 0xFF still reads
the correct busy/ready status. Each poll briefly locks the bus (~0.4us for
one `spi_ll_xfer()`). Between polls the bus is free for the flow deck.

```
DMA transfer:  lock -> CS low -> DMA 512 bytes -> CS high -> unlock
Busy poll:     lock -> CS low -> xfer(0xFF) -> CS high -> unlock -> sleep 1ms
               ... flow deck reads SPI freely between polls ...
```

### Typical write flow during flight

```
Logger actor (LOW priority, every 40ms):
    hive_file_write(csv_line, ~150 bytes)
        -> f_write() [FatFS buffers in sector cache]
        -> if sector full:
            -> disk_write(sector) [diskio.c, unchanged]
                -> spi_sd_write_blocks() [spi_sd.c]
                    -> send CMD24 + data token [byte-by-byte, ~10 bytes]
                    -> spi_ll_lock()                [acquire SPI bus]
                    -> spi_ll_dma_start(buf, 512)   [starts DMA, ~1 us]
                    -> hive_event_wait()            [yields to scheduler]
                        ... sensor_actor runs (CRITICAL) ...
                        ... flow deck skips SPI (bus locked, ~195us) ...
                        ... estimator, altitude, rate, motor run ...
                    -> DMA2 Stream 0 ISR signals event
                    -> spi_ll_unlock()              [release SPI bus]
                    -> actor wakes, sends CRC, checks response
                    -> wait_ready() with hive_timer_every() poll
                        ... other actors run between 1ms polls ...
                        ... flow deck reads SPI normally ...
                    -> card ready, return 0
        -> return HIVE_SUCCESS
```

The control loop runs uninterrupted at 250Hz. SD card writes happen in the
background, sector by sector, with the scheduler running other actors between
each sector transfer. Even SD card latency spikes (100ms+) are harmless -
the logger actor simply stays yielded longer while flight-critical actors
keep running.

### SD card busy-wait after DMA

SD cards have two sources of latency:
1. **SPI transfer** - Moving bytes over the bus. DMA handles this.
2. **Card-internal write** - After receiving data, the card may take up to
   250ms to program its internal flash (wear leveling, garbage collection).
   During this time, the card holds MISO low (busy signal).

The current SPI driver clocks 0xFF bytes in a tight loop, checking if the
response is 0xFF (card ready). With DMA yield, this busy-wait must also
yield to the scheduler so other actors can run during the wait.

Poll every 1ms using `hive_timer_every()`, clocking one byte per poll:
```c
#define SD_BUSY_POLL_INTERVAL_US 1000  // 1ms between polls
#define SD_BUSY_TIMEOUT_MS       500   // 500ms max busy time

hive_timer_id_t poll_timer;
hive_timer_every(SD_BUSY_POLL_INTERVAL_US, &poll_timer);
while (spi_ll_xfer(0xFF) != 0xFF) {
    hive_status_t s = hive_timer_recv(poll_timer, &msg, SD_BUSY_TIMEOUT_MS);
    if (HIVE_FAILED(s)) {
        hive_timer_cancel(poll_timer);
        return -1;  // Timeout - card stuck
    }
}
hive_timer_cancel(poll_timer);
```

Predictable overhead (~1ms granularity), uses existing timer primitive.
Worst case during a 250ms spike: 250 polls, each yielding for 1ms while
other actors run.

**Alternatives rejected:**

*hive_yield() loop* - Polls as fast as the scheduler reschedules. Works for
the pilot (250Hz = ~4ms between polls) but in applications with few or fast
actors, spins at a very high rate for up to 250ms. Not appropriate for a
general-purpose runtime.

*EXTI interrupt on MISO* - The SD busy signal is read by clocking 0xFF
through SPI, not by passively monitoring the GPIO level. Using EXTI would
require reconfiguring the pin from SPI alternate function to GPIO input
mid-transfer, or relying on underdocumented EXTI behavior with AF-mode
pins. Fragile, board-specific, and unnecessary.

## Implementation Plan

### Step 1 - DMA bulk transfer in spi_ll

Add DMA bulk transfer functions to the SPI low-level interface. These
replace byte-by-byte `spi_send()`/`spi_recv()` for 512-byte sector data.

Files to modify:
- `src/hal/stm32/spi_ll.h` - Add `spi_ll_dma_init()`, `spi_ll_dma_start()`,
  `spi_ll_dma_ok()`, `spi_ll_lock()`, `spi_ll_unlock()` to low-level SPI
  interface
- `examples/pilot/hal/crazyflie-2.1plus/spi_ll_sd.c` - Implement DMA
  using DMA2 Stream 0 (SPI1_RX) and DMA2 Stream 3 (SPI1_TX), add ISR

DMA streams are board-specific (depends on which SPI peripheral the SD card
is on and which DMA streams are free). On the Crazyflie 2.1+, SD card uses
SPI1 and syslink already uses DMA2 Stream 2, leaving Stream 0 and Stream 3
free for SPI1.

### Step 2 - DMA + yield in spi_sd

Replace blocking byte loops with DMA start + yield in the SD protocol layer.

Files to modify:
- `src/hal/stm32/spi_sd.c` - Replace `spi_send(buf, 512)` and
  `spi_recv(buf, 512)` with `spi_ll_dma_start()` + `hive_event_wait()`.
  Replace `wait_ready()` tight loop with `hive_timer_every()` +
  `hive_timer_recv()` poll loop.

This is the core change. The SD protocol (commands, tokens, CRC) stays
byte-by-byte. Only the 512-byte sector data transfers use DMA. The
`wait_ready()` busy loop becomes a periodic timer poll so other actors
run during card-internal write latency (up to 250ms).

`diskio.c` is unchanged - it calls `spi_sd_write_blocks()` /
`spi_sd_read_blocks()` which now yield internally.

### Step 3 - Testing

- Bench test: verify DMA transfers complete correctly (ground test mode)
- Timing test: measure scheduler latency during SD writes with stack
  watermarking
- Flight test: FIRST_TEST profile + SD logging, verify no deadman timeouts
- Stress test: rapid logging at maximum rate, verify no data corruption

### Step 4 - Documentation update

SD card file I/O is no longer blocking. Many places in the documentation
warn about file I/O stalling the scheduler and recommend LOW priority actors
for file work. After DMA yield, SD writes on STM32 yield cooperatively like
any other blocking call (`hive_ipc_recv`, `hive_event_wait`). Linux file I/O
is still synchronous POSIX but rarely blocks in practice (OS page cache).

The "file I/O is synchronous" framing changes to: file I/O is non-blocking
on STM32 (DMA + yield for SD, ring buffer for flash) and synchronous on
Linux (OS-buffered, rarely stalls).

Files to update:
- `CLAUDE.md` - File subsystem description (line 155: "Synchronous POSIX
  I/O (briefly pauses scheduler)"), man page description (line 35:
  "Synchronous file I/O"), platform differences (line 569)
- `README.md` - File I/O note (line 138: "synchronous and briefly stalls
  the scheduler"), code example comment (line 369)
- `docs/spec/api.md` - File API note (line 2062: "synchronous and briefly
  pauses the scheduler"), STM32 behavior (line 2117), sync description
  (line 2135)
- `docs/spec/design.md` - LOW priority recommendation (line 468)
- `docs/spec/internals.md` - Platform differences table (line 489:
  "Synchronous POSIX")
- `man/man3/hive_file.3` - Blocking note (line 23), scheduler stall
  description (line 163)

## Constraints

- **No heap allocation** - All DMA descriptors and buffers are static
- **No new runtime primitives** - Uses existing HAL events and
  `hive_event_wait()`
- **Backward compatible** - `hive_file_write()` API unchanged
- **Flash path unchanged** - Already has ring buffer, no modification needed
- **Linux path unchanged** - OS page cache handles buffering, no DMA needed
- **Single SPI bus** - Only one actor can do SD I/O at a time (logger owns
  the SD file descriptors)
- **FatFS unchanged** - No modifications to FatFS source
- **diskio unchanged** - All changes are in spi_sd.c and spi_ll_sd.c below
