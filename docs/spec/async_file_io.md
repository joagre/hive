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

DMA + yield in the FatFS diskio layer (STM32 only). Make SPI transfers
non-blocking by yielding to the scheduler during DMA.

No ring buffer needed for SD:
- On Linux, the OS page cache already buffers writes
- On STM32, DMA + yield makes the SPI transfer itself non-blocking
- FatFS has its own internal sector cache for small writes
- Adding a ring buffer would be buffering on top of buffering

### DMA + yield in diskio

**Modified diskio layer:**
```c
// src/hal/stm32/diskio.c (FatFS hardware abstraction)

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    // Start DMA transfer (returns immediately)
    spi_sd_start_dma_write(buff, sector, count);

    // Yield to scheduler - other actors run while DMA transfers data
    hive_event_wait(sd_dma_event, SD_WRITE_TIMEOUT_MS);

    // DMA complete - check result
    return spi_sd_check_result() ? RES_OK : RES_ERROR;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    // Same pattern for reads
    spi_sd_start_dma_read(buff, sector, count);
    hive_event_wait(sd_dma_event, SD_READ_TIMEOUT_MS);
    return spi_sd_check_result() ? RES_OK : RES_ERROR;
}
```

**How it works:**
1. `disk_write()` starts a DMA transfer on the SPI peripheral
2. DMA runs autonomously (no CPU involvement)
3. `hive_event_wait()` yields the current actor to the scheduler
4. The scheduler runs other actors (sensor, estimator, altitude, etc.)
5. When DMA completes, the SPI DMA ISR calls `hive_hal_event_signal()`
6. The scheduler wakes the yielded actor
7. `disk_write()` returns, FatFS continues

**Key insight:** FatFS doesn't need to change. It calls `disk_write()` and
gets a result. It doesn't know the scheduler ran flight-critical actors in
between. The diskio layer is already platform-specific, so this is a clean
abstraction boundary.

**ISR integration:**
```c
// SPI DMA transfer complete ISR
void DMA1_Stream4_IRQHandler(void) {
    if (DMA1->HISR & DMA_HISR_TCIF4) {
        DMA1->HIFCR = DMA_HIFCR_CTCIF4;  // Clear flag
        spi_sd_dma_finish();               // Deassert CS, etc.
        hive_hal_event_signal(SD_DMA_EVENT_ID);  // Wake actor
    }
}
```

`hive_hal_event_signal()` is already ISR-safe (single 32-bit store on ARM
Cortex-M). This uses the existing HAL event mechanism - no new primitives.

**What about disk_read()?** Same pattern. DMA + yield for reads too. This
makes log download via radio non-blocking as well.

### Typical write flow during flight

```
Logger actor (LOW priority, every 40ms):
    hive_file_write(csv_line, ~150 bytes)
        -> f_write() [FatFS buffers in sector cache]
        -> if sector full:
            -> disk_write(sector)
                -> start SPI DMA       [~1 us]
                -> hive_event_wait()   [yields to scheduler]
                    ... sensor_actor runs (CRITICAL) ...
                    ... estimator_actor runs ...
                    ... altitude_actor runs ...
                    ... rate_actor runs ...
                    ... motor_actor runs ...
                -> DMA ISR signals event
                -> actor wakes, continues
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

The current SPI driver polls MISO waiting for the card to finish. With DMA
yield, this busy-wait must also yield to the scheduler. Options:

**Option A - Timer-based polling with yield:**
```c
while (!spi_sd_card_ready()) {
    hive_yield();  // Let other actors run, check again next schedule
}
```
Simple but wastes scheduler cycles polling. At 250Hz control loop, each
yield is ~4ms, so worst case we check 60+ times during a 250ms spike.

**Option B - Busy signal interrupt + HAL event:**
```c
// Configure MISO pin as EXTI interrupt (rising edge = card ready)
spi_sd_enable_busy_irq();
hive_event_wait(sd_busy_event, SD_BUSY_TIMEOUT_MS);
spi_sd_disable_busy_irq();
```
Efficient but requires EXTI line for the MISO pin. Must verify pin is
EXTI-capable on the Crazyflie 2.1+ board.

**Option C - Periodic timer + yield:**
```c
hive_timer_id_t poll_timer;
hive_timer_after(1000, &poll_timer);  // Check every 1ms
while (!spi_sd_card_ready()) {
    hive_timer_recv(poll_timer, &msg, -1);  // Yield for 1ms
    hive_timer_after(1000, &poll_timer);
}
```
Predictable overhead (~1ms granularity), uses existing timer mechanism.

Option A is simplest and adequate. The scheduler overhead of extra yields is
negligible compared to the 250ms card-internal delay.

## Implementation Plan

### Step 1 - DMA SPI transfer functions

Files to modify:
- `src/hal/stm32/spi_sd.c` - Add `spi_sd_start_dma_write()`,
  `spi_sd_start_dma_read()`, `spi_sd_check_result()`
- `hal/crazyflie-2.1plus/spi_ll_sd.c` - Board-specific DMA stream/channel
  config, ISR handlers

Prerequisites:
- Identify free DMA streams on the STM32F405 (must not conflict with UART
  DMA used by syslink)
- Verify SPI peripheral supports DMA (SPI1/SPI2/SPI3 all do on STM32F405)

### Step 2 - Yield in diskio

Files to modify:
- `src/hal/stm32/diskio.c` - Replace synchronous SPI calls with DMA + yield
  pattern in `disk_write()` and `disk_read()`

This is the core change. FatFS calls `disk_write()`/`disk_read()` in the
diskio layer. We replace the blocking SPI transfer with DMA start + yield +
wake on completion.

### Step 3 - Card busy-wait with yield

Files to modify:
- `src/hal/stm32/spi_sd.c` - Replace busy-wait loop with yield loop

After DMA transfer completes, the SD card may internally busy for up to
250ms. The current code polls MISO in a tight loop. Replace with a yield
loop so other actors run during the wait.

### Step 4 - Testing

- Bench test: verify DMA transfers complete correctly (ground test mode)
- Timing test: measure scheduler latency during SD writes with stack
  watermarking
- Flight test: FIRST_TEST profile + SD logging, verify no deadman timeouts
- Stress test: rapid logging at maximum rate, verify no data corruption

## Constraints

- **No heap allocation** - All DMA descriptors and buffers are static
- **No new runtime primitives** - Uses existing HAL events and
  `hive_event_wait()`
- **Backward compatible** - `hive_file_write()` API unchanged
- **Flash path unchanged** - Already has ring buffer, no modification needed
- **Linux path unchanged** - OS page cache handles buffering, no DMA needed
- **Single SPI bus** - Only one actor can do SD I/O at a time (logger owns
  the SD file descriptors)
- **FatFS unchanged** - All changes are in the diskio layer below FatFS

## Alternatives Considered

**Ring buffer for SD card** - Adds a RAM buffer between `hive_file_write()`
and FatFS. Reduces blocking frequency but doesn't eliminate it - the flush
still blocks during SPI. On Linux, redundant with OS page cache. On STM32,
redundant with DMA yield. Buffering on top of buffering.

**Idle-time flush** - Flush writes during scheduler idle (WFI). Risks data
loss if the system never reaches idle. Doesn't help if writes happen faster
than idle time allows.

**Dedicated flush actor** - Separate actor for SD card I/O via IPC. Adds
message-passing overhead for every write and doesn't solve blocking (the
flush actor still blocks during SPI without DMA yield).

**Reduce log verbosity only** - Treats the symptom. Any future log message
during flight could trigger the same stall. Not acceptable for a flight
controller.

**Increase deadman timeout** - Makes the symptom less likely but doesn't fix
the scheduler stall. Other time-sensitive actors would still be starved.
