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

**SD card files** (`/sd/*`) - Fully synchronous, no buffering:
```
hive_file_write() -> f_write() [FatFS] -> disk_write() [SPI] -> hardware
                     entire call blocks scheduler for duration of SPI transfer
```

## Design

Two layers, each independently valuable:

1. **Ring buffer for SD card** (both platforms) - Absorbs write bursts into
   RAM, batches I/O. Same pattern as flash files.
2. **DMA + yield in diskio** (STM32 only) - Makes SPI transfers non-blocking
   by yielding to the scheduler during DMA.

### Layer 1 - SD card ring buffer

Add a ring buffer between `hive_file_write()` and FatFS, identical in concept
to the flash ring buffer. Writes push to RAM (fast). Actual FatFS writes
happen during flush.

```
Writer actor                    Ring buffer              FatFS/SPI
    |                               |                       |
    |-- hive_file_write(data) ----->|                       |
    |   [memcpy to ring, O(1)]      |                       |
    |<-- HIVE_SUCCESS --------------|                       |
    |                               |                       |
    |-- hive_file_sync() ---------->|                       |
    |                               |-- f_write(batch) ---->|
    |                               |   [SPI transfer]      |
    |                               |<-- done --------------|
    |<-- HIVE_SUCCESS --------------|                       |
```

**Flush triggers:**
- Explicit `hive_file_sync()` call (application-controlled)
- Ring buffer approaching full (e.g., 75% threshold)
- File close

**Ring buffer full behavior:**
- Default (`pool_block = false`): Return `HIVE_ERR_NOMEM`, caller decides
- With `pool_block = true`: Yield until space available (flush in progress)
- This follows the same pattern as IPC pool exhaustion

**Configuration:**
```c
// In hive_static_config.h
#define HIVE_SD_RING_SIZE       (8 * 1024)  // 8 KB default (same as flash)
```

**Memory:**
- Static ring buffer: `HIVE_SD_RING_SIZE` bytes (8 KB default)
- Allocated at init, not on hot path
- One ring buffer shared across all SD file descriptors (writes are
  serialized anyway - single SPI bus)

**Platform behavior:**
- **STM32** - Ring buffer absorbs writes, flush goes through FatFS/SPI
- **Linux** - Ring buffer absorbs writes, flush uses POSIX write() (already
  fast due to OS page cache, but ring still helps with burst absorption)

### Layer 2 - DMA yield (STM32 only)

Make the SPI transfer itself non-blocking by using DMA and yielding to the
scheduler while the transfer runs.

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

### Combined flow

With both layers active, a typical write path during flight:

```
Logger actor (LOW priority, every 40ms):
    hive_file_write(csv_line)
        -> ring_push()          [~1 us, never blocks scheduler]
        -> return HIVE_SUCCESS

Logger actor (every 1 second):
    hive_file_sync()
        -> flush ring to FatFS
        -> f_write(8KB batch)
            -> disk_write(sector)
                -> start SPI DMA   [~1 us]
                -> hive_event_wait [yields to scheduler]
                    ... sensor_actor runs ...
                    ... estimator_actor runs ...
                    ... altitude_actor runs ...
                    ... rate_actor runs ...
                    ... motor_actor runs ...
                -> DMA ISR signals event
                -> actor wakes, continues
            -> disk_write(next sector)
                -> [repeat yield pattern]
        -> return HIVE_SUCCESS
```

The control loop runs uninterrupted at 250Hz. The SD card write happens in
the background, sector by sector, with the scheduler running other actors
between each sector transfer.

## Implementation Plan

### Phase 1 - Ring buffer for SD card

Files to modify:
- `src/hal/stm32/hive_hal_file.c` - Add ring buffer to SD write path
- `src/hal/linux/hive_hal_file.c` - Add ring buffer to file write path
- `include/hive_static_config.h` - Add `HIVE_SD_RING_SIZE` config

The flash ring buffer implementation in `hive_hal_file.c` (STM32) is the
template. The SD path needs:
- Static ring buffer allocation
- `sd_write()` pushes to ring instead of calling `f_write()` directly
- `sd_sync()` flushes ring through `f_write()`
- `sd_close()` flushes before closing

On Linux, the same pattern applies to POSIX file writes.

**Testing:**
- Unit test: ring buffer push/flush/wrap behavior
- Integration test: file write + sync + read back
- QEMU test: SD card write path with ring buffer
- Flight test: verify no scheduler stalls during logging

### Phase 2 - DMA yield in diskio (STM32)

Files to modify:
- `src/hal/stm32/spi_sd.c` - Add DMA transfer functions
- `src/hal/stm32/diskio.c` - Use DMA + yield in disk_read/disk_write
- `hal/crazyflie-2.1plus/spi_ll_sd.c` - Board-specific DMA config
- `hal/crazyflie-2.1plus/hal_config.h` - DMA channel/stream assignments

The existing `hive_hal_event_signal()` / `hive_event_wait()` mechanism
handles the ISR-to-actor communication. The SPI DMA peripheral is already
available on the STM32F405 (DMA1 or DMA2 depending on SPI peripheral).

**Prerequisites:**
- Phase 1 must be complete (ring buffer provides the batching)
- DMA channel must not conflict with other peripherals (UART DMA for syslink)
- Need to verify which DMA streams are free on the Crazyflie 2.1+

**Testing:**
- Bench test: verify DMA transfers complete correctly
- Timing test: measure scheduler latency during SD writes
- Flight test: full flight with SD logging, verify no deadman timeouts

### Phase 3 - Verification

- Run flight test with FIRST_TEST profile + SD logging
- Verify motor deadman never fires during normal operation
- Measure worst-case scheduler latency during SD writes
- Compare telemetry quality (no gaps in 25Hz CSV)

## Constraints

- **No heap allocation** - Ring buffer is statically allocated
- **No new runtime primitives** - Uses existing HAL events and pool patterns
- **Backward compatible** - API unchanged, behavior improves transparently
- **Flash path unchanged** - Already has ring buffer, no modification needed
- **Single SPI bus** - Ring buffer serializes writes naturally (one flush at
  a time)

## Alternatives Considered

**Idle-time flush** - Flush SD ring buffer during scheduler idle (WFI). This
would never block actor execution but risks data loss if the system is busy
and never reaches idle. Also doesn't help if the ring fills up.

**Dedicated flush actor** - A separate actor that owns the SD card and
processes write requests via IPC. Adds complexity (message passing for every
write) and doesn't solve the blocking problem (the flush actor still blocks
during SPI).

**Reduce log verbosity only** - Treats the symptom. Any future log message
during flight could trigger the same stall. Not acceptable for a flight
controller.

**Increase deadman timeout** - Makes the symptom less likely to manifest but
doesn't fix the underlying scheduler stall. Other time-sensitive actors would
still be affected.
