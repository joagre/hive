# SD Card DMA File I/O

DMA-based SD card I/O for the Hive actor runtime on STM32. Sector data
transfers use DMA with spin-wait for speed, card busy-wait yields to the
scheduler so flight-critical actors keep running.

## Problem

Hive is single-threaded and cooperative. When an actor calls
`hive_file_write()` on an SD card path, the write goes synchronously through
FatFS -> SPI. SD cards have unpredictable write latency (50-250ms+ spikes
from internal garbage collection). During this time, no other actor can run -
including flight-critical control loops that need the CPU every 4ms.

## Solution

Two separate optimizations in `spi_sd.c`:

1. **DMA bulk transfer with spin-wait** - 512-byte sector data uses DMA
   instead of byte-by-byte SPI. DMA is ~23x faster (24us vs 700us per block
   at 21 MHz). The CPU spin-waits for the ~24us transfer - too short to
   benefit from a context switch.

2. **Yielding busy-wait** - After the card receives data, it may take
   10-250ms to program internal flash. During this time, the driver yields
   to the scheduler via `hive_sleep()` with 1ms polls, letting other actors
   run.

### Write flow

```
spi_sd_write_blocks():
    spi_ll_lock()                    acquire SPI bus
    spi_ll_cs_low()                  assert CS (full SPI reset)
    send_cmd(CMD24, addr)            command bytes (byte-by-byte)
    spi_ll_xfer(0xFE)                data token
    spi_ll_dma_start(buf, 512, TX)   start DMA
    spin-wait ~24us                  poll spi_ll_dma_ok()
    spi_ll_dma_cleanup()             wait BSY, drain RX FIFO
    spi_ll_xfer(CRC)                 2 CRC bytes
    spi_ll_xfer(0xFF)                read data response
    spi_ll_cs_high()                 deassert CS
    spi_ll_unlock()                  release SPI bus
    wait_ready_yield():              card programming (10-250ms)
        lock, cs_low_poll, read byte
        while busy:
            cs_high, unlock
            hive_sleep(1ms)          YIELD - other actors run here
            lock, cs_low_poll, read byte
        cs_high, unlock
```

FatFS and `diskio.c` are unchanged. The yield is invisible to them.

### Read flow

Same pattern: DMA spin-wait for the 512-byte data, no card busy-wait needed
after reads.

## DMA hardware (Crazyflie 2.1+)

SD card uses SPI1 (shared with Flow Deck PMW3901):

| Stream | Channel | Usage |
|--------|---------|-------|
| DMA2 Stream 0 | Ch 3 | SPI1_RX (SD card) |
| DMA2 Stream 3 | Ch 3 | SPI1_TX (SD card) |

SPI is full-duplex: both TX and RX DMA streams run for every transfer.
For writes, TX sends data and RX discards. For reads, TX sends 0xFF and
RX captures data. The RX stream ISR signals completion (RX finishes after
TX, so RX complete = transfer done).

## Quirks and lessons learned

### DMA enable order (critical)

DMA streams MUST be enabled BEFORE setting SPI CR2 TXDMAEN/RXDMAEN.

When SPI is idle, TXE=1. Setting TXDMAEN while TXE=1 causes SPI to
immediately generate a DMA request. If the DMA stream is not yet enabled,
the request is lost and DMA never completes (timeout).

```c
// Correct (matches Bitcraze firmware):
DMA2_Stream0->CR |= DMA_SxCR_EN;  // RX stream first
DMA2_Stream3->CR |= DMA_SxCR_EN;  // TX stream second
SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;  // Now safe

// WRONG - causes DMA timeout:
SPI1->CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;  // Request fires NOW
DMA2_Stream0->CR |= DMA_SxCR_EN;  // Too late, request lost
DMA2_Stream3->CR |= DMA_SxCR_EN;
```

### Spin-wait, not yield, during DMA

The original design used `hive_event_wait()` to yield during the ~24us DMA
transfer. This caused dresp=0x00 errors (card rejecting writes) and DMA
timeouts. The exact mechanism was never identified - the flow deck properly
checks the bus lock and skips SPI access when locked - but yielding during
DMA consistently corrupted the SPI transaction.

Since 512 bytes at 21 MHz takes only ~24us, yielding provides no meaningful
benefit. Spin-waiting is the correct approach. The scheduler gets its time
during the much longer card busy-wait phase (10-250ms).

### SPI bus locking scope

The bus must be locked for the ENTIRE SD card transaction (command + data +
CRC + response), not just during the DMA transfer. The flow deck sensor
actor runs at 250 Hz and shares SPI1. If it accesses SPI1 between the DMA
transfer and the data response read, it corrupts the SD card's response.

```c
// Correct - lock covers entire transaction:
spi_ll_lock();
spi_ll_cs_low();
send_cmd(CMD24, addr);
dma_xfer(buf, 512, TX);    // DMA spin-wait
spi_ll_xfer(CRC);
resp = spi_ll_xfer(0xFF);  // data response, still locked
spi_ll_cs_high();
spi_ll_unlock();

// WRONG - flow deck can corrupt response:
send_cmd(CMD24, addr);
spi_ll_lock();
dma_xfer(buf, 512, TX);
spi_ll_unlock();            // flow deck runs here
resp = spi_ll_xfer(0xFF);  // reads garbage
```

### SPI1 peripheral reset (RCC) vs lightweight reconfigure

Two ways to reconfigure SPI1 after the flow deck has used it:

- **`spi_ll_cs_low()`** - Full RCC peripheral reset. Clears all SPI1
  registers and FIFOs. Required before DMA transfers (clean slate). Side
  effect: corrupts MSB of first byte READ (0xFF reads as 0x7F). Callers
  that poll for exact 0xFF must discard the first byte.

- **`spi_ll_cs_low_poll()`** - Disables SPE, writes CR1, re-enables SPE.
  No RCC reset, no MSB corruption. Used during busy polling between yields.
  NOT safe for DMA (stale FIFO state can cause DMA errors).

### Busy-wait CS management

During `wait_ready_yield()`, CS is deasserted between polls so the flow
deck can use SPI1 freely during the 1ms sleep:

```
poll:  lock -> cs_low_poll -> discard byte -> read byte -> cs_high -> unlock
sleep: hive_sleep(1ms)  [flow deck reads SPI1 here]
poll:  lock -> cs_low_poll -> discard byte -> read byte -> ...
```

The discard byte after `cs_low_poll` is necessary because the SPE toggle
during reconfigure leaves a stale byte in the SPI receive buffer.

CS high uses `spi_ll_cs_high_no_dummy()` (no dummy clock) to avoid clock
glitches that cause MSB corruption on the next `cs_low_poll` cycle.

### Pre-scheduler fallback

Before the scheduler starts (FatFS mount, initial f_open), `dma_xfer()`
checks if DMA has been initialized. If not, it falls back to synchronous
byte-by-byte. `wait_ready_yield()` checks `in_actor_context()` and falls
back to tight polling (no yield, no bus locking needed since no other actor
is running).

### SD card power cycle requirement

The SD card retains state across MCU resets (st-flash). Failed writes can
put the card in an unrecoverable state. Recovery requires a physical power
cycle (battery disconnect for 10+ seconds). The init sequence includes an
aggressive recovery: 2048-byte flush with CS low, CMD12, 1024 clocks with
CS high, and up to 20 CMD0 retries.

### Data response 0x00

A data response of 0x00 (all zeros) after a write means the card is pulling
MISO low. This is NOT a valid data response token (valid tokens have bit 0
set: 0x05=accepted, 0x0B=CRC error, 0x0D=write error). It indicates the SPI
bus was corrupted during the transaction, typically from the flow deck
reconfiguring SPI1 mid-transaction or from yielding during DMA.

### Write retry logic

Single-block writes retry up to 3 times on data response failure. Recovery
between retries: deassert CS, wait for card ready via `wait_ready_yield()`,
then re-send the entire command + data block. No extra clocking with CS low
during recovery (that can put the card in an unrecoverable state).

## Files

| File | Role |
|------|------|
| `src/hal/stm32/spi_sd.c` | SD protocol, DMA spin-wait, busy-wait yield |
| `src/hal/stm32/spi_ll.h` | Low-level SPI interface (board BSP contract) |
| `examples/pilot/hal/crazyflie-2.1plus/spi_ll_sd.c` | SPI1 + DMA2 implementation, ISR |
| `lib/fatfs/diskio.c` | FatFS disk I/O (unchanged, calls spi_sd) |
