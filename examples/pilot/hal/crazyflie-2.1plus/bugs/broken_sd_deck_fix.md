# Broken SD Card Deck - Corrupted 1-Wire EEPROM

## Date

2026-02-14

## Symptom

SD card deck not detected. CMD0 returns 0xFF on every attempt.
Bitcraze firmware console shows:

    DECK_BACKEND_OW: Memory error: wrong header ID
    DECK_CORE: 0 deck(s) found

## Root Cause

The DMA-before-scheduler bug (fixed in c29eabe) left DMA2 running on SPI1
after hive_event_wait() returned HIVE_ERR_INVALID from main() context. The
runaway DMA clocked garbage on SPI1, which corrupted the DS28E05 1-wire
EEPROM on the SD card deck. The 1-wire bus shares the expansion header with
SPI1.

Without a valid EEPROM, neither the Bitcraze firmware nor our firmware can
detect the deck, so SPI chip select for the SD card is never configured,
and the card never responds.

## How to Diagnose

1. Flash Bitcraze firmware (CLOAD=0) and check cfclient console
2. If you see `Memory error: wrong header ID` - the EEPROM is corrupted
3. If the deck is detected but `mount SD-Card [FAIL]` - the EEPROM is fine,
   the problem is elsewhere (SPI bus state, card format, etc.)

## How to Fix

### Prerequisites

- Bitcraze firmware source at local/crazyflie-firmware
- cflib installed in examples/pilot/tests/.venv
- ST-Link connected
- SD card deck connected (can be without flow deck)

### Step 1 - Patch Bitcraze firmware

In `src/deck/backends/deck_backend_onewire.c`, function `infoDecode()`:

```c
if (info->header != DECK_INFO_HEADER_ID) {
    DEBUG_PRINT("Memory error: wrong header ID (continuing for repair)\n");
    return true;  // was: return false;
}
```

### Step 2 - Build, flash, and reset

```bash
cd local/crazyflie-firmware
make clean && make CLOAD=0 -j$(nproc)
st-flash --connect-under-reset write build/cf2.bin 0x8000000
st-flash reset
```

### Step 3 - Repair the EEPROM

```bash
source examples/pilot/tests/.venv/bin/activate
python3 examples/pilot/tools/repair_sd_deck_ow.py --uri radio://0/80/2M
```

Verify output shows `Board name=bcUSD` and `Valid: True`.

### Step 4 - Flash our firmware back

```bash
cd examples/pilot
make -f Makefile.crazyflie-2.1plus clean
make -f Makefile.crazyflie-2.1plus FLIGHT_PROFILE=FLIGHT_PROFILE_GROUND_TEST ENABLE_SD=1
st-flash --connect-under-reset write build_crazyflie-2.1plus/pilot_crazyflie-2.1plus.bin 0x8000000
```

Power cycle the drone. The SD card should now be detected.

### Step 5 - Revert the Bitcraze firmware patch

```bash
cd local/crazyflie-firmware
git checkout -- src/deck/backends/deck_backend_onewire.c
```

## Prevention

The fix in commit c29eabe prevents this from recurring. The dma_xfer()
function in src/hal/stm32/spi_sd.c checks in_actor_context() and falls
back to byte-by-byte SPI when called before the scheduler starts. DMA
hardware is never started outside actor context.
