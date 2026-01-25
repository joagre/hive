# Pilot HAL Tests

Tests for validating the pilot HAL and Hive file I/O subsystems.

## Architecture

These tests are organized by what they validate:

| Test | API Used | Description |
|------|----------|-------------|
| `sensors_motors` | Pilot HAL | HAL API validation (sensors, motors, calibration) |
| `flash` | Hive File API | Tests flash-backed `/log` via `hive_file_*()` |
| `sd` | Hive File API | Tests SD-backed `/sd` via `hive_file_*()` (placeholder) |

**Key distinction:**
- `sensors_motors` uses only the pilot HAL (`hal/hal.h`)
- `flash` and `sd` use the Hive runtime (`hive_file.h`, `hive_runtime.h`)

For bare-metal hardware bring-up tests (direct register access), see
`hal/crazyflie-2.1+/bringup/`.

## Quick Start

```bash
# Build and flash a specific test
make PLATFORM=crazyflie TEST=sensors_motors
make flash-crazyflie TEST=sensors_motors

# Build all tests
make PLATFORM=crazyflie all-tests
```

## Test Details

### sensors_motors

Validates the complete pilot HAL API stack.

**Test sequence:**
1. `hal_init()` - Initialize all hardware
2. `hal_self_test()` - Verify sensors respond
3. `hal_calibrate()` - Calibrate sensors (keep still and level!)
4. `hal_arm()` - Arm motors
5. Sensor loop - Read sensors for 5 seconds
6. Motor test - Brief motor spin (2 seconds at 15% thrust)
7. `hal_disarm()` - Disarm motors
8. `hal_cleanup()` - Cleanup

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | hal_init() passed, starting self-test |
| 3 blinks | hal_self_test() passed, starting calibration |
| 4 blinks | Calibration done, starting sensor read |
| Fast blink | Sensor read loop |
| 5 blinks | Sensor test done, starting motor test |
| Medium blink | Motors running |
| 6 blinks | Motor test done |
| LED on solid | All tests passed! |

### flash

Tests the Hive file API with flash-backed storage (`/log` path).

Uses `hive_file_open()`, `hive_file_write()`, `hive_file_pread()`, etc.
to exercise the STM32 flash file backend.

**WARNING:** This test erases `/log`! Any existing log data will be lost.

**Test sequence:**
1. Initialize Hive runtime
2. Open `/log` for writing (erases flash sector 8)
3. Write 4KB test pattern
4. Sync and close
5. Re-open for reading
6. Read back with `hive_file_pread()` and verify
7. Report results with timing

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | Starting flash test |
| Fast blink | Writing/reading in progress |
| LED on solid | All tests passed! |
| Slow blink | Test failed |

### sd

Tests the Hive file API with SD card storage (`/sd` path).

**NOTE:** This is a placeholder test. The SD card backend (`HIVE_ENABLE_SD`)
is not yet implemented in Hive. The test checks mount availability and
gracefully reports when SD is not available.

**Requirements (once implemented):**
- Micro SD Card Deck attached to expansion connector
- FAT32-formatted SD card inserted
- Hive built with `HIVE_ENABLE_SD=1`

**Test sequence:**
1. Initialize Hive runtime
2. Check if `/sd` mount is available
3. If not available: report status and exit gracefully
4. If available: write test file, read back, verify

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | Starting SD test |
| 3 blinks | SD not available (expected until implemented) |
| Fast blink | Writing/reading in progress |
| LED on solid | All tests passed! |
| Slow blink | Test failed (SD available but I/O error) |

## Build Instructions

### Crazyflie 2.1+

```bash
# Build specific test
make PLATFORM=crazyflie TEST=sensors_motors
make PLATFORM=crazyflie TEST=flash
make PLATFORM=crazyflie TEST=sd

# Build all tests
make PLATFORM=crazyflie all-tests

# Flash specific test
make flash-crazyflie TEST=sensors_motors
make flash-crazyflie TEST=flash
make flash-crazyflie TEST=sd
```

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path
make PLATFORM=webots TEST=sensors_motors
```

Then copy `build_webots/test_sensors_motors` to a Webots controller directory.

**Note:** `flash` and `sd` tests are Crazyflie-specific and test hardware
that doesn't exist in simulation.

## Build System

The Makefile automatically detects which tests need the Hive library:

- `sensors_motors`: Links against pilot HAL only (`libhal.a`)
- `flash`, `sd`: Links against Hive + pilot HAL (`libhive.a` + `libhal.a`)

The Hive library is built automatically when needed with minimal configuration
suitable for tests.

## Debug Output

All tests output progress via SWD debug (printf over ITM or semihosting).
Connect a debugger and monitor SWO output to see detailed test progress.

Example output for `flash` test:
```
========================================
  Flash Storage Test (Hive File API)
========================================

[FLASH] Initializing Hive runtime...
[FLASH] Hive runtime initialized
[FLASH] Generating test pattern (4096 bytes)...
[FLASH] Opening /log for writing (erases flash)...
[FLASH] Flash erased and opened (fd=0, 1523 ms)
[FLASH] Writing 4096 bytes in 256-byte blocks... OK (42 ms)
[FLASH] Syncing to flash... OK (15 ms)
[FLASH] Re-opening /log for reading...
[FLASH] Reading and verifying 4096 bytes... OK (3 ms)

========================================
  FLASH TEST PASSED!
========================================

  Path:       /log
  Size:       4096 bytes
  Erase time: 1523 ms
  Write time: 42 ms (97523 bytes/sec)
  Sync time:  15 ms
  Read time:  3 ms (1365333 bytes/sec)
```

## Adding New Tests

1. Create `test_<name>.c` in this directory
2. If the test uses Hive file API, add `<name>` to `HIVE_TESTS` in Makefile
3. Update this README with test documentation
