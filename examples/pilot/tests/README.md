# Pilot HAL Tests

Tests for validating the pilot HAL and Hive file I/O subsystems.

## Architecture

These tests are organized by what they validate:

| Test | API Used | Description |
|------|----------|-------------|
| `sensors_motors` | Pilot HAL | HAL API validation (sensors, motors, calibration) |
| `flash` | Hive File API | Tests flash-backed `/log` via `hive_file_*()` |
| `sd` | Hive File API | Tests SD-backed `/sd` via `hive_file_*()` (placeholder) |
| `main` | Combined | Runs all applicable tests (Webots dummy pilot) |

**Key distinction:**
- `sensors_motors` uses the pilot HAL (`hal/hal.h`) + Hive logging
- `flash` and `sd` use the Hive runtime (`hive_file.h`, `hive_runtime.h`)
- `main` combines all tests for use as a Webots controller

For bare-metal hardware bring-up tests (direct register access), see
`hal/crazyflie-2.1+/bringup/`.

## Quick Start

```bash
# Build and flash a specific test (Crazyflie)
make PLATFORM=crazyflie TEST=sensors_motors
make flash-crazyflie TEST=sensors_motors

# Build combined test for Webots (dummy pilot)
make PLATFORM=webots TEST=main

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

### main (Combined Test Runner)

Runs all applicable tests in sequence. Primarily intended for use as a
"dummy pilot" in Webots simulation.

**Platform behavior:**
- **Webots:** Runs `sensors_motors` only (flash/SD are hardware-specific)
- **Crazyflie:** Runs all tests sequentially (`sensors_motors`, `flash`, `sd`)

**Test sequence:**
1. Initialize HAL and Hive runtime (once)
2. Run `sensors_motors` test
3. Run `flash` test (Crazyflie only)
4. Run `sd` test (Crazyflie only)
5. Report summary (passed/failed/skipped)

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 1 blink | Starting combined test |
| (individual test patterns) | See individual test docs |
| LED on solid | All tests passed |
| Slow blink | Some tests failed |

## Build Instructions

### Crazyflie 2.1+

```bash
# Build specific test
make PLATFORM=crazyflie TEST=sensors_motors
make PLATFORM=crazyflie TEST=flash
make PLATFORM=crazyflie TEST=sd
make PLATFORM=crazyflie TEST=main

# Build all tests
make PLATFORM=crazyflie all-tests

# Flash specific test
make flash-crazyflie TEST=sensors_motors
make flash-crazyflie TEST=flash
make flash-crazyflie TEST=sd
make flash-crazyflie TEST=main
```

### Webots Simulation

```bash
export WEBOTS_HOME=/usr/local/webots  # adjust path

# Build combined test (recommended for Webots)
make PLATFORM=webots TEST=main

# Or build individual test
make PLATFORM=webots TEST=sensors_motors
```

Then copy `build_webots/test_main` (or `test_sensors_motors`) to a Webots
controller directory.

**Note:** `flash` and `sd` tests are Crazyflie-specific and test hardware
that doesn't exist in simulation. The `main` test automatically skips them
on Webots.

## Build System

The Makefile automatically detects which tests need the Hive library:

- `sensors_motors`: Links against pilot HAL only (`libhal.a`)
- `flash`, `sd`, `main`: Links against Hive + pilot HAL (`libhive.a` + `libhal.a`)

For the `main` test, individual test files are compiled as objects (with
`-DTEST_MAIN_BUILD`) to exclude their standalone `main()` functions.

The Hive library is built automatically when needed with minimal configuration
suitable for tests.

## Debug Output

All tests output progress via `HIVE_LOG_*` macros (which use `hal_printf()`).
On Crazyflie, connect a debugger and monitor SWO output to see detailed test
progress. On Webots, output goes to stdout.

Example output for `flash` test:
```
========================================
  Flash Storage Test (Hive File API)
========================================
Generating test pattern (4096 bytes)...
Opening /log for writing (erases flash)...
Flash erased and opened (fd=0, 1523 ms)
Writing 4096 bytes in 256-byte blocks...
Write OK (42 ms)
Syncing to flash...
Sync OK (15 ms)
Re-opening /log for reading...
Reading and verifying 4096 bytes...
Read and verify OK (3 ms)

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

Example output for `main` (combined) test on Webots:
```
========================================
  Combined Test Runner
========================================

--- Running: sensors_motors ---
========================================
  Sensors and Motors Test (Pilot HAL)
========================================
Phase 1: hal_init() PASS
...
sensors_motors: PASSED

--- Skipping: flash (not available on this platform) ---

--- Skipping: sd (not available on this platform) ---

========================================
  TEST SUMMARY
========================================
  Passed:  1
  Failed:  0
  Skipped: 2
========================================
  ALL TESTS PASSED!
```

## Adding New Tests

1. Create `test_<name>.c` in this directory
2. Add a `test_<name>_run(bool standalone)` function
3. Wrap the standalone `main()` in `#ifndef TEST_MAIN_BUILD`
4. If the test uses Hive file API, add `<name>` to `HIVE_TESTS` in Makefile
5. If including in `main`, add object rules and link it
6. Update this README with test documentation
