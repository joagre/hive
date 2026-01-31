# Pilot HAL Tests

Tests for validating the pilot HAL and Hive file I/O subsystems.

## Architecture

These tests are organized by what they validate:

| Test | API Used | Description |
|------|----------|-------------|
| `sensors_motors` | Pilot HAL | HAL API validation (sensors, motors, calibration) |
| `thrust` | Pilot HAL | Thrust calibration (find HAL_BASE_THRUST for hover) |
| `flash` | Hive File API | Tests flash-backed `/log` via `hive_file_*()` |
| `sd` | Hive File API | Tests SD-backed `/sd` via `hive_file_*()` |
| `esb` | Pilot HAL + Hive | ESB radio communication test (DMA-based syslink) |
| `main` | Combined | Runs all applicable tests (Webots dummy pilot) |

**Key distinction:**
- `sensors_motors` uses the pilot HAL (`hal/hal.h`) + Hive logging
- `flash` and `sd` use the Hive runtime (`hive_file.h`, `hive_runtime.h`)
- `main` combines all tests for use as a Webots controller

For bare-metal hardware bring-up tests (direct register access), see
`hal/crazyflie-2.1plus/bringup/`.

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
6. Individual motor test - Each motor separately (2s each, 2s pause between)
   - M1 Front-Right (CCW), M2 Back-Right (CW), M3 Back-Left (CCW), M4 Front-Left (CW)
7. All motors test - All 4 motors together (2 seconds at 15% thrust)
8. `hal_disarm()` - Disarm motors
9. `hal_cleanup()` - Cleanup

**Motor test is disabled by default.** Enable with `ENABLE_MOTOR_TEST=1`:
```bash
make PLATFORM=crazyflie TEST=sensors_motors ENABLE_MOTOR_TEST=1
```

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | hal_init() passed, starting self-test |
| 3 blinks | hal_self_test() passed, starting calibration |
| 4 blinks | Calibration done, starting sensor read |
| Fast blink | Sensor read loop |
| 5 blinks | Sensor test done, starting motor test |
| Medium blink | Individual motor running |
| Fast blink | All motors running |
| 6 blinks | Motor test done |
| LED on solid | All tests passed! |

### thrust

Calibrates the base thrust value needed for hover.

The `HAL_BASE_THRUST` parameter in `hal_config.h` must be calibrated for each
Crazyflie based on battery charge and weight. This test helps find the correct
value by running motors at a specified thrust level.

**Calibration procedure:**
1. Start with `THRUST_TEST_VALUE=0.30f`
2. Build and flash the test
3. Place Crazyflie on flat surface
4. Observe during 5-second motor run:
   - **No movement:** increase thrust by 0.02
   - **Light on skids:** correct value found!
   - **Lifts off:** decrease thrust by 0.01
5. Update `HAL_BASE_THRUST` in `hal/crazyflie-2.1plus/hal_config.h`

**Usage:**
```bash
# Default thrust (0.35)
make PLATFORM=crazyflie TEST=thrust
make flash-crazyflie TEST=thrust

# Custom thrust value
make PLATFORM=crazyflie TEST=thrust CFLAGS+='-DTHRUST_TEST_VALUE=0.40f'
make flash-crazyflie TEST=thrust
```

**Safety:**
- REMOVE PROPELLERS for initial testing!
- Once props are on, test in a safe area with clearance
- Keep hands clear - motors spin at significant speed
- Test runs for 5 seconds then stops automatically

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | Init complete, starting calibration |
| 3 blinks | Calibration done, motors starting in 3 seconds |
| Slow blink | Motors running at test thrust |
| LED on solid | Test complete |

**Example output:**
```
========================================
  Thrust Calibration Test
========================================
Test thrust: 0.38 (38%)
Duration: 5000 ms
Running self-test...
Self-test PASSED
Calibrating sensors (keep still and level)...
Calibration done
Motors starting in 3 seconds...
Place drone on flat surface NOW!
  3...
  2...
  1...
MOTORS STARTING at 0.38 thrust!
Running at 0.38 thrust... 1000 ms / 5000 ms
Running at 0.38 thrust... 2000 ms / 5000 ms
Running at 0.38 thrust... 3000 ms / 5000 ms
Running at 0.38 thrust... 4000 ms / 5000 ms
========================================
  Test complete!
========================================
Tested thrust: 0.38
Observations:
  - No movement: increase to 0.40
  - Light on skids: GOOD! Use 0.38 as HAL_BASE_THRUST
  - Lifted off: decrease to 0.37
Update hal/crazyflie-2.1plus/hal_config.h with found value
```

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

**Requirements:**
- Micro SD Card Deck attached to expansion connector
- FAT32-formatted SD card inserted
- Hive built with `ENABLE_SD=1`

```bash
make PLATFORM=crazyflie TEST=sd ENABLE_SD=1
make flash-crazyflie TEST=sd
```

**Test sequence:**
1. Initialize Hive runtime
2. Check if `/sd` mount is available
3. If not available: report status (built without `ENABLE_SD=1` or card not present)
4. If available: write 4KB test file, read back, verify

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | Starting SD test |
| 3 blinks | SD not available (build without `ENABLE_SD=1` or no card) |
| Fast blink | Writing/reading in progress |
| LED on solid | All tests passed! |
| Slow blink | Test failed (SD available but I/O error) |

### esb

Tests ESB radio communication with Crazyradio PA/2.0 via DMA-based syslink.

The Crazyflie communicates with the ground via the nRF51822 co-processor using
the syslink protocol over USART6 at 1Mbaud. This test validates the DMA-based
RX implementation required for reliable high-speed reception.

**Requirements:**
- Crazyradio PA or Crazyradio 2.0 USB dongle
- Python 3 with cflib: `pip install cflib`

**Build and flash:**
```bash
make PLATFORM=crazyflie TEST=esb
make flash-crazyflie TEST=esb
```

**Run ground station (on laptop):**
```bash
python test_radio_ground.py [--uri URI] [--duration SECONDS]
```

**Test sequence (Crazyflie side):**
1. Initialize HAL and Hive runtime
2. Initialize DMA-based ESB radio (`hal_esb_init()`)
3. Wait for battery packet from nRF51 (proves DMA RX works)
4. Wait for ground station connection (RADIO_RAW packet)
5. Send telemetry packets at 10Hz for 30 seconds:
   - Attitude packets (gyro, roll/pitch/yaw)
   - Echo packets (sequence + data)
6. Report TX/RX statistics

**Test sequence (ground station):**
1. Scan for Crazyflie (or use specified URI)
2. Connect via Crazyradio
3. Receive and decode telemetry packets
4. Display real-time statistics

**Packet types:**
| Type | ID | Contents |
|------|-----|----------|
| Attitude | 0x01 | timestamp, gyro_xyz, roll/pitch/yaw |
| Position | 0x02 | timestamp, altitude, velocities, thrust |
| Echo | 0xE0 | sequence, data string |

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 1 blink | Starting test |
| 2 blinks | Radio initialized (DMA enabled) |
| 3 blinks | Battery packet received |
| 4 blinks | Ground station connected |
| Fast blink | Sending/receiving packets |
| LED on solid | Test complete (success) |
| Slow blink | Error |

**Example ground station output:**
```
============================================================
  Radio Communication Test - Ground Station
============================================================

Scanning for Crazyflie...
Found 1 Crazyflie(s):
  [0] radio://0/80/2M/E7E7E7E7E7
Connecting to radio://0/80/2M/E7E7E7E7E7...
Connected!

Receiving packets for 60 seconds...
Press Ctrl+C to stop

[ 30.0s] RX:   300 (att: 150 pos:   0 echo: 150) | R:+0.00 P:+0.00 Y:+1.50

============================================================
  TEST COMPLETE
============================================================
  Duration:     30.0 seconds
  Total RX:     300
  Attitude:     150
  Position:     0
  Echo:         150
  Unknown:      0
  Rate:         10.0 packets/sec
============================================================
```

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
make PLATFORM=crazyflie TEST=thrust
make PLATFORM=crazyflie TEST=flash
make PLATFORM=crazyflie TEST=sd
make PLATFORM=crazyflie TEST=esb
make PLATFORM=crazyflie TEST=main

# Build all tests
make PLATFORM=crazyflie all-tests

# Flash specific test
make flash-crazyflie TEST=sensors_motors
make flash-crazyflie TEST=thrust
make flash-crazyflie TEST=flash
make flash-crazyflie TEST=sd
make flash-crazyflie TEST=esb
make flash-crazyflie TEST=main

# Thrust test with custom value
make PLATFORM=crazyflie TEST=thrust CFLAGS+='-DTHRUST_TEST_VALUE=0.40f'
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

**Note:** `flash`, `sd`, and `esb` tests are Crazyflie-specific. The `main`
test automatically skips `flash` and `sd` on Webots. The `esb` test is not
included in `main` since it requires external ground station hardware.

## Build System

The Makefile automatically detects which tests need the Hive library:

- `sensors_motors`: Links against pilot HAL only (`libhal.a`)
- `flash`, `sd`, `esb`, `main`: Links against Hive + pilot HAL (`libhive.a` + `libhal.a`)

The `esb` test also requires `HAL_HAS_RADIO` to be defined and disables SD
support to avoid linker conflicts.

For the `main` test, individual test files are compiled as objects (with
`-DTEST_MAIN_BUILD`) to exclude their standalone `main()` functions.

The Hive library is built automatically when needed with minimal configuration
suitable for tests.

## Debug Output

All tests output progress via `HIVE_LOG_*` macros (which use `hal_printf()`).
On Crazyflie, connect a debugger and monitor SWO output to see detailed test
progress. On Webots, output goes to stdout.

**Viewing SWO output on Crazyflie:**
```bash
# After flashing, run trace from any directory:
../tools/st-trace.sh -t 30    # 30s timeout
../tools/st-trace.sh -t 0     # No timeout (Ctrl-C to stop)
```

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
