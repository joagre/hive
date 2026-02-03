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
| `esb` | Pilot HAL + Hive | ESB radio communication test (interrupt-based syslink) |
| `main` | Combined | Runs all applicable tests (Webots dummy pilot) |

**Key distinction:**
- `sensors_motors` uses the pilot HAL (`hal/hal.h`) + Hive logging
- `flash` and `sd` use the Hive runtime (`hive_file.h`, `hive_runtime.h`)
- `main` combines all tests for use as a Webots controller

For bare-metal hardware bring-up, use the `sensors_motors` test with motor
testing disabled (default), then enable with `ENABLE_MOTOR_TEST=1`.

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
   - M1 Front-Left (CCW), M2 Front-Right (CW), M3 Rear-Right (CCW), M4 Rear-Left (CW)
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

**Note:** The SD card deck shares SPI1 with the Flow deck. Both decks can be
used simultaneously - the test works with or without the Flow deck attached.

```bash
make PLATFORM=crazyflie TEST=sd ENABLE_SD=1
make flash-crazyflie TEST=sd
./st-trace.sh
```

**Filename limitation:** FatFS is configured for 8.3 filenames only (no long
filenames) to save ~2KB flash. Use short names like `data.bin`, `log001.txt`.

**Test sequence:**
1. Initialize Hive runtime
2. Check if `/sd` mount is available
3. If not available: report status (built without `ENABLE_SD=1` or card not present)
4. If available: write 4KB test file (`/sd/test.bin`), read back, verify

**Typical performance:** Write ~450 KB/s, Read ~1000 KB/s.

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 2 blinks | Starting SD test |
| 3 blinks | SD not available (build without `ENABLE_SD=1` or no card) |
| Fast blink | Writing/reading in progress |
| LED on solid | All tests passed! |
| Slow blink | Test failed (SD available but I/O error) |

### esb

Tests ESB radio communication with Crazyradio PA/2.0.

The Crazyflie communicates with the ground via the nRF51822 co-processor using
the syslink protocol over USART6 at 1Mbaud. This test validates the DMA-based
syslink implementation and ESB protocol.

**ESB Protocol:**

The nRF51 on the Crazyflie is configured as PRX (Primary Receiver), while the
Crazyradio on the ground station is PTX (Primary Transmitter). This means:

- Ground station initiates all communication by sending packets
- Drone can only respond via ACK payloads (cannot send spontaneously)
- Telemetry from drone piggybacks on ACK packets
- Ground station poll rate determines maximum telemetry rate

```
Ground (PTX)                    Drone nRF51 (PRX)
     |                                |
     |-------- poll packet --------->|
     |<------- ACK + telemetry ------|
     |                                |
```

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
python3 ../tools/ground_station.py --uri radio://0/80/2M
```

**Test sequence (Crazyflie):**
1. Initialize HAL and radio (`hal_esb_init()`)
2. Wait for battery voltage from nRF51 (proves internal syslink works)
3. Queue telemetry packets at 100Hz via `hal_esb_send()`
4. Packets are sent when ground station polls (ACK payload)

**Test sequence (ground station):**
1. Scan for Crazyflie (or use specified URI)
2. Connect via Crazyradio
3. Poll continuously
4. Receive and decode telemetry packets from ACK payloads

**Packet formats (must match comms_actor.c and ground_station.py):**

ESB max payload is 32 bytes. HAL uses 1 byte for framing, so max payload is 30 bytes.

| Type | Name | Payload | Contents |
|------|------|---------|----------|
| 0x01 | Attitude | 17 bytes | type + timestamp_ms + gyro_xyz + roll/pitch/yaw |
| 0x02 | Position | 17 bytes | type + timestamp_ms + alt + vz/vx/vy + thrust + battery_mv |

**LED feedback:**
| Pattern | Meaning |
|---------|---------|
| 1 blink | Starting test |
| 2 blinks | Radio initialized |
| 3 blinks | Battery received from nRF51 |
| Fast blink | Sending telemetry |
| LED on solid | Test complete |

**Example output (ground_station.py):**
```
Connecting to channel 80, 2M...
Connected!
Waiting for telemetry... (Ctrl+C to stop)
ATT  gyro=( +0.00,  +0.00,  +0.00) rad/s  rpy=(+0.00, +0.00, +0.00) rad
POS  alt=+0.00m  vz=+0.00m/s  vxy=(+0.00, +0.00)m/s  thrust=0.0%  bat=4.15V
ATT  gyro=( +0.00,  +0.00,  +0.00) rad/s  rpy=(+0.00, +0.00, +0.00) rad
POS  alt=+0.00m  vz=+0.00m/s  vxy=(+0.00, +0.00)m/s  thrust=0.0%  bat=4.15V
...
```

**Note:** Test sends zeros for sensor values since no sensors are read. Battery
voltage in position packets shows the actual nRF51-reported battery level.

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
./st-trace.sh -t 30    # 30s timeout
./st-trace.sh -t 0     # No timeout (Ctrl-C to stop)
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
