# Crazyflie 2.1+ Hardware Bring-Up Tests

Standalone firmware for systematically verifying each hardware component
before running the full pilot firmware.

## Hardware Setup

### Required Equipment

1. **Crazyflie 2.1+** - The target platform
2. **Bitcraze Debug Adapter Kit** - Connects to the 0.05" debug header
3. **ST-Link V3** - For flashing and SWO debug output

### Connections

```
Crazyflie 2.1+          Debug Adapter          ST-Link V3
+--------------+        +------------+        +----------+
| Debug Header |------->| 10-pin     |------->| 20-pin   |
| (0.05" pitch)|        | to 20-pin  |        | SWD+SWO  |
+--------------+        +------------+        +----------+
                                                   |
                                                   v
                                              USB to PC
                                              (view with st-trace)
```

### Debug Header Pinout (Crazyflie)

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | VCC | 3.0V reference |
| 2 | SWDIO | SWD data |
| 3 | GND | Ground |
| 4 | SWCLK | SWD clock |
| 5 | GND | Ground |
| 6 | SWO | Trace output (debug output) |
| 7 | GND | Ground |
| 8 | PA9 | USART1_TX (unused) |
| 9 | GND | Ground |
| 10 | PA10 | USART1_RX (unused) |

### SWO Debug Output

Debug output uses SWO (Serial Wire Output) via the ITM (Instrumentation
Trace Macrocell). This is a one-way output channel - no input is possible.

1. Connect ST-Link V2 or V3 to PC via USB
2. Flash the firmware
3. View output: `./st-trace.sh`

The `st-trace.sh` script uses a locally built stlink-tools (the system
version 1.8.0 has bugs with SWO trace). If you need to build it:

```bash
cd local/stlink
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make && make install
```

Alternatively, run manually: `st-trace --clock=168m --trace=2m`

## Hardware Architecture

### I2C Bus Configuration

The Crazyflie 2.1+ has two separate I2C buses:

| Bus | Pins | Speed | Devices |
|-----|------|-------|---------|
| I2C3 | PA8 (SCL), PC9 (SDA) | 400 kHz | On-board sensors (BMI088, BMP388) |
| I2C1 | PB6 (SCL), PB7 (SDA) | 400 kHz | Expansion connector (EEPROM, Flow deck VL53L1x) |

**Important**: The Flow deck's VL53L1x ToF sensor is on **I2C1** (expansion connector),
not I2C3 (on-board sensors). This is a common source of confusion.

### SPI Configuration

| Bus | Pins | Devices |
|-----|------|---------|
| SPI1 | PA5 (SCK), PA6 (MISO), PA7 (MOSI), PB4 (CS) | PMW3901 optical flow (Flow deck) |
| SPI3 | PB3 (SCK), PB4 (MISO), PB5 (MOSI), PB6 (CS) | SD card (Micro SD Card deck) |

### Expansion Deck Connector GPIO

| Signal | STM32 Pin | Description |
|--------|-----------|-------------|
| IO1 | PB8 | General purpose I/O |
| IO2 | PB5 | General purpose I/O |
| IO3 | PB4 | PMW3901 chip select (Flow deck) |
| IO4 | PC12 | General purpose I/O |
| OW | PC11 | 1-Wire deck detection |

### Deck Detection Architecture

**Important**: The 1-Wire deck EEPROM is connected to the **NRF51** power management
processor, NOT the STM32 main processor. PC11 is NOT connected to the deck EEPROM.

```
                      +-----------+
Expansion Deck ------>| NRF51     |<------ Syslink (1Mbaud) ------>| STM32
EEPROM (DS28E05)      | GPIO 8    |         USART6 (PC6/PC7)       | Main CPU
                      +-----------+
```

The NRF51 handles:
- 1-Wire protocol to read deck EEPROMs
- Deck enumeration at boot
- Forwarding deck info to STM32 via syslink

The STM32 must request deck info from the NRF51 via syslink commands:
- `SYSLINK_OW_SCAN (0x20)` - Get number of detected decks
- `SYSLINK_OW_GETINFO (0x21)` - Get deck serial number
- `SYSLINK_OW_READ (0x22)` - Read deck EEPROM (VID/PID)

**Limitation**: Syslink at 1Mbaud requires DMA/interrupt-driven UART for reliable
communication. The bringup firmware uses simple polling which causes overrun errors.
As a workaround, deck detection uses sensor probing - if VL53L1x and PMW3901 are
detected, we infer a Flow deck is present.

### Stock Firmware Reference

**The Bitcraze crazyflie-firmware repository is the normative truth for this
platform.** When in doubt about initialization sequences, sensor configuration,
or hardware behavior, refer to the Bitcraze firmware implementation at:
https://github.com/bitcraze/crazyflie-firmware

The file `crazflie_firmware_log.txt` contains console output from the stock
Bitcraze firmware (captured via cfclient). Useful reference values:

| Item | Value | Notes |
|------|-------|-------|
| PMW3901 chip ID | 0x49 | Product ID register 0x00 |
| PMW3901 inverse ID | 0xB6 | Register 0x5F (sanity check) |
| VL53L1x | "ZR2: Z-down sensor [OK]" | Detected on I2C1 |
| Flow deck driver | bcFlow2 | PID 0x0A via 1-Wire |
| BMI088 | "Using I2C interface" | On I2C3 |
| BMP388 | "I2C connection [OK]" | On I2C3 |
| EEPROM | "I2C connection [OK]" | On I2C1 at 0x50 |

The log also shows FreeRTOS task stack usage which may be useful for sizing
actor stacks in the Hive pilot firmware.

### VL53L1x I2C Address Note

The VL53L1x ToF sensor has a default I2C address of **0x29**, but the Bitcraze
firmware reassigns it to avoid conflicts when multiple ranging decks are used.
The address is stored in the sensor's volatile memory and persists until power
cycle, but some boards may have NVM that persists the address change.

**Important**: The bringup firmware scans for VL53L1x at multiple addresses:
- 0x29 (default)
- 0x6A (observed on some units)
- 0x30-0x32 (Bitcraze reassignment range)
- 0x52 (alternative address)

If you've previously run the Bitcraze firmware, the VL53L1x may be at a
different address. The bringup code will find it automatically by checking the
model ID register (0x010F) which should return 0xEACC for VL53L1x.

## Building and Flashing

```bash
# From this directory (bringup/)
make clean
make

# Flash using ST-Link
make flash

# Or manually:
st-flash write build/bringup.bin 0x08000000

# Reset to start
st-flash reset
```

## Bring-Up Sequence

The firmware runs through each hardware component systematically,
**ordered from most primitive to most complex**. Each phase can pass (OK),
fail (FAIL), or be skipped (SKIP).

### Phase 1: Boot & Clock

Verifies basic system initialization:
- LED blink proves Reset_Handler executed
- SystemCoreClock value confirms PLL setup (expect 168 MHz)
- SysTick operational

**Expected output**
```
=== Phase 1: Boot & Clock ===
[BOOT] LED blink test... OK
[BOOT] SystemCoreClock = 168000000 Hz... OK
```

### Phase 2: LEDs

Tests all controllable LEDs on the Crazyflie 2.1+ using simple GPIO output.

**Hardware**:
- Blue LED: PD2 (status indicator)
- Red LEDs: PC0-PC3 (motor position indicators M1-M4)

Note: The red/green charge LEDs are controlled by the nRF51 and cannot
be tested directly from the STM32.

| Test | Description |
|------|-------------|
| GPIO Init | Configures LED pins as outputs |
| Blue LED | Blinks blue LED 3 times |
| Red M1-M4 | Blinks each motor LED 2 times in sequence |
| All Pattern | All LEDs on/off pattern |
| Rotation | Rotating pattern through M1-M4 |

**Expected output**
```
=== Phase 2: LED Test ===
[LED] Testing all controllable LEDs
[LED] Watch the Crazyflie for LED activity
[LED] Initializing GPIO... OK
[LED] Blue LED (PD2) - 3 blinks... OK
[LED] Red LED M1 (PC0, front-left) - 2 blinks... OK
[LED] Red LED M2 (PC1, front-right) - 2 blinks... OK
[LED] Red LED M3 (PC2, rear-right) - 2 blinks... OK
[LED] Red LED M4 (PC3, rear-left) - 2 blinks... OK
[LED] All LEDs - on/off pattern... OK
[LED] Rotating pattern (M1->M2->M3->M4)... OK
[LED] All LED tests complete!
```

### Phase 3: I2C Bus Scan

Scans I2C3 bus (on-board sensors) for connected devices:

| Address | Device | Bus | Required |
|---------|--------|-----|----------|
| 0x18 | BMI088 Accelerometer | I2C3 | Yes |
| 0x69 | BMI088 Gyroscope (SDO=VDD) | I2C3 | Yes |
| 0x77 | BMP388 Barometer | I2C3 | Yes |

**Note**: The VL53L1x ToF sensor (Flow deck) is on I2C1 (expansion connector),
not I2C3. It is detected separately during the sensor init phase.

**Expected output**
```
=== Phase 3: I2C Bus Scan ===
[I2C] Scanning I2C3 bus...
[I2C] Found device at 0x18 (BMI088 Accel)
[I2C] Found device at 0x69 (BMI088 Gyro)
[I2C] Found device at 0x77 (BMP388 Baro)
[I2C] Scan complete: 3 devices found... OK
```

### Phase 4: Sensors

Reads chip IDs and sensor data via I2C and SPI:

**Chip IDs:**

| Sensor | Bus | Register | Expected Value |
|--------|-----|----------|----------------|
| BMI088 Accel | I2C3 (0x18) | 0x00 | 0x1E |
| BMI088 Gyro | I2C3 (0x69) | 0x00 | 0x0F |
| BMP388 | I2C3 (0x77) | 0x00 | 0x50 |
| VL53L1x | I2C1 (0x29) | Model ID | 0xEACC |
| PMW3901 | SPI1 (PB4 CS) | Product ID | 0x49 |

**Sensor Data (at rest, level):**

| Sensor | Check | Expected |
|--------|-------|----------|
| Accelerometer | X, Y near 0; Z near +1g | +/- 0.1g for X,Y; 0.9-1.1g for Z |
| Gyroscope | All axes near 0 | +/- 5 deg/s |
| Barometer | Pressure | 95-106 kPa (varies with altitude) |
| Barometer | Temperature | 15-35 C (room temp) |
| ToF | Range | 0-2000 mm (depends on surface) |

**Expected output**
```
=== Phase 4: Sensors ===
[SENSOR] BMI088 Accel chip ID = 0x1E... OK
[SENSOR] BMI088 Gyro chip ID = 0x0F... OK
[SENSOR] BMP388 chip ID = 0x50... OK
[DATA] Accelerometer: X=-0.02g Y=0.01g Z=1.01g... OK
[DATA] Gyroscope: X=0.5 Y=-0.3 Z=0.1 deg/s... OK
[DATA] Barometer: 101325 Pa, 23.5 C... OK
```

### Phase 5: EEPROM

Tests the AT24C64 configuration EEPROM on I2C1 (separate bus from sensors).

**Hardware**: AT24C64 8KB EEPROM
- I2C1: PB6 (SCL), PB7 (SDA)
- Address: 0x50

| Test | Description |
|------|-------------|
| I2C Init | Initializes I2C1 peripheral (400 kHz fast mode) |
| Detect | Scans 0x50-0x57 for EEPROM |
| Write | Writes 16-byte test pattern to last page (0x1FE0) |
| Verify | Reads back and verifies data integrity |

**Expected output**
```
=== Phase 5: EEPROM Test ===
[EEPROM] Configuration EEPROM (I2C1: PB6=SCL, PB7=SDA)
[EEPROM] Initializing I2C1... OK
[EEPROM] Scanning I2C1 for EEPROM (0x50-0x57)...
[EEPROM] Found EEPROM at 0x50... OK
[EEPROM] Writing 16 bytes to address 0x1FE0... OK (6 ms)
[EEPROM] Reading and verifying 16 bytes... OK (1 ms)
[EEPROM] All tests passed!
```

### Phase 6: Deck Detection

Detects expansion decks by probing for deck-specific sensors. The deck EEPROM
(DS28E05) is connected to the NRF51 processor, not the STM32, so direct 1-Wire
access is not possible without DMA-based syslink communication.

**Detection Method**: Sensor probing (indirect)

| Deck | Detection Method |
|------|------------------|
| Flow deck v2 | VL53L1x (I2C1) + PMW3901 (SPI1) present |
| Z-ranger deck | VL53L1x (I2C1) present, no PMW3901 |
| Other decks | Not detected (would require syslink DMA) |

**Why not 1-Wire?**

The deck EEPROM is on NRF51 GPIO 8, not STM32 PC11. Reading the EEPROM requires
syslink commands to the NRF51, but syslink at 1Mbaud causes UART overrun errors
without DMA. See "Deck Detection Architecture" section above.

**Known Bitcraze Decks (VID=0xBC)**

| PID | Deck Name | Bringup Detection |
|-----|-----------|-------------------|
| 0x01 | LED ring deck | Not detected |
| 0x06 | Loco positioning deck | Not detected |
| 0x08 | Micro SD card deck | Not detected (use Phase 10) |
| 0x09 | Flow deck | VL53L1x + PMW3901 |
| 0x0A | Flow deck v2 | VL53L1x + PMW3901 |
| 0x0C | Z-ranger deck | VL53L1x only |
| 0x0F | Z-ranger deck v2 | VL53L1x only |
| 0x10 | Lighthouse deck | Not detected |
| 0x12 | AI deck | Not detected |

**Expected output (Flow deck present)**
```
=== Phase 6: Deck Detection ===
[DECK] Note: Deck EEPROM is on NRF51 1-Wire bus
[DECK] Using I2C/SPI sensor detection as proxy
[DECK] Checking for deck sensors...
[DECK] VL53L1x + PMW3901 detected in Phase 4 = Flow deck present
[DECK]   VID: 0xBC (Bitcraze)
[DECK]   PID: 0x0A (inferred from sensors)
[DECK]   Name: Flow deck v2 (inferred)
[DECK] Detected 1 deck(s)
```

**Expected output (no deck)**
```
=== Phase 6: Deck Detection ===
[DECK] Note: Deck EEPROM is on NRF51 1-Wire bus
[DECK] Using I2C/SPI sensor detection as proxy
[DECK] Checking for deck sensors...
[DECK] No deck-specific sensors detected
[DECK] (This is normal if no expansion deck is attached)
```

### Phase 7: Motors

**WARNING: REMOVE PROPELLERS BEFORE THIS TEST!**

Tests timer-based PWM output. Each motor is tested individually.

**Motor layout (X-config, viewed from above):**
```
         FRONT
     M1(CCW)  M2(CW)
         +--+
         |  |
         +--+
     M4(CW)  M3(CCW)
         REAR
```

**Motor to pin mapping (Bitcraze reference):**

| Motor | Position | Rotation | Pin | Timer |
|-------|----------|----------|-----|-------|
| M1 | Front-left | CCW | PA1 | TIM2_CH2 |
| M2 | Front-right | CW | PB11 | TIM2_CH4 |
| M3 | Rear-right | CCW | PA15 | TIM2_CH1 |
| M4 | Rear-left | CW | PB9 | TIM4_CH4 |

**Auto-Proceed Mode**

By default, `MOTOR_TEST_AUTO_PROCEED` is enabled in `bringup_motors.c` for
automated testing. Motors will spin without user confirmation after delays:
- 3 second initial delay
- 1 second delay before each motor test
- 1 second spin time for rotation verification

To disable auto-proceed and require manual confirmation via SWO input,
set `#define MOTOR_TEST_AUTO_PROCEED 0` in `bringup_motors.c`.

The test:
1. Shows motor layout diagram with expected rotations
2. Waits (or prompts if auto-proceed disabled)
3. Slowly ramps up each motor so rotation direction is visible
4. Tests all motors together at 10%

**Expected output (auto-proceed mode)**
```
=== Phase 7: Motor Test ===
[MOTOR] !!! WARNING: REMOVE PROPELLERS !!!
[MOTOR] AUTO-PROCEED enabled - starting in 3 seconds...
[MOTOR] Starting motor test...

[MOTOR] Motor layout (X-config, viewed from above):
...

----------------------------------------
[MOTOR] Testing M1 (front-left)
[MOTOR] Expected rotation: CCW
[MOTOR] AUTO-PROCEED: spinning in 1 second...
[MOTOR] Ramping up M1...
[MOTOR] M1 spinning at 15% - verify CCW rotation
[MOTOR] AUTO-PROCEED: running for 1 second...
[MOTOR] M1 test complete
----------------------------------------
... (repeat for M2, M3, M4) ...
----------------------------------------
[MOTOR] Individual tests complete

[MOTOR] AUTO-PROCEED: spinning all motors in 1 second...
[MOTOR] Spinning all motors at 10%...
[MOTOR] All motors stopped

[MOTOR] Motor test complete
```

### Phase 8: Radio (Syslink)

Tests USART serial communication with the nRF51822 via syslink protocol.
Waits for battery voltage packet from nRF51.

**Hardware**: USART6 at 1Mbaud
- PC6: TX to NRF51
- PC7: RX from NRF51
- PA4: TXEN flow control (input, nRF51 signals when busy)

**Implementation**: The bringup test uses DMA2 Stream 1 Channel 5 for USART6 RX
in circular buffer mode, matching the approach used in the Bitcraze firmware.

**Syslink Activation Sequence**

The nRF51 requires explicit activation before it starts sending data to STM32:

1. **OW_SCAN (0x20)** - Any valid syslink packet activates the `isSyslinkActive`
   flag on the nRF51. We send OW_SCAN (one-wire scan) as a safe activation packet.

2. **SYS_NRF_VERSION (0x30)** - Request nRF51 firmware version (optional, may
   trigger a response packet).

3. **PM_BATTERY_AUTOUPDATE (0x14)** - Enable automatic battery status updates.
   This starts the 100Hz battery packets from nRF51 to STM32.

Without this activation sequence, the nRF51 will not send any data even though
the USART and DMA are correctly configured.

**Expected output**
```
=== Phase 8: Radio Test ===
[RADIO] Initializing syslink (USART6)... OK
[RADIO] PA4 (TXEN) initial state: LOW
[RADIO] Activating syslink...
[RADIO]   Sending OW_SCAN (0x20)...
[RADIO]   Sending SYS_NRF_VERSION (0x30)...
[RADIO]   Sending PM_BATTERY_AUTOUPDATE (0x14)...
[RADIO] Waiting for battery packet...
[RADIO] Battery voltage: 3.67V... OK
```

### Phase 9: Flash Storage

Tests internal flash sector 8 (128KB at 0x08080000) for telemetry logging.
This is complex because flash programming must run from RAM.

**WARNING: This test erases sector 8! Existing log data will be lost.**

| Test | Description |
|------|-------------|
| Erase | Erases flash sector 8 (128KB) |
| Write | Writes 4KB test pattern (256-byte blocks) |
| Verify | Reads back and verifies data integrity |

**Expected output**
```
=== Phase 9: Flash Storage Test ===
[FLASH] Erasing sector 8 at 0x08080000 (128 KB)... OK
[FLASH] Writing 4096 bytes in 256-byte blocks... OK (12 ms)
[FLASH] Reading and verifying 4096 bytes... OK (1 ms)
[FLASH] Write speed: 4096 bytes in 12 ms (341333 bytes/sec)
```

### Phase 10: SD Card (Optional)

Tests SD card via SPI3 + SD protocol (requires Micro SD Card Deck attached).
This is the most complex test, combining SPI with the SD card protocol.

**Hardware**: Micro SD Card Deck
- SPI3: PB3 (SCK), PB4 (MISO), PB5 (MOSI)
- CS: PB6

**SWO Pin Conflict**: SPI3 SCK (PB3) shares the same pin as SWO trace output.
When the SD test initializes SPI3, it reconfigures PB3 from SWO (AF0) to
SPI3_SCK (AF6), which kills SWO debug output. The test runs "blind" after
this point - you won't see any more trace output until the board is reset.

**Workaround**: Check the final LED status after the test completes:
- **Solid LED** = All critical tests passed (phases 1-4)
- **Fast blinking LED** = One or more critical tests failed

| Test | Description |
|------|-------------|
| SPI Init | Initializes SPI3 peripheral |
| Card Detect | Sends CMD0, CMD8, ACMD41 to detect card type |
| Write | Writes 512-byte test block to block 65536 |
| Verify | Reads back and verifies data integrity |

**Expected output (deck present)**
```
=== Phase 10: SD Card Test ===
[SD] Initializing SPI3 (PB3=SCK, PB4=MISO, PB5=MOSI, PB6=CS)... OK
[SD] Detecting card...
[SD]   CMD0 (GO_IDLE)... OK
[SD]   CMD8 (IF_COND)... OK (SDv2)
[SD]   ACMD41 (SD_SEND_OP_COND)... OK
[SD]   CMD58 (READ_OCR)... OK (SDHC/SDXC)
[SD] Card type: SDHC
[SD] Writing test block 65536 (512 bytes)... OK (8 ms)
[SD] Reading and verifying block 65536... OK (2 ms)
[SD] All tests passed!
```

**Expected output (no deck)**
```
=== Phase 10: SD Card Test ===
[SD] Initializing SPI3 (PB3=SCK, PB4=MISO, PB5=MOSI, PB6=CS)... OK
[SD] Detecting card...
[SD]   CMD0 (GO_IDLE)... FAIL (R1=0xFF, expected 0x01)
[SD]   Card not detected or not responding
[SD] Card not detected - deck not attached?
```

## Troubleshooting

### No output on SWO

1. Use `./st-trace.sh` (system st-trace v1.8.0 has bugs)
2. Verify SWO pin connected (pin 6 on debug header, pin 13 on 20-pin)
3. Try ST-Link V2 if V3 doesn't work (some V3 units have issues)
4. Try resetting the target: `st-flash reset`
5. If building st-trace locally, ensure libusb-dev is installed

### I2C devices not found

**On-board sensors (I2C3 - PA8/PC9):**
1. Check I2C3 pull-ups present on board
2. Verify PA8 (SCL) and PC9 (SDA) not shorted
3. Try slower I2C clock (reduce from 400kHz to 100kHz)
4. Check for bus contention (other masters)

**Expansion deck sensors (I2C1 - PB6/PB7):**
1. Ensure deck is fully seated in expansion headers
2. Check I2C1 pull-ups on expansion connector (PB6, PB7)
3. Verify the correct I2C bus is being used (VL53L1x is on I2C1, not I2C3)
4. EEPROM and VL53L1x share I2C1 - if EEPROM works, bus is OK

### Sensor chip ID wrong

1. Verify correct I2C address
2. Check for counterfeit/different sensor variant
3. Ensure power supply stable (check 3.0V rail)

### Motor doesn't spin

1. Verify propellers REMOVED
2. Check motor connector seated properly
3. Verify PWM output with oscilloscope
4. Try higher duty cycle (20%)
5. Check battery charged (>3.0V)

### Sensor data out of range

1. Accelerometer: Ensure board is level
2. Gyroscope: Ensure board is stationary
3. Barometer: Check for pressure leaks, verify altitude
4. ToF: Ensure clear line of sight to surface

### Flash erase/write fails

1. Check power supply stability (flash operations need stable 3.0V)
2. Verify sector 8 is not write-protected in option bytes
3. Check flash is not busy (previous operation in progress)
4. Ensure no debugger breakpoints in flash code

### SD card not detected

1. Verify Micro SD Card Deck is properly attached
2. Check card is FAT32 formatted (exFAT not supported)
3. Ensure card is fully inserted in deck slot
4. Try a different SD card (some cards have compatibility issues)
5. Check SPI3 pins not shorted (PB3, PB4, PB5, PB6)

### SD write/read fails

1. Ensure card is not write-protected (no lock tab)
2. Check card has sufficient free space
3. Try slower SPI clock if card is older/slower
4. Remove and reinsert card to reset state

### EEPROM not found

1. Check I2C1 pull-ups present (PB6, PB7)
2. Verify EEPROM address pins (A0-A2) match expected address
3. Try slower I2C clock (100 kHz instead of 400 kHz)
4. Check for bus contention with other I2C devices

### Deck not detected

1. Ensure deck is fully seated in expansion headers
2. Check PC11 (1-Wire) connection
3. Try cleaning expansion header contacts
4. Verify deck has proper EEPROM (some third-party decks may not)

### LEDs don't blink

1. Check GPIO clock enabled for GPIOC and GPIOD
2. Verify LED polarity (active high on Crazyflie)
3. Check for GPIO pin conflicts with other peripherals

## Test Summary

At the end, a summary is printed (in order of test execution). Note that
due to the SWO/SD pin conflict, the summary may not be visible if the SD
test runs (SWO output stops when SPI3 is initialized).

**Verified working (as of 2026-01-29)**:

| Phase | Test | Status |
|-------|------|--------|
| 1 | Boot & Clock (168 MHz) | PASS |
| 2 | LEDs (Blue + 4x Red) | PASS |
| 3 | I2C Bus Scan (BMI088, BMP388) | PASS |
| 4 | Sensors (all IDs and data) | PASS |
| 5 | EEPROM (I2C1 write/verify) | PASS |
| 6 | Deck Detection (Flow deck v2) | PASS |
| 7 | Motors (TIM2/TIM4 PWM, all 4) | PASS |
| 8 | Radio (USART6 syslink, 3.67V) | PASS |
| 9 | Flash Storage (273 KB/sec) | PASS |
| 10 | SD Card (SPI3) | BLIND (SWO conflict) |

**LED Status Indicator**:
- **Solid blue LED** = Phases 1-4 all passed
- **Fast blinking blue LED** = One or more of phases 1-4 failed

**Note**: The pass/fail LED only considers phases 1-4 (Boot, LEDs, I2C, Sensors).
Later phases (EEPROM, Deck, Motors, Radio, Flash, SD) are informational and
don't affect the final LED status.

## Tests To Be Written

The following tests are planned but not yet implemented:

| Test | Priority | Description |
|------|----------|-------------|
| USB | Medium | USB OTG FS (PA11/PA12) enumeration and data transfer |
| Debug UART | Low | USART1 (PA9/PA10) for debug console |
| Watchdog | Low | Independent watchdog (IWDG) timeout and reset |
| nRF51 Reset | Low | nRF51 reset control via GPIO |
| nRF51 Extended | Medium | Extended syslink commands (firmware version, reset) |

## Files

| File | Description |
|------|-------------|
| `bringup_main.c` | Main test sequence and coordination |
| `bringup_swo.c/h` | SWO debug output via ITM |
| `bringup_i2c1.c/h` | I2C1 bus communication (expansion connector: EEPROM, VL53L1x) |
| `bringup_i2c3.c/h` | I2C3 bus scan and communication (on-board sensors: BMI088, BMP388) |
| `bringup_sensors.c/h` | Sensor chip ID and data readout |
| `bringup_motors.c/h` | Motor PWM test (TIM2/TIM4) with rotation verification |
| `bringup_radio.c/h` | Syslink radio test (USART6) |
| `bringup_flash.c/h` | Internal flash storage test (sector 8) |
| `bringup_sd.c/h` | SD card test via SPI3 (Micro SD Card Deck) |
| `bringup_eeprom.c/h` | Configuration EEPROM test (I2C1) |
| `bringup_deck.c/h` | Expansion deck detection (1-Wire) |
| `bringup_leds.c/h` | LED test (blue status, red motor LEDs) |
| `Makefile` | Build system |
