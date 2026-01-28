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

Scans I2C3 bus for all connected devices:

| Address | Device | Required |
|---------|--------|----------|
| 0x18 | BMI088 Accelerometer | Yes |
| 0x69 | BMI088 Gyroscope (SDO=VDD) | Yes |
| 0x77 | BMP388 Barometer | Yes |
| 0x29 | VL53L1x ToF (Flow deck) | No |

**Expected output**
```
=== Phase 3: I2C Bus Scan ===
[I2C] Scanning I2C3 bus...
[I2C] Found device at 0x18 (BMI088 Accel)
[I2C] Found device at 0x29 (VL53L1x ToF) [Flow deck detected]
[I2C] Found device at 0x69 (BMI088 Gyro)
[I2C] Found device at 0x77 (BMP388 Baro)
[I2C] Scan complete: 4 devices found... OK
```

### Phase 4: Sensors

Reads chip IDs and sensor data via I2C and SPI:

**Chip IDs:**

| Sensor | Register | Expected Value |
|--------|----------|----------------|
| BMI088 Accel | 0x00 | 0x1E |
| BMI088 Gyro | 0x00 | 0x0F |
| BMP388 | 0x00 | 0x50 |
| VL53L1x | Model ID | 0xEACC |
| PMW3901 | Product ID | 0x49 |

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

Detects expansion decks via the 1-Wire bit-banged interface. Each deck
contains a DS28E05 EEPROM with vendor/product identification.

**Hardware**: 1-Wire interface on PC11

| Test | Description |
|------|-------------|
| OW Init | Initializes 1-Wire GPIO (open-drain with pull-up) |
| Presence | Sends reset pulse, checks for presence response |
| ROM Read | Reads 8-byte ROM ID from deck EEPROM |
| Memory Read | Reads VID/PID from deck memory |

**Known Bitcraze Decks (VID=0xBC)**

| PID | Deck Name |
|-----|-----------|
| 0x01 | LED ring deck |
| 0x06 | Loco positioning deck |
| 0x08 | Micro SD card deck |
| 0x09 | Flow deck |
| 0x0A | Flow deck v2 |
| 0x0C | Z-ranger deck |
| 0x0F | Z-ranger deck v2 |
| 0x10 | Lighthouse deck |
| 0x12 | AI deck |

**Expected output (deck present)**
```
=== Phase 6: Deck Detection ===
[DECK] 1-Wire interface (PC11)
[DECK] Initializing 1-Wire... OK
[DECK] Checking for deck presence... Deck detected!
[DECK] Reading deck ROM ID... OK
[DECK]   ROM ID: 42-00-00-0A-BC-83-29-6E
[DECK] Reading deck identity... OK
[DECK]   VID: 0xBC (Bitcraze)
[DECK]   PID: 0x0A
[DECK]   Name: Flow deck v2
[DECK] Detected 1 deck(s)
```

**Expected output (no deck)**
```
=== Phase 6: Deck Detection ===
[DECK] 1-Wire interface (PC11)
[DECK] Initializing 1-Wire... OK
[DECK] Checking for deck presence... No deck detected
[DECK] (This is normal if no expansion deck is attached)
```

### Phase 7: Motors

**WARNING: REMOVE PROPELLERS BEFORE THIS TEST!**

Tests timer-based PWM output. Spins each motor individually at low power
(10%) for 1 second.

| Motor | Position | Rotation | Pin |
|-------|----------|----------|-----|
| M1 | Front-left | CCW | PA0 |
| M2 | Front-right | CW | PA1 |
| M3 | Rear-right | CCW | PA2 |
| M4 | Rear-left | CW | PA3 |

The test requires user confirmation before starting motors.

**Expected output**
```
=== Phase 7: Motor Test ===
[MOTOR] !!! REMOVE PROPELLERS !!!
[MOTOR] Press ENTER to continue or 's' to skip...
[MOTOR] Spinning M1 (front-left, CCW) at 10%... OK
[MOTOR] Spinning M2 (front-right, CW) at 10%... OK
[MOTOR] Spinning M3 (rear-right, CCW) at 10%... OK
[MOTOR] Spinning M4 (rear-left, CW) at 10%... OK
[MOTOR] All motors tested... OK
```

### Phase 8: Radio (Syslink)

Tests USART serial communication with the nRF51822 via syslink protocol.
Waits for battery voltage packet from nRF51.

**Expected output**
```
=== Phase 8: Radio Test ===
[RADIO] Initializing syslink (USART6)... OK
[RADIO] Waiting for battery packet...
[RADIO] Battery voltage: 3.92V... OK
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

1. Check I2C3 pull-ups present on board
2. Verify PA8 (SCL) and PC9 (SDA) not shorted
3. Try slower I2C clock (reduce from 400kHz to 100kHz)
4. Check for bus contention (other masters)

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

At the end, a summary is printed (in order of test execution):

```
=== Bring-Up Test Summary ===
1. Boot & Clock:    OK
2. LEDs:            OK
3. I2C Bus Scan:    OK
4. Sensors:         OK
5. EEPROM:          OK
6. Deck Detection:  OK
7. Motors:          OK
8. Radio:           OK
9. Flash Storage:   OK
10. SD Card:        OK

Flow deck:          DETECTED
SD card deck:       DETECTED
Expansion decks:    1 detected

All required tests passed!
```

**Note**: SD Card shows "NO DECK" if the Micro SD Card Deck is not attached.
This is not a failure - the SD card deck is optional. Similarly, Deck Detection
will report "No deck detected" if no expansion deck is attached.

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
| `bringup_i2c.c/h` | I2C3 bus scan and communication |
| `bringup_sensors.c/h` | Sensor chip ID and data readout |
| `bringup_motors.c/h` | Motor PWM test (TIM2) |
| `bringup_radio.c/h` | Syslink radio test (USART6) |
| `bringup_flash.c/h` | Internal flash storage test (sector 8) |
| `bringup_sd.c/h` | SD card test via SPI3 (Micro SD Card Deck) |
| `bringup_eeprom.c/h` | Configuration EEPROM test (I2C1) |
| `bringup_deck.c/h` | Expansion deck detection (1-Wire) |
| `bringup_leds.c/h` | LED test (blue status, red motor LEDs) |
| `Makefile` | Build system |
