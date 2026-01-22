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

1. Connect ST-Link V3 to PC via USB
2. Flash the firmware
3. View output: `st-trace -c 168`

The `-c 168` parameter specifies the CPU clock frequency in MHz (168 MHz for
the STM32F405). The SWO baud rate is automatically derived from this.

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
reporting results via SWO. Each phase can pass (OK), fail (FAIL),
or be skipped (SKIP).

### Phase 1: Boot & Clock

Verifies basic system initialization:
- LED blink proves Reset_Handler executed
- SystemCoreClock value confirms PLL setup (expect 168 MHz)
- SysTick operational

**Expected output:**
```
=== Crazyflie 2.1+ Bring-Up Test ===
[BOOT] LED blink test... OK
[BOOT] SystemCoreClock = 168000000 Hz... OK
```

### Phase 2: I2C Bus Scan

Scans I2C3 bus for all connected devices:

| Address | Device | Required |
|---------|--------|----------|
| 0x18 | BMI088 Accelerometer | Yes |
| 0x68 | BMI088 Gyroscope | Yes |
| 0x77 | BMP388 Barometer | Yes |
| 0x29 | VL53L1x ToF (Flow deck) | No |

**Expected output:**
```
[I2C] Scanning I2C3 bus...
[I2C] Found device at 0x18 (BMI088 Accel)
[I2C] Found device at 0x29 (VL53L1x ToF) [Flow deck detected]
[I2C] Found device at 0x68 (BMI088 Gyro)
[I2C] Found device at 0x77 (BMP388 Baro)
[I2C] Scan complete: 4 devices found... OK
```

### Phase 3: Sensor Chip IDs

Reads and verifies chip identification registers:

| Sensor | Register | Expected Value |
|--------|----------|----------------|
| BMI088 Accel | 0x00 | 0x1E |
| BMI088 Gyro | 0x00 | 0x0F |
| BMP388 | 0x00 | 0x50 |
| VL53L1x | Model ID | 0xEACC |
| PMW3901 | Product ID | 0x49 |

**Expected output:**
```
[SENSOR] BMI088 Accel chip ID = 0x1E... OK
[SENSOR] BMI088 Gyro chip ID = 0x0F... OK
[SENSOR] BMP388 chip ID = 0x50... OK
[SENSOR] VL53L1x model ID = 0xEACC... OK (Flow deck)
[SENSOR] PMW3901 product ID = 0x49... OK (Flow deck)
```

### Phase 4: Sensor Data Readout

Reads actual sensor data and checks for sane values:

| Sensor | Check | Expected (at rest, level) |
|--------|-------|---------------------------|
| Accelerometer | X, Y near 0; Z near +1g | +/- 0.1g for X,Y; 0.9-1.1g for Z |
| Gyroscope | All axes near 0 | +/- 5 deg/s |
| Barometer | Pressure | 95-106 kPa (varies with altitude) |
| Barometer | Temperature | 15-35 C (room temp) |
| ToF | Range | 0-2000 mm (depends on surface) |

**Expected output:**
```
[DATA] Accelerometer: X=-0.02g Y=0.01g Z=1.01g... OK
[DATA] Gyroscope: X=0.5 Y=-0.3 Z=0.1 deg/s... OK
[DATA] Barometer: 101325 Pa, 23.5 C... OK
[DATA] ToF range: 152 mm... OK (Flow deck)
```

### Phase 5: Motor Test

**WARNING: REMOVE PROPELLERS BEFORE THIS TEST!**

Spins each motor individually at low power (10%) for 1 second:

| Motor | Position | Rotation | Pin |
|-------|----------|----------|-----|
| M1 | Front-left | CCW | PA0 |
| M2 | Front-right | CW | PA1 |
| M3 | Rear-right | CCW | PA2 |
| M4 | Rear-left | CW | PA3 |

The test requires user confirmation before starting motors.

**Expected output:**
```
[MOTOR] !!! REMOVE PROPELLERS !!!
[MOTOR] Press ENTER to continue or 's' to skip...
[MOTOR] Spinning M1 (front-left, CCW) at 10%... OK
[MOTOR] Spinning M2 (front-right, CW) at 10%... OK
[MOTOR] Spinning M3 (rear-right, CCW) at 10%... OK
[MOTOR] Spinning M4 (rear-left, CW) at 10%... OK
[MOTOR] All motors tested... OK
```

### Phase 6: Radio (Syslink)

Tests communication with the nRF51822 via USART6 syslink protocol.
Waits for battery voltage packet from nRF51.

**Expected output:**
```
[RADIO] Initializing syslink (USART6)... OK
[RADIO] Waiting for battery packet...
[RADIO] Battery voltage: 3.92V... OK
```

## Test Summary

At the end, a summary is printed:

```
=== Bring-Up Test Summary ===
Boot & Clock:     OK
I2C Bus Scan:     OK
Sensor Chip IDs:  OK
Sensor Data:      OK
Motors:           OK
Radio:            OK

Flow deck:        DETECTED

All tests passed!
```

## Troubleshooting

### No output on SWO

1. Ensure `st-trace -c 168` is running
2. Verify SWO pin connected (pin 6 on debug header, pin 13 on 20-pin)
3. Check CPU frequency matches `-c` parameter (168 for STM32F405)
4. Try resetting the target: `st-flash reset`

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

## Files

| File | Description |
|------|-------------|
| `bringup_main.c` | Main test sequence and coordination |
| `bringup_swo.c/h` | SWO debug output via ITM |
| `bringup_i2c.c/h` | I2C3 bus scan and communication |
| `bringup_sensors.c/h` | Sensor chip ID and data readout |
| `bringup_motors.c/h` | Motor PWM test (TIM2) |
| `bringup_radio.c/h` | Syslink radio test (USART6) |
| `Makefile` | Build system |
| `stm32f405_bringup.ld` | Linker script |
