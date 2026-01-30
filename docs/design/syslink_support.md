# Syslink Radio Support for Crazyflie

This document describes how to implement radio communication for the Hive runtime
on Crazyflie 2.1, enabling real-time telemetry logging over the radio link.

**Use case** - High-frequency logging (100Hz+) during flight for PID tuning and
control analysis as described in [stabilization.md](stabilization.md).

---

## Architecture Overview

The Crazyflie has two MCUs that handle radio communication:

```
┌─────────────────────────────────────────────────────────────────┐
│ Crazyflie 2.1                                                   │
│                                                                 │
│  ┌──────────────────────┐         ┌──────────────────────┐     │
│  │ STM32F405 (Main MCU) │  UART   │ nRF51822 (Radio MCU) │     │
│  │                      │ 1Mbaud  │                      │     │
│  │ - Your Hive firmware │◄───────►│ - Bitcraze firmware  │     │
│  │ - Flight control     │ Syslink │ - ESB radio protocol │     │
│  │ - 1MB flash, 192KB   │         │ - Power management   │     │
│  │   RAM                │         │ - 128KB flash        │     │
│  └──────────────────────┘         └──────────┬───────────┘     │
│                                              │                  │
└──────────────────────────────────────────────┼──────────────────┘
                                               │ 2.4GHz ESB
                                               │
                                    ┌──────────▼───────────┐
                                    │ Crazyradio 2.0 (USB)  │
                                    │ - nRF24LU1+ chip     │
                                    │ - ESB compatible     │
                                    └──────────┬───────────┘
                                               │ USB
                                               │
                                    ┌──────────▼───────────┐
                                    │ PC (Python/cflib)    │
                                    └──────────────────────┘
```

**Key insight** - The nRF51 firmware stays intact when you flash custom STM32
firmware. You only need to implement the syslink protocol on the STM32 side.

---

## No Custom Firmware Required

A common question: Do you need custom firmware on the Crazyradio 2.0 dongle?

**No.** The stock firmware on both the nRF51 and Crazyradio 2.0 handles everything
at the radio layer:

```
Your STM32 firmware          Stock firmware           Stock firmware
┌─────────────────┐         ┌───────────────┐        ┌──────────────┐
│  Hive + syslink │──UART──▶│ nRF51 on      │~~ESB~~▶│ Crazyradio   │──USB──▶ PC
│  (custom)       │         │ Crazyflie     │  radio │ PA dongle    │
└─────────────────┘         │ (stock)       │        │ (stock)      │
                            └───────────────┘        └──────────────┘
```

### What stays stock (no changes needed):

| Component | Firmware | Why it works |
|-----------|----------|--------------|
| nRF51 on Crazyflie | Bitcraze stock | Flashing STM32 doesn't touch it |
| Crazyradio 2.0 | Bitcraze stock | Ships ready to use |
| ESB radio protocol | Built into both | Already compatible |

### What you customize:

| Component | What to do |
|-----------|------------|
| STM32 firmware | Your Hive application using syslink |
| Python ground station | Use cflib to receive packets |

### How cflib handles the complexity

The Crazyradio 2.0 is essentially a USB-to-ESB bridge. On the PC side, cflib
abstracts all USB and radio details:

```python
import cflib.crtp
from cflib.crtp.radiodriver import RadioDriver

# cflib talks to stock Crazyradio 2.0 firmware
cflib.crtp.init_drivers()
link = cflib.crtp.get_link_driver("radio://0/80/2M")

# Receive your custom packets - just raw bytes
while True:
    packet = link.receive_packet(timeout=1)
    if packet:
        # Your telemetry data, as sent via syslink
        process_telemetry(packet.data)
```

The Crazyradio 2.0 firmware:
1. Receives ESB packets from the air (sent by nRF51)
2. Passes them to the PC via USB
3. cflib reads them and gives you the raw bytes

You send whatever data you want via: **syslink -> nRF51 -> ESB -> Crazyradio 2.0 -> USB -> cflib**.
The radio layer doesn't care about packet contents; it just moves bytes.

---

## Syslink Protocol

Syslink is the packet-based protocol between STM32 and nRF51 over UART.

### Physical Layer

- **Baud rate** - 1,000,000 (1 Mbaud)
- **Format** - 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow control** - Hardware flow via TXEN pin (nRF51 -> STM32)

### UART Pins (Crazyflie 2.1)

| Function | STM32 Pin | Notes |
|----------|-----------|-------|
| TX (STM32 -> nRF51) | PA2 (USART2_TX) | |
| RX (nRF51 -> STM32) | PA3 (USART2_RX) | |
| TXEN (flow control) | PC4 | High = nRF51 ready to receive |

### Packet Format

```
┌───────┬───────┬──────┬────────┬─────────────┬────────┬────────┐
│ START │ START │ TYPE │ LENGTH │    DATA     │ CKSUM  │ CKSUM  │
│ 0xBC  │ 0xCF  │      │        │ (0-64 bytes)│   A    │   B    │
└───────┴───────┴──────┴────────┴─────────────┴────────┴────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| START | 2 bytes | Magic bytes: 0xBC, 0xCF |
| TYPE | 1 byte | Packet type (see below) |
| LENGTH | 1 byte | Data length (0-64) |
| DATA | 0-64 bytes | Payload |
| CKSUM | 2 bytes | Fletcher-8 checksum over TYPE + LENGTH + DATA |

### Packet Types

| Type | Name | Direction | Description |
|------|------|-----------|-------------|
| 0x00 | RADIO_RAW | Both | Raw radio packet data |
| 0x01 | RADIO_CHANNEL | STM32->nRF | Set radio channel (0-125) |
| 0x02 | RADIO_DATARATE | STM32->nRF | Set data rate (0=250K, 1=1M, 2=2M) |
| 0x03 | RADIO_CONTWAVE | STM32->nRF | Continuous wave test mode |
| 0x04 | RADIO_RSSI | nRF->STM32 | Received signal strength |
| 0x10 | RADIO_ADDRESS | STM32->nRF | Set radio address (5 bytes) |
| 0x11 | PM_SWITCHOFF | STM32->nRF | Power down system |
| 0x13 | PM_BATTERY | nRF->STM32 | Battery voltage and state |

### Flow Control (Critical!)

The nRF51 has limited buffer space. Flow control rule:

> **STM32 may only send one RADIO_RAW packet after receiving one.**

The nRF51 periodically sends NULL packets (empty RADIO_RAW) to enable TX.
Your driver must track this and only transmit when permitted.

### Checksum Calculation (Fletcher-8)

```c
void syslink_checksum(uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b) {
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += data[i];
        *ck_b += *ck_a;
    }
}
```

---

## Minimal Implementation for Hive

### API Design

Two new HAL functions for radio telemetry:

```c
// hive_radio.h

// Initialize radio subsystem (call from hive_init on Crazyflie)
hive_status_t hive_radio_init(void);

// Send raw data over radio (max 31 bytes)
// Returns HIVE_ERR_WOULDBLOCK if flow control disallows TX
hive_status_t hive_radio_send(const void *data, size_t len);

// Check if radio TX is allowed (flow control)
bool hive_radio_tx_ready(void);

// Receive callback registration (for incoming commands)
typedef void (*hive_radio_rx_callback)(const void *data, size_t len);
void hive_radio_set_rx_callback(hive_radio_rx_callback cb);

// Get battery voltage (received via syslink PM_BATTERY)
float hive_radio_get_battery_voltage(void);
```

### Implementation Sketch

```c
// src/hive_radio_crazyflie.c

#include "hive_radio.h"
#include <stdbool.h>
#include <string.h>

// Syslink constants
#define SYSLINK_START1      0xBC
#define SYSLINK_START2      0xCF
#define SYSLINK_RADIO_RAW   0x00
#define SYSLINK_PM_BATTERY  0x13
#define SYSLINK_MTU         64

// Flow control state
static volatile bool tx_allowed = false;
static volatile float battery_voltage = 0.0f;
static hive_radio_rx_callback rx_callback = NULL;

// UART registers (STM32F405 USART2)
#define USART2_BASE     0x40004400
#define USART_SR        (*(volatile uint32_t *)(USART2_BASE + 0x00))
#define USART_DR        (*(volatile uint32_t *)(USART2_BASE + 0x04))
#define USART_BRR       (*(volatile uint32_t *)(USART2_BASE + 0x08))
#define USART_CR1       (*(volatile uint32_t *)(USART2_BASE + 0x0C))
#define USART_CR2       (*(volatile uint32_t *)(USART2_BASE + 0x10))
#define USART_CR3       (*(volatile uint32_t *)(USART2_BASE + 0x14))

#define USART_SR_TXE    (1 << 7)
#define USART_SR_RXNE   (1 << 5)
#define USART_CR1_UE    (1 << 13)
#define USART_CR1_TE    (1 << 3)
#define USART_CR1_RE    (1 << 2)
#define USART_CR1_RXNEIE (1 << 5)

// GPIO for TXEN (PC4) - flow control from nRF51
#define GPIOC_BASE      0x40020800
#define GPIOC_IDR       (*(volatile uint32_t *)(GPIOC_BASE + 0x10))
#define TXEN_PIN        4
#define TXEN_READY()    ((GPIOC_IDR & (1 << TXEN_PIN)) != 0)

// -----------------------------------------------------------------------------
// Low-level UART
// -----------------------------------------------------------------------------

static void uart_putc(uint8_t c) {
    while (!(USART_SR & USART_SR_TXE));
    USART_DR = c;
}

static void uart_write(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uart_putc(data[i]);
    }
}

// -----------------------------------------------------------------------------
// Syslink TX
// -----------------------------------------------------------------------------

static hive_status_t syslink_send(uint8_t type, const void *data, size_t len) {
    if (len > SYSLINK_MTU) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Packet too large");
    }

    // Check hardware flow control
    if (!TXEN_READY()) {
        return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "nRF51 not ready");
    }

    // Calculate checksum over type + length + data
    uint8_t ck_a = type + (uint8_t)len;
    uint8_t ck_b = ck_a;
    const uint8_t *bytes = (const uint8_t *)data;
    for (size_t i = 0; i < len; i++) {
        ck_a += bytes[i];
        ck_b += ck_a;
    }

    // Send packet
    uart_putc(SYSLINK_START1);
    uart_putc(SYSLINK_START2);
    uart_putc(type);
    uart_putc((uint8_t)len);
    uart_write(bytes, len);
    uart_putc(ck_a);
    uart_putc(ck_b);

    return HIVE_SUCCESS;
}

// -----------------------------------------------------------------------------
// Syslink RX (called from USART2 IRQ or polling loop)
// -----------------------------------------------------------------------------

typedef enum {
    RX_START1,
    RX_START2,
    RX_TYPE,
    RX_LENGTH,
    RX_DATA,
    RX_CKSUM_A,
    RX_CKSUM_B
} rx_state_t;

static rx_state_t rx_state = RX_START1;
static uint8_t rx_type;
static uint8_t rx_length;
static uint8_t rx_data[SYSLINK_MTU];
static uint8_t rx_index;
static uint8_t rx_ck_a, rx_ck_b;

static void syslink_process_packet(void) {
    switch (rx_type) {
    case SYSLINK_RADIO_RAW:
        // Flow control: receiving a packet allows us to send one
        tx_allowed = true;

        // Pass to application if callback registered
        if (rx_callback && rx_length > 0) {
            rx_callback(rx_data, rx_length);
        }
        break;

    case SYSLINK_PM_BATTERY:
        // Battery packet: flags (1 byte) + voltage (4 bytes float) + current (2 bytes)
        if (rx_length >= 5) {
            memcpy(&battery_voltage, &rx_data[1], sizeof(float));
        }
        break;

    default:
        // Ignore other packet types
        break;
    }
}

static void syslink_rx_byte(uint8_t byte) {
    switch (rx_state) {
    case RX_START1:
        if (byte == SYSLINK_START1) rx_state = RX_START2;
        break;

    case RX_START2:
        rx_state = (byte == SYSLINK_START2) ? RX_TYPE : RX_START1;
        break;

    case RX_TYPE:
        rx_type = byte;
        rx_ck_a = byte;
        rx_ck_b = byte;
        rx_state = RX_LENGTH;
        break;

    case RX_LENGTH:
        rx_length = byte;
        rx_ck_a += byte;
        rx_ck_b += rx_ck_a;
        rx_index = 0;
        rx_state = (rx_length > 0) ? RX_DATA : RX_CKSUM_A;
        break;

    case RX_DATA:
        rx_data[rx_index++] = byte;
        rx_ck_a += byte;
        rx_ck_b += rx_ck_a;
        if (rx_index >= rx_length) rx_state = RX_CKSUM_A;
        break;

    case RX_CKSUM_A:
        rx_state = (byte == rx_ck_a) ? RX_CKSUM_B : RX_START1;
        break;

    case RX_CKSUM_B:
        if (byte == rx_ck_b) {
            syslink_process_packet();
        }
        rx_state = RX_START1;
        break;
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

hive_status_t hive_radio_init(void) {
    // Enable USART2 and GPIOA/GPIOC clocks
    // Configure PA2 (TX) and PA3 (RX) for USART2 alternate function
    // Configure PC4 (TXEN) as input
    // Set up USART2: 1Mbaud, 8N1, enable TX/RX

    // TODO: Actual STM32 peripheral initialization
    // This is board-specific and depends on your HAL

    // For now, assume UART is already configured by startup code

    tx_allowed = false;
    rx_state = RX_START1;

    return HIVE_SUCCESS;
}

hive_status_t hive_radio_send(const void *data, size_t len) {
    if (len > 31) {
        return HIVE_ERROR(HIVE_ERR_INVALID, "Max 31 bytes");
    }

    if (!tx_allowed) {
        return HIVE_ERROR(HIVE_ERR_WOULDBLOCK, "Flow control");
    }

    tx_allowed = false;  // Consumed our TX slot

    return syslink_send(SYSLINK_RADIO_RAW, data, len);
}

bool hive_radio_tx_ready(void) {
    return tx_allowed;
}

void hive_radio_set_rx_callback(hive_radio_rx_callback cb) {
    rx_callback = cb;
}

float hive_radio_get_battery_voltage(void) {
    return battery_voltage;
}

// Call this from main loop or USART2 IRQ handler
void hive_radio_poll(void) {
    while (USART_SR & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)USART_DR;
        syslink_rx_byte(byte);
    }
}
```

### Usage in Telemetry Actor

```c
// telemetry_actor.c - Sends binary sensor data at 100Hz
//
// NOTE: This is TELEMETRY, not logging. Telemetry is:
//   - Binary packed structs (not text)
//   - High-frequency periodic data (100Hz)
//   - For PID tuning and analysis
//
// For text log messages (HIVE_LOG_*), see "Radio as Third Log Target" below.

#define PACKET_TYPE_TELEMETRY  0x01  // Distinguishes from log packets

typedef struct __attribute__((packed)) {
    uint8_t  type;            // PACKET_TYPE_TELEMETRY
    uint32_t seq;
    uint32_t timestamp_us;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t roll_sp, roll_meas;
    int16_t pitch_sp, pitch_meas;
    int16_t yaw_sp, yaw_meas;
    int16_t thrust;
} telemetry_packet_t;  // 27 bytes - fits in 31-byte limit

static uint32_t seq = 0;

static void telemetry_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    // Subscribe to sensor and setpoint buses
    hive_bus_subscribe(sensor_bus);
    hive_bus_subscribe(setpoint_bus);

    // Create 10ms timer (100Hz telemetry)
    timer_id_t telem_timer;
    hive_timer_every(10000, &telem_timer);  // 10ms = 100Hz

    while (1) {
        hive_message_t msg;
        hive_ipc_recv(&msg, -1);

        if (hive_msg_is_timer(&msg) && msg.tag == telem_timer) {
            // Only send if flow control allows
            if (hive_radio_tx_ready()) {
                telemetry_packet_t pkt = {
                    .type = PACKET_TYPE_TELEMETRY,
                    .seq = seq++,
                    .timestamp_us = hive_get_time(),
                    // Fill from latest sensor/setpoint data...
                };

                hive_radio_send(&pkt, sizeof(pkt));
            }
            // If not ready, skip this sample (don't block)
        }

        // Poll radio RX (or do this in a separate actor/ISR)
        hive_radio_poll();
    }
}
```

---

## Ground Station (Python)

### Minimal Receiver

```python
#!/usr/bin/env python3
"""
Minimal Crazyflie telemetry receiver for Hive firmware.
Receives raw packets and logs to CSV.
"""

import struct
import time
import csv
from cflib.crtp import init_drivers
from cflib.crtp.radiodriver import RadioDriver

# Must match telemetry_packet_t in firmware
PACKET_FORMAT = '<IIhhhhhhhhhh'  # seq, timestamp, 10x int16
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

def main():
    # Initialize radio
    init_drivers(enable_debug_driver=False)

    uri = 'radio://0/80/2M/E7E7E7E7E7'
    print(f"Connecting to {uri}...")

    link = RadioDriver()
    link.connect(uri, None, None)

    print("Connected! Receiving telemetry...")

    # Open CSV for logging
    with open('telemetry.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'seq', 'timestamp_us',
            'gyro_x', 'gyro_y', 'gyro_z',
            'roll_sp', 'roll_meas',
            'pitch_sp', 'pitch_meas',
            'yaw_sp', 'yaw_meas',
            'thrust'
        ])

        try:
            while True:
                pk = link.receive_packet(wait=True)
                if pk is None:
                    continue

                # Skip if wrong size
                if len(pk.data) < PACKET_SIZE:
                    continue

                # Unpack telemetry
                values = struct.unpack(PACKET_FORMAT, pk.data[:PACKET_SIZE])
                writer.writerow(values)

                # Print progress
                seq, ts = values[0], values[1]
                if seq % 100 == 0:
                    print(f"seq={seq}, t={ts/1000:.1f}ms")

        except KeyboardInterrupt:
            print("\nStopping...")
        finally:
            link.close()

if __name__ == '__main__':
    main()
```

### Running the Receiver

```bash
# Install cflib if needed
pip install cflib

# Run receiver (Crazyradio 2.0 must be plugged in)
python3 examples/pilot/tools/telemetry_receiver.py

# Output: telemetry.csv with all logged data
```

---

## Integration with Pilot HAL

The pilot example already has a Hardware Abstraction Layer (HAL) with two main
functions used by actors:

```c
// Existing HAL functions (examples/pilot/hal/hal.h)
void hal_read_sensors(sensor_data_t *sensors);  // Used by sensor_actor
void hal_write_torque(const torque_cmd_t *cmd); // Used by motor_actor
```

Syslink support adds radio functions to this same HAL interface.

### Existing HAL Structure

```
examples/pilot/
├── hal/
│   ├── hal.h                      # Common interface (all platforms)
│   ├── crazyflie-2.1plus/
│   │   ├── hal_crazyflie.c        # HAL implementation
│   │   ├── platform_crazyflie.h   # Platform-specific functions
│   │   └── Makefile
│   └── webots-crazyflie/
│       ├── hal_webots.c
│       └── ...
```

### Adding Syslink to HAL

New functions in `hal/hal.h`:

```c
// ----------------------------------------------------------------------------
// Radio Interface (Crazyflie only)
// ----------------------------------------------------------------------------

#ifdef HAL_HAS_RADIO

// Initialize syslink UART to nRF51.
// Called by hal_init() on platforms with radio.
int hal_esb_init(void);

// Send raw data over radio (max 31 bytes).
// Returns 0 on success, -1 if flow control disallows TX.
int hal_esb_send(const void *data, size_t len);

// Check if radio TX is permitted (flow control).
bool hal_esb_tx_ready(void);

// Poll for incoming radio data (call from main loop).
void hal_esb_poll(void);

// Register callback for incoming radio packets.
typedef void (*hal_esb_rx_callback)(const void *data, size_t len, void *user_data);
void hal_esb_set_rx_callback(hal_esb_rx_callback cb, void *user_data);

// Get battery voltage (received via syslink PM_BATTERY).
// Note: This is in the power interface, separate from ESB.
float hal_power_get_battery(void);

#endif // HAL_HAS_RADIO
```

### Platform Implementation

The Crazyflie HAL implements these functions using syslink:

```
examples/pilot/hal/crazyflie-2.1plus/
├── hal_init.c               # Existing: init, cleanup, self_test, calibrate, arm, disarm
├── hal_sensors.c            # Existing: hal_read_sensors
├── hal_motors.c             # Existing: hal_write_torque
├── hal_syslink.c            # Syslink implementation (ESB + power)
├── platform.h               # Platform declarations
└── Makefile                 # Add hal_syslink.c, define HAL_HAS_RADIO
```

**hal_syslink.c** contains the syslink state machine, UART driver, and flow
control logic (see "Minimal Implementation for Hive" section above).

### Stubs for Other Platforms

Platforms without radio provide empty stubs or compile-time exclusion:

```c
// hal/webots-crazyflie/ (simulation, no radio)

#ifdef HAL_HAS_RADIO
// Not defined for this platform - ESB functions excluded
#endif
```

Or with stubs if HAL_HAS_RADIO is defined globally:

```c
// Simulation stub (not used in webots-crazyflie)

int hal_esb_init(void) { return 0; }
int hal_esb_send(const void *data, size_t len) { return -1; }
bool hal_esb_tx_ready(void) { return false; }
void hal_esb_poll(void) { }
void hal_esb_set_rx_callback(hal_esb_rx_callback cb, void *user_data) { }
float hal_power_get_battery(void) { return 0.0f; }
```

### Makefile Changes

```makefile
# hal/crazyflie-2.1plus/Makefile

# Enable radio support
CFLAGS += -DHAL_HAS_RADIO

# Add syslink source
SRCS += hal_syslink.c
```

### Usage in Actors

Actors use HAL functions directly, staying hardware-independent:

```c
// comms_actor.c

#include "hal/hal.h"

#define PACKET_TYPE_TELEMETRY  0x01

static void comms_actor(void *args, ...) {
    #ifdef HAL_HAS_RADIO
    // Send telemetry if radio available and flow control permits
    if (hal_esb_tx_ready()) {
        telemetry_packet_t pkt = {
            .type = PACKET_TYPE_TELEMETRY,
            // ... fill sensor data
        };
        hal_esb_send(&pkt, sizeof(pkt));
    }
    #endif
}
```

### Complete HAL Function Summary

After adding syslink, the HAL has three categories:

| Category | Functions | Used By |
|----------|-----------|---------|
| **Lifecycle** | `hal_init`, `hal_cleanup`, `hal_calibrate`, `hal_arm`, `hal_disarm` | pilot.c |
| **Sensors/Motors** | `hal_read_sensors`, `hal_write_torque` | sensor_actor, motor_actor |
| **ESB Radio** | `hal_esb_init`, `hal_esb_send`, `hal_esb_tx_ready`, `hal_esb_poll`, `hal_esb_set_rx_callback` | comms_actor |
| **Power** | `hal_power_get_battery` | comms_actor |

---

## Telemetry vs Logging

Radio packets carry two distinct data types. Keep them separate:

| Aspect | Telemetry | Logging (HIVE_LOG_*) |
|--------|-----------|----------------------|
| **Format** | Binary packed structs | Text strings |
| **Rate** | Periodic, 100Hz | Sporadic, event-driven |
| **Purpose** | PID tuning, analysis | Debugging, diagnostics |
| **Size** | Fixed (27 bytes) | Variable (up to 25 chars) |
| **Source** | telemetry_actor | HIVE_LOG_WARN/ERROR macros |
| **Packet type** | `0x01` | `0x02` |

**Do NOT use HIVE_LOG_* for telemetry data.** HIVE_LOG formats text strings,
which is inefficient for high-frequency binary sensor data.

---

## Post-Flight Log Download

After landing, the complete flash log file can be downloaded over radio. This
keeps flight phase simple (telemetry only) while enabling full log retrieval.

### Design Principles

- **Flight phase** - Only telemetry packets (attitude, position) at 100Hz
- **Post-flight** - Ground station requests log, drone sends in chunks
- **Complete file** - All log entries, not filtered by level
- **Simple protocol** - Request/response, ESB handles flow control

### Protocol

Ground station initiates download by sending a command packet:

```
Ground Station                       Drone (telemetry_actor)
      │                                       │
      │──── CMD_REQUEST_LOG (0x10) ──────────>│
      │                                       │ Opens /log file
      │<──── LOG_CHUNK (0x11) ────────────────│ Sends chunk 0
      │<──── LOG_CHUNK (0x11) ────────────────│ Sends chunk 1
      │<──── LOG_CHUNK (0x11) ────────────────│ ...
      │<──── LOG_COMPLETE (0x12) ─────────────│ EOF reached
      │                                       │
```

### Packet Formats

**Command: Request Log (ground -> drone)**

```
┌──────┬─────────┐
│ TYPE │ RESERVED│
│ 0x10 │ 0x00    │
└──────┴─────────┘
```

**Response: Log Chunk (drone -> ground)**

```
┌──────┬────────┬────────┬─────────────────────────────┐
│ TYPE │ SEQ_LO │ SEQ_HI │          DATA               │
│ 0x11 │ 1 byte │ 1 byte │       0-28 bytes            │
└──────┴────────┴────────┴─────────────────────────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| TYPE | 1 byte | `0x11` = log chunk |
| SEQ | 2 bytes | Chunk sequence number (little-endian) |
| DATA | 0-28 bytes | Raw log file data |

**Response: Log Complete (drone -> ground)**

```
┌──────┬──────────┬──────────┐
│ TYPE │ TOTAL_LO │ TOTAL_HI │
│ 0x12 │  2 bytes │  2 bytes │
└──────┴──────────┴──────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| TYPE | 1 byte | `0x12` = log complete |
| TOTAL | 4 bytes | Total bytes sent (little-endian) |

### Implementation in telemetry_actor

The telemetry actor handles log download as a secondary mode:

```c
// State machine
typedef enum {
    TELEM_MODE_FLIGHT,      // Normal telemetry
    TELEM_MODE_LOG_DOWNLOAD // Sending log file
} telem_mode_t;

// RX callback handles CMD_REQUEST_LOG
static void on_radio_rx(const void *data, size_t len) {
    const uint8_t *pkt = data;
    if (len >= 1 && pkt[0] == CMD_REQUEST_LOG) {
        // Transition to log download mode
        start_log_download();
    }
}

// In main loop, check mode
if (mode == TELEM_MODE_LOG_DOWNLOAD) {
    send_next_log_chunk();
} else {
    send_telemetry();
}
```

### Ground Station Usage

```bash
# Download log after flight (from project root)
./examples/pilot/tools/telemetry_receiver.py --download-log flight.log

# Or from examples/pilot directory
./tools/telemetry_receiver.py --download-log flight.log
```

### Advantages Over Real-Time Log Streaming

| Aspect | Real-Time Streaming | Post-Flight Download |
|--------|---------------------|----------------------|
| Bandwidth during flight | Competes with telemetry | Zero impact |
| Log completeness | Filtered (WARN+ only) | Complete (all levels) |
| Complexity | Callback hooks in hive_log | Self-contained in telemetry_actor |
| Runtime coupling | hive_log knows about radio | No runtime changes |

### Log File Format

The downloaded file is the raw binary log from flash (`/log`). Use the existing
decoder tool:

```bash
# Decode downloaded log
python3 tools/decode_log.py flight.log
```

---

## Why Not BLE?

The nRF51822 supports BLE, but it's not recommended for this use case:

- **Requires custom nRF51 firmware** - Stock firmware uses ESB, not BLE. You'd
  need to reflash the radio MCU, adding complexity (two MCUs to program instead
  of one).
- **Lower throughput** - BLE connection intervals (7.5-30ms) limit telemetry to
  ~50Hz max. ESB easily handles 100Hz.
- **More protocol complexity** - BLE requires GATT services. ESB/syslink is just
  raw packets.

**Bottom line** - ESB + Crazyradio 2.0 (~$30) lets you keep stock nRF51 firmware
and only write STM32 code. Much simpler.

---

## Limitations and Future Work

### Current Limitations

1. **No acknowledgment** - Lost packets are not retransmitted
2. **31-byte payload limit** - Multiple packets needed for larger data
3. **~100Hz practical limit** - Flow control restricts throughput

### Implemented Features

1. **Bidirectional communication** - Ground station can send commands
2. **Post-flight log download** - Complete flash log retrieval over radio

### Possible Future Improvements

1. **PID tuning commands** - Adjust gains over radio during hover
2. **Compression** - Delta encoding for sensor data
3. **Retransmission** - ACK/NAK for reliable log download

---

## References

- [Syslink Protocol Specification](https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/protocols/syslink/)
- [Crazyflie Hardware Architecture](https://www.bitcraze.io/documentation/system/platform/cf2-architecture/)
- [nRF51 Firmware Source](https://github.com/bitcraze/crazyflie2-nrf-firmware)
- [Bitcraze UART Syslink Driver](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/uart_syslink.c)
- [cflib Python Library](https://github.com/bitcraze/crazyflie-lib-python)
