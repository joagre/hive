# Syslink Radio Support for Crazyflie

This document describes how to implement radio communication for the Hive runtime
on Crazyflie 2.1, enabling real-time telemetry logging over the radio link.

**Use case:** High-frequency logging (100Hz+) during flight for PID tuning and
control analysis as described in [STABILIZATION.md](STABILIZATION.md).

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
                                    │ Crazyradio PA (USB)  │
                                    │ - nRF24LU1+ chip     │
                                    │ - ESB compatible     │
                                    └──────────┬───────────┘
                                               │ USB
                                               │
                                    ┌──────────▼───────────┐
                                    │ PC (Python/cflib)    │
                                    └──────────────────────┘
```

**Key insight:** The nRF51 firmware stays intact when you flash custom STM32
firmware. You only need to implement the syslink protocol on the STM32 side.

---

## No Custom Firmware Required

A common question: Do you need custom firmware on the Crazyradio PA dongle?

**No.** The stock firmware on both the nRF51 and Crazyradio PA handles everything
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
| Crazyradio PA | Bitcraze stock | Ships ready to use |
| ESB radio protocol | Built into both | Already compatible |

### What you customize:

| Component | What to do |
|-----------|------------|
| STM32 firmware | Your Hive application using syslink |
| Python ground station | Use cflib to receive packets |

### How cflib handles the complexity

The Crazyradio PA is essentially a USB-to-ESB bridge. On the PC side, cflib
abstracts all USB and radio details:

```python
import cflib.crtp
from cflib.crtp.radiodriver import RadioDriver

# cflib talks to stock Crazyradio PA firmware
cflib.crtp.init_drivers()
link = cflib.crtp.get_link_driver("radio://0/80/2M")

# Receive your custom packets - just raw bytes
while True:
    packet = link.receive_packet(timeout=1)
    if packet:
        # Your telemetry data, as sent via syslink
        process_telemetry(packet.data)
```

The Crazyradio PA firmware:
1. Receives ESB packets from the air (sent by nRF51)
2. Passes them to the PC via USB
3. cflib reads them and gives you the raw bytes

You send whatever data you want via: **syslink → nRF51 → ESB → Crazyradio PA → USB → cflib**.
The radio layer doesn't care about packet contents—it just moves bytes.

---

## Syslink Protocol

Syslink is the packet-based protocol between STM32 and nRF51 over UART.

### Physical Layer

- **Baud rate:** 1,000,000 (1 Mbaud)
- **Format:** 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow control:** Hardware flow via TXEN pin (nRF51 → STM32)

### UART Pins (Crazyflie 2.1)

| Function | STM32 Pin | Notes |
|----------|-----------|-------|
| TX (STM32 → nRF51) | PA2 (USART2_TX) | |
| RX (nRF51 → STM32) | PA3 (USART2_RX) | |
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
| 0x01 | RADIO_CHANNEL | STM32→nRF | Set radio channel (0-125) |
| 0x02 | RADIO_DATARATE | STM32→nRF | Set data rate (0=250K, 1=1M, 2=2M) |
| 0x03 | RADIO_CONTWAVE | STM32→nRF | Continuous wave test mode |
| 0x04 | RADIO_RSSI | nRF→STM32 | Received signal strength |
| 0x10 | RADIO_ADDRESS | STM32→nRF | Set radio address (5 bytes) |
| 0x11 | PM_SWITCHOFF | STM32→nRF | Power down system |
| 0x13 | PM_BATTERY | nRF→STM32 | Battery voltage and state |

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
hive_status hive_radio_init(void);

// Send raw data over radio (max 31 bytes)
// Returns HIVE_ERR_WOULDBLOCK if flow control disallows TX
hive_status hive_radio_send(const void *data, size_t len);

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

static hive_status syslink_send(uint8_t type, const void *data, size_t len) {
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

hive_status hive_radio_init(void) {
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

hive_status hive_radio_send(const void *data, size_t len) {
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

static void telemetry_actor(void *args, const hive_spawn_info *siblings,
                            size_t sibling_count) {
    (void)args; (void)siblings; (void)sibling_count;

    // Subscribe to sensor and setpoint buses
    hive_bus_subscribe(sensor_bus);
    hive_bus_subscribe(setpoint_bus);

    // Create 10ms timer (100Hz telemetry)
    timer_id telem_timer;
    hive_timer_every(10000, &telem_timer);  // 10ms = 100Hz

    while (1) {
        hive_message msg;
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

# Run receiver (Crazyradio PA must be plugged in)
python3 tools/telemetry_receiver.py

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
│   ├── crazyflie-2.1+/
│   │   ├── hal_crazyflie.c        # HAL implementation
│   │   ├── platform_crazyflie.h   # Platform-specific functions
│   │   └── Makefile
│   ├── STEVAL-DRONE01/
│   │   ├── hal_stm32.c
│   │   └── ...
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
int hal_radio_init(void);

// Send raw data over radio (max 31 bytes).
// Returns 0 on success, -1 if flow control disallows TX.
int hal_radio_send(const void *data, size_t len);

// Check if radio TX is permitted (flow control).
bool hal_radio_tx_ready(void);

// Poll for incoming radio data (call from main loop).
void hal_radio_poll(void);

// Register callback for incoming radio packets.
typedef void (*hal_radio_rx_callback)(const void *data, size_t len);
void hal_radio_set_rx_callback(hal_radio_rx_callback cb);

// Get battery voltage (received via syslink PM_BATTERY).
float hal_radio_get_battery(void);

#endif // HAL_HAS_RADIO
```

### Platform Implementation

The Crazyflie HAL implements these functions using syslink:

```
examples/pilot/hal/crazyflie-2.1+/
├── hal_crazyflie.c          # Existing: sensors, motors
├── hal_radio.c              # NEW: syslink implementation
├── platform_crazyflie.h     # Add syslink declarations
└── Makefile                 # Add hal_radio.c, define HAL_HAS_RADIO
```

**hal_radio.c** contains the syslink state machine, UART driver, and flow
control logic (see "Minimal Implementation for Hive" section above).

### Stubs for Other Platforms

Platforms without radio provide empty stubs or compile-time exclusion:

```c
// hal/STEVAL-DRONE01/hal_stm32.c (no radio)

#ifdef HAL_HAS_RADIO
// Not defined for this platform - radio functions excluded
#endif
```

Or with stubs if HAL_HAS_RADIO is defined globally:

```c
// hal/webots-crazyflie/hal_webots.c (simulation stub)

int hal_radio_init(void) { return 0; }
int hal_radio_send(const void *data, size_t len) { (void)data; (void)len; return -1; }
bool hal_radio_tx_ready(void) { return false; }
void hal_radio_poll(void) { }
void hal_radio_set_rx_callback(hal_radio_rx_callback cb) { (void)cb; }
float hal_radio_get_battery(void) { return 0.0f; }
```

### Makefile Changes

```makefile
# hal/crazyflie-2.1+/Makefile

# Enable radio support
CFLAGS += -DHAL_HAS_RADIO

# Add radio source
SRCS += hal_radio.c
```

### Usage in Actors

Actors use HAL functions directly, staying hardware-independent:

```c
// telemetry_actor.c

#include "hal/hal.h"

#define PACKET_TYPE_TELEMETRY  0x01

static void telemetry_actor(void *args, ...) {
    #ifdef HAL_HAS_RADIO
    // Send telemetry if radio available and flow control permits
    if (hal_radio_tx_ready()) {
        telemetry_packet_t pkt = {
            .type = PACKET_TYPE_TELEMETRY,
            // ... fill sensor data
        };
        hal_radio_send(&pkt, sizeof(pkt));
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
| **Radio** (new) | `hal_radio_init`, `hal_radio_send`, `hal_radio_tx_ready`, `hal_radio_poll`, `hal_radio_set_rx_callback`, `hal_radio_get_battery` | telemetry_actor, hive_log |

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

## Radio as Third Log Target (Optional)

> **Note:** This section describes optional text logging over radio, which is
> **separate from telemetry**. For most use cases, telemetry_actor alone is
> sufficient. Radio logging is useful for real-time warnings/errors during flight.

The Hive logging system currently supports two output targets:
- **stdout** - Console output (Linux default, disabled on STM32)
- **file** - Binary log file (`/var/tmp/hive.log` on Linux, `/log` on STM32)

Radio can be added as a **third target**, enabling real-time log streaming over
the Crazyflie radio link. This is for **text messages only** (HIVE_LOG_WARN,
HIVE_LOG_ERROR), not telemetry data.

### Configuration

New compile-time options in `hive_static_config.h`:

```c
// Radio logging (Crazyflie only)
#ifndef HIVE_LOG_TO_RADIO
#define HIVE_LOG_TO_RADIO 0      // Disabled by default
#endif

#ifndef HIVE_LOG_RADIO_LEVEL
#define HIVE_LOG_RADIO_LEVEL HIVE_LOG_LEVEL_WARN  // Only WARN+ over radio
#endif
```

### Why a Separate Level?

Radio has limited bandwidth (~100 packets/sec, 31 bytes each). Streaming all
logs would saturate the link. The `HIVE_LOG_RADIO_LEVEL` setting allows:

- Full logging to file (TRACE level for post-flight analysis)
- Filtered logging to radio (WARN+ for real-time monitoring)

### Integration with Existing Macros

The existing `HIVE_LOG_*` macros would gain radio support:

```c
// In hive_log.c - simplified sketch

void hive_log_write(int level, const char *fmt, ...) {
    char buf[HIVE_LOG_MAX_ENTRY_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Target 1: stdout (if enabled and level meets threshold)
    #if HIVE_LOG_TO_STDOUT
    if (level >= HIVE_LOG_LEVEL) {
        fprintf(stderr, "[%s] %s\n", level_str(level), buf);
    }
    #endif

    // Target 2: file (if enabled and open)
    #if HIVE_LOG_TO_FILE
    if (level >= HIVE_LOG_LEVEL && log_file_open) {
        write_binary_log_entry(level, buf, len);
    }
    #endif

    // Target 3: radio (if enabled and level meets radio threshold)
    #if HIVE_LOG_TO_RADIO
    if (level >= HIVE_LOG_RADIO_LEVEL && hive_radio_tx_ready()) {
        write_radio_log_entry(level, buf, len);
    }
    #endif
}
```

### Radio Log Packet Format

Compact binary format optimized for 31-byte radio packets:

```
┌──────┬───────┬───────────┬────────┬─────────────────────────┐
│ TYPE │ LEVEL │ TIMESTAMP │ LENGTH │       MESSAGE           │
│ 0x02 │ 1byte │  4 bytes  │ 1 byte │     0-24 bytes          │
└──────┴───────┴───────────┴────────┴─────────────────────────┘
```

| Field | Size | Description |
|-------|------|-------------|
| TYPE | 1 byte | Packet type: `0x02` = log entry |
| LEVEL | 1 byte | Log level (0=TRACE to 4=ERROR) |
| TIMESTAMP | 4 bytes | Microseconds since boot (wraps at ~71 min) |
| LENGTH | 1 byte | Message length (0-24) |
| MESSAGE | 0-24 bytes | Truncated log message |

**Total:** 7 bytes header + 24 bytes message = 31 bytes max

The TYPE byte allows the ground station to distinguish log packets (`0x02`)
from telemetry packets (`0x01`).

### Implementation Sketch

```c
// New function in hive_log.c (Crazyflie build only)

#if HIVE_LOG_TO_RADIO

#define PACKET_TYPE_LOG  0x02  // Distinguishes from telemetry (0x01)

typedef struct __attribute__((packed)) {
    uint8_t  type;            // PACKET_TYPE_LOG
    uint8_t  level;
    uint32_t timestamp_us;
    uint8_t  length;
    char     message[24];
} radio_log_packet_t;

static void write_radio_log_entry(int level, const char *msg, int len) {
    radio_log_packet_t pkt = {
        .type = PACKET_TYPE_LOG,
        .level = (uint8_t)level,
        .timestamp_us = hive_get_time(),
        .length = (len > 24) ? 24 : len,
    };
    memcpy(pkt.message, msg, pkt.length);

    // Non-blocking send - drop if flow control disallows
    hive_radio_send(&pkt, 7 + pkt.length);
}

#endif
```

### Ground Station Receiver

The Python receiver handles both packet types:

```python
import struct

PACKET_TYPE_TELEMETRY = 0x01
PACKET_TYPE_LOG = 0x02
LOG_LEVELS = ['TRACE', 'DEBUG', 'INFO', 'WARN', 'ERROR']

def decode_packet(data):
    if len(data) < 1:
        return None

    pkt_type = data[0]

    if pkt_type == PACKET_TYPE_TELEMETRY:
        # Telemetry: type(1) + seq(4) + timestamp(4) + 10x int16(20) = 29 bytes
        if len(data) < 29:
            return None
        values = struct.unpack('<BIIhhhhhhhhhh', data[:29])
        return {
            'type': 'telemetry',
            'seq': values[1],
            'timestamp_us': values[2],
            'gyro': values[3:6],
            'roll': (values[6], values[7]),    # (setpoint, measured)
            'pitch': (values[8], values[9]),
            'yaw': (values[10], values[11]),
            'thrust': values[12]
        }

    elif pkt_type == PACKET_TYPE_LOG:
        # Log: type(1) + level(1) + timestamp(4) + length(1) + message
        if len(data) < 7:
            return None
        _, level, timestamp, length = struct.unpack('<BBIB', data[:7])
        message = data[7:7+length].decode('utf-8', errors='replace')
        return {
            'type': 'log',
            'level': LOG_LEVELS[level] if level < 5 else f'L{level}',
            'timestamp_us': timestamp,
            'message': message
        }

    return None  # Unknown packet type

# In receive loop:
packet = link.receive_packet(timeout=1)
if packet:
    decoded = decode_packet(packet.data)
    if decoded:
        if decoded['type'] == 'telemetry':
            # Log to CSV, update plots, etc.
            print(f"TELEM seq={decoded['seq']}")
        elif decoded['type'] == 'log':
            # Display log message
            t_ms = decoded['timestamp_us'] / 1000.0
            print(f"[{t_ms:10.1f}ms] {decoded['level']:5s} {decoded['message']}")
```

### Platform Summary

| Target | Linux | STM32 | Crazyflie |
|--------|-------|-------|-----------|
| stdout | ✓ (default on) | ✗ | ✗ |
| file | ✓ | ✓ (flash) | ✓ (flash) |
| radio | ✗ | ✗ | ✓ (optional) |

### Usage Example

```c
// In your Crazyflie Hive application

hive_status pilot_init(void) {
    // Initialize radio for logging
    hive_radio_init();

    // Now all HIVE_LOG_WARN/ERROR calls also go to radio
    HIVE_LOG_INFO("Pilot starting");   // File only (below radio threshold)
    HIVE_LOG_WARN("Low battery");      // File + radio
    HIVE_LOG_ERROR("Sensor failure"); // File + radio

    return HIVE_SUCCESS;
}
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

**Bottom line:** ESB + Crazyradio PA (~$30) lets you keep stock nRF51 firmware
and only write STM32 code. Much simpler.

---

## Limitations and Future Work

### Current Limitations

1. **One-way telemetry only** - Commands from ground not implemented
2. **No acknowledgment** - Lost packets are not retransmitted
3. **31-byte payload limit** - Multiple packets needed for larger data
4. **~100Hz practical limit** - Flow control restricts throughput

### Possible Improvements

1. **Command channel** - Receive PID tuning commands over radio
2. **Packet sequencing** - Detect dropped packets on ground
3. **Compression** - Delta encoding for sensor data
4. **Multiple log streams** - Different rates for different data

---

## References

- [Syslink Protocol Specification](https://www.bitcraze.io/documentation/repository/crazyflie2-nrf-firmware/master/protocols/syslink/)
- [Crazyflie Hardware Architecture](https://www.bitcraze.io/documentation/system/platform/cf2-architecture/)
- [nRF51 Firmware Source](https://github.com/bitcraze/crazyflie2-nrf-firmware)
- [Bitcraze UART Syslink Driver](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/uart_syslink.c)
- [cflib Python Library](https://github.com/bitcraze/crazyflie-lib-python)
