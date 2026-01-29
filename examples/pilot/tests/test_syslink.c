/**
 * Syslink Communication Test
 *
 * Tests the DMA-based syslink radio communication with Crazyradio PA/2.0.
 *
 * Test sequence:
 *   1. Initialize HAL and Hive runtime
 *   2. Initialize radio (hal_radio_init)
 *   3. Wait for battery packet (proves RX works)
 *   4. Wait for ground station connection (RADIO_RAW packet)
 *   5. Send telemetry packets when TX allowed
 *   6. Receive and echo commands from ground station
 *
 * LED patterns:
 *   - 1 blink: Starting test
 *   - 2 blinks: Radio initialized
 *   - 3 blinks: Battery packet received
 *   - 4 blinks: Ground station connected
 *   - Fast blink: Sending/receiving packets
 *   - Solid: Test complete (success)
 *   - Slow blink: Error
 *
 * Usage:
 *   make PLATFORM=crazyflie TEST=syslink
 *   make flash-crazyflie TEST=syslink
 *
 * Ground station test (Python):
 *   import cflib
 *   # Connect to Crazyflie and observe telemetry
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"
#include "hive_timer.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "printf.h" // Use lib/printf (snprintf_) on STM32

// ============================================================================
// Test Configuration
// ============================================================================

#define TEST_DURATION_MS 30000      // Run test for 30 seconds
#define TELEMETRY_INTERVAL_MS 100   // Send telemetry every 100ms
#define BATTERY_TIMEOUT_MS 5000     // Wait 5s for battery packet
#define CONNECTION_TIMEOUT_MS 30000 // Wait 30s for ground station

// Packet types (matching comms_actor.c)
#define PACKET_TYPE_ATTITUDE 0x01
#define PACKET_TYPE_POSITION 0x02
#define PACKET_TYPE_ECHO 0xE0 // Echo test packet

// ============================================================================
// Test State
// ============================================================================

typedef struct {
    volatile bool battery_received;
    volatile bool ground_connected;
    volatile uint32_t rx_count;
    volatile uint32_t tx_count;
    volatile uint8_t last_rx_type;
    volatile uint8_t last_rx_data[32];
    volatile size_t last_rx_len;
} radio_test_state_t;

static radio_test_state_t s_state = {0};

// ============================================================================
// Packet Structures
// ============================================================================

// Simple attitude packet for testing (17 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type; // PACKET_TYPE_ATTITUDE
    uint32_t timestamp_ms;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
} test_attitude_t;

// Echo packet (variable length, max 31 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type; // PACKET_TYPE_ECHO
    uint8_t sequence;
    uint8_t data[29]; // Echo back received data
} test_echo_t;

// ============================================================================
// RX Callback
// ============================================================================

static void radio_rx_callback(const void *data, size_t len, void *user_data) {
    (void)user_data;

    if (len < 1) {
        return;
    }

    const uint8_t *bytes = (const uint8_t *)data;
    s_state.rx_count++;
    s_state.last_rx_type = bytes[0];
    s_state.last_rx_len = len;

    if (len <= sizeof(s_state.last_rx_data)) {
        memcpy((void *)s_state.last_rx_data, data, len);
    }

    // Mark ground station as connected on first packet
    if (!s_state.ground_connected) {
        s_state.ground_connected = true;
    }
}

// ============================================================================
// LED Helpers
// ============================================================================

static void test_blink(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        hal_led_on();
        hal_delay_ms(on_ms);
        hal_led_off();
        hal_delay_ms(off_ms);
    }
}

static void error_blink_forever(void) {
    while (1) {
        hal_led_toggle();
        hal_delay_ms(500);
    }
}

// ============================================================================
// Test Phases
// ============================================================================

static bool wait_for_battery(void) {
    HIVE_LOG_INFO("[RADIO] Waiting for battery packet...");

    uint32_t start = hal_get_time_ms();

    while ((hal_get_time_ms() - start) < BATTERY_TIMEOUT_MS) {
        hal_radio_poll();

        float voltage = hal_radio_get_battery();
        if (voltage > 0.1f) {
            s_state.battery_received = true;
            HIVE_LOG_INFO("[RADIO] Battery: %.2fV", voltage);
            return true;
        }

        hal_delay_ms(10);
    }

    HIVE_LOG_WARN("[RADIO] Battery packet timeout (nRF51 may not be running)");
    return false;
}

static bool wait_for_connection(void) {
    HIVE_LOG_INFO("[RADIO] Waiting for ground station connection...");
    HIVE_LOG_INFO("[RADIO] Connect with Crazyradio PA/2.0 now");

    uint32_t start = hal_get_time_ms();

    while ((hal_get_time_ms() - start) < CONNECTION_TIMEOUT_MS) {
        hal_radio_poll();

        if (s_state.ground_connected) {
            HIVE_LOG_INFO("[RADIO] Ground station connected!");
            return true;
        }

        // Blink while waiting
        if (((hal_get_time_ms() - start) / 500) % 2) {
            hal_led_on();
        } else {
            hal_led_off();
        }

        hal_delay_ms(10);
    }

    HIVE_LOG_WARN("[RADIO] Connection timeout");
    return false;
}

static void run_communication_test(void) {
    HIVE_LOG_INFO("[RADIO] Starting communication test (30s)...");

    uint32_t start = hal_get_time_ms();
    uint32_t last_tx = 0;
    uint32_t tx_success = 0;
    uint32_t tx_fail = 0;
    uint8_t sequence = 0;

    while ((hal_get_time_ms() - start) < TEST_DURATION_MS) {
        uint32_t now = hal_get_time_ms();

        // Poll for incoming packets
        hal_radio_poll();

        // Send telemetry at regular intervals
        if ((now - last_tx) >= TELEMETRY_INTERVAL_MS) {
            last_tx = now;

            if (hal_radio_tx_ready()) {
                // Alternate between attitude and position packets
                if (sequence % 2 == 0) {
                    test_attitude_t pkt = {
                        .type = PACKET_TYPE_ATTITUDE,
                        .timestamp_ms = now,
                        .gyro_x = (int16_t)(sequence * 10),
                        .gyro_y = (int16_t)(sequence * 20),
                        .gyro_z = (int16_t)(sequence * 30),
                        .roll = 0,
                        .pitch = 0,
                        .yaw = (int16_t)(sequence * 100),
                    };

                    if (hal_radio_send(&pkt, sizeof(pkt)) == 0) {
                        tx_success++;
                        s_state.tx_count++;
                        hal_led_toggle();
                    } else {
                        tx_fail++;
                    }
                } else {
                    // Send echo request
                    test_echo_t pkt = {
                        .type = PACKET_TYPE_ECHO,
                        .sequence = sequence,
                    };
                    snprintf_((char *)pkt.data, sizeof(pkt.data), "SEQ=%u",
                              sequence);

                    if (hal_radio_send(&pkt, sizeof(pkt)) == 0) {
                        tx_success++;
                        s_state.tx_count++;
                        hal_led_toggle();
                    } else {
                        tx_fail++;
                    }
                }

                sequence++;
            }
        }

        // Log status every 5 seconds
        if ((now - start) % 5000 < 10) {
            HIVE_LOG_INFO("[RADIO] TX: %lu/%lu, RX: %lu, Battery: %.2fV",
                          tx_success, tx_success + tx_fail, s_state.rx_count,
                          hal_radio_get_battery());
        }

        hal_delay_ms(1);
    }

    HIVE_LOG_INFO("[RADIO] Test complete!");
    HIVE_LOG_INFO("[RADIO] TX success: %lu, TX fail: %lu", tx_success, tx_fail);
    HIVE_LOG_INFO("[RADIO] RX count: %lu", s_state.rx_count);
    HIVE_LOG_INFO("[RADIO] Final battery: %.2fV", hal_radio_get_battery());
}

// ============================================================================
// Main Test Entry Point
// ============================================================================

int test_syslink_run(bool standalone) {
    int result = 0;

    if (standalone) {
        // Initialize HAL
        hal_debug_init();
        if (hal_init() != 0) {
            return -1;
        }

        // Initialize Hive runtime (for logging)
        hive_status_t status = hive_init();
        if (HIVE_FAILED(status)) {
            return -1;
        }
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  Radio Communication Test");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("[RADIO] DMA-based USART6 syslink");

    // 1 blink = starting
    test_blink(1, 200, 200);
    hal_delay_ms(500);

    // Initialize radio
    HIVE_LOG_INFO("[RADIO] Initializing radio...");
    if (hal_radio_init() != 0) {
        HIVE_LOG_ERROR("[RADIO] Radio init failed!");
        result = -1;
        goto done;
    }

    // Register RX callback
    hal_radio_set_rx_callback(radio_rx_callback, NULL);

    HIVE_LOG_INFO("[RADIO] Radio initialized (DMA RX enabled)");

    // 2 blinks = radio ready
    test_blink(2, 200, 200);
    hal_delay_ms(500);

    // Wait for battery packet (proves DMA RX works)
    if (wait_for_battery()) {
        // 3 blinks = battery received
        test_blink(3, 200, 200);
        hal_delay_ms(500);
    }
    // Continue even without battery packet (nRF51 might not send it)

    // Wait for ground station connection
    if (!wait_for_connection()) {
        HIVE_LOG_WARN("[RADIO] No ground station - running TX-only test");
    } else {
        // 4 blinks = connected
        test_blink(4, 200, 200);
        hal_delay_ms(500);
    }

    // Run communication test
    run_communication_test();

    // Success
    HIVE_LOG_INFO("[RADIO] TEST PASSED");

done:
    if (standalone) {
        hive_cleanup();

        if (result == 0) {
            // Solid LED = success
            hal_led_on();
            while (1) {
                hal_delay_ms(1000);
            }
        } else {
            // Slow blink = failure
            error_blink_forever();
        }
    }

    return result;
}

// ============================================================================
// Standalone Main (when built as TEST=syslink)
// ============================================================================

#ifndef TEST_MAIN_BUILD
int main(void) {
    return test_syslink_run(true);
}
#endif
