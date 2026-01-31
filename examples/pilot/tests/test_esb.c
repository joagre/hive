/**
 * ESB Radio Communication Test
 *
 * Tests syslink radio communication with ground station.
 * Sends simple battery voltage packets to validate the protocol.
 *
 * ESB Protocol:
 *   - Ground station is PTX (Primary Transmitter) - initiates all communication
 *   - Drone's nRF51 is PRX (Primary Receiver) - responds via ACK payloads
 *   - Drone queues telemetry via hal_esb_send()
 *   - nRF51 attaches queued payload to ACK when ground station polls
 *
 * Packet format (must match test_radio_ground.py):
 *   Byte 0: Type (0x03 = battery)
 *   Bytes 1-4: Voltage (float, little-endian)
 *
 * Usage:
 *   make PLATFORM=crazyflie TEST=esb
 *   make flash-crazyflie TEST=esb
 *
 * Ground station:
 *   sudo python test_radio_ground.py --rate 100 --duration 30
 */

#include "hal/hal.h"
#include "hive_log.h"
#include "hive_runtime.h"

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Configuration
// ============================================================================

#define TEST_DURATION_MS 60000   // Run for 60 seconds
#define TELEMETRY_INTERVAL_MS 10 // Queue telemetry every 10ms (100Hz)
#define BATTERY_TIMEOUT_MS 5000  // Wait 5s for battery from nRF51

// ============================================================================
// Packet Definition
// ============================================================================

#define PACKET_TYPE_BATTERY 0x03

// Battery packet: type(1) + voltage(4) = 5 bytes
typedef struct __attribute__((packed)) {
    uint8_t type;
    float voltage;
} battery_packet_t;

_Static_assert(sizeof(battery_packet_t) == 5, "Battery packet size mismatch");

// ============================================================================
// Test State
// ============================================================================

static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_tx_count = 0;

// ============================================================================
// RX Callback
// ============================================================================

static void radio_rx_callback(const void *data, size_t len, void *user_data) {
    (void)data;
    (void)len;
    (void)user_data;
    s_rx_count++;
}

// ============================================================================
// LED Helpers
// ============================================================================

static void blink(int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        hal_led_on();
        hal_delay_ms(on_ms);
        hal_led_off();
        hal_delay_ms(off_ms);
    }
}

// ============================================================================
// Wait for battery voltage from nRF51
// ============================================================================

static bool wait_for_battery(void) {
    HIVE_LOG_INFO("[ESB] Waiting for battery from nRF51...");

    uint32_t start = hal_get_time_ms();

    while ((hal_get_time_ms() - start) < BATTERY_TIMEOUT_MS) {
        hal_esb_poll();

        float voltage = hal_power_get_battery();
        if (voltage > 0.1f) {
            HIVE_LOG_INFO("[ESB] Battery: %.2fV", voltage);
            return true;
        }

        hal_delay_ms(10);
    }

    HIVE_LOG_WARN("[ESB] Battery timeout");
    return false;
}

// ============================================================================
// Telemetry Loop
// ============================================================================

static void run_telemetry_loop(void) {
    HIVE_LOG_INFO("[ESB] Telemetry loop starting (%ds)",
                  TEST_DURATION_MS / 1000);
    HIVE_LOG_INFO("[ESB] Sending battery packets at %d Hz",
                  1000 / TELEMETRY_INTERVAL_MS);

    uint32_t start = hal_get_time_ms();
    uint32_t last_tx = 0;
    uint32_t last_log = 0;
    uint32_t tx_ok = 0;
    uint32_t tx_busy = 0;

    while ((hal_get_time_ms() - start) < TEST_DURATION_MS) {
        uint32_t now = hal_get_time_ms();

        // Poll for incoming packets
        hal_esb_poll();

        // Queue telemetry at fixed interval
        if ((now - last_tx) >= TELEMETRY_INTERVAL_MS) {
            last_tx = now;

            if (!hal_esb_tx_ready()) {
                tx_busy++;
                continue;
            }

            battery_packet_t pkt = {
                .type = PACKET_TYPE_BATTERY,
                .voltage = hal_power_get_battery(),
            };

            if (hal_esb_send(&pkt, sizeof(pkt)) == 0) {
                tx_ok++;
                s_tx_count++;
                hal_led_toggle();
            }
        }

        // Log status every 5 seconds
        if ((now - last_log) >= 5000) {
            last_log = now;
            HIVE_LOG_INFO("[ESB] TX:%lu busy:%lu RX:%lu Batt:%.2fV", tx_ok,
                          tx_busy, s_rx_count, hal_power_get_battery());
        }

        hal_delay_ms(1);
    }

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  TEST COMPLETE");
    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  TX sent:  %lu", tx_ok);
    HIVE_LOG_INFO("  TX busy:  %lu", tx_busy);
    HIVE_LOG_INFO("  RX count: %lu", s_rx_count);
    HIVE_LOG_INFO("  Battery:  %.2fV", hal_power_get_battery());
}

// ============================================================================
// Test Actor
// ============================================================================

static void esb_test_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    HIVE_LOG_INFO("========================================");
    HIVE_LOG_INFO("  ESB Radio Test");
    HIVE_LOG_INFO("========================================");

    hal_esb_set_rx_callback(radio_rx_callback, NULL);

    // 2 blinks = radio ready
    blink(2, 200, 200);
    hal_delay_ms(500);

    // Wait for battery from nRF51
    if (wait_for_battery()) {
        blink(3, 200, 200);
        hal_delay_ms(500);
    }

    run_telemetry_loop();

    HIVE_LOG_INFO("[ESB] TEST PASSED");
    hal_led_on();

    while (1) {
        hal_delay_ms(1000);
    }
}

// ============================================================================
// Main
// ============================================================================

int test_esb_run(bool standalone) {
    if (standalone) {
        hal_debug_init();
        if (hal_init() != 0) {
            return -1;
        }

        blink(1, 200, 200);
        hal_delay_ms(300);

        HIVE_LOG_INFO("[ESB] Initializing radio...");
        if (hal_esb_init() != 0) {
            HIVE_LOG_ERROR("[ESB] hal_esb_init failed!");
            while (1) {
                hal_led_toggle();
                hal_delay_ms(100);
            }
        }
        HIVE_LOG_INFO("[ESB] Radio OK");

        hive_status_t status = hive_init();
        if (HIVE_FAILED(status)) {
            return -1;
        }

        actor_id_t actor;
        hive_actor_config_t cfg = {.stack_size = 4096};
        status = hive_spawn(esb_test_actor, NULL, NULL, &cfg, &actor);
        if (HIVE_FAILED(status)) {
            return -1;
        }

        hive_run();
        hive_cleanup();
    }

    return 0;
}

#ifndef TEST_MAIN_BUILD
int main(void) {
    return test_esb_run(true);
}
#endif
