/**
 * ESB Radio Communication Test
 *
 * Tests syslink radio communication with ground station.
 * Sends telemetry packets in the same format as comms_actor.c.
 *
 * ESB Protocol:
 *   - Ground station is PTX (Primary Transmitter) - initiates all communication
 *   - Drone's nRF51 is PRX (Primary Receiver) - responds via ACK payloads
 *   - Drone queues telemetry via hal_esb_send()
 *   - nRF51 attaches queued payload to ACK when ground station polls
 *
 * Packet limits: ESB max payload is 32 bytes. HAL uses 1 byte for framing,
 * so max application payload is 30 bytes.
 *
 * Packet formats (must match comms_actor.c and ground_station.py):
 *   HAL adds protocol framing automatically.
 *   Type 0x01 - Attitude (18 bytes on wire): type + timestamp + gyro_xyz + roll/pitch/yaw
 *   Type 0x02 - Position (18 bytes on wire): type + timestamp + alt + vz/vx/vy + thrust + battery
 *
 * Usage:
 *   make PLATFORM=crazyflie TEST=esb
 *   make flash-crazyflie TEST=esb
 *
 * Ground station:
 *   python3 ../tools/ground_station.py --uri radio://0/80/2M
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
// Packet Definitions (must match comms_actor.c)
// ============================================================================

// Maximum telemetry packet size (HAL uses 1 byte for framing, ESB limit is 32)
#define MAX_TELEMETRY_SIZE 30

// Packet type identifiers
#define PACKET_TYPE_ATTITUDE 0x01
#define PACKET_TYPE_POSITION 0x02

// Packet type 0x01: Attitude and rates (17 bytes payload)
typedef struct __attribute__((packed)) {
    uint8_t type;          // 0x01
    uint32_t timestamp_ms; // Milliseconds since boot
    int16_t gyro_x;        // Raw gyro X (millirad/s)
    int16_t gyro_y;        // Raw gyro Y (millirad/s)
    int16_t gyro_z;        // Raw gyro Z (millirad/s)
    int16_t roll;          // Roll angle (millirad)
    int16_t pitch;         // Pitch angle (millirad)
    int16_t yaw;           // Yaw angle (millirad)
} telemetry_attitude_t;

// Packet type 0x02: Position and altitude (17 bytes payload)
typedef struct __attribute__((packed)) {
    uint8_t type;          // 0x02
    uint32_t timestamp_ms; // Milliseconds since boot
    int16_t altitude;      // Altitude (mm)
    int16_t vz;            // Vertical velocity (mm/s)
    int16_t vx;            // X velocity (mm/s)
    int16_t vy;            // Y velocity (mm/s)
    uint16_t thrust;       // Thrust (0-65535)
    uint16_t battery_mv;   // Battery voltage (millivolts)
} telemetry_position_t;

// Verify packet sizes at compile time
_Static_assert(sizeof(telemetry_attitude_t) <= MAX_TELEMETRY_SIZE,
               "Attitude packet too large");
_Static_assert(sizeof(telemetry_position_t) <= MAX_TELEMETRY_SIZE,
               "Position packet too large");

// ============================================================================
// Test State
// ============================================================================

static volatile uint32_t s_rx_count = 0;
static volatile uint32_t s_tx_count = 0;

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
    uint8_t rx_buf[32];
    size_t rx_len;

    while ((hal_get_time_ms() - start) < BATTERY_TIMEOUT_MS) {
        // Process incoming packets (battery handled internally by HAL)
        while (hal_esb_recv(rx_buf, sizeof(rx_buf), &rx_len)) {
            // Discard any radio packets during init
        }

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
    HIVE_LOG_INFO(
        "[ESB] Response mode: send telemetry when radio packet received");

    uint32_t start = hal_get_time_ms();
    uint32_t last_log = 0;
    uint32_t tx_ok = 0;
    uint32_t tx_busy = 0;
    bool next_is_attitude = true;
    uint8_t rx_buf[32];
    size_t rx_len;

    while ((hal_get_time_ms() - start) < TEST_DURATION_MS) {
        uint32_t now = hal_get_time_ms();

        // Check for incoming packets
        bool got_packet = hal_esb_recv(rx_buf, sizeof(rx_buf), &rx_len);
        if (got_packet) {
            s_rx_count++;
        }

        // Send telemetry in response to received radio packet
        if (got_packet) {
            if (!hal_esb_tx_ready()) {
                tx_busy++;
            } else if (next_is_attitude) {
                // Send attitude packet with zeros (no sensors in test)
                telemetry_attitude_t pkt = {
                    .type = PACKET_TYPE_ATTITUDE,
                    .timestamp_ms = hal_get_time_ms(),
                    .gyro_x = 0,
                    .gyro_y = 0,
                    .gyro_z = 0,
                    .roll = 0,
                    .pitch = 0,
                    .yaw = 0,
                };
                if (hal_esb_send(&pkt, sizeof(pkt)) == 0) {
                    tx_ok++;
                    s_tx_count++;
                    hal_led_toggle();
                }
                next_is_attitude = false;
            } else {
                // Send position packet with battery voltage
                float voltage = hal_power_get_battery();
                telemetry_position_t pkt = {
                    .type = PACKET_TYPE_POSITION,
                    .timestamp_ms = hal_get_time_ms(),
                    .altitude = 0,
                    .vz = 0,
                    .vx = 0,
                    .vy = 0,
                    .thrust = 0,
                    .battery_mv = (uint16_t)(voltage * 1000.0f),
                };
                if (hal_esb_send(&pkt, sizeof(pkt)) == 0) {
                    tx_ok++;
                    s_tx_count++;
                    hal_led_toggle();
                }
                next_is_attitude = true;
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

        hive_actor_id_t actor;
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
