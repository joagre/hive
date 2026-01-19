// Telemetry actor - Radio transmission of flight data
//
// Sends binary telemetry over radio for ground station logging/analysis.
// Uses two alternating packet types due to 31-byte ESB limit.
// Runs at LOW priority to avoid blocking control loops (~370us per send).
//
// Packet format:
//   Type 0x01: Attitude/rates (gyro, roll/pitch/yaw)
//   Type 0x02: Position/altitude (height, velocities, thrust)
//
// Future: Add post-flight log download over radio.
// Ground station sends request, drone reads flash log and sends in chunks.
// Keeps flight phase simple (telemetry only), transfers complete log after landing.

#include "telemetry_actor.h"
#include "pilot_buses.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include <string.h>

#ifdef HAL_HAS_RADIO

// Telemetry rate: 100Hz total (50Hz per packet type)
#define TELEMETRY_INTERVAL_US 10000 // 10ms = 100Hz

// Packet type identifiers
#define PACKET_TYPE_ATTITUDE 0x01
#define PACKET_TYPE_POSITION 0x02

// Scale factors for int16 encoding
#define SCALE_ANGLE 1000   // rad -> millirad
#define SCALE_RATE 1000    // rad/s -> millirad/s
#define SCALE_POS 1000     // m -> mm
#define SCALE_VEL 1000     // m/s -> mm/s
#define SCALE_THRUST 65535 // 0.0-1.0 -> 0-65535

// Packed telemetry packet structures (must fit in 31 bytes)
// Using __attribute__((packed)) to ensure no padding

// Packet type 0x01: Attitude and rates (17 bytes)
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

// Packet type 0x02: Position and altitude (15 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;          // 0x02
    uint32_t timestamp_ms; // Milliseconds since boot
    int16_t altitude;      // Altitude (mm)
    int16_t vz;            // Vertical velocity (mm/s)
    int16_t vx;            // X velocity (mm/s)
    int16_t vy;            // Y velocity (mm/s)
    uint16_t thrust;       // Thrust (0-65535)
} telemetry_position_t;

// Verify packet sizes at compile time
_Static_assert(sizeof(telemetry_attitude_t) <= 31, "Attitude packet too large");
_Static_assert(sizeof(telemetry_position_t) <= 31, "Position packet too large");

// Helper: clamp float to int16 range
static inline int16_t float_to_i16(float val, float scale) {
    float scaled = val * scale;
    if (scaled > 32767.0f)
        return 32767;
    if (scaled < -32768.0f)
        return -32768;
    return (int16_t)scaled;
}

// Helper: clamp float 0.0-1.0 to uint16
static inline uint16_t float_to_u16(float val) {
    if (val < 0.0f)
        return 0;
    if (val > 1.0f)
        return 65535;
    return (uint16_t)(val * 65535.0f);
}

// Actor state
typedef struct {
    bus_id state_bus;
    bus_id sensor_bus;
    bus_id thrust_bus;
    bool next_is_attitude; // Alternate between packet types
    uint32_t tick_count;   // Tick counter for timestamps (10ms per tick)
} telemetry_state;

void *telemetry_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static telemetry_state state;
    state.state_bus = buses->state_bus;
    state.sensor_bus = buses->sensor_bus;
    state.thrust_bus = buses->thrust_bus;
    state.next_is_attitude = true;
    state.tick_count = 0;
    return &state;
}

void telemetry_actor(void *args, const hive_spawn_info *siblings,
                     size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    telemetry_state *state = args;

    // Initialize radio
    if (hal_radio_init() != 0) {
        HIVE_LOG_ERROR("[TELEM] Radio init failed");
        hive_exit();
        return; // Never reached, but silences compiler warning
    }
    HIVE_LOG_INFO("[TELEM] Radio initialized");

    // Subscribe to buses
    hive_bus_subscribe(state->state_bus);
    hive_bus_subscribe(state->sensor_bus);
    hive_bus_subscribe(state->thrust_bus);

    // Start telemetry timer
    timer_id timer;
    hive_timer_every(TELEMETRY_INTERVAL_US, &timer);

    HIVE_LOG_INFO("[TELEM] Started at 100Hz");

    // Latest data from buses
    state_estimate_t latest_state = STATE_ESTIMATE_ZERO;
    sensor_data_t latest_sensors = SENSOR_DATA_ZERO;
    thrust_cmd_t latest_thrust = THRUST_CMD_ZERO;

    while (1) {
        hive_message msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

        // Increment tick counter (10ms per tick)
        state->tick_count++;

        // Poll radio for incoming packets (maintains flow control)
        hal_radio_poll();

        // Read latest bus data (non-blocking)
        size_t bytes_read;
        state_estimate_t tmp_state;
        if (HIVE_SUCCEEDED(hive_bus_read(state->state_bus, &tmp_state,
                                         sizeof(tmp_state), &bytes_read))) {
            if (bytes_read == sizeof(state_estimate_t)) {
                latest_state = tmp_state;
            }
        }

        sensor_data_t tmp_sensors;
        if (HIVE_SUCCEEDED(hive_bus_read(state->sensor_bus, &tmp_sensors,
                                         sizeof(tmp_sensors), &bytes_read))) {
            if (bytes_read == sizeof(sensor_data_t)) {
                latest_sensors = tmp_sensors;
            }
        }

        thrust_cmd_t tmp_thrust;
        if (HIVE_SUCCEEDED(hive_bus_read(state->thrust_bus, &tmp_thrust,
                                         sizeof(tmp_thrust), &bytes_read))) {
            if (bytes_read == sizeof(thrust_cmd_t)) {
                latest_thrust = tmp_thrust;
            }
        }

        // Check if radio is ready (flow control)
        if (!hal_radio_tx_ready()) {
            continue; // Skip this cycle, try again next tick
        }

        // Calculate timestamp (tick_count * 10ms)
        uint32_t now_ms = state->tick_count * 10;

        if (state->next_is_attitude) {
            // Send attitude/rates packet
            telemetry_attitude_t pkt = {
                .type = PACKET_TYPE_ATTITUDE,
                .timestamp_ms = now_ms,
                .gyro_x = float_to_i16(latest_sensors.gyro[0], SCALE_RATE),
                .gyro_y = float_to_i16(latest_sensors.gyro[1], SCALE_RATE),
                .gyro_z = float_to_i16(latest_sensors.gyro[2], SCALE_RATE),
                .roll = float_to_i16(latest_state.roll, SCALE_ANGLE),
                .pitch = float_to_i16(latest_state.pitch, SCALE_ANGLE),
                .yaw = float_to_i16(latest_state.yaw, SCALE_ANGLE),
            };
            hal_radio_send(&pkt, sizeof(pkt));
        } else {
            // Send position/altitude packet
            telemetry_position_t pkt = {
                .type = PACKET_TYPE_POSITION,
                .timestamp_ms = now_ms,
                .altitude = float_to_i16(latest_state.altitude, SCALE_POS),
                .vz = float_to_i16(latest_state.vertical_velocity, SCALE_VEL),
                .vx = float_to_i16(latest_state.x_velocity, SCALE_VEL),
                .vy = float_to_i16(latest_state.y_velocity, SCALE_VEL),
                .thrust = float_to_u16(latest_thrust.thrust),
            };
            hal_radio_send(&pkt, sizeof(pkt));
        }

        // Alternate packet type
        state->next_is_attitude = !state->next_is_attitude;
    }
}

#endif // HAL_HAS_RADIO
