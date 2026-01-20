// Telemetry actor - Radio transmission of flight data and log download
//
// Two modes of operation:
//   1. Flight mode: Sends binary telemetry at 100Hz for ground station logging
//   2. Download mode: Transfers flash log file to ground station on request
//
// Runs at LOW priority so control loops run first each cycle.
// Radio send blocks ~370us (37 bytes * 10 bits/byte / 1Mbaud).
//
// Packet types:
//   0x01: Attitude/rates (gyro, roll/pitch/yaw) - flight telemetry
//   0x02: Position/altitude (height, velocities, thrust) - flight telemetry
//   0x10: CMD_REQUEST_LOG - ground station requests log download
//   0x11: LOG_CHUNK - drone sends log data chunk
//   0x12: LOG_COMPLETE - drone signals end of log file

#include "telemetry_actor.h"
#include "pilot_buses.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include "hive_file.h"
#include <string.h>

#ifdef HAL_HAS_RADIO

// Telemetry rate: 100Hz total (50Hz per packet type)
#define TELEMETRY_INTERVAL_US 10000 // 10ms = 100Hz

// Packet type identifiers - telemetry
#define PACKET_TYPE_ATTITUDE 0x01
#define PACKET_TYPE_POSITION 0x02

// Packet type identifiers - log download
#define CMD_REQUEST_LOG 0x10  // Ground -> drone: request log download
#define PACKET_LOG_CHUNK 0x11 // Drone -> ground: log data chunk
#define PACKET_LOG_DONE 0x12  // Drone -> ground: download complete

// Log chunk data size (31 - 3 byte header = 28 bytes)
#define LOG_CHUNK_DATA_SIZE 28

// Operating modes
typedef enum {
    TELEM_MODE_FLIGHT,      // Normal telemetry transmission
    TELEM_MODE_LOG_DOWNLOAD // Sending log file to ground station
} telem_mode_t;

// Scale factors for int16 encoding
#define SCALE_ANGLE 1000   // rad -> millirad
#define SCALE_RATE 1000    // rad/s -> millirad/s
#define SCALE_POS 1000     // m -> mm
#define SCALE_VEL 1000     // m/s -> mm/s
#define SCALE_THRUST 65535 // 0.0-1.0 -> 0-65535

// Packed structs for wire format (must fit in 31-byte ESB limit).
// Packed is appropriate here: bandwidth is tight, both ends are controlled
// by this codebase, and ARM Cortex-M handles unaligned access.

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

// Packet type 0x11: Log chunk (31 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;                      // 0x11
    uint16_t sequence;                 // Chunk sequence number
    uint8_t data[LOG_CHUNK_DATA_SIZE]; // File data
} log_chunk_packet_t;

// Packet type 0x12: Log complete (3 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;          // 0x12
    uint16_t total_chunks; // Total chunks sent
} log_complete_packet_t;

// Verify packet sizes at compile time
_Static_assert(sizeof(telemetry_attitude_t) <= 31, "Attitude packet too large");
_Static_assert(sizeof(telemetry_position_t) <= 31, "Position packet too large");
_Static_assert(sizeof(log_chunk_packet_t) <= 31, "Log chunk packet too large");
_Static_assert(sizeof(log_complete_packet_t) <= 31,
               "Log complete packet too large");

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
    // Log download state
    telem_mode_t mode;
    int log_fd;
    uint32_t log_offset;
    uint16_t log_sequence;
} telemetry_state;

// RX callback - handles incoming commands from ground station
static void radio_rx_callback(const void *data, size_t len, void *user_data) {
    telemetry_state *state = (telemetry_state *)user_data;
    if (len < 1 || !state) {
        return;
    }

    const uint8_t *bytes = (const uint8_t *)data;
    uint8_t cmd = bytes[0];

    if (cmd == CMD_REQUEST_LOG) {
        // Ground station requests log download
        // Open log file and switch to download mode
        int fd;
        hive_status s = hive_file_open("/log", HIVE_O_RDONLY, 0, &fd);
        if (HIVE_FAILED(s)) {
            HIVE_LOG_WARN(
                "[TELEM] Log download request failed: cannot open /log");
            return;
        }

        state->log_fd = fd;
        state->log_offset = 0;
        state->log_sequence = 0;
        state->mode = TELEM_MODE_LOG_DOWNLOAD;
        HIVE_LOG_INFO("[TELEM] Log download started");
    }
}

void *telemetry_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static telemetry_state state;
    state.state_bus = buses->state_bus;
    state.sensor_bus = buses->sensor_bus;
    state.thrust_bus = buses->thrust_bus;
    state.next_is_attitude = true;
    state.tick_count = 0;
    state.mode = TELEM_MODE_FLIGHT;
    state.log_fd = -1;
    state.log_offset = 0;
    state.log_sequence = 0;
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
        return; // No hive_exit() - supervisor sees CRASH
    }

    // Register RX callback for ground station commands
    hal_radio_set_rx_callback(radio_rx_callback, state);

    HIVE_LOG_INFO("[TELEM] Radio initialized");

    // Subscribe to buses
    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->sensor_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus))) {
        HIVE_LOG_ERROR("[TELEM] Bus subscribe failed");
        return;
    }

    // Start telemetry timer
    timer_id timer;
    if (HIVE_FAILED(hive_timer_every(TELEMETRY_INTERVAL_US, &timer))) {
        HIVE_LOG_ERROR("[TELEM] Timer setup failed");
        return;
    }

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

        // Handle current mode
        if (state->mode == TELEM_MODE_LOG_DOWNLOAD) {
            // Log download mode: send next chunk or complete
            log_chunk_packet_t chunk = {
                .type = PACKET_LOG_CHUNK,
                .sequence = state->log_sequence,
            };

            // Read next chunk from file
            size_t bytes_read;
            hive_status s =
                hive_file_pread(state->log_fd, chunk.data, LOG_CHUNK_DATA_SIZE,
                                state->log_offset, &bytes_read);

            if (HIVE_FAILED(s) || bytes_read == 0) {
                // End of file or error - send completion packet
                log_complete_packet_t done = {
                    .type = PACKET_LOG_DONE,
                    .total_chunks = state->log_sequence,
                };
                hal_radio_send(&done, sizeof(done));

                // Close file and return to flight mode
                hive_file_close(state->log_fd);
                state->log_fd = -1;
                state->mode = TELEM_MODE_FLIGHT;
                HIVE_LOG_INFO("[TELEM] Log download complete: %u chunks",
                              state->log_sequence);
            } else {
                // Pad remaining bytes with zeros if partial chunk
                if (bytes_read < LOG_CHUNK_DATA_SIZE) {
                    memset(chunk.data + bytes_read, 0,
                           LOG_CHUNK_DATA_SIZE - bytes_read);
                }

                hal_radio_send(&chunk, sizeof(chunk));
                state->log_offset += bytes_read;
                state->log_sequence++;
            }
        } else {
            // Flight telemetry mode: send attitude/position packets
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
                    .vz =
                        float_to_i16(latest_state.vertical_velocity, SCALE_VEL),
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
}

#endif // HAL_HAS_RADIO
