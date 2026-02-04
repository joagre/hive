// Comms actor - Ground station communication
//
// Handles bidirectional radio communication with ground station:
//   - Downlink: Telemetry in response to ground station polls
//   - Downlink: Log file transfer on request
//   - Uplink: Commands from ground station
//
// Architecture: Interrupt-driven RX with 100Hz TX refresh
//   - Ground station (PTX) polls, drone (PRX) responds via ACK payload
//   - Each poll triggers UART IDLE interrupt -> HAL event -> actor wakes
//   - Actor uses hive_event_wait() to wait on RX event (no polling)
//   - 100Hz timer refreshes telemetry payload between polls
//   - See hal/crazyflie-2.1plus/README.md "Radio Communication Flow" for diagram
//
// Runs at LOW priority so control loops run first each cycle.
// Radio send blocks ~370us (37 bytes * 10 bits/byte / 1Mbaud).
//
// Telemetry packets can be up to 30 bytes payload (HAL adds protocol overhead).
// ESB max payload is 32 bytes; HAL uses 1 byte for framing.
//
// Packet types (first byte of payload):
//   0x01: Attitude/rates (17 bytes) - timestamp, gyro xyz, roll/pitch/yaw
//   0x02: Position/altitude (17 bytes) - timestamp, alt, velocities, thrust, battery
//   0x10: CMD_REQUEST_LOG - ground station requests log download
//   0x11: LOG_CHUNK - drone sends log data chunk
//   0x12: LOG_COMPLETE - drone signals end of log file

#include "comms_actor.h"
#include "pilot_buses.h"
#include "tunable_params.h"
#include "notifications.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"
#include "hive_file.h"
#include <string.h>

#ifdef HAL_HAS_RADIO

// Packet type identifiers - telemetry (first byte of payload)
#define PACKET_TYPE_ATTITUDE 0x01
#define PACKET_TYPE_POSITION 0x02

// Packet type identifiers - log download
#define CMD_REQUEST_LOG 0x10  // Ground -> drone: request log download
#define PACKET_LOG_CHUNK 0x11 // Drone -> ground: log data chunk
#define PACKET_LOG_DONE 0x12  // Drone -> ground: download complete

// Packet type identifiers - flight control
#define CMD_GO 0x20     // Ground -> drone: start flight sequence
#define CMD_ABORT 0x21  // Ground -> drone: abort countdown/flight
#define CMD_STATUS 0x22 // Ground -> drone: request status

// Response packet types - flight control
#define RESP_STATUS 0x23 // Drone -> ground: status [state:u8][countdown_s:u8]

// Packet type identifiers - parameter tuning (0x30-0x32)
#define CMD_SET_PARAM 0x30 // Ground -> drone: set parameter [id:u8][value:f32]
#define CMD_GET_PARAM 0x31 // Ground -> drone: get parameter [id:u8]
#define CMD_LIST_PARAMS 0x32 // Ground -> drone: list all parameters

// Response packet types
#define RESP_PARAM_ACK \
    0x33 // Drone -> ground: param set acknowledged [status:u8]
#define RESP_PARAM_VALUE 0x34 // Drone -> ground: param value [id:u8][value:f32]
#define RESP_PARAM_LIST \
    0x35 // Drone -> ground: param list [count:u8][id:u8][val:f32]...

// Log chunk data size (30 - 3 byte header = 27 bytes)
// HAL uses 1 byte for framing, leaving 30 bytes for payload
// Header: type(1) + sequence(2) = 3 bytes
#define LOG_CHUNK_DATA_SIZE 27

// Operating modes
typedef enum {
    COMMS_MODE_FLIGHT,      // Normal telemetry transmission
    COMMS_MODE_LOG_DOWNLOAD // Sending log file to ground station
} comms_mode_t;

// Scale factors for int16 encoding
#define SCALE_ANGLE 1000   // rad -> millirad
#define SCALE_RATE 1000    // rad/s -> millirad/s
#define SCALE_POS 1000     // m -> mm
#define SCALE_VEL 1000     // m/s -> mm/s
#define SCALE_THRUST 65535 // 0.0-1.0 -> 0-65535

// Packed structs for wire format.
// Maximum payload is 30 bytes (HAL uses 1 byte for framing, ESB limit is 32).
// Packed is appropriate here: bandwidth is tight, both ends are controlled
// by this codebase, and ARM Cortex-M handles unaligned access.

// Maximum telemetry packet size (30 bytes payload, HAL adds 1 byte framing)
#define MAX_TELEMETRY_SIZE 30

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

// Packet type 0x11: Log chunk (30 bytes payload)
typedef struct __attribute__((packed)) {
    uint8_t type;                      // 0x11
    uint16_t sequence;                 // Chunk sequence number
    uint8_t data[LOG_CHUNK_DATA_SIZE]; // File data
} log_chunk_packet_t;

// Packet type 0x12: Log complete (3 bytes payload)
typedef struct __attribute__((packed)) {
    uint8_t type;          // 0x12
    uint16_t total_chunks; // Total chunks sent
} log_complete_packet_t;

// Verify packet sizes at compile time
// All packets must be <= 30 bytes (HAL adds 1 byte framing, ESB limit is 32)
_Static_assert(sizeof(telemetry_attitude_t) <= MAX_TELEMETRY_SIZE,
               "Attitude packet too large for nRF51 syslink");
_Static_assert(sizeof(telemetry_position_t) <= MAX_TELEMETRY_SIZE,
               "Position packet too large for nRF51 syslink");
_Static_assert(sizeof(log_chunk_packet_t) <= 30, "Log chunk packet too large");
_Static_assert(sizeof(log_complete_packet_t) <= 30,
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

// Param response packets
typedef struct __attribute__((packed)) {
    uint8_t type;   // RESP_PARAM_ACK
    uint8_t status; // 0 = success, 1 = error
} param_ack_packet_t;

typedef struct __attribute__((packed)) {
    uint8_t type; // RESP_PARAM_VALUE
    uint8_t id;   // Parameter ID
    float value;  // Parameter value
} param_value_packet_t;

// Param list response (up to 5 params per packet to fit in 30 bytes)
// 1 + 1 + 5*(1+4) = 27 bytes
#define PARAM_LIST_MAX_PER_PACKET 5
typedef struct __attribute__((packed)) {
    uint8_t type;   // RESP_PARAM_LIST
    uint8_t offset; // Starting parameter ID
    struct __attribute__((packed)) {
        uint8_t id;
        float value;
    } params[PARAM_LIST_MAX_PER_PACKET];
} param_list_packet_t;

// Packet type RESP_STATUS: Flight status (3 bytes payload)
typedef struct __attribute__((packed)) {
    uint8_t type;        // RESP_STATUS (0x23)
    uint8_t state;       // FM_STATE_* enum value
    uint8_t countdown_s; // Seconds remaining in countdown (0 if not in ARMED)
} status_response_packet_t;

// Actor state
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t sensor_bus;
    hive_bus_id_t thrust_bus;
    hive_actor_id_t flight_manager; // For ARM notification
    tunable_params_t *params;       // Tunable parameters
    bool next_is_attitude;          // Alternate between packet types
    // Log download state
    comms_mode_t mode;
    int log_fd;
    uint32_t log_offset;
    uint16_t log_sequence;
    // Param list state (for multi-packet responses)
    uint8_t param_list_offset;
} comms_state_t;

// Handle incoming command from ground station
static void handle_rx_command(comms_state_t *state, const uint8_t *data,
                              size_t len) {
    // Need at least HAL frame header + command byte
    if (len < 2) {
        return;
    }

    // Skip HAL frame header (byte 0), command is in byte 1
    uint8_t cmd = data[1];

    if (cmd == CMD_GO) {
        // Ground station sends GO to start flight sequence
        HIVE_LOG_INFO("[COMMS] GO command received from ground station");
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            hive_ipc_notify(state->flight_manager, NOTIFY_GO, NULL, 0);
        }
        return;
    }

    if (cmd == CMD_ABORT) {
        // Ground station sends ABORT to stop countdown/flight
        HIVE_LOG_INFO("[COMMS] ABORT command received from ground station");
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            hive_ipc_notify(state->flight_manager, NOTIFY_ABORT, NULL, 0);
        }
        return;
    }

    if (cmd == CMD_STATUS) {
        // Ground station requests status
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            // Request status from flight manager
            hive_message_t reply;
            hive_status_t s = hive_ipc_request(state->flight_manager, NULL, 0,
                                               &reply, 500); // 500ms timeout
            if (HIVE_SUCCEEDED(s) && reply.len >= 2) {
                // Flight manager returns [state:u8, countdown_s:u8]
                status_response_packet_t pkt = {
                    .type = RESP_STATUS,
                    .state = reply.data[0],
                    .countdown_s = reply.data[1],
                };
                hal_esb_send(&pkt, sizeof(pkt));
            } else {
                HIVE_LOG_WARN("[COMMS] STATUS request failed or timeout");
            }
        }
        return;
    }

    if (cmd == CMD_REQUEST_LOG) {
        // Ground station requests log download
        // Find log file path (must match flight_manager_actor.c)
        const char *log_path = NULL;
        if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
            log_path = "/sd/flm.log";
        } else if (HIVE_SUCCEEDED(hive_file_mount_available("/log"))) {
            log_path = "/log";
        } else if (HIVE_SUCCEEDED(hive_file_mount_available("/tmp"))) {
            log_path = "/tmp/flm.log";
        }

        if (!log_path) {
            HIVE_LOG_WARN("[COMMS] Log download failed: no storage available");
            return;
        }

        int fd;
        hive_status_t s = hive_file_open(log_path, HIVE_O_RDONLY, 0, &fd);
        if (HIVE_FAILED(s)) {
            HIVE_LOG_WARN("[COMMS] Log download failed: cannot open %s",
                          log_path);
            return;
        }

        state->log_fd = fd;
        state->log_offset = 0;
        state->log_sequence = 0;
        state->mode = COMMS_MODE_LOG_DOWNLOAD;
        HIVE_LOG_INFO("[COMMS] Log download started: %s", log_path);
        return;
    }

    if (cmd == CMD_SET_PARAM) {
        // Set parameter: [id:u8][value:f32] - need 6 bytes total (header + id + value)
        if (len < 7) {
            HIVE_LOG_WARN("[COMMS] SET_PARAM too short: %zu", len);
            return;
        }
        uint8_t param_id = data[2];
        float value;
        memcpy(&value, &data[3], sizeof(float));

        hive_status_t s = tunable_params_set(
            state->params, (tunable_param_id_t)param_id, value);
        param_ack_packet_t ack = {
            .type = RESP_PARAM_ACK,
            .status = HIVE_SUCCEEDED(s) ? 0 : 1,
        };
        hal_esb_send(&ack, sizeof(ack));
        return;
    }

    if (cmd == CMD_GET_PARAM) {
        // Get parameter: [id:u8] - need 3 bytes total (header + id)
        if (len < 3) {
            HIVE_LOG_WARN("[COMMS] GET_PARAM too short: %zu", len);
            return;
        }
        uint8_t param_id = data[2];
        float value =
            tunable_params_get(state->params, (tunable_param_id_t)param_id);
        param_value_packet_t resp = {
            .type = RESP_PARAM_VALUE,
            .id = param_id,
            .value = value,
        };
        hal_esb_send(&resp, sizeof(resp));
        return;
    }

    if (cmd == CMD_LIST_PARAMS) {
        // Start param list transmission from offset 0
        state->param_list_offset = 0;
        state->mode =
            COMMS_MODE_FLIGHT; // Stay in flight mode, list is one-shot
        // Send first batch immediately
        param_list_packet_t pkt = {
            .type = RESP_PARAM_LIST,
            .offset = 0,
        };
        for (int i = 0;
             i < PARAM_LIST_MAX_PER_PACKET && i < TUNABLE_PARAM_COUNT; i++) {
            pkt.params[i].id = (uint8_t)i;
            pkt.params[i].value =
                tunable_params_get(state->params, (tunable_param_id_t)i);
        }
        hal_esb_send(&pkt, sizeof(pkt));
        HIVE_LOG_INFO("[COMMS] Sent param list offset=%u", pkt.offset);
        return;
    }
}

void *comms_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static comms_state_t state;
    state.state_bus = buses->state_bus;
    state.sensor_bus = buses->sensor_bus;
    state.thrust_bus = buses->thrust_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    state.next_is_attitude = true;
    state.mode = COMMS_MODE_FLIGHT;
    state.log_fd = -1;
    state.log_offset = 0;
    state.log_sequence = 0;
    state.param_list_offset = 0;
    return &state;
}

void comms_actor(void *args, const hive_spawn_info_t *siblings,
                 size_t sibling_count) {
    comms_state_t *state = args;

    // Find flight_manager sibling for ARM notification
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_WARN(
            "[COMMS] flight_manager sibling not found - ARM disabled");
    }

    // Radio already initialized from main() via hal_esb_init()
    HIVE_LOG_INFO("[COMMS] Radio initialized");

    // Subscribe to buses
    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->sensor_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus))) {
        HIVE_LOG_ERROR("[COMMS] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Get HAL event for RX notification (UART IDLE interrupt)
    hive_hal_event_id_t rx_event = hal_esb_get_rx_event();
    if (rx_event == HIVE_HAL_EVENT_INVALID) {
        HIVE_LOG_ERROR("[COMMS] No RX event available");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    HIVE_LOG_INFO("[COMMS] Started (event-driven), pkt sizes: att=%zu pos=%zu",
                  sizeof(telemetry_attitude_t), sizeof(telemetry_position_t));

    // Latest data from buses
    state_estimate_t latest_state = STATE_ESTIMATE_ZERO;
    sensor_data_t latest_sensors = SENSOR_DATA_ZERO;
    thrust_cmd_t latest_thrust = THRUST_CMD_ZERO;

    static uint32_t wake_count = 0;
    while (1) {
        // Wait for RX event (UART IDLE interrupt signals ground station poll)
        hive_status_t status = hive_event_wait(rx_event, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[COMMS] event wait failed: %s",
                           HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }
        wake_count++;
        if ((wake_count % 500) == 1) {
            HIVE_LOG_INFO("[COMMS] wake %lu", wake_count);
        }

        // Process any pending RX packets (commands from ground station)
        uint8_t rx_buf[32];
        size_t rx_len;
        while (hal_esb_recv(rx_buf, sizeof(rx_buf), &rx_len)) {
            HIVE_LOG_INFO("[COMMS] RX len=%zu: %02x %02x", rx_len,
                          rx_len > 0 ? rx_buf[0] : 0,
                          rx_len > 1 ? rx_buf[1] : 0);
            handle_rx_command(state, rx_buf, rx_len);
        }

        // Read latest bus data (non-blocking)
        size_t bytes_read;
        state_estimate_t tmp_state;
        if (HIVE_SUCCEEDED(hive_bus_read(state->state_bus, &tmp_state,
                                         sizeof(tmp_state), &bytes_read,
                                         HIVE_TIMEOUT_NONBLOCKING))) {
            if (bytes_read == sizeof(state_estimate_t)) {
                latest_state = tmp_state;
            }
        }

        sensor_data_t tmp_sensors;
        if (HIVE_SUCCEEDED(hive_bus_read(state->sensor_bus, &tmp_sensors,
                                         sizeof(tmp_sensors), &bytes_read,
                                         HIVE_TIMEOUT_NONBLOCKING))) {
            if (bytes_read == sizeof(sensor_data_t)) {
                latest_sensors = tmp_sensors;
            }
        }

        thrust_cmd_t tmp_thrust;
        if (HIVE_SUCCEEDED(hive_bus_read(state->thrust_bus, &tmp_thrust,
                                         sizeof(tmp_thrust), &bytes_read,
                                         HIVE_TIMEOUT_NONBLOCKING))) {
            if (bytes_read == sizeof(thrust_cmd_t)) {
                latest_thrust = tmp_thrust;
            }
        }

        // Check hardware flow control before sending
        if (!hal_esb_tx_ready()) {
            if ((wake_count % 1000) == 1) {
                HIVE_LOG_WARN("[COMMS] TX not ready");
            }
            continue;
        }

        // Handle current mode
        if (state->mode == COMMS_MODE_LOG_DOWNLOAD) {
            // Log download mode: send next chunk or complete
            log_chunk_packet_t chunk = {
                .type = PACKET_LOG_CHUNK,
                .sequence = state->log_sequence,
            };

            // Read next chunk from file
            size_t file_bytes_read;
            hive_status_t s =
                hive_file_pread(state->log_fd, chunk.data, LOG_CHUNK_DATA_SIZE,
                                state->log_offset, &file_bytes_read);

            if (HIVE_FAILED(s) || file_bytes_read == 0) {
                // End of file or error - send completion packet
                log_complete_packet_t done = {
                    .type = PACKET_LOG_DONE,
                    .total_chunks = state->log_sequence,
                };
                hal_esb_send(&done, sizeof(done));

                // Close file and return to flight mode
                hive_file_close(state->log_fd);
                state->log_fd = -1;
                state->mode = COMMS_MODE_FLIGHT;
                HIVE_LOG_INFO("[COMMS] Log download complete: %u chunks",
                              state->log_sequence);
            } else {
                // Pad remaining bytes with zeros if partial chunk
                if (file_bytes_read < LOG_CHUNK_DATA_SIZE) {
                    memset(chunk.data + file_bytes_read, 0,
                           LOG_CHUNK_DATA_SIZE - file_bytes_read);
                }

                hal_esb_send(&chunk, sizeof(chunk));
                state->log_offset += file_bytes_read;
                state->log_sequence++;
            }
        } else {
            // Flight telemetry mode: alternate between attitude and position
            if (state->next_is_attitude) {
                // Send attitude packet
                telemetry_attitude_t pkt = {
                    .type = PACKET_TYPE_ATTITUDE,
                    .timestamp_ms = hal_get_time_ms(),
                    .gyro_x = float_to_i16(latest_sensors.gyro[0], SCALE_RATE),
                    .gyro_y = float_to_i16(latest_sensors.gyro[1], SCALE_RATE),
                    .gyro_z = float_to_i16(latest_sensors.gyro[2], SCALE_RATE),
                    .roll = float_to_i16(latest_state.roll, SCALE_ANGLE),
                    .pitch = float_to_i16(latest_state.pitch, SCALE_ANGLE),
                    .yaw = float_to_i16(latest_state.yaw, SCALE_ANGLE),
                };
                hal_esb_send(&pkt, sizeof(pkt));
            } else {
                // Send position packet
                telemetry_position_t pkt = {
                    .type = PACKET_TYPE_POSITION,
                    .timestamp_ms = hal_get_time_ms(),
                    .altitude = float_to_i16(latest_state.altitude, SCALE_POS),
                    .vz =
                        float_to_i16(latest_state.vertical_velocity, SCALE_VEL),
                    .vx = float_to_i16(latest_state.x_velocity, SCALE_VEL),
                    .vy = float_to_i16(latest_state.y_velocity, SCALE_VEL),
                    .thrust = float_to_u16(latest_thrust.thrust),
                    .battery_mv = (uint16_t)(hal_power_get_battery() * 1000.0f),
                };
                hal_esb_send(&pkt, sizeof(pkt));
            }

            // Alternate packet type
            state->next_is_attitude = !state->next_is_attitude;
        }
    }
}

#endif // HAL_HAS_RADIO
