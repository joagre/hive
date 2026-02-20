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
//   0x03: tlog_state (29 bytes) - timestamp, attitude, rates, position, velocity
//   0x04: tlog_sensors (27 bytes) - timestamp, thrust, targets, gyro, accel
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
#include <stdbool.h>

#ifdef HAL_HAS_RADIO

// Packet type identifiers - telemetry (first byte of payload)
// 0x03/0x04 carry all 24 tlog.csv columns; old 0x01/0x02 retired so
// mismatched ground station fails cleanly instead of producing garbage.
#define PACKET_TYPE_TLOG_STATE 0x03
#define PACKET_TYPE_TLOG_SENSORS 0x04

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

// RX packet layout: HAL prepends 1 framing byte to all received packets.
// [0] = HAL frame header, [1] = command byte, [2..] = command payload.
#define RX_CMD_OFFSET 1         // Command byte position
#define RX_PAYLOAD_OFFSET 2     // First payload byte (e.g., param ID)
#define RX_PARAM_VALUE_OFFSET 3 // Float value position in SET_PARAM
// Minimum packet lengths for command validation
#define RX_MIN_SET_PARAM_LEN 7 // frame(1) + cmd(1) + id(1) + value(4)
#define RX_MIN_GET_PARAM_LEN 3 // frame(1) + cmd(1) + id(1)

// Operating modes
typedef enum {
    COMMS_MODE_FLIGHT,       // Normal telemetry transmission
    COMMS_MODE_LOG_DOWNLOAD, // Sending log file to ground station
    COMMS_MODE_PARAM_LIST    // Sending parameter list to ground station
} comms_mode_t;

// Scale factors for int16 encoding
#define SCALE_ANGLE 1000   // rad -> millirad
#define SCALE_RATE 1000    // rad/s -> millirad/s
#define SCALE_POS 1000     // m -> mm
#define SCALE_VEL 1000     // m/s -> mm/s
#define SCALE_THRUST 65535 // 0.0-1.0 -> 0-65535
#define SCALE_ACCEL 100    // m/s^2 -> cm/s^2 (range +/-327 m/s^2)

// Packed structs for wire format.
// Maximum payload is 30 bytes (HAL uses 1 byte for framing, ESB limit is 32).
// Packed is appropriate here: bandwidth is tight, both ends are controlled
// by this codebase, and ARM Cortex-M handles unaligned access.

// Maximum telemetry packet size (30 bytes payload, HAL adds 1 byte framing)
#define MAX_TELEMETRY_SIZE 30

// Packet type 0x03: tlog state - attitude, rates, position, velocity (29 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;       // 0x03
    uint32_t time_ms;   // Milliseconds since boot
    int16_t roll;       // Roll angle (millirad)
    int16_t pitch;      // Pitch angle (millirad)
    int16_t yaw;        // Yaw angle (millirad)
    int16_t roll_rate;  // Roll rate (millirad/s)
    int16_t pitch_rate; // Pitch rate (millirad/s)
    int16_t yaw_rate;   // Yaw rate (millirad/s)
    int16_t x;          // X position (mm)
    int16_t y;          // Y position (mm)
    int16_t altitude;   // Altitude (mm)
    int16_t vx;         // X velocity (mm/s)
    int16_t vy;         // Y velocity (mm/s)
    int16_t vz;         // Vertical velocity (mm/s)
} tlog_state_packet_t;

// Packet type 0x04: tlog sensors - thrust, targets, gyro, accel (27 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;       // 0x04
    uint32_t time_ms;   // Milliseconds since boot
    uint16_t thrust;    // Thrust (0-65535)
    int16_t target_x;   // Target X (mm)
    int16_t target_y;   // Target Y (mm)
    int16_t target_z;   // Target Z / altitude (mm)
    int16_t target_yaw; // Target yaw (millirad)
    int16_t gyro_x;     // Raw gyro X (millirad/s)
    int16_t gyro_y;     // Raw gyro Y (millirad/s)
    int16_t gyro_z;     // Raw gyro Z (millirad/s)
    int16_t accel_x;    // Accel X (cm/s^2)
    int16_t accel_y;    // Accel Y (cm/s^2)
    int16_t accel_z;    // Accel Z (cm/s^2)
} tlog_sensors_packet_t;

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
_Static_assert(sizeof(tlog_state_packet_t) <= MAX_TELEMETRY_SIZE,
               "tlog_state packet too large for nRF51 syslink");
_Static_assert(sizeof(tlog_sensors_packet_t) <= MAX_TELEMETRY_SIZE,
               "tlog_sensors packet too large for nRF51 syslink");
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
    hive_bus_id_t position_target_bus;
    hive_actor_id_t flight_manager; // For ARM notification
    tunable_params_t *params;       // Tunable parameters
    bool next_is_state;             // Alternate between packet types
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

    uint8_t cmd = data[RX_CMD_OFFSET];

    if (cmd == CMD_GO) {
        // Ground station sends GO to start flight sequence
        HIVE_LOG_INFO("[COMMS] GO command received from ground station");
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            hive_status_t s =
                hive_ipc_notify(state->flight_manager, NOTIFY_GO, NULL, 0);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[COMMS] GO notify failed: %s", HIVE_ERR_STR(s));
            }
        }
        return;
    }

    if (cmd == CMD_ABORT) {
        // Ground station sends ABORT to stop countdown/flight
        HIVE_LOG_INFO("[COMMS] ABORT command received from ground station");
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            hive_status_t s =
                hive_ipc_notify(state->flight_manager, NOTIFY_ABORT, NULL, 0);
            if (HIVE_FAILED(s)) {
                HIVE_LOG_ERROR("[COMMS] ABORT notify failed: %s",
                               HIVE_ERR_STR(s));
            }
        }
        return;
    }

    if (cmd == CMD_STATUS) {
        // Ground station requests status
        if (state->flight_manager != HIVE_ACTOR_ID_INVALID) {
            // Request status from flight manager
            hive_message_t reply;
            hive_status_t s =
                hive_ipc_request(state->flight_manager, HIVE_ID_ANY, NULL, 0,
                                 &reply, 500); // 500ms timeout
            if (HIVE_SUCCEEDED(s) && reply.len >= 2) {
                // Flight manager returns [state:u8, countdown_s:u8]
                const uint8_t *data = (const uint8_t *)reply.data;
                status_response_packet_t pkt = {
                    .type = RESP_STATUS,
                    .state = data[0],
                    .countdown_s = data[1],
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
            log_path = "/sd/hive.log";
        } else if (HIVE_SUCCEEDED(hive_file_mount_available("/log"))) {
            log_path = "/log";
        } else if (HIVE_SUCCEEDED(hive_file_mount_available("/tmp"))) {
            log_path = "/tmp/hive.log";
        }

        if (log_path == NULL) {
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
        if (len < RX_MIN_SET_PARAM_LEN) {
            HIVE_LOG_WARN("[COMMS] SET_PARAM too short: %zu", len);
            return;
        }
        uint8_t param_id = data[RX_PAYLOAD_OFFSET];
        float value;
        memcpy(&value, &data[RX_PARAM_VALUE_OFFSET], sizeof(float));

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
        if (len < RX_MIN_GET_PARAM_LEN) {
            HIVE_LOG_WARN("[COMMS] GET_PARAM too short: %zu", len);
            return;
        }
        uint8_t param_id = data[RX_PAYLOAD_OFFSET];
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
        // Enter param list mode - sends one chunk per main loop iteration
        state->param_list_offset = 0;
        state->mode = COMMS_MODE_PARAM_LIST;
        HIVE_LOG_INFO("[COMMS] Starting param list");
        return;
    }
}

void *comms_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static comms_state_t state;
    state.state_bus = buses->state_bus;
    state.sensor_bus = buses->sensor_bus;
    state.thrust_bus = buses->thrust_bus;
    state.position_target_bus = buses->position_target_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID; // Set from siblings in actor
    state.next_is_state = true;
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

    // Radio already initialized from main() via hal_esb_init().
    // Flush stale DMA data from boot/grace period.
    hal_esb_flush_rx();
    HIVE_LOG_INFO("[COMMS] Radio initialized");

    // Subscribe to buses
    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->sensor_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->position_target_bus))) {
        HIVE_LOG_ERROR("[COMMS] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Get HAL event for RX notification (UART IDLE interrupt)
    hive_hal_event_id_t rx_event = hal_esb_get_rx_event();
    if (rx_event == HIVE_HAL_EVENT_INVALID) {
        HIVE_LOG_ERROR("[COMMS] No RX event available");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    HIVE_LOG_INFO(
        "[COMMS] Started (event-driven), pkt sizes: state=%zu sens=%zu",
        sizeof(tlog_state_packet_t), sizeof(tlog_sensors_packet_t));

    // Latest data from buses
    state_estimate_t latest_state = STATE_ESTIMATE_ZERO;
    sensor_data_t latest_sensors = SENSOR_DATA_ZERO;
    thrust_cmd_t latest_thrust = THRUST_CMD_ZERO;
    position_target_t latest_target = POSITION_TARGET_ZERO;

    // Select sources: HAL event (radio RX) + IPC (RESET notification)
    hive_select_source_t sources[] = {
        {.type = HIVE_SEL_HAL_EVENT, .event = rx_event},
        {.type = HIVE_SEL_IPC,
         .ipc = {.class = HIVE_MSG_REQUEST, .id = NOTIFY_RESET}},
    };

    static uint32_t wake_count = 0;
    while (true) {
        // Wait for radio RX event or RESET notification
        hive_select_result_t result;
        hive_status_t status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[COMMS] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        // Handle RESET: flush syslink parser to resync after DMA overflow
        if (result.index == 1) {
            hal_esb_flush_rx();
            HIVE_LOG_INFO("[COMMS] RESET - syslink flushed");
            {
                hive_status_t rs = hive_ipc_reply(&result.ipc, NULL, 0);
                if (HIVE_FAILED(rs)) {
                    HIVE_LOG_ERROR("[COMMS] RESET reply failed: %s",
                                   HIVE_ERR_STR(rs));
                }
            }
            continue;
        }

        wake_count++;
        if ((wake_count % COMMS_TRACE_INTERVAL) == 1) {
            HIVE_LOG_TRACE("[COMMS] wake %lu", wake_count);
        }

        // Process any pending RX packets (commands from ground station)
        uint8_t rx_buf[32];
        size_t rx_len;
        while (hal_esb_recv(rx_buf, sizeof(rx_buf), &rx_len)) {
            HIVE_LOG_DEBUG("[COMMS] RX len=%zu: %02x %02x", rx_len,
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

        position_target_t tmp_target;
        if (HIVE_SUCCEEDED(hive_bus_read(
                state->position_target_bus, &tmp_target, sizeof(tmp_target),
                &bytes_read, HIVE_TIMEOUT_NONBLOCKING))) {
            if (bytes_read == sizeof(position_target_t)) {
                latest_target = tmp_target;
            }
        }

        // Check hardware flow control before sending
        if (!hal_esb_tx_ready()) {
            if ((wake_count % COMMS_TRACE_INTERVAL) == 1) {
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
        } else if (state->mode == COMMS_MODE_PARAM_LIST) {
            // Param list mode: send one chunk per iteration
            uint8_t offset = state->param_list_offset;
            if (offset >= TUNABLE_PARAM_COUNT) {
                // All params sent, return to flight mode
                state->mode = COMMS_MODE_FLIGHT;
                HIVE_LOG_INFO("[COMMS] Param list complete");
            } else {
                param_list_packet_t pkt = {
                    .type = RESP_PARAM_LIST,
                    .offset = offset,
                };
                int count = 0;
                for (int i = offset; i < TUNABLE_PARAM_COUNT &&
                                     count < PARAM_LIST_MAX_PER_PACKET;
                     i++, count++) {
                    pkt.params[count].id = (uint8_t)i;
                    pkt.params[count].value = tunable_params_get(
                        state->params, (tunable_param_id_t)i);
                }
                hal_esb_send(&pkt, sizeof(pkt));
                state->param_list_offset += count;
            }
        } else {
            // Flight telemetry mode: alternate tlog_state / tlog_sensors
            if (state->next_is_state) {
                tlog_state_packet_t pkt = {
                    .type = PACKET_TYPE_TLOG_STATE,
                    .time_ms = hal_get_time_ms(),
                    .roll = float_to_i16(latest_state.roll, SCALE_ANGLE),
                    .pitch = float_to_i16(latest_state.pitch, SCALE_ANGLE),
                    .yaw = float_to_i16(latest_state.yaw, SCALE_ANGLE),
                    .roll_rate =
                        float_to_i16(latest_state.roll_rate, SCALE_RATE),
                    .pitch_rate =
                        float_to_i16(latest_state.pitch_rate, SCALE_RATE),
                    .yaw_rate = float_to_i16(latest_state.yaw_rate, SCALE_RATE),
                    .x = float_to_i16(latest_state.x, SCALE_POS),
                    .y = float_to_i16(latest_state.y, SCALE_POS),
                    .altitude = float_to_i16(latest_state.altitude, SCALE_POS),
                    .vx = float_to_i16(latest_state.x_velocity, SCALE_VEL),
                    .vy = float_to_i16(latest_state.y_velocity, SCALE_VEL),
                    .vz =
                        float_to_i16(latest_state.vertical_velocity, SCALE_VEL),
                };
                hal_esb_send(&pkt, sizeof(pkt));
            } else {
                tlog_sensors_packet_t pkt = {
                    .type = PACKET_TYPE_TLOG_SENSORS,
                    .time_ms = hal_get_time_ms(),
                    .thrust = float_to_u16(latest_thrust.thrust),
                    .target_x = float_to_i16(latest_target.x, SCALE_POS),
                    .target_y = float_to_i16(latest_target.y, SCALE_POS),
                    .target_z = float_to_i16(latest_target.z, SCALE_POS),
                    .target_yaw = float_to_i16(latest_target.yaw, SCALE_ANGLE),
                    .gyro_x = float_to_i16(latest_sensors.gyro[0], SCALE_RATE),
                    .gyro_y = float_to_i16(latest_sensors.gyro[1], SCALE_RATE),
                    .gyro_z = float_to_i16(latest_sensors.gyro[2], SCALE_RATE),
                    .accel_x =
                        float_to_i16(latest_sensors.accel[0], SCALE_ACCEL),
                    .accel_y =
                        float_to_i16(latest_sensors.accel[1], SCALE_ACCEL),
                    .accel_z =
                        float_to_i16(latest_sensors.accel[2], SCALE_ACCEL),
                };
                hal_esb_send(&pkt, sizeof(pkt));
            }

            // Alternate packet type
            state->next_is_state = !state->next_is_state;
        }
    }
}

#endif // HAL_HAS_RADIO
