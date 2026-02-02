// Telemetry logger actor - CSV logging for PID tuning
//
// Logs flight data to CSV file at 25Hz for analysis.
// Automatically selects storage: /sd (SD card) or /tmp (local filesystem).
// If no storage is available, logs a warning and exits gracefully.

#include "telemetry_logger_actor.h"
#include "types.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_file.h"
#include "hive_log.h"
#include "printf.h"

#include <string.h>

// Logging interval in microseconds (25Hz = 40ms)
#define LOG_INTERVAL_US (1000000 / TELEMETRY_LOG_RATE_HZ)

// Line buffer size (enough for one CSV row)
#define LINE_BUF_SIZE 512

// Log filename (8.3 compatible for SD card)
#define TLOG_FILENAME "tlog.csv"

// Actor state
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t sensor_bus;
    hive_bus_id_t thrust_bus;
    hive_bus_id_t position_target_bus;
    char log_path[64];
    int log_fd;
} telemetry_logger_state_t;

void *telemetry_logger_init(void *init_args) {
    const telemetry_logger_config_t *cfg = init_args;
    static telemetry_logger_state_t state;
    state.state_bus = cfg->buses->state_bus;
    state.sensor_bus = cfg->buses->sensor_bus;
    state.thrust_bus = cfg->buses->thrust_bus;
    state.position_target_bus = cfg->buses->position_target_bus;
    state.log_path[0] = '\0';
    state.log_fd = -1;
    return &state;
}

void telemetry_logger_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    telemetry_logger_state_t *state = args;
    char line_buf[LINE_BUF_SIZE];

    // Select storage path based on mount availability
    // Prefer SD card, fall back to /tmp (simulation)
    if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
        snprintf_(state->log_path, sizeof(state->log_path), "/sd/%s",
                  TLOG_FILENAME);
    } else if (HIVE_SUCCEEDED(hive_file_mount_available("/tmp"))) {
        snprintf_(state->log_path, sizeof(state->log_path), "/tmp/%s",
                  TLOG_FILENAME);
    } else {
        HIVE_LOG_WARN("[TLOG] No storage available (/sd or /tmp) - "
                      "telemetry logging disabled");
        return;
    }

    // Open CSV file
    hive_status_t status = hive_file_open(
        state->log_path, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644,
        &state->log_fd);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_WARN("[TLOG] Cannot open %s: %s - telemetry logging disabled",
                      state->log_path, HIVE_ERR_STR(status));
        return;
    }

    // Write CSV header
    int len = snprintf_(line_buf, sizeof(line_buf),
                        "time_ms,"
                        "roll,pitch,yaw,"
                        "roll_rate,pitch_rate,yaw_rate,"
                        "x,y,altitude,"
                        "vx,vy,vz,"
                        "thrust,"
                        "target_x,target_y,target_z,target_yaw,"
                        "gyro_x,gyro_y,gyro_z,"
                        "accel_x,accel_y,accel_z\n");
    size_t bytes_written;
    hive_file_write(state->log_fd, line_buf, (size_t)len, &bytes_written);
    hive_file_sync(state->log_fd);

    HIVE_LOG_INFO("[TLOG] Logging to %s at %d Hz", state->log_path,
                  TELEMETRY_LOG_RATE_HZ);

    // Subscribe to buses
    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->sensor_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->thrust_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->position_target_bus))) {
        HIVE_LOG_ERROR("[TLOG] Bus subscribe failed");
        hive_file_close(state->log_fd);
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Start logging timer
    hive_timer_id_t timer;
    if (HIVE_FAILED(hive_timer_every(LOG_INTERVAL_US, &timer))) {
        HIVE_LOG_ERROR("[TLOG] Timer setup failed");
        hive_file_close(state->log_fd);
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Latest data from buses
    state_estimate_t latest_state = STATE_ESTIMATE_ZERO;
    sensor_data_t latest_sensors = SENSOR_DATA_ZERO;
    thrust_cmd_t latest_thrust = THRUST_CMD_ZERO;
    position_target_t latest_target = POSITION_TARGET_ZERO;

    uint32_t start_time = hive_get_time();
    uint32_t log_count = 0;

    while (1) {
        hive_message_t msg;
        status = hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer,
                                     &msg, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[TLOG] recv_match failed: %s",
                           HIVE_ERR_STR(status));
            break;
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

        // Calculate timestamp
        uint32_t now = hive_get_time();
        uint32_t time_ms = (now - start_time) / 1000;

        // Format CSV row
        len = snprintf_(line_buf, sizeof(line_buf),
                        "%u,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,"
                        "%.4f,%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f,"
                        "%.4f,%.4f,%.4f\n",
                        (unsigned)time_ms, latest_state.roll,
                        latest_state.pitch, latest_state.yaw,
                        latest_state.roll_rate, latest_state.pitch_rate,
                        latest_state.yaw_rate, latest_state.x, latest_state.y,
                        latest_state.altitude, latest_state.x_velocity,
                        latest_state.y_velocity, latest_state.vertical_velocity,
                        latest_thrust.thrust, latest_target.x, latest_target.y,
                        latest_target.z, latest_target.yaw,
                        latest_sensors.gyro[0], latest_sensors.gyro[1],
                        latest_sensors.gyro[2], latest_sensors.accel[0],
                        latest_sensors.accel[1], latest_sensors.accel[2]);

        // Write CSV row
        hive_file_write(state->log_fd, line_buf, (size_t)len, &bytes_written);

        log_count++;

        // Sync every 25 samples (once per second)
        if (log_count % TELEMETRY_LOG_RATE_HZ == 0) {
            hive_file_sync(state->log_fd);
        }
    }

    // Cleanup
    hive_file_close(state->log_fd);
    HIVE_LOG_INFO("[TLOG] Closed log file (%u samples)", (unsigned)log_count);
}
