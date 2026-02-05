// Logger actor - Manages hive runtime log and telemetry CSV
//
// Owns lifecycle of both logs:
// - Hive log: open at startup, sync periodically, close on exit
// - Telemetry CSV: 25Hz flight data logging, truncated on RESET
//
// Automatically selects storage: /sd (SD card) or /tmp (local filesystem).
// If no storage is available, logs a warning and exits gracefully.

#include "logger_actor.h"
#include "notifications.h"
#include "types.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_file.h"
#include "hive_log.h"
#include "printf.h"

#include <string.h>

// Logging interval in microseconds (25Hz = 40ms)
#define LOG_INTERVAL_US (1000000 / TELEMETRY_LOG_RATE_HZ)

// Hive log sync interval (4 seconds, matches flight_manager's old interval)
#define HIVE_LOG_SYNC_INTERVAL 100 // Every 100 CSV samples = 4 seconds

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
    hive_actor_id_t flight_manager;
    char csv_path[64];
    int csv_fd;
    bool hive_log_open;
} logger_state_t;

void *logger_actor_init(void *init_args) {
    const logger_config_t *cfg = init_args;
    static logger_state_t state;
    state.state_bus = cfg->buses->state_bus;
    state.sensor_bus = cfg->buses->sensor_bus;
    state.thrust_bus = cfg->buses->thrust_bus;
    state.position_target_bus = cfg->buses->position_target_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    state.csv_path[0] = '\0';
    state.csv_fd = -1;
    state.hive_log_open = false;
    return &state;
}

// Helper to write CSV header
static void write_csv_header(int fd, char *line_buf, size_t buf_size) {
    int len = snprintf_(line_buf, buf_size,
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
    hive_file_write(fd, line_buf, (size_t)len, &bytes_written);
    hive_file_sync(fd);
}

void logger_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    logger_state_t *state = args;
    char line_buf[LINE_BUF_SIZE];

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_WARN(
            "[LOG] flight_manager sibling not found - RESET disabled");
    }

    // Select storage path based on mount availability
    // Prefer SD card, fall back to /tmp (simulation)
    const char *storage_base = NULL;
    if (HIVE_SUCCEEDED(hive_file_mount_available("/sd"))) {
        storage_base = "/sd";
    } else if (HIVE_SUCCEEDED(hive_file_mount_available("/tmp"))) {
        storage_base = "/tmp";
    } else {
        HIVE_LOG_WARN("[LOG] No storage available (/sd or /tmp) - "
                      "logging disabled");
        return;
    }

    // Hive log is opened by pilot.c before actors start
    // We just sync it periodically
    state->hive_log_open = true;

    // Build CSV path
    snprintf_(state->csv_path, sizeof(state->csv_path), "%s/%s", storage_base,
              TLOG_FILENAME);

    // Open CSV file
    hive_status_t status = hive_file_open(
        state->csv_path, HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC, 0644,
        &state->csv_fd);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_WARN("[LOG] Cannot open %s: %s - CSV logging disabled",
                      state->csv_path, HIVE_ERR_STR(status));
        // Continue - hive log still works
        if (!state->hive_log_open) {
            return; // Both logs failed, exit
        }
    } else {
        // Write CSV header
        write_csv_header(state->csv_fd, line_buf, sizeof(line_buf));
        HIVE_LOG_INFO("[LOG] CSV log open: %s at %d Hz", state->csv_path,
                      TELEMETRY_LOG_RATE_HZ);
    }

    // Subscribe to buses (only if CSV logging is active)
    if (state->csv_fd >= 0) {
        if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
            HIVE_FAILED(hive_bus_subscribe(state->sensor_bus)) ||
            HIVE_FAILED(hive_bus_subscribe(state->thrust_bus)) ||
            HIVE_FAILED(hive_bus_subscribe(state->position_target_bus))) {
            HIVE_LOG_ERROR("[LOG] Bus subscribe failed");
            hive_file_close(state->csv_fd);
            if (state->hive_log_open) {
                hive_log_file_close();
            }
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }
    }

    // Start logging timer
    hive_timer_id_t timer;
    if (HIVE_FAILED(hive_timer_every(LOG_INTERVAL_US, &timer))) {
        HIVE_LOG_ERROR("[LOG] Timer setup failed");
        if (state->csv_fd >= 0) {
            hive_file_close(state->csv_fd);
        }
        if (state->hive_log_open) {
            hive_log_file_close();
        }
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Latest data from buses
    state_estimate_t latest_state = STATE_ESTIMATE_ZERO;
    sensor_data_t latest_sensors = SENSOR_DATA_ZERO;
    thrust_cmd_t latest_thrust = THRUST_CMD_ZERO;
    position_target_t latest_target = POSITION_TARGET_ZERO;

    uint32_t start_time = hive_get_time();
    uint32_t log_count = 0;

    // Set up hive_select() sources: timer + RESET notification
    enum { SEL_TIMER, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_TIMER] = {HIVE_SEL_IPC, .ipc = {.sender = HIVE_SENDER_ANY,
                                             .class = HIVE_MSG_TIMER,
                                             .id = HIVE_ID_ANY,
                                             .tag = timer}},
        [SEL_RESET] = {HIVE_SEL_IPC, .ipc = {.sender = state->flight_manager,
                                             .class = HIVE_MSG_NOTIFY,
                                             .id = NOTIFY_RESET,
                                             .tag = HIVE_TAG_ANY}},
    };
    // Only include RESET source if flight_manager is valid
    size_t num_sources =
        (state->flight_manager != HIVE_ACTOR_ID_INVALID) ? 2 : 1;

    while (1) {
        hive_select_result_t result;
        status = hive_select(sources, num_sources, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[LOG] select failed: %s", HIVE_ERR_STR(status));
            break;
        }

        if (result.index == SEL_RESET) {
            // RESET notification - truncate CSV file for new flight
            // (hive log is kept continuous for debugging)
            if (state->csv_fd >= 0) {
                HIVE_LOG_INFO("[LOG] RESET - truncating CSV file");
                hive_file_close(state->csv_fd);

                // Reopen file with truncation
                status =
                    hive_file_open(state->csv_path,
                                   HIVE_O_WRONLY | HIVE_O_CREAT | HIVE_O_TRUNC,
                                   0644, &state->csv_fd);
                if (HIVE_FAILED(status)) {
                    HIVE_LOG_ERROR("[LOG] Cannot reopen %s: %s",
                                   state->csv_path, HIVE_ERR_STR(status));
                    state->csv_fd = -1;
                } else {
                    // Write CSV header
                    write_csv_header(state->csv_fd, line_buf, sizeof(line_buf));
                }
            }

            // Reset counters
            start_time = hive_get_time();
            log_count = 0;
            latest_state = (state_estimate_t)STATE_ESTIMATE_ZERO;
            latest_sensors = (sensor_data_t)SENSOR_DATA_ZERO;
            latest_thrust = (thrust_cmd_t)THRUST_CMD_ZERO;
            latest_target = (position_target_t)POSITION_TARGET_ZERO;
            continue;
        }

        // SEL_TIMER: Read latest bus data and write CSV (if active)
        if (state->csv_fd >= 0) {
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
            int len = snprintf_(
                line_buf, sizeof(line_buf),
                "%u,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,"
                "%.4f,%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f,"
                "%.4f,%.4f,%.4f\n",
                (unsigned)time_ms, latest_state.roll, latest_state.pitch,
                latest_state.yaw, latest_state.roll_rate,
                latest_state.pitch_rate, latest_state.yaw_rate, latest_state.x,
                latest_state.y, latest_state.altitude, latest_state.x_velocity,
                latest_state.y_velocity, latest_state.vertical_velocity,
                latest_thrust.thrust, latest_target.x, latest_target.y,
                latest_target.z, latest_target.yaw, latest_sensors.gyro[0],
                latest_sensors.gyro[1], latest_sensors.gyro[2],
                latest_sensors.accel[0], latest_sensors.accel[1],
                latest_sensors.accel[2]);

            // Write CSV row
            size_t bytes_written;
            status = hive_file_write(state->csv_fd, line_buf, (size_t)len,
                                     &bytes_written);
            if (HIVE_FAILED(status) || bytes_written != (size_t)len) {
                HIVE_LOG_WARN("[LOG] CSV write failed: wrote %zu of %d bytes",
                              bytes_written, len);
            }
        }

        log_count++;

        // Sync CSV every 25 samples (once per second)
        if (state->csv_fd >= 0 && log_count % TELEMETRY_LOG_RATE_HZ == 0) {
            hive_file_sync(state->csv_fd);
        }

        // Sync hive log every 100 samples (4 seconds)
        if (state->hive_log_open && log_count % HIVE_LOG_SYNC_INTERVAL == 0) {
            hive_log_file_sync();
        }
    }

    // Cleanup
    if (state->csv_fd >= 0) {
        hive_file_close(state->csv_fd);
        HIVE_LOG_INFO("[LOG] Closed CSV file (%u samples)",
                      (unsigned)log_count);
    }
    // Note: hive log is closed by pilot.c, we just sync on exit
    if (state->hive_log_open) {
        hive_log_file_sync();
    }
}
