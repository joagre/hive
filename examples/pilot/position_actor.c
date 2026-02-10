// Position actor - Horizontal position hold control
//
// Subscribes to state bus, runs simple PD position control,
// publishes attitude setpoints for the attitude actor to track.
//
// Sign conventions:
//   Internal: Positive error -> positive command -> accelerate toward target
//   Aerospace: Positive pitch (nose up) -> -X accel, positive roll -> -Y accel
//
// Roll is negated when publishing to convert from internal to aerospace.

#include "position_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "tunable_params.h"
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"
#include <string.h>
#include <math.h>
#include <stdbool.h>

// Actor state - initialized by position_actor_init
typedef struct {
    hive_bus_id_t state_bus;
    hive_bus_id_t attitude_setpoint_bus;
    hive_bus_id_t position_target_bus;
    tunable_params_t *params;
    hive_actor_id_t flight_manager;
} position_state_t;

void *position_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static position_state_t state;
    state.state_bus = buses->state_bus;
    state.attitude_setpoint_bus = buses->attitude_setpoint_bus;
    state.position_target_bus = buses->position_target_bus;
    state.params = buses->params;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void position_actor(void *args, const hive_spawn_info_t *siblings,
                    size_t sibling_count) {
    position_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[POS] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    if (HIVE_FAILED(hive_bus_subscribe(state->state_bus)) ||
        HIVE_FAILED(hive_bus_subscribe(state->position_target_bus))) {
        HIVE_LOG_ERROR("[POS] Bus subscribe failed");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Current target (updated from waypoint actor)
    position_target_t target = POSITION_TARGET_DEFAULT;

    // Set up hive_select() sources: state bus + RESET notification
    enum { SEL_STATE, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_STATE] = {.type = HIVE_SEL_BUS, .bus = state->state_bus},
        [SEL_RESET] = {.type = HIVE_SEL_IPC,
                       .ipc = {.sender = state->flight_manager,
                               .class = HIVE_MSG_NOTIFY,
                               .id = NOTIFY_RESET}},
    };

    while (true) {
        state_estimate_t est;
        position_target_t new_target;
        size_t len;
        hive_status_t status;

        // Wait for state OR RESET request
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[POS] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            HIVE_LOG_INFO("[POS] RESET - clearing state");
            target = (position_target_t)POSITION_TARGET_DEFAULT;
            continue;
        }

        // SEL_STATE: Copy state data from select result
        if (result.bus.len != sizeof(est)) {
            HIVE_LOG_ERROR("[POS] State bus corrupted: size=%zu expected=%zu",
                           result.bus.len, sizeof(est));
            continue;
        }
        memcpy(&est, result.bus.data, sizeof(est));

        // Read target from waypoint actor (non-blocking, use last known)
        if (hive_bus_read(state->position_target_bus, &new_target,
                          sizeof(new_target), &len, HIVE_TIMEOUT_NONBLOCKING)
                .code == HIVE_OK) {
            target = new_target;
        }

        // Simple PD controller in world frame (use tunable params)
        // Note: When GPS unavailable, state.x/y = 0 and waypoints at origin
        // result in zero error, naturally outputting roll=0, pitch=0
        tunable_params_t *p = state->params;
        float x_error = target.x - est.x;
        float y_error = target.y - est.y;

        // Desired acceleration in world frame
        float accel_x = p->pos_kp * x_error - p->pos_kd * est.x_velocity;
        float accel_y = p->pos_kp * y_error - p->pos_kd * est.y_velocity;

        // Rotate from world frame to body frame based on current yaw
        // Body X (forward) = World X * cos(yaw) + World Y * sin(yaw)
        // Body Y (right)   = -World X * sin(yaw) + World Y * cos(yaw)
        float cos_yaw = cosf(est.yaw);
        float sin_yaw = sinf(est.yaw);

        float pitch_cmd = accel_x * cos_yaw + accel_y * sin_yaw;
        float roll_cmd = -accel_x * sin_yaw + accel_y * cos_yaw;

        // Clamp to maximum tilt angle for safety (use tunable param)
        pitch_cmd = CLAMPF(pitch_cmd, -p->max_tilt_angle, p->max_tilt_angle);
        roll_cmd = CLAMPF(roll_cmd, -p->max_tilt_angle, p->max_tilt_angle);

        // Sign conversion to aerospace convention:
        // - Roll negated: positive body Y error -> negative roll -> +Y accel
        float target_yaw = target.yaw;
        if (!isfinite(target_yaw)) {
            HIVE_LOG_ERROR("[POS] NaN target yaw - waypoint corrupted!");
            target_yaw = est.yaw;
        }
        attitude_setpoint_t setpoint = {
            .roll = -roll_cmd, .pitch = pitch_cmd, .yaw = target_yaw};

        status = hive_bus_publish(state->attitude_setpoint_bus, &setpoint,
                                  sizeof(setpoint));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[POS] bus publish failed: %s", HIVE_ERR_STR(status));
        }
    }
}
