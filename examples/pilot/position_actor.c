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
#include "types.h"
#include "config.h"
#include "math_utils.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <math.h>

// Actor state - initialized by position_actor_init
typedef struct {
    bus_id state_bus;
    bus_id attitude_setpoint_bus;
    bus_id position_target_bus;
} position_state;

void *position_actor_init(void *init_args) {
    const pilot_buses *buses = init_args;
    static position_state state;
    state.state_bus = buses->state_bus;
    state.attitude_setpoint_bus = buses->attitude_setpoint_bus;
    state.position_target_bus = buses->position_target_bus;
    return &state;
}

void position_actor(void *args, const hive_spawn_info *siblings,
                    size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;

    position_state *state = args;

    hive_status status = hive_bus_subscribe(state->state_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[POS] Failed to subscribe to state bus: %s",
                       HIVE_ERR_STR(status));
        return;
    }
    status = hive_bus_subscribe(state->position_target_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[POS] Failed to subscribe to position target bus: %s",
                       HIVE_ERR_STR(status));
        return;
    }

    // Current target (updated from waypoint actor)
    position_target_t target = POSITION_TARGET_DEFAULT;

    while (1) {
        state_estimate_t est;
        position_target_t new_target;
        size_t len;

        // Block until state available
        hive_bus_read_wait(state->state_bus, &est, sizeof(est), &len, -1);

        // Read target from waypoint actor (non-blocking, use last known)
        if (hive_bus_read(state->position_target_bus, &new_target,
                          sizeof(new_target), &len)
                .code == HIVE_OK) {
            target = new_target;
        }

        // Simple PD controller in world frame
        // Note: When GPS unavailable, state.x/y = 0 and waypoints at origin
        // result in zero error, naturally outputting roll=0, pitch=0
        float x_error = target.x - est.x;
        float y_error = target.y - est.y;

        // Desired acceleration in world frame
        float accel_x = POS_KP * x_error - POS_KD * est.x_velocity;
        float accel_y = POS_KP * y_error - POS_KD * est.y_velocity;

        // Rotate from world frame to body frame based on current yaw
        // Body X (forward) = World X * cos(yaw) + World Y * sin(yaw)
        // Body Y (right)   = -World X * sin(yaw) + World Y * cos(yaw)
        float cos_yaw = cosf(est.yaw);
        float sin_yaw = sinf(est.yaw);

        float pitch_cmd = accel_x * cos_yaw + accel_y * sin_yaw;
        float roll_cmd = -accel_x * sin_yaw + accel_y * cos_yaw;

        // Clamp to maximum tilt angle for safety
        pitch_cmd = CLAMPF(pitch_cmd, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
        roll_cmd = CLAMPF(roll_cmd, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);

        // Sign conversion to aerospace convention:
        // - Roll negated: positive body Y error -> negative roll -> +Y accel
        attitude_setpoint_t setpoint = {
            .roll = -roll_cmd, .pitch = pitch_cmd, .yaw = target.yaw};

        hive_bus_publish(state->attitude_setpoint_bus, &setpoint,
                         sizeof(setpoint));
    }
}
