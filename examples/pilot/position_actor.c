// Position actor - Horizontal position hold control
//
// Subscribes to state bus, runs simple PD position control,
// publishes angle setpoints for the angle actor to track.
//
// Internal convention (intuitive physics):
//   - Positive position error → positive command → accelerate toward target
//
// Webots/aerospace convention (attitude → acceleration):
//   - Positive pitch (nose up) → thrust tilts back → -X acceleration
//   - Positive roll (right wing down) → thrust tilts left → -Y acceleration
//
// We negate both roll and pitch when publishing to convert from our
// internal convention to Webots convention.

#include "position_actor.h"
#include "types.h"
#include "config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include <math.h>

static bus_id s_state_bus;
static bus_id s_angle_setpoint_bus;
static bus_id s_target_bus;

void position_actor_init(bus_id state_bus, bus_id angle_setpoint_bus, bus_id target_bus) {
    s_state_bus = state_bus;
    s_angle_setpoint_bus = angle_setpoint_bus;
    s_target_bus = target_bus;
}

void position_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_state_bus);
    hive_bus_subscribe(s_target_bus);

    // Current target (updated from waypoint actor)
    position_target_t target = POSITION_TARGET_ZERO;
    int count = 0;

    while (1) {
        state_estimate_t state;
        position_target_t new_target;

        // Read target from waypoint actor
        if (BUS_READ(s_target_bus, &new_target)) {
            target = new_target;
        }

        if (BUS_READ(s_state_bus, &state)) {
            // Simple PD controller in world frame
            float x_error = target.x - state.x;
            float y_error = target.y - state.y;

            // Desired acceleration in world frame
            float accel_x = POS_KP * x_error - POS_KD * state.x_velocity;
            float accel_y = POS_KP * y_error - POS_KD * state.y_velocity;

            // Rotate from world frame to body frame based on current yaw
            // Body X (forward) = World X * cos(yaw) + World Y * sin(yaw)
            // Body Y (right)   = -World X * sin(yaw) + World Y * cos(yaw)
            float cos_yaw = cosf(state.yaw);
            float sin_yaw = sinf(state.yaw);

            float pitch_cmd = accel_x * cos_yaw + accel_y * sin_yaw;
            float roll_cmd  = -accel_x * sin_yaw + accel_y * cos_yaw;

            // Clamp to maximum tilt angle for safety
            pitch_cmd = CLAMPF(pitch_cmd, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
            roll_cmd  = CLAMPF(roll_cmd,  -MAX_TILT_ANGLE, MAX_TILT_ANGLE);

            // Sign conversion for Webots Crazyflie (matching Bitcraze):
            // - Roll is negated: positive body Y error → negative roll → +Y body accel
            // - Pitch is NOT negated: positive body X error → positive pitch → forward
            angle_setpoint_t setpoint = {
                .roll = -roll_cmd,
                .pitch = pitch_cmd,
                .yaw = target.yaw
            };
            hive_bus_publish(s_angle_setpoint_bus, &setpoint, sizeof(setpoint));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                HIVE_LOG_DEBUG("[POS] tgt=(%.1f,%.1f) x=%.2f y=%.2f pitch=%.1f roll=%.1f",
                               target.x, target.y, state.x, state.y,
                               pitch_cmd * RAD_TO_DEG, roll_cmd * RAD_TO_DEG);
            }
        }

        hive_yield();
    }
}
