// Altitude actor - Altitude hold control
//
// Subscribes to state and target buses, runs altitude PI with velocity damping,
// publishes thrust commands.
//
// Uses measured vertical velocity for damping instead of differentiating
// the position error. This provides smoother response with less noise.

#include "altitude_actor.h"
#include "types.h"
#include "config.h"
#include "pid.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include <stdio.h>

static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_target_bus;

void altitude_actor_init(bus_id state_bus, bus_id thrust_bus, bus_id target_bus) {
    s_state_bus = state_bus;
    s_thrust_bus = thrust_bus;
    s_target_bus = target_bus;
}

void altitude_actor(void *arg) {
    (void)arg;

    hive_bus_subscribe(s_state_bus);
    hive_bus_subscribe(s_target_bus);

    pid_state_t alt_pid;
    pid_init_full(&alt_pid, ALT_PID_KP, ALT_PID_KI, ALT_PID_KD, ALT_PID_IMAX, ALT_PID_OMAX);

    // Target altitude (updated from waypoint actor)
    float target_altitude = 1.0f;  // Default to 1m
    int count = 0;

    while (1) {
        state_estimate_t state;
        position_target_t target;

        // Read target altitude from waypoint actor
        if (BUS_READ(s_target_bus, &target)) {
            target_altitude = target.z;
        }

        if (BUS_READ(s_state_bus, &state)) {
            // Position control (PI)
            float pos_correction = pid_update(&alt_pid, target_altitude, state.altitude, TIME_STEP_S);

            // Velocity damping: reduce thrust when moving up, increase when moving down
            float vel_damping = -VVEL_DAMPING_GAIN * state.vertical_velocity;

            float thrust = CLAMPF(BASE_THRUST + pos_correction + vel_damping, 0.0f, 1.0f);

            thrust_cmd_t cmd = {.thrust = thrust};
            hive_bus_publish(s_thrust_bus, &cmd, sizeof(cmd));

            if (++count % DEBUG_PRINT_INTERVAL == 0) {
                printf("[ALT] tgt=%.2f alt=%.2f vvel=%.2f thrust=%.3f\n",
                       target_altitude, state.altitude, state.vertical_velocity, thrust);
            }
        }

        hive_yield();
    }
}
