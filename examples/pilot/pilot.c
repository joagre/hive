// clang-format off
// Pilot example - Quadcopter waypoint navigation using actor runtime
//
// Demonstrates waypoint navigation for a quadcopter using the hive actor
// runtime. Nine actors work together in a pipeline, supervised by one
// supervisor actor (10 actors total):
//
//   flight_manager  - Flight authority and safety monitoring
//   sensor_actor    - Reads raw sensors via HAL -> sensor bus
//   estimator_actor - Complementary filter fusion -> state bus
//   altitude_actor  - Altitude PID -> thrust command
//   waypoint_actor  - Waypoint manager -> position target bus
//   position_actor  - Position PD -> attitude setpoints
//   attitude_actor  - Attitude PIDs -> rate setpoints
//   rate_actor      - Rate PIDs -> torque commands
//   motor_actor     - Output to hardware via HAL
//
// Data flows through buses:
//
//   Sensor -> Sensor Bus -> Estimator -> State Bus ─┬-> Altitude -> Thrust Bus ───────────┐
//                                                   ├-> Position -> Attitude SP Bus -> Attitude │
//                                                   │                                 v         │
//                                                   └-> Rate <- Rate SP Bus <─────────┘         │
//                                                         v                                     │
//                                                   Torque Bus -> Motor <- Thrust Bus <─────────┘
//
// IPC coordination via name registry:
//   flight_manager, waypoint, altitude, motor register themselves
//   Uses hive_whereis() to look up actor IDs for IPC
//
// Supervision:
//   All 9 actors are supervised with ONE_FOR_ALL strategy.
//   If any actor crashes, all are restarted together.
//
// Hardware abstraction:
//   All hardware access goes through the HAL (hal/hal.h).
//   Supported platforms:
//     - hal/webots-crazyflie/ - Webots simulation
//     - hal/STEVAL-DRONE01/   - STM32 real hardware
// clang-format on

#include "hal/hal.h"
#include "hal_config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_actor.h"
#include "hive_supervisor.h"

#include "types.h"
#include "config.h"
#include "sensor_actor.h"
#include "estimator_actor.h"
#include "altitude_actor.h"
#include "waypoint_actor.h"
#include "position_actor.h"
#include "attitude_actor.h"
#include "rate_actor.h"
#include "motor_actor.h"
#include "flight_manager_actor.h"

#include <assert.h>

// Bus configuration from HAL (platform-specific)
#define PILOT_BUS_CONFIG HAL_BUS_CONFIG

// ============================================================================
// BUSES
// ============================================================================

static bus_id s_sensor_bus;
static bus_id s_state_bus;
static bus_id s_thrust_bus;
static bus_id s_position_target_bus;
static bus_id s_attitude_setpoint_bus;
static bus_id s_rate_setpoint_bus;
static bus_id s_torque_bus;

// ============================================================================
// SUPERVISOR CALLBACK
// ============================================================================

static void on_pipeline_shutdown(void *ctx) {
    (void)ctx;
    HIVE_LOG_WARN(
        "[PILOT] Pipeline supervisor shut down - max restarts exceeded");
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    // Initialize hardware via HAL
    if (hal_init() != 0) {
        return 1;
    }
    hal_calibrate();
    hal_arm();

    // Initialize actor runtime
    hive_init();

    // Create buses (single entry = latest value only)
    hive_bus_config cfg = PILOT_BUS_CONFIG;
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_sensor_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_state_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_thrust_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_position_target_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_attitude_setpoint_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_rate_setpoint_bus)));
    assert(HIVE_SUCCEEDED(hive_bus_create(&cfg, &s_torque_bus)));

    // Initialize actors with bus connections (no actor IDs - use name registry)
    flight_manager_actor_init();
    sensor_actor_init(s_sensor_bus);
    estimator_actor_init(s_sensor_bus, s_state_bus);
    waypoint_actor_init(s_state_bus, s_position_target_bus);
    altitude_actor_init(s_state_bus, s_thrust_bus, s_position_target_bus);
    position_actor_init(s_state_bus, s_attitude_setpoint_bus,
                        s_position_target_bus);
    attitude_actor_init(s_state_bus, s_attitude_setpoint_bus,
                        s_rate_setpoint_bus);
    rate_actor_init(s_state_bus, s_thrust_bus, s_rate_setpoint_bus,
                    s_torque_bus);
    motor_actor_init(s_torque_bus);

    // clang-format off
    // Define child specs for supervisor (9 actors)
    // Spawn order matters: flight_manager last so its whereis() targets are registered.
    // Control loop order: sensor -> estimator -> waypoint -> altitude ->
    //                     position -> attitude -> rate -> motor -> flight_manager
    hive_child_spec children[] = {
        {.id = "sensor",        .fn = sensor_actor,         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "sensor"}},
        {.id = "estimator",     .fn = estimator_actor,      .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "estimator"}},
        {.id = "waypoint",      .fn = waypoint_actor,       .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "waypoint"}},
        {.id = "altitude",      .fn = altitude_actor,       .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "altitude"}},
        {.id = "position",      .fn = position_actor,       .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "position"}},
        {.id = "attitude",      .fn = attitude_actor,       .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "attitude"}},
        {.id = "rate",          .fn = rate_actor,           .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "rate"}},
        {.id = "motor",         .fn = motor_actor,          .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "motor"}},
        {.id = "flight_manager",.fn = flight_manager_actor, .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "flight_mgr"}},
    };
    // clang-format on

    // Configure supervisor with ONE_FOR_ALL strategy:
    // If any actor crashes, all are killed and restarted together.
    // This ensures consistent pipeline state after recovery.
    hive_supervisor_config sup_cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ALL,
        .max_restarts = 3,
        .restart_period_ms = 10000,
        .children = children,
        .num_children = 9,
        .on_shutdown = on_pipeline_shutdown,
        .shutdown_ctx = NULL,
    };

    actor_id supervisor;
    hive_status status = hive_supervisor_start(&sup_cfg, NULL, &supervisor);
    assert(HIVE_SUCCEEDED(status));
    (void)supervisor;

    HIVE_LOG_INFO("10 actors spawned (9 children + 1 supervisor)");

    // Main loop - time control differs between real-time and simulation
#ifdef SIMULATED_TIME
    // Simulation: External loop advances time, then runs actors
    while (hal_step()) {
        hive_advance_time(HAL_TIME_STEP_US);
        hive_run_until_blocked();
    }
#else
    // Real-time: Scheduler runs event loop with hardware timers
    hive_run();
#endif

    // Cleanup
    hal_disarm();
    hive_cleanup();
    hal_cleanup();

    return 0;
}
