// clang-format off
// Pilot example - Quadcopter waypoint navigation using actor runtime
//
// Demonstrates waypoint navigation for a quadcopter using the hive actor
// runtime. Nine flight actors work together in a pipeline, supervised by one
// supervisor actor. On platforms with radio (HAL_HAS_RADIO), a telemetry
// actor is also included (10-11 actors total):
//
//   sensor_actor    - Reads raw sensors via HAL -> sensor bus
//   estimator_actor - Complementary filter fusion -> state bus
//   altitude_actor  - Altitude PID -> thrust command
//   waypoint_actor  - Waypoint manager -> position target bus
//   position_actor  - Position PD -> attitude setpoints
//   attitude_actor  - Attitude PIDs -> rate setpoints
//   rate_actor      - Rate PIDs -> torque commands
//   motor_actor     - Output to hardware via HAL
//   flight_manager  - Flight authority and safety monitoring
//   telemetry_actor - Radio transmission of flight data (optional)
//
// Data flows through buses:
//
//   Sensor --> Sensor Bus --> Estimator --> State Bus
//                                              |
//        +------------------+------------------+
//        |                  |                  |
//        v                  v                  v
//    Waypoint           Altitude           Position
//        |                  |                  |
//        v                  v                  v
//   Pos Target Bus      Thrust Bus       Att SP Bus
//        |                  |                  |
//        +-------+----------+                  v
//                |                         Attitude
//                v                             |
//              Rate  <-------- Rate SP Bus <---+
//                |
//                v
//           Torque Bus --> Motor <-- Thrust Bus
//
// Actor initialization:
//   All actors receive pilot_buses via init_args
//   Each actor's init function extracts the buses it needs
//   Actors use hive_find_sibling() to look up sibling actor IDs for IPC
//
// Supervision:
//   All actors are supervised with ONE_FOR_ALL strategy.
//   If any flight-critical actor crashes, all are restarted together.
//   Telemetry uses TEMPORARY restart (not flight-critical, won't trigger restarts).
//
// Hardware abstraction:
//   All hardware access goes through the HAL (hal/hal.h).
//   Supported platforms:
//     - hal/webots-crazyflie/ - Webots simulation
//     - hal/crazyflie-2.1+/   - Crazyflie 2.1+ hardware
//     - hal/STEVAL-DRONE01/   - STEVAL-DRONE01 hardware
// clang-format on

#include "hal/hal.h"
#include "hal_config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_actor.h"
#include "hive_supervisor.h"

#include "pilot_buses.h"
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
#include "stack_profile.h"
#ifdef HAL_HAS_RADIO
#include "telemetry_actor.h"
#endif

// Bus configuration from HAL (platform-specific)
#define PILOT_BUS_CONFIG HAL_BUS_CONFIG

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
    // All actors share these via pilot_buses struct passed through init_args
    static pilot_buses buses;
    hive_bus_config cfg = PILOT_BUS_CONFIG;
    hive_status status;

    status = hive_bus_create(&cfg, &buses.sensor_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create sensor_bus: %s", HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.state_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create state_bus: %s", HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.thrust_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create thrust_bus: %s", HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.position_target_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create position_target_bus: %s",
                       HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.attitude_setpoint_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create attitude_setpoint_bus: %s",
                       HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.rate_setpoint_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create rate_setpoint_bus: %s",
                       HIVE_ERR_STR(status));
        return 1;
    }
    status = hive_bus_create(&cfg, &buses.torque_bus);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to create torque_bus: %s", HIVE_ERR_STR(status));
        return 1;
    }

    // clang-format off
    // Define child specs for supervisor (9 flight actors + optional telemetry)
    // Each actor's init function receives pilot_buses and extracts what it needs.
    // Control loop order: sensor -> estimator -> waypoint -> altitude ->
    //                     position -> attitude -> rate -> motor -> flight_manager
    // Telemetry runs at LOW priority and uses TEMPORARY restart.
    // clang-format on
    hive_child_spec children[] = {
        {.start = sensor_actor,
         .init = sensor_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "sensor",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "sensor"}},
        {.start = estimator_actor,
         .init = estimator_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "estimator",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "estimator"}},
        {.start = waypoint_actor,
         .init = waypoint_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "waypoint",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "waypoint"}},
        {.start = altitude_actor,
         .init = altitude_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "altitude",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "altitude"}},
        {.start = position_actor,
         .init = position_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "position",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "position"}},
        {.start = attitude_actor,
         .init = attitude_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "attitude",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "attitude"}},
        {.start = rate_actor,
         .init = rate_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "rate",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "rate"}},
        {.start = motor_actor,
         .init = motor_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "motor",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL, .name = "motor"}},
        {.start = flight_manager_actor,
         .init = flight_manager_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "flight_manager",
         .auto_register = true,
         .restart = HIVE_CHILD_TRANSIENT, // Normal exit = mission complete
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "flight_mgr"}},
#ifdef HAL_HAS_RADIO
        {.start = telemetry_actor,
         .init = telemetry_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "telemetry",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY, // Not flight-critical, don't restart
         .actor_cfg = {.priority = HIVE_PRIORITY_LOW, .name = "telemetry"}},
#endif
    };

    // Configure supervisor with ONE_FOR_ALL strategy:
    // If any actor crashes, all are killed and restarted together.
    // This ensures consistent pipeline state after recovery.
    // Note: telemetry is TEMPORARY so its exit won't trigger restarts.
    hive_supervisor_config sup_cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ALL,
        .max_restarts = 3,
        .restart_period_ms = 10000,
        .children = children,
        .num_children = sizeof(children) / sizeof(children[0]),
        .on_shutdown = on_pipeline_shutdown,
        .shutdown_ctx = NULL,
    };

    actor_id supervisor;
    status = hive_supervisor_start(&sup_cfg, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to start supervisor: %s", HIVE_ERR_STR(status));
        return 1;
    }
    (void)supervisor;

    // Log actor count (9 flight actors + optional telemetry + 1 supervisor)
#ifdef HAL_HAS_RADIO
    HIVE_LOG_INFO("11 actors spawned (10 children + 1 supervisor)");
#else
    HIVE_LOG_INFO("10 actors spawned (9 children + 1 supervisor)");
#endif

    // Main loop - time control differs between real-time and simulation
#ifdef SIMULATED_TIME
    // Simulation: External loop advances time, then runs actors
    while (hal_step()) {
        hive_advance_time(HAL_TIME_STEP_US);
        hive_run_until_blocked();

        // Check if stack profile report was requested (flight complete)
        if (stack_profile_check()) {
            break;
        }
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
