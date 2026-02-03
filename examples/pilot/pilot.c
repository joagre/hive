// Pilot - Quadcopter autopilot using the Hive actor runtime
//
// 11 actors in a supervised control pipeline: sensor fusion, cascaded PID
// control (altitude/position/attitude/rate), and telemetry logging.
//
// See README.md for architecture, data flow, and build instructions.

#include "hal/hal.h"
#include "hal_config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
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
#include "comms_actor.h"
#endif
#include "telemetry_logger_actor.h"

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

#ifdef HAL_HAS_RADIO
    // Initialize ESB radio (blocks until nRF51 responds or timeout)
    if (hal_esb_init() != 0) {
        return 1;
    }
    hal_printf("[PILOT] Radio ready, battery=%.2fV\n", hal_power_get_battery());
#endif

    if (!hal_self_test()) {
        hal_cleanup();
        return 1;
    }

    // Grace period - 15 seconds to step back and place drone
    hal_printf(
        "[PILOT] 15 second grace period - place drone on level surface\n");
    for (int i = 15; i > 0; i--) {
        hal_printf("[PILOT] %2d seconds...\n", i);
        hal_led_on();
        hal_delay_ms(500);
        hal_led_off();
        hal_delay_ms(500);
    }
    hal_printf("[PILOT] Starting calibration - KEEP STILL!\n");

    hal_calibrate();
    hal_arm();

    // Initialize actor runtime
    hive_init();

    // Open log file early to capture HAL init messages
    // Try each mount in order until one succeeds
    const char *log_paths[] = {"/sd/hive.log", "/log", "/tmp/hive.log", NULL};
    for (int i = 0; log_paths[i] != NULL; i++) {
        hive_status_t log_status = hive_log_file_open(log_paths[i]);
        if (HIVE_SUCCEEDED(log_status)) {
            hal_flush_early_log();
            hal_printf("[PILOT] Log file: %s\n", log_paths[i]);
            break;
        }
    }

    // Create buses (single entry = latest value only)
    // All actors share these via pilot_buses_t struct passed through init_args
    static pilot_buses_t buses;
    hive_bus_config_t cfg = PILOT_BUS_CONFIG;
    hive_status_t status;

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

    // Telemetry logger configuration
    static telemetry_logger_config_t tlog_config = {
        .buses = &buses,
    };

    // clang-format off
    // Define child specs for supervisor (9 flight actors + optional comms)
    // Each actor's init function receives pilot_buses_t and extracts what it needs.
    // Control loop order: sensor -> estimator -> waypoint -> altitude ->
    //                     position -> attitude -> rate -> motor -> flight_manager
    // Comms runs at LOW priority and uses TEMPORARY restart.
    // clang-format on
    hive_child_spec_t children[] = {
        {.start = sensor_actor,
         .init = sensor_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "sensor",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "sensor",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = estimator_actor,
         .init = estimator_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "estimator",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "estimator",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = waypoint_actor,
         .init = waypoint_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "waypoint",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "waypoint",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = altitude_actor,
         .init = altitude_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "altitude",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "altitude",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = position_actor,
         .init = position_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "position",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "position",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = attitude_actor,
         .init = attitude_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "attitude",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "attitude",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = rate_actor,
         .init = rate_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "rate",
         .auto_register = false,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "rate",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = motor_actor,
         .init = motor_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "motor",
         .auto_register = true,
         .restart = HIVE_CHILD_PERMANENT,
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "motor",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
        {.start = flight_manager_actor,
         .init = flight_manager_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "flight_manager",
         .auto_register = true,
         .restart = HIVE_CHILD_TRANSIENT, // Normal exit = mission complete
         .actor_cfg = {.priority = HIVE_PRIORITY_CRITICAL,
                       .name = "flight_mgr",
                       .pool_block =
                           false}}, // CRITICAL: never block control loop
#ifdef HAL_HAS_RADIO
        {.start = comms_actor,
         .init = comms_actor_init,
         .init_args = &buses,
         .init_args_size = sizeof(buses),
         .name = "comms",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY,
         .actor_cfg = {.priority = HIVE_PRIORITY_LOW,
                       .name = "comms",
                       .pool_block = true}},
#endif
        {.start = telemetry_logger_actor,
         .init = telemetry_logger_init,
         .init_args = &tlog_config,
         .init_args_size = sizeof(tlog_config),
         .name = "tlog",
         .auto_register = false,
         .restart = HIVE_CHILD_TEMPORARY, // Not flight-critical, don't restart
         .actor_cfg = {.priority = HIVE_PRIORITY_LOW,
                       .name = "tlog",
                       .pool_block = true}},
    };

    // Configure supervisor with ONE_FOR_ALL strategy:
    // If any actor crashes, all are killed and restarted together.
    // This ensures consistent pipeline state after recovery.
    // Note: comms is TEMPORARY so its exit won't trigger restarts.
    // First flight safety: max_restarts=0 means any crash triggers immediate
    // shutdown and landing. No restart attempts that could cause erratic behavior.
    hive_supervisor_config_t sup_cfg = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ALL,
        .max_restarts = 0,
        .restart_period_ms = 10000,
        .children = children,
        .num_children = sizeof(children) / sizeof(children[0]),
        .on_shutdown = on_pipeline_shutdown,
        .shutdown_ctx = NULL,
    };

    hive_actor_id_t supervisor;
    status = hive_supervisor_start(&sup_cfg, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("Failed to start supervisor: %s", HIVE_ERR_STR(status));
        return 1;
    }
    (void)supervisor;

    // Log actor count (9 flight actors + tlog + optional comms + 1 supervisor)
#ifdef HAL_HAS_RADIO
    HIVE_LOG_INFO("12 actors spawned (11 children + 1 supervisor)");
#else
    HIVE_LOG_INFO("11 actors spawned (10 children + 1 supervisor)");
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
    hive_log_file_close();
    hal_disarm();
    hive_cleanup();
    hal_cleanup();

    return 0;
}
