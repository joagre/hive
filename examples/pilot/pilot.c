// Pilot example - Quadcopter waypoint navigation using actor runtime
//
// Demonstrates waypoint navigation for a quadcopter using the hive actor
// runtime. Nine actors work together in a pipeline:
//
//   sensor_actor    - Reads raw sensors via HAL → sensor bus
//   estimator_actor - Complementary filter fusion → state bus
//   altitude_actor  - Altitude PID → thrust command
//   waypoint_actor  - Waypoint manager → position target bus
//   position_actor  - Position PD → attitude setpoints
//   attitude_actor  - Attitude PIDs → rate setpoints
//   rate_actor      - Rate PIDs → torque commands
//   motor_actor     - Output to hardware via HAL
//
// Data flows through buses:
//
//   Sensor → Sensor Bus → Estimator → State Bus ─┬→ Altitude → Thrust Bus ────────────┐
//                                                ├→ Position → Attitude SP Bus → Attitude │
//                                                │                                ↓        │
//                                                └→ Rate ← Rate SP Bus ←──────────┘        │
//                                                      ↓                                    │
//                                                Torque Bus → Motor ← Thrust Bus ←─────────┘
//
// Hardware abstraction:
//   All hardware access goes through the HAL (hal/hal.h).
//   Supported platforms:
//     - hal/webots-crazyflie/ - Webots simulation
//     - hal/STEVAL-DRONE01/   - STM32 real hardware

#include "hal/hal.h"
#include "hal_config.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_log.h"
#include "hive_actor.h"  // For HIVE_ACTOR_CONFIG_DEFAULT

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
#include "supervisor_actor.h"

#include <assert.h>

// ============================================================================
// HELPER MACROS
// ============================================================================

// Spawn an actor at specified priority. Assigns result to 'id'.
#define SPAWN_ACTOR(func, name_str, prio, id) do { \
    actor_config _cfg = HIVE_ACTOR_CONFIG_DEFAULT; \
    _cfg.priority = (prio); \
    _cfg.name = name_str; \
    hive_status _status = hive_spawn_ex(func, NULL, &_cfg, &id); \
    assert(HIVE_SUCCEEDED(_status)); \
} while (0)

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

    // Initialize actors with bus connections
    sensor_actor_init(s_sensor_bus);
    estimator_actor_init(s_sensor_bus, s_state_bus);
    position_actor_init(s_state_bus, s_attitude_setpoint_bus, s_position_target_bus);
    attitude_actor_init(s_state_bus, s_attitude_setpoint_bus, s_rate_setpoint_bus);
    rate_actor_init(s_state_bus, s_thrust_bus, s_rate_setpoint_bus, s_torque_bus);
    motor_actor_init(s_torque_bus);

    // Spawn order matters for IPC dependencies:
    // - Supervisor needs waypoint, altitude, and motor IDs (for START, LANDING, STOP)
    // - Altitude needs supervisor ID (for landing complete notification)
    //
    // All actors use CRITICAL priority. Execution order within a tick follows
    // spawn order (round-robin), which must match the data flow:
    // sensor → estimator → waypoint → altitude → position → attitude → rate → motor
    //
    // Note: Differentiated priorities (CRITICAL/HIGH/NORMAL) don't work here because
    // priority determines execution order, not importance. Higher priority actors
    // run first, which would break the pipeline (motor would run before controllers).
    actor_id sensor, estimator, altitude, waypoint, position, attitude, rate, motor, supervisor;
    SPAWN_ACTOR(sensor_actor,    "sensor",    HIVE_PRIORITY_CRITICAL, sensor);
    SPAWN_ACTOR(estimator_actor, "estimator", HIVE_PRIORITY_CRITICAL, estimator);

    // Spawn supervisor early (waypoint and altitude IDs filled in after)
    supervisor_actor_init(0, 0, motor);
    SPAWN_ACTOR(supervisor_actor, "supervisor", HIVE_PRIORITY_CRITICAL, supervisor);

    // Spawn waypoint before altitude/position (publishes position target they read)
    waypoint_actor_init(s_state_bus, s_position_target_bus);
    SPAWN_ACTOR(waypoint_actor,  "waypoint",  HIVE_PRIORITY_CRITICAL, waypoint);

    // Spawn altitude with supervisor ID
    altitude_actor_init(s_state_bus, s_thrust_bus, s_position_target_bus, supervisor);
    SPAWN_ACTOR(altitude_actor,  "altitude",  HIVE_PRIORITY_CRITICAL, altitude);

    // Update supervisor with waypoint and altitude IDs
    supervisor_actor_init(waypoint, altitude, motor);

    SPAWN_ACTOR(position_actor,  "position",  HIVE_PRIORITY_CRITICAL, position);
    SPAWN_ACTOR(attitude_actor,  "attitude",  HIVE_PRIORITY_CRITICAL, attitude);
    SPAWN_ACTOR(rate_actor,      "rate",      HIVE_PRIORITY_CRITICAL, rate);
    SPAWN_ACTOR(motor_actor,     "motor",     HIVE_PRIORITY_CRITICAL, motor);

    (void)sensor; (void)estimator; (void)altitude; (void)waypoint;
    (void)position; (void)attitude; (void)rate; (void)motor; (void)supervisor;

    HIVE_LOG_INFO("9 actors spawned");

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
