// Pilot example - Quadcopter hover using actor runtime with Webots
//
// This example demonstrates altitude-hold hover control for a Crazyflie
// quadcopter simulated in Webots. The code is organized for portability
// to real hardware - the control algorithms are hardware-independent,
// while sensor reading and motor control are isolated in a platform layer.
//
// Architecture:
//   1. PORTABLE TYPES - Data structures used by control code
//   2. PORTABLE CONTROL CODE - PID controllers and motor mixer
//   3. PLATFORM ABSTRACTION - Interface between actors and hardware
//   4. ATTITUDE ACTOR - The flight controller running as a hive actor
//   5. WEBOTS PLATFORM LAYER - Webots-specific sensor/motor code
//   6. MAIN LOOP - Integrates Webots simulation with hive runtime
//
// To port to real hardware, replace section 5 (Webots platform layer)
// with hardware-specific implementations.

// ============================================================================
// INCLUDES
// ============================================================================

// Webots API headers for robot simulation
#include <webots/robot.h>          // Core robot API (init, step, cleanup)
#include <webots/motor.h>          // Motor control (set velocity)
#include <webots/gyro.h>           // Gyroscope (angular rates)
#include <webots/inertial_unit.h>  // IMU (roll, pitch, yaw angles)
#include <webots/gps.h>            // GPS (position, including altitude)

// Hive actor runtime
#include "hive_runtime.h"  // Core runtime (init, spawn, step, cleanup)
#include "hive_bus.h"      // Publish-subscribe bus for sensor data

#include <stdio.h>  // printf for debug output

// ============================================================================
// CONFIGURATION
// ============================================================================

// Simulation timestep in milliseconds. This determines the control loop rate.
// 4ms = 250 Hz, which is typical for quadcopter flight controllers.
// Webots will simulate 4ms of physics per iteration.
#define TIME_STEP 4

// Maximum motor velocity in rad/s. This scales normalized motor commands
// (0.0-1.0) to actual velocities. The value was tuned for the Webots
// Crazyflie model to achieve hover at ~55% throttle.
#define MOTOR_MAX_VELOCITY 100.0f

// ============================================================================
// SECTION 1: PORTABLE TYPES
// These data structures are used by the control algorithms and are
// independent of any specific hardware or simulator.
// ============================================================================

// Sensor data from IMU and GPS, packaged for the control actor.
// On real hardware, this would be populated from SPI/I2C sensor drivers.
typedef struct {
    float roll, pitch, yaw;        // Euler angles in radians
    float gyro_x, gyro_y, gyro_z;  // Angular rates in rad/s (body frame)
    float altitude;                 // Height above ground in meters
} imu_data_t;

// Motor commands as normalized values (0.0 to 1.0).
// The platform layer converts these to actual motor velocities.
typedef struct {
    float motor[4];  // [0]=M1(front), [1]=M2(right), [2]=M3(rear), [3]=M4(left)
} motor_cmd_t;

// PID controller state. Each axis (roll, pitch, yaw, altitude) has its own.
typedef struct {
    float kp, ki, kd;       // PID gains (proportional, integral, derivative)
    float integral;         // Accumulated integral term
    float prev_error;       // Previous error (for derivative calculation)
    float integral_max;     // Anti-windup: max absolute value of integral
    float output_max;       // Output clamping: max absolute value of output
} pid_state_t;

// ============================================================================
// SECTION 2: PORTABLE CONTROL CODE
// These functions implement the control algorithms. They have no hardware
// dependencies and can be unit-tested independently.
// ============================================================================

// Initialize a PID controller with given gains.
// The integral and derivative limits use reasonable defaults.
static void pid_init(pid_state_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = 0.5f;  // Prevent integral windup
    pid->output_max = 1.0f;    // Full range by default
}

// Update PID controller and return control output.
//
// Arguments:
//   pid         - Controller state (modified: integral and prev_error updated)
//   setpoint    - Desired value (e.g., 0 for level flight, 1.0m for altitude)
//   measurement - Current sensor reading
//   dt          - Time step in seconds
//
// Returns:
//   Control output, clamped to [-output_max, +output_max]
//
// The derivative term uses error derivative (not measurement derivative),
// which is simpler but can cause "derivative kick" on setpoint changes.
// For hover with constant setpoint, this is not an issue.
static float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt) {
    // Error = how far we are from the desired value
    float error = setpoint - measurement;

    // Proportional term: immediate response proportional to error
    float p = pid->kp * error;

    // Integral term: accumulated error over time (eliminates steady-state error)
    // Anti-windup: clamp to prevent runaway integration
    pid->integral += error * dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i = pid->ki * pid->integral;

    // Derivative term: rate of change of error (provides damping)
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Sum all terms and clamp output
    float output = p + i + d;
    if (output > pid->output_max) output = pid->output_max;
    if (output < -pid->output_max) output = -pid->output_max;

    return output;
}

// Motor mixer for Crazyflie "+" configuration.
//
// Converts desired thrust and torques into individual motor commands.
// The Crazyflie uses a "+" configuration (not "X"):
//
//         Front
//           M1
//            |
//     M4 ----+---- M2
//            |
//           M3
//          Rear
//
// Sign convention (right-hand rule, Z-up):
//   - Positive roll torque  -> right side down -> M4 up, M2 down
//   - Positive pitch torque -> nose down       -> M3 up, M1 down
//   - Positive yaw torque   -> rotate CCW      -> M1,M3 up (CW motors)
//
// Motor rotation directions (for yaw control):
//   - M1 (front): CW  -> creates CCW reaction torque on body
//   - M2 (right): CCW -> creates CW reaction torque on body
//   - M3 (rear):  CW  -> creates CCW reaction torque on body
//   - M4 (left):  CCW -> creates CW reaction torque on body
static void mixer_update(float thrust, float roll, float pitch, float yaw, motor_cmd_t *cmd) {
    cmd->motor[0] = thrust - pitch - yaw;  // M1 (front): pitch down & yaw CCW -> slower
    cmd->motor[1] = thrust - roll  + yaw;  // M2 (right): roll right & yaw CW -> slower
    cmd->motor[2] = thrust + pitch - yaw;  // M3 (rear):  pitch down & yaw CCW -> faster
    cmd->motor[3] = thrust + roll  + yaw;  // M4 (left):  roll right & yaw CW -> faster

    // Clamp motor commands to valid range [0, 1]
    for (int i = 0; i < 4; i++) {
        if (cmd->motor[i] < 0.0f) cmd->motor[i] = 0.0f;
        if (cmd->motor[i] > 1.0f) cmd->motor[i] = 1.0f;
    }
}

// ============================================================================
// SECTION 3: PLATFORM ABSTRACTION
// This provides a clean interface between actors and hardware.
// Motor commands go through a global variable because the main loop
// (which writes to Webots motors) runs outside actor context.
// ============================================================================

// Global motor command storage. The attitude actor writes here,
// and the main loop reads from here to send to Webots.
static motor_cmd_t g_motor_cmd;

// Flag indicating new motor commands are available.
// Volatile because it's written by actor and read by main loop.
// (In single-threaded code this is not strictly necessary, but
// it documents the intent and prevents optimizer issues.)
static volatile int g_motor_cmd_ready = 0;

// Called by the attitude actor to output motor commands.
// The commands are stored globally for the main loop to apply.
static void motor_output_set(const motor_cmd_t *cmd) {
    g_motor_cmd = *cmd;
    g_motor_cmd_ready = 1;
}

// ============================================================================
// SECTION 4: ATTITUDE ACTOR
// This is the flight controller, implemented as a hive actor.
// It subscribes to sensor data via the bus, runs PID controllers,
// and outputs motor commands.
// ============================================================================

// Bus ID for sensor data. Set during initialization in main().
static bus_id g_imu_bus;

// The attitude actor function. This runs as a hive actor, called
// once per hive_step() invocation (i.e., once per simulation step).
//
// Control architecture:
//   - Inner loop: Rate control (gyro feedback) for roll, pitch, yaw
//   - Outer loop: Altitude control (GPS feedback) for thrust
//
// The rate controllers stabilize angular rates to zero (level flight).
// The altitude controller adjusts thrust to maintain target height.
void attitude_actor(void *arg) {
    (void)arg;  // Unused parameter

    // Subscribe to the IMU bus to receive sensor data
    hive_bus_subscribe(g_imu_bus);

    // ========================================================================
    // PID CONTROLLER INITIALIZATION
    // ========================================================================

    pid_state_t roll_pid, pitch_pid, yaw_pid, alt_pid;

    // Rate controllers for attitude stabilization.
    // These use gyro feedback to keep angular rates at zero.
    // Gains are conservative - higher gains give faster response but
    // risk oscillation. These were tuned empirically for the Webots model.
    pid_init(&roll_pid,  0.02f, 0.0f, 0.001f);   // Roll rate control
    pid_init(&pitch_pid, 0.02f, 0.0f, 0.001f);   // Pitch rate control
    pid_init(&yaw_pid,   0.02f, 0.0f, 0.001f);   // Yaw rate control

    // Limit attitude corrections to ±10-15% of thrust.
    // This prevents aggressive maneuvers that could destabilize the drone.
    roll_pid.output_max = 0.1f;
    pitch_pid.output_max = 0.1f;
    yaw_pid.output_max = 0.15f;  // Yaw needs more authority

    // Altitude controller.
    // This uses GPS Z feedback to maintain target height.
    // Higher gains than rate controllers because altitude changes slowly.
    pid_init(&alt_pid, 0.3f, 0.05f, 0.15f);
    alt_pid.output_max = 0.15f;   // Limit thrust adjustment to ±15%
    alt_pid.integral_max = 0.2f;  // Prevent excessive integral buildup

    // ========================================================================
    // CONTROL PARAMETERS
    // ========================================================================

    // Time step for PID calculations (must match TIME_STEP)
    const float dt = 0.004f;  // 4ms = 0.004s

    // Base thrust for hover. This was determined experimentally:
    // - Too low: drone sinks
    // - Too high: drone rises
    // - Just right: drone hovers (altitude PID makes small corrections)
    const float base_thrust = 0.553f;

    // Target altitude in meters
    const float target_altitude = 1.0f;

    // Counter for periodic debug output
    int count = 0;

    // ========================================================================
    // MAIN CONTROL LOOP
    // ========================================================================

    while (1) {
        imu_data_t imu;
        size_t len;

        // Read latest sensor data from the bus.
        // This is non-blocking - if no data available, skip this iteration.
        if (hive_bus_read(g_imu_bus, &imu, sizeof(imu), &len).code == HIVE_OK) {

            // ================================================================
            // RATE CONTROL (inner loop)
            // ================================================================
            // The setpoint is 0 for all axes (level flight, no rotation).
            // The gyro values are negated to match the control sign convention:
            // positive gyro_x means rolling right, so we need negative
            // roll_torque to correct (roll left).

            float roll_torque  = pid_update(&roll_pid,  0.0f, -imu.gyro_x, dt);
            float pitch_torque = pid_update(&pitch_pid, 0.0f, -imu.gyro_y, dt);
            float yaw_torque   = pid_update(&yaw_pid,   0.0f, -imu.gyro_z, dt);

            // ================================================================
            // ALTITUDE CONTROL (outer loop)
            // ================================================================
            // Compares current altitude to target and adjusts thrust.
            // Positive error (below target) -> positive correction -> more thrust

            float alt_correction = pid_update(&alt_pid, target_altitude, imu.altitude, dt);
            float thrust = base_thrust + alt_correction;

            // Clamp thrust to valid range
            if (thrust < 0.0f) thrust = 0.0f;
            if (thrust > 1.0f) thrust = 1.0f;

            // ================================================================
            // MOTOR MIXING
            // ================================================================
            // Convert thrust and torques to individual motor commands

            motor_cmd_t cmd;
            mixer_update(thrust, roll_torque, pitch_torque, yaw_torque, &cmd);

            // Output commands (stored in global for main loop to apply)
            motor_output_set(&cmd);

            // ================================================================
            // DEBUG OUTPUT
            // ================================================================
            // Print status every 250 iterations (once per second at 250 Hz)

            if (++count % 250 == 0) {
                printf("alt=%.2f thrust=%.2f | roll=%5.1f pitch=%5.1f\n",
                       imu.altitude, thrust,
                       imu.roll * 57.3f,   // Convert radians to degrees
                       imu.pitch * 57.3f);
            }
        }

        // Yield control back to the scheduler.
        // This is required for cooperative multitasking.
        // The actor will resume on the next hive_step() call.
        hive_yield();
    }
}

// ============================================================================
// SECTION 5: WEBOTS PLATFORM LAYER
// This section contains all Webots-specific code. To port to real hardware,
// replace these functions with hardware drivers.
// ============================================================================

// Webots device handles (like file descriptors for hardware)
static WbDeviceTag motors[4];  // Motor devices
static WbDeviceTag gyro_dev;   // Gyroscope
static WbDeviceTag imu_dev;    // Inertial measurement unit
static WbDeviceTag gps_dev;    // GPS (used for altitude)

// Initialize Webots devices.
// Returns 0 on success, -1 on failure.
static int platform_init(void) {
    // Initialize Webots robot API
    wb_robot_init();

    // ========================================================================
    // MOTOR INITIALIZATION
    // ========================================================================
    // Find all four motors by name and configure for velocity control.

    const char *motor_names[] = {"m1_motor", "m2_motor", "m3_motor", "m4_motor"};
    for (int i = 0; i < 4; i++) {
        motors[i] = wb_robot_get_device(motor_names[i]);
        if (motors[i] == 0) {
            printf("Error: motor %s not found\n", motor_names[i]);
            return -1;
        }
        // Set position to infinity for velocity control mode.
        // This tells Webots we want to control velocity, not position.
        wb_motor_set_position(motors[i], INFINITY);
        wb_motor_set_velocity(motors[i], 0.0);  // Start with motors off
    }

    // ========================================================================
    // SENSOR INITIALIZATION
    // ========================================================================

    gyro_dev = wb_robot_get_device("gyro");
    imu_dev = wb_robot_get_device("inertial_unit");
    gps_dev = wb_robot_get_device("gps");

    if (gyro_dev == 0 || imu_dev == 0 || gps_dev == 0) {
        printf("Error: sensors not found\n");
        return -1;
    }

    // Enable sensors with the simulation timestep.
    // Sensors only provide data if enabled.
    wb_gyro_enable(gyro_dev, TIME_STEP);
    wb_inertial_unit_enable(imu_dev, TIME_STEP);
    wb_gps_enable(gps_dev, TIME_STEP);

    return 0;
}

// Read all sensors and populate the imu_data_t structure.
// This abstracts the Webots API into our portable data format.
static void platform_read_imu(imu_data_t *imu) {
    // Get gyroscope values (angular rates in rad/s)
    const double *g = wb_gyro_get_values(gyro_dev);

    // Get attitude angles (roll, pitch, yaw in radians)
    const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_dev);

    // Get position (X, Y, Z in meters - we use Z for altitude)
    const double *pos = wb_gps_get_values(gps_dev);

    // Copy to our portable structure
    imu->roll = rpy[0];
    imu->pitch = rpy[1];
    imu->yaw = rpy[2];
    imu->gyro_x = g[0];
    imu->gyro_y = g[1];
    imu->gyro_z = g[2];
    imu->altitude = pos[2];  // Z coordinate is altitude in Webots
}

// Apply motor commands to Webots motors.
// This reads from the global g_motor_cmd set by the attitude actor.
static void platform_write_motors(void) {
    if (g_motor_cmd_ready) {
        // Motor velocity signs for Webots Crazyflie to cancel reaction torque.
        // This was determined experimentally:
        // - Without sign correction: drone spins rapidly due to unbalanced torque
        // - With these signs: torques cancel and drone hovers stably
        //
        // M1, M3 (front, rear): negative velocity
        // M2, M4 (right, left): positive velocity
        static const float signs[4] = {-1.0f, 1.0f, -1.0f, 1.0f};

        for (int i = 0; i < 4; i++) {
            // Convert normalized command (0-1) to velocity (rad/s)
            // and apply sign correction
            wb_motor_set_velocity(motors[i],
                signs[i] * g_motor_cmd.motor[i] * MOTOR_MAX_VELOCITY);
        }
    }
}

// ============================================================================
// SECTION 6: MAIN LOOP
// This integrates the Webots simulation with the hive actor runtime.
// The key insight is that Webots controls time (via wb_robot_step),
// so we use hive_step() instead of hive_run() to run actors once per
// simulation step.
// ============================================================================

int main(void) {
    // Initialize Webots platform (sensors and motors)
    if (platform_init() < 0) return 1;

    // Initialize hive actor runtime
    hive_init();

    // Create a bus for sensor data.
    // max_entries=1 means only the latest value is kept (no history).
    // This is appropriate for sensor data where only current values matter.
    hive_bus_config cfg = HIVE_BUS_CONFIG_DEFAULT;
    cfg.max_entries = 1;
    hive_bus_create(&cfg, &g_imu_bus);

    // Spawn the attitude control actor.
    // It will start running on the first hive_step() call.
    actor_id attitude;
    hive_spawn(attitude_actor, NULL, &attitude);

    printf("Pilot started - hover mode\n");

    // ========================================================================
    // MAIN SIMULATION LOOP
    // ========================================================================
    // This loop runs once per simulation timestep (4ms).
    // wb_robot_step() blocks until Webots has simulated TIME_STEP milliseconds.

    while (wb_robot_step(TIME_STEP) != -1) {

        // Step 1: Read sensors from Webots and publish to bus.
        // The attitude actor will read this data.
        imu_data_t imu;
        platform_read_imu(&imu);
        hive_bus_publish(g_imu_bus, &imu, sizeof(imu));

        // Step 2: Run all ready actors exactly once.
        // This is different from hive_run() which blocks until all actors exit.
        // hive_step() returns immediately after running each actor once.
        hive_step();

        // Step 3: Apply motor commands to Webots.
        // The attitude actor has computed new commands in step 2.
        platform_write_motors();
    }

    // ========================================================================
    // CLEANUP
    // ========================================================================
    // The loop exits when the user stops the simulation in Webots.

    hive_cleanup();
    wb_robot_cleanup();
    return 0;
}
