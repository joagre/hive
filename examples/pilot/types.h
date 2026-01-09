// Portable types for quadcopter control
//
// These data structures are hardware-independent and used by control actors.

#ifndef PILOT_TYPES_H
#define PILOT_TYPES_H

// Raw sensor data from IMU and GPS.
// On real hardware, this would be populated from SPI/I2C sensor drivers.
// For Webots, inertial_unit provides fused attitude (not truly raw).
// Note: gyro_x/y/z are body-frame rates, mapped to roll/pitch/yaw in estimator.
typedef struct {
    float roll, pitch, yaw;        // Euler angles in radians
    float gyro_x, gyro_y, gyro_z;  // Angular rates in rad/s (body frame: x=roll, y=pitch, z=yaw)
    float x, y;                    // GPS position (meters, world frame)
    float altitude;                // Height above ground in meters (Z)
} imu_data_t;

#define IMU_DATA_ZERO { \
    .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f, \
    .gyro_x = 0.0f, .gyro_y = 0.0f, .gyro_z = 0.0f, \
    .x = 0.0f, .y = 0.0f, .altitude = 0.0f \
}

// State estimate from estimator actor.
// Controllers use this instead of raw sensor data.
// Includes derived values like vertical velocity.
typedef struct {
    float roll, pitch, yaw;        // Attitude estimate (rad)
    float roll_rate;               // Roll rate (rad/s)
    float pitch_rate;              // Pitch rate (rad/s)
    float yaw_rate;                // Yaw rate (rad/s)
    float x, y;                    // Position estimate (m, world frame)
    float x_velocity, y_velocity;  // Horizontal velocity (m/s, world frame)
    float altitude;                // Altitude estimate (m)
    float vertical_velocity;       // Vertical velocity (m/s), positive = up
} state_estimate_t;

#define STATE_ESTIMATE_ZERO { \
    .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f, \
    .roll_rate = 0.0f, .pitch_rate = 0.0f, .yaw_rate = 0.0f, \
    .x = 0.0f, .y = 0.0f, .x_velocity = 0.0f, .y_velocity = 0.0f, \
    .altitude = 0.0f, .vertical_velocity = 0.0f \
}

// Motor commands as normalized values (0.0 to 1.0).
// The platform layer converts these to actual motor velocities.
typedef struct {
    float motor[4];  // [0]=M1(rear-left), [1]=M2(front-left), [2]=M3(front-right), [3]=M4(rear-right)
} motor_cmd_t;

// Zero motor command initializer
#define MOTOR_CMD_ZERO  {.motor = {0.0f, 0.0f, 0.0f, 0.0f}}

// Thrust command from altitude actor to attitude actor.
typedef struct {
    float thrust;  // Normalized thrust (0.0 to 1.0)
} thrust_cmd_t;

#define THRUST_CMD_ZERO {.thrust = 0.0f}

// Rate setpoint from angle actor to attitude actor.
// Attitude actor tracks these angular rates.
typedef struct {
    float roll;   // Target roll rate (rad/s)
    float pitch;  // Target pitch rate (rad/s)
    float yaw;    // Target yaw rate (rad/s)
} rate_setpoint_t;

#define RATE_SETPOINT_ZERO {.roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f}

// Angle setpoint from position actor to angle actor.
// Angle actor tracks these target angles.
typedef struct {
    float roll;   // Target roll angle (rad)
    float pitch;  // Target pitch angle (rad)
    float yaw;    // Target yaw angle (rad)
} angle_setpoint_t;

#define ANGLE_SETPOINT_ZERO {.roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f}

// Position target from waypoint actor to position and altitude actors.
// Position actor tracks x, y, yaw. Altitude actor tracks z.
typedef struct {
    float x, y;   // Target position (meters, world frame)
    float z;      // Target altitude (meters)
    float yaw;    // Target heading (radians)
} position_target_t;

#define POSITION_TARGET_ZERO {.x = 0.0f, .y = 0.0f, .z = 1.0f, .yaw = 0.0f}

// Torque command from attitude actor to motor actor.
// Motor actor applies mixer to convert to motor commands.
typedef struct {
    float thrust;  // Normalized thrust (0.0 to 1.0)
    float roll;    // Roll torque
    float pitch;   // Pitch torque
    float yaw;     // Yaw torque
} torque_cmd_t;

#define TORQUE_CMD_ZERO {.thrust = 0.0f, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f}

// PID controller state. Each axis (roll, pitch, yaw, altitude) has its own.
typedef struct {
    float kp, ki, kd;       // PID gains (proportional, integral, derivative)
    float integral;         // Accumulated integral term
    float prev_error;       // Previous error (for derivative calculation)
    float integral_max;     // Anti-windup: max absolute value of integral
    float output_max;       // Output clamping: max absolute value of output
} pid_state_t;

#endif // PILOT_TYPES_H
