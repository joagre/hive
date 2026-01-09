// Portable types for quadcopter control
//
// These data structures are hardware-independent and used by control actors.

#ifndef PILOT_TYPES_H
#define PILOT_TYPES_H

// Raw sensor data from IMU and GPS.
// On real hardware, this would be populated from SPI/I2C sensor drivers.
// For Webots, inertial_unit provides fused attitude (not truly raw).
typedef struct {
    float roll, pitch, yaw;        // Euler angles in radians
    float gyro_x, gyro_y, gyro_z;  // Angular rates in rad/s (body frame)
    float altitude;                 // Height above ground in meters
} imu_data_t;

// State estimate from estimator actor.
// Controllers use this instead of raw sensor data.
// Includes derived values like vertical velocity.
typedef struct {
    float roll, pitch, yaw;        // Attitude estimate (rad)
    float roll_rate;               // Roll rate (rad/s)
    float pitch_rate;              // Pitch rate (rad/s)
    float yaw_rate;                // Yaw rate (rad/s)
    float altitude;                // Altitude estimate (m)
    float vertical_velocity;       // Vertical velocity (m/s), positive = up
} state_estimate_t;

#define STATE_ESTIMATE_ZERO { \
    .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f, \
    .roll_rate = 0.0f, .pitch_rate = 0.0f, .yaw_rate = 0.0f, \
    .altitude = 0.0f, .vertical_velocity = 0.0f \
}

// Motor commands as normalized values (0.0 to 1.0).
// The platform layer converts these to actual motor velocities.
typedef struct {
    float motor[4];  // [0]=M1(front), [1]=M2(right), [2]=M3(rear), [3]=M4(left)
} motor_cmd_t;

// Zero motor command initializer
#define MOTOR_CMD_ZERO  {.motor = {0.0f, 0.0f, 0.0f, 0.0f}}

// Thrust command from altitude actor to attitude actor.
typedef struct {
    float thrust;  // Normalized thrust (0.0 to 1.0)
} thrust_cmd_t;

// Rate setpoint from angle actor to attitude actor.
// Attitude actor tracks these angular rates.
typedef struct {
    float roll;   // Target roll rate (rad/s)
    float pitch;  // Target pitch rate (rad/s)
    float yaw;    // Target yaw rate (rad/s)
} rate_setpoint_t;

#define RATE_SETPOINT_ZERO {.roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f}

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
