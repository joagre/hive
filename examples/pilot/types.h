// Portable types for quadcopter control
//
// These data structures are hardware-independent and used by control actors.

#ifndef PILOT_TYPES_H
#define PILOT_TYPES_H

// Sensor data from IMU and GPS, packaged for control actors.
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

// Zero motor command initializer
#define MOTOR_CMD_ZERO  {.motor = {0.0f, 0.0f, 0.0f, 0.0f}}

// Thrust command from altitude actor to attitude actor.
typedef struct {
    float thrust;  // Normalized thrust (0.0 to 1.0)
} thrust_cmd_t;

// PID controller state. Each axis (roll, pitch, yaw, altitude) has its own.
typedef struct {
    float kp, ki, kd;       // PID gains (proportional, integral, derivative)
    float integral;         // Accumulated integral term
    float prev_error;       // Previous error (for derivative calculation)
    float integral_max;     // Anti-windup: max absolute value of integral
    float output_max;       // Output clamping: max absolute value of output
} pid_state_t;

#endif // PILOT_TYPES_H
