// PID controller
//
// Generic discrete PID controller with anti-windup and output clamping.

#ifndef PID_H
#define PID_H

#include "types.h"

// Initialize a PID controller with given gains.
// Sets default limits: integral_max=0.5, output_max=1.0
void pid_init(pid_state_t *pid, float kp, float ki, float kd);

// Initialize a PID controller with all parameters.
void pid_init_full(pid_state_t *pid, float kp, float ki, float kd,
                   float integral_max, float output_max);

// Reset PID state (integral and previous error).
void pid_reset(pid_state_t *pid);

// Update PID controller and return control output.
//
// Arguments:
//   pid         - Controller state (modified: integral and prev_error updated)
//   setpoint    - Desired value
//   measurement - Current sensor reading
//   dt          - Time step in seconds
//
// Returns:
//   Control output, clamped to [-output_max, +output_max]
float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt);

// Update PID controller for angular quantities (handles wrap-around at +/-PI).
//
// Same as pid_update but normalizes error to [-PI, PI] range to ensure
// the controller takes the shortest path around the circle.
float pid_update_angle(pid_state_t *pid, float setpoint, float measurement,
                       float dt);

// Update PID gains without resetting integrator state.
// Allows smooth gain changes during flight.
void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd);

// Update PID limits without resetting integrator state.
void pid_set_limits(pid_state_t *pid, float imax, float omax);

#endif // PID_H
