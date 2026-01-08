// PID controller implementation

#include "pid.h"
#include "config.h"

void pid_init(pid_state_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = 0.5f;
    pid->output_max = 1.0f;
}

void pid_reset(pid_state_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_state_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;

    // Proportional
    float p = pid->kp * error;

    // Integral with anti-windup
    pid->integral += error * dt;
    pid->integral = CLAMPF(pid->integral, -pid->integral_max, pid->integral_max);
    float i = pid->ki * pid->integral;

    // Derivative
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Sum and clamp
    float output = p + i + d;
    return CLAMPF(output, -pid->output_max, pid->output_max);
}
