// PID controller implementation

#include "pid.h"
#include "math_utils.h"
#include "hive_log.h"

void pid_init(pid_state_t *pid, float kp, float ki, float kd) {
    pid_init_full(pid, kp, ki, kd, 0.5f, 1.0f);
}

void pid_init_full(pid_state_t *pid, float kp, float ki, float kd,
                   float integral_max, float output_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_max = integral_max;
    pid->output_max = output_max;
}

void pid_reset(pid_state_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// Minimum dt to avoid division by zero or extreme derivative spikes
// 0.1ms is well below any realistic control loop period
#define PID_MIN_DT 0.0001f

static float pid_update_internal(pid_state_t *pid, float error, float dt) {
    // Proportional
    float p = pid->kp * error;

    // Guard against bad dt (zero, negative, or extremely small)
    if (dt < PID_MIN_DT) {
        // Skip integral and derivative, return P-only output
        HIVE_LOG_WARN("[PID] bad dt=%.6f, using P-only", dt);
        return CLAMPF(p, -pid->output_max, pid->output_max);
    }

    // Integral with anti-windup
    pid->integral += error * dt;
    pid->integral =
        CLAMPF(pid->integral, -pid->integral_max, pid->integral_max);
    float i = pid->ki * pid->integral;

    // Derivative
    float d = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // Sum and clamp
    float output = p + i + d;
    return CLAMPF(output, -pid->output_max, pid->output_max);
}

float pid_update(pid_state_t *pid, float setpoint, float measurement,
                 float dt) {
    float error = setpoint - measurement;
    return pid_update_internal(pid, error, dt);
}

float pid_update_angle(pid_state_t *pid, float setpoint, float measurement,
                       float dt) {
    float error = normalize_angle(setpoint - measurement);
    return pid_update_internal(pid, error, dt);
}

void pid_set_gains(pid_state_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    // Note: Integrator state preserved for smooth transitions
}

void pid_set_limits(pid_state_t *pid, float imax, float omax) {
    pid->integral_max = imax;
    pid->output_max = omax;
    // Re-clamp integrator if needed
    pid->integral = CLAMPF(pid->integral, -imax, imax);
}
