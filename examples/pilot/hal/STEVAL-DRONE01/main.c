// Example main.c for STEVAL-DRONE01
//
// Demonstrates how to use the platform with a simple attitude stabilization
// controller. This is a minimal example - a full implementation would include
// waypoint navigation, position control, and RC input handling.
//
// Build with STM32CubeIDE or Makefile targeting STM32F401.

#include "platform.h"
#include <math.h>

// TODO: Include STM32 HAL for GPIO (LED, button)
// #include "stm32f4xx_hal.h"

// ----------------------------------------------------------------------------
// Configuration
// ----------------------------------------------------------------------------

// Attitude PID gains (tune these for your drone!)
#define ROLL_KP     2.0f
#define ROLL_KI     0.0f
#define ROLL_KD     0.3f

#define PITCH_KP    2.0f
#define PITCH_KI    0.0f
#define PITCH_KD    0.3f

#define YAW_KP      1.0f
#define YAW_KI      0.0f
#define YAW_KD      0.1f

// Altitude PID gains
#define ALT_KP      0.5f
#define ALT_KI      0.1f
#define ALT_KD      0.2f

// Target setpoints
#define TARGET_ALTITUDE     1.0f    // meters
#define TARGET_ROLL         0.0f    // radians (level)
#define TARGET_PITCH        0.0f    // radians (level)
#define TARGET_YAW          0.0f    // radians (north)

// Motor mixing
#define THROTTLE_HOVER      0.5f    // Base throttle for hover (tune this!)
#define CONTROL_AUTHORITY   0.3f    // Max control adjustment

// ----------------------------------------------------------------------------
// PID Controller
// ----------------------------------------------------------------------------

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float integral_limit;
} pid_t;

static void pid_init(pid_t *pid, float kp, float ki, float kd, float i_limit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral_limit = i_limit;
}

static float pid_update(pid_t *pid, float error, float dt) {
    // Proportional
    float p = pid->kp * error;

    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i = pid->ki * pid->integral;

    // Derivative
    float derivative = (error - pid->prev_error) / dt;
    float d = pid->kd * derivative;
    pid->prev_error = error;

    return p + i + d;
}

static void pid_reset(pid_t *pid) {
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

// ----------------------------------------------------------------------------
// Flight Controller State
// ----------------------------------------------------------------------------

static pid_t s_roll_pid;
static pid_t s_pitch_pid;
static pid_t s_yaw_pid;
static pid_t s_alt_pid;

static bool s_flying = false;
static float s_target_altitude = TARGET_ALTITUDE;
static float s_target_yaw = TARGET_YAW;

// ----------------------------------------------------------------------------
// Motor Mixing (X configuration)
// ----------------------------------------------------------------------------
//
//         Front
//       M2    M3
//         \  /
//          \/
//          /\
//         /  \
//       M1    M4
//         Rear
//
// M1 (rear-left):   CCW, +roll, +pitch, -yaw
// M2 (front-left):  CW,  +roll, -pitch, +yaw
// M3 (front-right): CCW, -roll, -pitch, -yaw
// M4 (rear-right):  CW,  -roll, +pitch, +yaw

static void mix_motors(float throttle, float roll, float pitch, float yaw,
                       platform_motors_t *motors) {
    // Limit control inputs
    if (roll > CONTROL_AUTHORITY) roll = CONTROL_AUTHORITY;
    if (roll < -CONTROL_AUTHORITY) roll = -CONTROL_AUTHORITY;
    if (pitch > CONTROL_AUTHORITY) pitch = CONTROL_AUTHORITY;
    if (pitch < -CONTROL_AUTHORITY) pitch = -CONTROL_AUTHORITY;
    if (yaw > CONTROL_AUTHORITY) yaw = CONTROL_AUTHORITY;
    if (yaw < -CONTROL_AUTHORITY) yaw = -CONTROL_AUTHORITY;

    // Mix for X configuration
    motors->m1 = throttle + roll + pitch - yaw;  // Rear-left, CCW
    motors->m2 = throttle + roll - pitch + yaw;  // Front-left, CW
    motors->m3 = throttle - roll - pitch - yaw;  // Front-right, CCW
    motors->m4 = throttle - roll + pitch + yaw;  // Rear-right, CW

    // Clamp to valid range
    if (motors->m1 < 0.0f) motors->m1 = 0.0f;
    if (motors->m1 > 1.0f) motors->m1 = 1.0f;
    if (motors->m2 < 0.0f) motors->m2 = 0.0f;
    if (motors->m2 > 1.0f) motors->m2 = 1.0f;
    if (motors->m3 < 0.0f) motors->m3 = 0.0f;
    if (motors->m3 > 1.0f) motors->m3 = 1.0f;
    if (motors->m4 < 0.0f) motors->m4 = 0.0f;
    if (motors->m4 > 1.0f) motors->m4 = 1.0f;
}

// ----------------------------------------------------------------------------
// Angle wrapping for yaw error
// ----------------------------------------------------------------------------

static float wrap_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// ----------------------------------------------------------------------------
// Platform Callbacks
// ----------------------------------------------------------------------------

static void on_init(void) {
    // Initialize PID controllers
    pid_init(&s_roll_pid, ROLL_KP, ROLL_KI, ROLL_KD, 0.5f);
    pid_init(&s_pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD, 0.5f);
    pid_init(&s_yaw_pid, YAW_KP, YAW_KI, YAW_KD, 0.5f);
    pid_init(&s_alt_pid, ALT_KP, ALT_KI, ALT_KD, 0.3f);

    // TODO: Initialize RC input, telemetry, etc.
}

static void on_control(const platform_sensors_t *sensors,
                       platform_motors_t *motors) {
    const float dt = PLATFORM_LOOP_DT;

    if (!s_flying) {
        // Not flying - motors off
        motors->m1 = 0.0f;
        motors->m2 = 0.0f;
        motors->m3 = 0.0f;
        motors->m4 = 0.0f;
        return;
    }

    // -------------------------------------------------------------------------
    // Altitude control (outer loop)
    // -------------------------------------------------------------------------

    float alt_error = s_target_altitude - sensors->altitude;
    float throttle_adj = pid_update(&s_alt_pid, alt_error, dt);
    float throttle = THROTTLE_HOVER + throttle_adj;

    // Clamp throttle
    if (throttle < 0.1f) throttle = 0.1f;
    if (throttle > 0.9f) throttle = 0.9f;

    // -------------------------------------------------------------------------
    // Attitude control (inner loop)
    // -------------------------------------------------------------------------

    // Roll control
    float roll_error = TARGET_ROLL - sensors->roll;
    float roll_cmd = pid_update(&s_roll_pid, roll_error, dt);

    // Pitch control
    float pitch_error = TARGET_PITCH - sensors->pitch;
    float pitch_cmd = pid_update(&s_pitch_pid, pitch_error, dt);

    // Yaw control (with angle wrapping)
    float yaw_error = wrap_angle(s_target_yaw - sensors->yaw);
    float yaw_cmd = pid_update(&s_yaw_pid, yaw_error, dt);

    // -------------------------------------------------------------------------
    // Motor mixing
    // -------------------------------------------------------------------------

    mix_motors(throttle, roll_cmd, pitch_cmd, yaw_cmd, motors);
}

static void on_state_change(platform_state_t old_state,
                            platform_state_t new_state) {
    (void)old_state;

    switch (new_state) {
    case PLATFORM_STATE_READY:
        // TODO: Turn on green LED
        s_flying = false;
        break;

    case PLATFORM_STATE_ARMED:
        // TODO: Turn on yellow LED
        // Reset PID integrators
        pid_reset(&s_roll_pid);
        pid_reset(&s_pitch_pid);
        pid_reset(&s_yaw_pid);
        pid_reset(&s_alt_pid);
        break;

    case PLATFORM_STATE_FLYING:
        // TODO: Turn on blue LED
        s_flying = true;
        break;

    case PLATFORM_STATE_ERROR:
        // TODO: Flash red LED
        s_flying = false;
        break;

    default:
        break;
    }
}

// ----------------------------------------------------------------------------
// Main Entry Point
// ----------------------------------------------------------------------------

int main(void) {
    // TODO: STM32 HAL initialization
    // HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();
    // MX_SPI1_Init();
    // MX_I2C1_Init();
    // MX_TIM4_Init();
    // MX_TIM2_Init();  // For microsecond timing

    // Configure platform callbacks
    platform_callbacks_t callbacks = {
        .on_init = on_init,
        .on_control = on_control,
        .on_state_change = on_state_change
    };

    // Initialize platform
    if (!platform_init(&callbacks)) {
        // Initialization failed - flash error LED
        while (1) {
            // TODO: Toggle error LED
            platform_delay_ms(100);
        }
    }

    // Calibrate sensors (drone must be stationary and level!)
    // TODO: Wait for user button press or RC command
    platform_delay_ms(2000);  // 2 second delay for user to set drone down

    if (!platform_calibrate()) {
        // Calibration failed
        while (1) {
            // TODO: Flash error LED differently
            platform_delay_ms(200);
        }
    }

    // Ready for flight!
    // TODO: Wait for arm command from RC or button

    // For testing: auto-arm after 3 seconds
    // WARNING: Remove this in production! Use RC arm command instead.
    platform_delay_ms(3000);
    platform_arm();
    s_flying = true;

    // Start main control loop (never returns)
    platform_run();

    return 0;
}

// ----------------------------------------------------------------------------
// STM32 HAL Callbacks (required stubs)
// ----------------------------------------------------------------------------

// TODO: Implement these when integrating with STM32 HAL

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//     // Used for system tick or control loop timing
// }

// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//     // Used for RC input capture or button press
// }

// void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
//     // Used for DMA-based sensor reads
// }
