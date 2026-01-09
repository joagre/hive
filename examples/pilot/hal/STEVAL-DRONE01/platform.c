// Platform initialization and main loop for STEVAL-DRONE01
//
// Main control loop running at 400Hz with sensor fusion.

#include "platform.h"
#include "system_config.h"
#include "gpio_config.h"
#include "i2c1.h"
#include "lsm6dsl.h"
#include "lis2mdl.h"
#include "lps22hd.h"
#include "motors.h"
#include "attitude.h"

// ----------------------------------------------------------------------------
// Static state
// ----------------------------------------------------------------------------

static platform_callbacks_t s_callbacks;
static platform_state_t s_state = PLATFORM_STATE_INIT;
static platform_sensors_t s_sensors;
static uint32_t s_loop_count = 0;
static uint32_t s_time_ms = 0;

// Calibration data
static float s_gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static float s_baro_reference = 0.0f;

// ----------------------------------------------------------------------------
// Time functions
// ----------------------------------------------------------------------------

static uint32_t get_time_us(void) {
    return system_get_us();
}

uint32_t platform_get_time_ms(void) {
    return system_get_tick();
}

void platform_delay_ms(uint32_t ms) {
    system_delay_ms(ms);
}

static void delay_us(uint32_t us) {
    system_delay_us(us);
}

// ----------------------------------------------------------------------------
// State management
// ----------------------------------------------------------------------------

static void set_state(platform_state_t new_state) {
    if (new_state != s_state) {
        platform_state_t old_state = s_state;
        s_state = new_state;

        if (s_callbacks.on_state_change) {
            s_callbacks.on_state_change(old_state, new_state);
        }
    }
}

platform_state_t platform_get_state(void) {
    return s_state;
}

// ----------------------------------------------------------------------------
// Sensor reading
// ----------------------------------------------------------------------------

static void read_imu(float accel[3], float gyro[3]) {
    lsm6dsl_data_t accel_data, gyro_data;
    lsm6dsl_read_all(&accel_data, &gyro_data);

    accel[0] = accel_data.x;
    accel[1] = accel_data.y;
    accel[2] = accel_data.z;

    // Apply gyro bias correction
    gyro[0] = gyro_data.x - s_gyro_bias[0];
    gyro[1] = gyro_data.y - s_gyro_bias[1];
    gyro[2] = gyro_data.z - s_gyro_bias[2];
}

static void read_mag(float mag[3]) {
    lis2mdl_data_t mag_data;
    lis2mdl_read(&mag_data);

    mag[0] = mag_data.x;
    mag[1] = mag_data.y;
    mag[2] = mag_data.z;
}

static float read_altitude(void) {
    float pressure = lps22hd_read_pressure();
    return lps22hd_altitude(pressure);
}

// ----------------------------------------------------------------------------
// Calibration
// ----------------------------------------------------------------------------

#define CALIBRATION_SAMPLES 500  // ~1.25 seconds at 400Hz

bool platform_calibrate(void) {
    set_state(PLATFORM_STATE_CALIBRATING);

    // -------------------------------------------------------------------------
    // Gyro bias calibration
    // -------------------------------------------------------------------------
    // Average gyro readings while stationary to find bias

    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    lsm6dsl_data_t accel_data, gyro_data;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        lsm6dsl_read_all(&accel_data, &gyro_data);

        gyro_sum[0] += gyro_data.x;
        gyro_sum[1] += gyro_data.y;
        gyro_sum[2] += gyro_data.z;

        delay_us(PLATFORM_LOOP_PERIOD_US);
    }

    s_gyro_bias[0] = gyro_sum[0] / CALIBRATION_SAMPLES;
    s_gyro_bias[1] = gyro_sum[1] / CALIBRATION_SAMPLES;
    s_gyro_bias[2] = gyro_sum[2] / CALIBRATION_SAMPLES;

    // -------------------------------------------------------------------------
    // Barometer reference calibration
    // -------------------------------------------------------------------------
    // Average pressure readings to establish ground level

    float pressure_sum = 0.0f;

    for (int i = 0; i < 50; i++) {
        pressure_sum += lps22hd_read_pressure();
        platform_delay_ms(20);
    }

    s_baro_reference = pressure_sum / 50.0f;
    lps22hd_set_reference(s_baro_reference);

    // -------------------------------------------------------------------------
    // Initialize attitude filter with current orientation
    // -------------------------------------------------------------------------

    float accel[3], gyro[3];
    read_imu(accel, gyro);

    attitude_t initial = {
        .roll = attitude_accel_roll(accel),
        .pitch = attitude_accel_pitch(accel),
        .yaw = 0.0f  // No absolute yaw reference without mag calibration
    };

    attitude_reset(&initial);

    set_state(PLATFORM_STATE_READY);
    return true;
}

// ----------------------------------------------------------------------------
// Motor control
// ----------------------------------------------------------------------------

bool platform_arm(void) {
    if (s_state != PLATFORM_STATE_READY) {
        return false;
    }

    motors_arm();
    set_state(PLATFORM_STATE_ARMED);
    return true;
}

bool platform_disarm(void) {
    motors_disarm();
    set_state(PLATFORM_STATE_READY);
    return true;
}

void platform_emergency_stop(void) {
    motors_emergency_stop();
    set_state(PLATFORM_STATE_READY);
}

static void apply_motors(const platform_motors_t *cmd) {
    motors_cmd_t motor_cmd = {
        .motor = {cmd->m1, cmd->m2, cmd->m3, cmd->m4}
    };
    motors_set(&motor_cmd);
}

// ----------------------------------------------------------------------------
// Platform initialization
// ----------------------------------------------------------------------------

bool platform_init(const platform_callbacks_t *callbacks) {
    s_state = PLATFORM_STATE_INIT;

    // Store callbacks
    if (callbacks) {
        s_callbacks = *callbacks;
    }

    // -------------------------------------------------------------------------
    // Initialize hardware peripherals
    // -------------------------------------------------------------------------

    // System clocks (84MHz), SysTick (1ms), DWT (microseconds)
    if (!system_init()) {
        set_state(PLATFORM_STATE_ERROR);
        return false;
    }

    // Initialize I2C1 for LIS2MDL and LPS22HD (400kHz Fast Mode)
    i2c1_init(I2C1_SPEED_400KHZ);

    // -------------------------------------------------------------------------
    // Initialize sensors
    // -------------------------------------------------------------------------

    // IMU (accelerometer + gyroscope) - SPI1 is initialized internally
    if (!lsm6dsl_init(NULL)) {
        set_state(PLATFORM_STATE_ERROR);
        return false;
    }

    // Magnetometer
    if (!lis2mdl_init(NULL)) {
        set_state(PLATFORM_STATE_ERROR);
        return false;
    }

    // Barometer
    if (!lps22hd_init(NULL)) {
        set_state(PLATFORM_STATE_ERROR);
        return false;
    }

    // -------------------------------------------------------------------------
    // Initialize motors (disarmed)
    // -------------------------------------------------------------------------

    if (!motors_init(NULL)) {
        set_state(PLATFORM_STATE_ERROR);
        return false;
    }

    // -------------------------------------------------------------------------
    // Initialize attitude filter
    // -------------------------------------------------------------------------

    attitude_config_t att_config = ATTITUDE_CONFIG_DEFAULT;
    att_config.use_mag = true;  // Enable magnetometer fusion
    attitude_init(&att_config, NULL);

    // -------------------------------------------------------------------------
    // Call user init callback
    // -------------------------------------------------------------------------

    if (s_callbacks.on_init) {
        s_callbacks.on_init();
    }

    return true;
}

// ----------------------------------------------------------------------------
// Main control loop
// ----------------------------------------------------------------------------

void platform_run(void) {
    float accel[3], gyro[3], mag[3];
    platform_motors_t motors_cmd = {0};
    attitude_t att;
    attitude_rates_t rates;

    uint32_t loop_start_us;
    uint32_t mag_counter = 0;
    uint32_t baro_counter = 0;

    // -------------------------------------------------------------------------
    // Main loop - runs at PLATFORM_LOOP_FREQ_HZ (400 Hz)
    // -------------------------------------------------------------------------

    while (1) {
        loop_start_us = get_time_us();

        // ---------------------------------------------------------------------
        // Read IMU (every iteration - 400 Hz)
        // ---------------------------------------------------------------------

        read_imu(accel, gyro);

        // Update attitude filter
        attitude_update(accel, gyro, PLATFORM_LOOP_DT);

        // ---------------------------------------------------------------------
        // Read magnetometer (every MAG_DIVIDER iterations - 50 Hz)
        // ---------------------------------------------------------------------

        mag_counter++;
        if (mag_counter >= PLATFORM_MAG_DIVIDER) {
            mag_counter = 0;

            read_mag(mag);
            attitude_update_mag(mag);
        }

        // ---------------------------------------------------------------------
        // Read barometer (every BARO_DIVIDER iterations - 50 Hz)
        // ---------------------------------------------------------------------

        baro_counter++;
        if (baro_counter >= PLATFORM_BARO_DIVIDER) {
            baro_counter = 0;

            s_sensors.altitude = read_altitude();
            s_sensors.pressure = lps22hd_read_pressure();
        }

        // ---------------------------------------------------------------------
        // Update sensor snapshot
        // ---------------------------------------------------------------------

        attitude_get(&att);
        attitude_get_rates(&rates);

        s_sensors.roll = att.roll;
        s_sensors.pitch = att.pitch;
        s_sensors.yaw = att.yaw;

        s_sensors.roll_rate = rates.roll_rate;
        s_sensors.pitch_rate = rates.pitch_rate;
        s_sensors.yaw_rate = rates.yaw_rate;

        s_sensors.accel_x = accel[0];
        s_sensors.accel_y = accel[1];
        s_sensors.accel_z = accel[2];

        s_sensors.timestamp_ms = platform_get_time_ms();
        s_sensors.loop_count = s_loop_count;

        // ---------------------------------------------------------------------
        // Run control callback
        // ---------------------------------------------------------------------

        if (s_callbacks.on_control) {
            s_callbacks.on_control(&s_sensors, &motors_cmd);

            // Apply motor commands if armed
            if (s_state == PLATFORM_STATE_ARMED ||
                s_state == PLATFORM_STATE_FLYING) {
                apply_motors(&motors_cmd);
            }
        }

        // ---------------------------------------------------------------------
        // Loop timing - maintain constant rate
        // ---------------------------------------------------------------------

        s_loop_count++;

        // Calculate elapsed time and wait for next iteration
        uint32_t elapsed_us = get_time_us() - loop_start_us;
        if (elapsed_us < PLATFORM_LOOP_PERIOD_US) {
            delay_us(PLATFORM_LOOP_PERIOD_US - elapsed_us);
        }

        // Update millisecond counter (approximate)
        s_time_ms += PLATFORM_LOOP_PERIOD_US / 1000;
    }
}

// ----------------------------------------------------------------------------
// Sensor data access
// ----------------------------------------------------------------------------

void platform_get_sensors(platform_sensors_t *sensors) {
    // TODO: Add critical section for thread safety if using interrupts
    // __disable_irq();
    *sensors = s_sensors;
    // __enable_irq();
}
