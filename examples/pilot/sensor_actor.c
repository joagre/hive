// Sensor actor - Timer-driven sensor reading
//
// Periodically reads raw sensors via HAL, applies low-pass filters,
// publishes to sensor bus. Sensor fusion is done by the estimator actor.

#include "sensor_actor.h"
#include "pilot_buses.h"
#include "notifications.h"
#include "config.h"
#include "hal/hal.h"
#include "hive_runtime.h"
#include "hive_bus.h"
#include "hive_timer.h"
#include "hive_ipc.h"
#include "hive_select.h"
#include "hive_log.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_INTERVAL_US (TIME_STEP_MS * 1000)

// ----------------------------------------------------------------------------
// 2nd order Butterworth low-pass filter
// ----------------------------------------------------------------------------
// Matches Bitcraze firmware filter.c implementation.
// Accel 30 Hz: removes motor vibration causing MEMS rectification error.
// Gyro 80 Hz: removes motor vibration from rate PID input.

#define ACCEL_LPF_CUTOFF_HZ 30
#define GYRO_LPF_CUTOFF_HZ 80
#define SENSOR_SAMPLE_HZ (1000 / TIME_STEP_MS)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

typedef struct {
    float a1, a2;     // Denominator coefficients
    float b0, b1, b2; // Numerator coefficients
    float d1, d2;     // Delay elements
} lpf2p_t;

static lpf2p_t s_accel_lpf[3];
static lpf2p_t s_gyro_lpf[3];

static void lpf2p_init(lpf2p_t *f, float sample_freq, float cutoff_freq) {
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf((float)M_PI / fr);
    float c = 1.0f + 2.0f * cosf((float)M_PI / 4.0f) * ohm + ohm * ohm;
    f->b0 = ohm * ohm / c;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    f->a2 = (1.0f - 2.0f * cosf((float)M_PI / 4.0f) * ohm + ohm * ohm) / c;
    f->d1 = 0.0f;
    f->d2 = 0.0f;
}

static float lpf2p_apply(lpf2p_t *f, float sample) {
    float d0 = sample - f->d1 * f->a1 - f->d2 * f->a2;
    if (!isfinite(d0)) {
        d0 = sample;
    }
    float out = d0 * f->b0 + f->d1 * f->b1 + f->d2 * f->b2;
    f->d2 = f->d1;
    f->d1 = d0;
    return out;
}

static void sensor_lpf_init(void) {
    for (int i = 0; i < 3; i++) {
        lpf2p_init(&s_accel_lpf[i], SENSOR_SAMPLE_HZ, ACCEL_LPF_CUTOFF_HZ);
        lpf2p_init(&s_gyro_lpf[i], SENSOR_SAMPLE_HZ, GYRO_LPF_CUTOFF_HZ);
    }
}

static void sensor_lpf_apply(sensor_data_t *sensors) {
    if (sensors->accel_valid) {
        for (int i = 0; i < 3; i++) {
            sensors->accel[i] = lpf2p_apply(&s_accel_lpf[i], sensors->accel[i]);
        }
    }
    if (sensors->gyro_valid) {
        for (int i = 0; i < 3; i++) {
            sensors->gyro[i] = lpf2p_apply(&s_gyro_lpf[i], sensors->gyro[i]);
        }
    }
}

// ----------------------------------------------------------------------------
// Flow processing - converts raw HAL data to position/velocity
// ----------------------------------------------------------------------------
// Moved here from HAL to keep signal processing platform-independent.
// HAL provides only raw data: range_height, flow_dpixel_x/y, raw_gps_x/y/z.
// This function populates: gps_x/y/z, gps_valid, velocity_x/y, velocity_valid.

// Flow integration state (Crazyflie optical flow path)
static float s_integrated_x = 0.0f;
static float s_integrated_y = 0.0f;
static float s_prev_yaw = 0.0f;
static uint64_t s_prev_flow_time_us = 0;

// GPS differentiation state (Webots GPS path)
static float s_prev_gps[3] = {0.0f, 0.0f, 0.0f};
static bool s_prev_gps_valid = false;

static void flow_process_reset(void) {
    s_integrated_x = 0.0f;
    s_integrated_y = 0.0f;
    s_prev_yaw = 0.0f;
    s_prev_flow_time_us = 0;
    s_prev_gps[0] = 0.0f;
    s_prev_gps[1] = 0.0f;
    s_prev_gps[2] = 0.0f;
    s_prev_gps_valid = false;
}

static void flow_process(sensor_data_t *sensors) {
    // Initialize processed fields as invalid
    sensors->gps_x = 0.0f;
    sensors->gps_y = 0.0f;
    sensors->gps_z = 0.0f;
    sensors->gps_valid = false;
    sensors->velocity_x = 0.0f;
    sensors->velocity_y = 0.0f;
    sensors->velocity_valid = false;

    // Path 1 - Optical flow (Crazyflie): convert pixel deltas to velocity
    if (sensors->flow_valid && sensors->range_valid &&
        sensors->range_height > FLOW_MIN_HEIGHT) {
        float height_m = sensors->range_height;

        // Compute dt from previous flow read
        uint64_t now_us = hive_get_time();
        float dt = 0.0f;
        if (s_prev_flow_time_us > 0) {
            dt = (now_us - s_prev_flow_time_us) / 1000000.0f;
        }
        s_prev_flow_time_us = now_us;

        if (dt > 0.0f && dt < 0.1f) {
            // Convert pixel deltas to velocity (m/s, body frame)
            // Formula: velocity = pixel_delta * SCALE * height / dt
            float vx_body = sensors->flow_dpixel_x * FLOW_SCALE * height_m / dt;
            float vy_body = sensors->flow_dpixel_y * FLOW_SCALE * height_m / dt;

            // Compensate for body rotation.
            // The PMW3901 sees rotational flow when the drone pitches/rolls.
            // From Bitcraze mm_flow.c:79 predicted_x ~ (vx/z - pitch_rate),
            // solving for vx: vx = flow_vel + pitch_rate * z.
            // From mm_flow.c:92 predicted_y ~ (vy/z + roll_rate),
            // solving for vy: vy = flow_vel - roll_rate * z.
            // gyro[1] = -raw_pitch_rate (negated in HAL), so
            // -= gives +raw_pitch_rate*h. gyro[0] = raw_roll_rate, so
            // -= gives -raw_roll_rate*h. Both match Bitcraze's model.
            vx_body -= sensors->gyro[1] * height_m;
            vy_body -= sensors->gyro[0] * height_m;

            // Integrate yaw from gyro (simple dead-reckoning)
            s_prev_yaw += sensors->gyro[2] * dt;

            // Rotate body velocity to world frame
            float cos_yaw = cosf(s_prev_yaw);
            float sin_yaw = sinf(s_prev_yaw);
            float vx_world = vx_body * cos_yaw - vy_body * sin_yaw;
            float vy_world = vx_body * sin_yaw + vy_body * cos_yaw;

            sensors->velocity_x = vx_world;
            sensors->velocity_y = vy_world;
            sensors->velocity_valid = true;

            // Integrate velocity to position
            s_integrated_x += vx_world * dt;
            s_integrated_y += vy_world * dt;
        }

        sensors->gps_x = s_integrated_x;
        sensors->gps_y = s_integrated_y;
        sensors->gps_z = height_m;
        sensors->gps_valid = true;
        return;
    }

    // Path 2 - GPS (Webots): pass through position, differentiate for velocity
    if (sensors->raw_gps_valid) {
        // Use noisy rangefinder for height check (matches real sensor behavior)
        float height =
            sensors->range_valid ? sensors->range_height : sensors->raw_gps_z;

        // Height range check (flow deck operational range)
        if (height >= FLOW_MIN_HEIGHT && height <= FLOW_MAX_HEIGHT) {
            sensors->gps_x = sensors->raw_gps_x;
            sensors->gps_y = sensors->raw_gps_y;
            sensors->gps_z = height; // Noisy altitude for KF
            sensors->gps_valid = true;

            // Differentiate clean GPS for velocity (no noise amplification)
            float dt = TIME_STEP_MS / 1000.0f;
            if (s_prev_gps_valid && dt > 0.0f) {
                sensors->velocity_x = (sensors->raw_gps_x - s_prev_gps[0]) / dt;
                sensors->velocity_y = (sensors->raw_gps_y - s_prev_gps[1]) / dt;
                sensors->velocity_valid = true;
            }
        } else {
            // Out of range - provide clean height but mark invalid
            sensors->gps_z = sensors->raw_gps_z;
        }

        s_prev_gps[0] = sensors->raw_gps_x;
        s_prev_gps[1] = sensors->raw_gps_y;
        s_prev_gps[2] = sensors->raw_gps_z;
        s_prev_gps_valid =
            (height >= FLOW_MIN_HEIGHT && height <= FLOW_MAX_HEIGHT);
        return;
    }

    // Path 3 - Range only (fallback): provide altitude, hold last position
    if (sensors->range_valid && sensors->range_height > FLOW_MIN_HEIGHT) {
        sensors->gps_x = s_integrated_x;
        sensors->gps_y = s_integrated_y;
        sensors->gps_z = sensors->range_height;
        sensors->gps_valid = true;
    } else if (sensors->range_valid) {
        // Range below minimum - still provide height for ground detection
        sensors->gps_z = sensors->range_height;
        sensors->gps_valid = (sensors->range_height > 0.0f);
    }
}

// ----------------------------------------------------------------------------
// Actor
// ----------------------------------------------------------------------------

// Actor state - initialized by sensor_actor_init
typedef struct {
    hive_bus_id_t sensor_bus;
    hive_actor_id_t flight_manager;
} sensor_state_t;

void *sensor_actor_init(void *init_args) {
    const pilot_buses_t *buses = init_args;
    static sensor_state_t state;
    state.sensor_bus = buses->sensor_bus;
    state.flight_manager = HIVE_ACTOR_ID_INVALID;
    return &state;
}

void sensor_actor(void *args, const hive_spawn_info_t *siblings,
                  size_t sibling_count) {
    sensor_state_t *state = args;

    // Look up flight_manager from sibling info
    state->flight_manager =
        hive_find_sibling(siblings, sibling_count, "flight_manager");
    if (state->flight_manager == HIVE_ACTOR_ID_INVALID) {
        HIVE_LOG_ERROR("[SENSOR] Failed to find flight_manager sibling");
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Initialize low-pass filters
    sensor_lpf_init();
    HIVE_LOG_INFO("[SENSOR] LPF: accel %d Hz, gyro %d Hz (sample %d Hz)",
                  ACCEL_LPF_CUTOFF_HZ, GYRO_LPF_CUTOFF_HZ, SENSOR_SAMPLE_HZ);

    hive_timer_id_t timer;
    hive_status_t status = hive_timer_every(SENSOR_INTERVAL_US, &timer);
    if (HIVE_FAILED(status)) {
        HIVE_LOG_ERROR("[SENSOR] Failed to create periodic timer: %s",
                       HIVE_ERR_STR(status));
        hive_exit(HIVE_EXIT_REASON_CRASH);
    }

    // Set up hive_select() sources: timer + RESET notification
    enum { SEL_TIMER, SEL_RESET };
    hive_select_source_t sources[] = {
        [SEL_TIMER] = {.type = HIVE_SEL_IPC,
                       .ipc = {.class = HIVE_MSG_TIMER, .tag = timer}},
        [SEL_RESET] = {.type = HIVE_SEL_IPC,
                       .ipc = {.sender = state->flight_manager,
                               .class = HIVE_MSG_REQUEST,
                               .id = NOTIFY_RESET}},
    };

    while (true) {
        hive_select_result_t result;
        status = hive_select(sources, 2, &result, -1);
        if (HIVE_FAILED(status)) {
            HIVE_LOG_ERROR("[SENSOR] select failed: %s", HIVE_ERR_STR(status));
            hive_exit(HIVE_EXIT_REASON_CRASH);
        }

        if (result.index == SEL_RESET) {
            // Re-initialize filters so delay elements don't carry stale state
            sensor_lpf_init();
            flow_process_reset();
            HIVE_LOG_INFO("[SENSOR] RESET received");
            hive_status_t rs = hive_ipc_reply(&result.ipc, NULL, 0);
            if (HIVE_FAILED(rs)) {
                HIVE_LOG_ERROR("[SENSOR] RESET reply failed: %s",
                               HIVE_ERR_STR(rs));
            }
            continue;
        }

        // SEL_TIMER: Read sensors, apply LPF, process flow, publish
        sensor_data_t sensors;
        hal_read_sensors(&sensors);
        sensor_lpf_apply(&sensors);
        flow_process(&sensors);
        status = hive_bus_publish(state->sensor_bus, &sensors, sizeof(sensors));
        if (HIVE_FAILED(status)) {
            HIVE_LOG_WARN("[SENSOR] bus publish failed: %s",
                          HIVE_ERR_STR(status));
        }
    }
}
