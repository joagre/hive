# Tunable Radio Parameters

Runtime-configurable parameters for PID tuning and flight adjustment via ESB radio.
This eliminates the build-flash-wait-test cycle and enables live tuning during flight.

## Architecture

### Design Decisions

1. **All parameters are `float`** - KISS. Even integers and booleans are stored as float.
   Actors cast to appropriate type when reading (e.g., `(uint32_t)params->deadman_ms`).

2. **Shared struct with pointer** - Single `tunable_params_t` instance allocated in
   `pilot.c`, pointer passed to all actors via `init_args`.

3. **No notifications needed** - Actors read directly from shared struct each control
   loop iteration. Single float reads/writes are atomic on ARM Cortex-M, so no
   synchronization is needed even if comms_actor (LOW priority) is interrupted
   mid-write by a control actor.

4. **Individual parameter commands** - Parameters sent one at a time over radio (not
   bulk transfer). Fits easily in 30-byte ESB payload.

5. **Enum for param_id** - Type-safe parameter identification, not byte offsets.

6. **PID gain changes do not reset integrators** - Changing Ki mid-flight leaves
   the existing I-term accumulator intact. This avoids thrust discontinuities but
   means the I-term may be sized for the old gain. For large Ki changes, consider
   landing first or accepting a brief transient.

### Platform-Specific Defaults

Default values in this spec are for **Crazyflie 2.1+ hardware**. Webots simulation
uses different tuning due to different dynamics:

| Parameter | Crazyflie | Webots |
|-----------|-----------|--------|
| `att_kp` | 1.8 | 2.5 |
| `att_kd` | 0.10 | 0.15 |
| `rate_kp` | 0.020 | 0.028 |
| `rate_ki` | 0.001 | 0.002 |
| `rate_kd` | 0.0015 | 0.003 |
| `pos_kd` | 0.10 | 0.06 |
| `max_tilt` | 0.25 | 0.20 |
| `vvel_damping` | 0.25 | 0.35 |

### Header File Structure

```c
// tunable_params.h

#include <stdbool.h>
#include <stdint.h>

// Parameter IDs - explicit enum for type safety
typedef enum {
    // Rate PID (0-6)
    PARAM_RATE_KP = 0,
    PARAM_RATE_KI = 1,
    PARAM_RATE_KD = 2,
    PARAM_RATE_IMAX = 3,
    PARAM_RATE_OMAX_ROLL = 4,
    PARAM_RATE_OMAX_PITCH = 5,
    PARAM_RATE_OMAX_YAW = 6,

    // Attitude PID (7-11)
    PARAM_ATT_KP = 7,
    PARAM_ATT_KI = 8,
    PARAM_ATT_KD = 9,
    PARAM_ATT_IMAX = 10,
    PARAM_ATT_OMAX = 11,

    // Altitude PID (12-17)
    PARAM_ALT_KP = 12,
    PARAM_ALT_KI = 13,
    PARAM_ALT_KD = 14,
    PARAM_ALT_IMAX = 15,
    PARAM_ALT_OMAX = 16,
    PARAM_VVEL_DAMPING = 17,

    // Emergency limits (18-19)
    PARAM_EMERGENCY_TILT_LIMIT = 18,
    PARAM_EMERGENCY_ALT_MAX = 19,

    // Landing (20-21)
    PARAM_LANDING_DESCENT_RATE = 20,
    PARAM_LANDING_VELOCITY_GAIN = 21,

    // Position control (22-24)
    PARAM_POS_KP = 22,
    PARAM_POS_KD = 23,
    PARAM_MAX_TILT_ANGLE = 24,

    // Complementary filter (25-29)
    PARAM_CF_ALPHA = 25,
    PARAM_CF_MAG_ALPHA = 26,
    PARAM_CF_USE_MAG = 27,
    PARAM_CF_ACCEL_THRESH_LO = 28,
    PARAM_CF_ACCEL_THRESH_HI = 29,

    // Waypoint navigation (30-34)
    PARAM_WP_TOLERANCE_XY = 30,
    PARAM_WP_TOLERANCE_Z = 31,
    PARAM_WP_TOLERANCE_YAW = 32,
    PARAM_WP_TOLERANCE_VEL = 33,
    PARAM_WP_HOVER_TIME_S = 34,

    // Altitude Kalman filter (35-41)
    PARAM_KF_Q_ALTITUDE = 35,
    PARAM_KF_Q_VELOCITY = 36,
    PARAM_KF_Q_BIAS = 37,
    PARAM_KF_R_ALTITUDE = 38,
    PARAM_KF_P0_ALTITUDE = 39,
    PARAM_KF_P0_VELOCITY = 40,
    PARAM_KF_P0_BIAS = 41,

    // Horizontal velocity filter (42)
    PARAM_HVEL_FILTER_ALPHA = 42,

    // Flight manager lifecycle (43-44)
    PARAM_ARMED_COUNTDOWN_S = 43,
    PARAM_AUTO_GO_DELAY_S = 44,

    // Yaw rate PID - separate from roll/pitch (45-47)
    PARAM_RATE_YAW_KP = 45,
    PARAM_RATE_YAW_KI = 46,
    PARAM_RATE_YAW_KD = 47,
} tunable_param_id_t;

#define TUNABLE_PARAM_COUNT 48

// Shared tunable parameters - all floats for simplicity
typedef struct {
    // Rate PID (same Kp/Ki/Kd for all axes, per-axis output limits)
    float rate_kp;            // Proportional gain
    float rate_ki;
    float rate_kd;
    float rate_imax;
    float rate_omax_roll;
    float rate_omax_pitch;
    float rate_omax_yaw;

    // Attitude PID (7-11)
    float att_kp;
    float att_ki;
    float att_kd;
    float att_imax;
    float att_omax;

    // Altitude PID (12-17)
    float alt_kp;
    float alt_ki;
    float alt_kd;
    float alt_imax;
    float alt_omax;
    float vvel_damping;

    // Emergency limits (18-19)
    float emergency_tilt_limit;
    float emergency_alt_max;

    // Landing (20-21)
    float landing_descent_rate;
    float landing_velocity_gain;

    // Position control (22-24)
    float pos_kp;
    float pos_kd;
    float max_tilt_angle;

    // Complementary filter (25-29)
    float cf_alpha;
    float cf_mag_alpha;
    float cf_use_mag;         // 0.0 = false, 1.0 = true
    float cf_accel_thresh_lo;
    float cf_accel_thresh_hi;

    // Waypoint navigation (30-34)
    float wp_tolerance_xy;
    float wp_tolerance_z;
    float wp_tolerance_yaw;
    float wp_tolerance_vel;
    float wp_hover_time_s;

    // Altitude Kalman filter (35-41)
    float kf_q_altitude;
    float kf_q_velocity;
    float kf_q_bias;
    float kf_r_altitude;
    float kf_p0_altitude;
    float kf_p0_velocity;
    float kf_p0_bias;

    // Horizontal velocity filter (42)
    float hvel_filter_alpha;

    // Flight manager lifecycle (43-44)
    float armed_countdown_s;
    float auto_go_delay_s;

    // Yaw rate PID (45-47)
    float rate_yaw_kp;
    float rate_yaw_ki;
    float rate_yaw_kd;
} tunable_params_t;

// Initialize with defaults from hal_config.h
void tunable_params_init(tunable_params_t *params);

// Set parameter by ID (returns HIVE_OK on success, HIVE_ERR_INVALID if invalid)
hive_status_t tunable_params_set(tunable_params_t *params, tunable_param_id_t id, float value);

// Get parameter by ID (returns 0.0f if invalid ID)
float tunable_params_get(const tunable_params_t *params, tunable_param_id_t id);

// Get parameter name string for debugging
const char *tunable_params_name(tunable_param_id_t id);
```

### Usage in pilot.c

```c
#include "tunable_params.h"

// Single shared instance
static tunable_params_t g_tunable_params;

int main(void) {
    // Initialize with defaults
    tunable_params_init(&g_tunable_params);

    // Pass pointer to actors via init_args
    altitude_actor_args_t alt_args = {
        .params = &g_tunable_params,
        // ...
    };

    // comms_actor also gets pointer to update params from radio
    comms_actor_args_t comms_args = {
        .params = &g_tunable_params,
        // ...
    };
}
```

### Usage in Actors

```c
// In altitude_actor.c
void altitude_actor(void *args, ...) {
    altitude_actor_args_t *a = args;
    tunable_params_t *params = a->params;

    while (true) {
        // Read current values each iteration (no caching)
        float kp = params->alt_kp;
        float ki = params->alt_ki;
        float kd = params->alt_kd;

        // Use in PID calculation
        // ...
    }
}
```

---

## Parameters

### Rate PID (innermost loop - controls rotation speed)

Same Kp/Ki/Kd gains used for roll/pitch; yaw has separate gains (no
aerodynamic restoring force, needs higher P and much higher I).

| Parameter | Description | Default |
|-----------|-------------|---------|
| `rate_kp` | Proportional gain (roll/pitch) | 0.020 |
| `rate_ki` | Integral gain (roll/pitch) | 0.001 |
| `rate_kd` | Derivative gain (roll/pitch) | 0.0015 |
| `rate_imax` | Integral windup limit | 0.3 |
| `rate_omax_roll` | Output limit for roll | 0.12 |
| `rate_omax_pitch` | Output limit for pitch | 0.12 |
| `rate_omax_yaw` | Output limit for yaw | 0.15 |
| `rate_yaw_kp` | Yaw proportional gain | 0.08 |
| `rate_yaw_ki` | Yaw integral gain | 0.02 |
| `rate_yaw_kd` | Yaw derivative gain | 0.001 |

### Attitude PID (middle loop - controls angle)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `att_kp` | Proportional gain | 1.8 |
| `att_ki` | Integral gain | 0.0 |
| `att_kd` | Derivative gain | 0.10 |
| `att_imax` | Integral windup limit | 0.5 |
| `att_omax` | Max rate setpoint (rad/s) | 2.0 |

### Altitude PID (controls height)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `alt_kp` | Proportional gain | 0.12 |
| `alt_ki` | Integral gain | 0.01 |
| `alt_kd` | Derivative gain | 0.0 |
| `alt_imax` | Integral windup limit | 0.2 |
| `alt_omax` | Output limit | 0.15 |
| `vvel_damping` | Vertical velocity damping | 0.25 |

### Position PD (controls XY - requires Flow deck)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `pos_kp` | Proportional gain | 0.08 |
| `pos_kd` | Derivative gain | 0.10 |
| `max_tilt_angle` | Max tilt angle for position control | 0.25 |

### Emergency Limits
| Parameter | Description | Default |
|-----------|-------------|---------|
| `emergency_tilt_limit` | Max tilt before crash cutoff (rad) | 0.78 (~45 deg) |
| `emergency_alt_max` | Max altitude before cutoff (m) | 2.0 |

### Landing
| Parameter | Description | Default |
|-----------|-------------|---------|
| `landing_descent_rate` | Target descent velocity (m/s, negative) | -0.15 |
| `landing_velocity_gain` | Thrust adjustment per m/s velocity error | 0.5 |

### Estimator / Filters

#### Altitude Kalman Filter
| Parameter | Description | Default |
|-----------|-------------|---------|
| `kf_q_altitude` | Process noise - position (m^2) | 0.0001 |
| `kf_q_velocity` | Process noise - velocity (m^2/s^2) | 1.0 |
| `kf_q_bias` | Process noise - accel bias | 0.0001 |
| `kf_r_altitude` | Measurement noise (m^2) | 0.001 |

#### Velocity Filters
| Parameter | Description | Default |
|-----------|-------------|---------|
| `hvel_filter_alpha` | Horizontal velocity LPF (0-1) | 0.95 |

#### Complementary Filter (attitude estimation)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `cf_alpha` | Gyro trust coefficient (0-1, higher = more gyro) | 0.995 |
| `cf_mag_alpha` | Magnetometer trust for yaw (0-1) | 0.95 |
| `cf_use_mag` | Enable magnetometer for yaw (0=off, 1=on) | 1.0 |
| `cf_accel_thresh_lo` | Reject accel below this (g) | 0.8 |
| `cf_accel_thresh_hi` | Reject accel above this (g) | 1.2 |

### Mission / Waypoint
| Parameter | Description | Default |
|-----------|-------------|---------|
| `wp_tolerance_xy` | Horizontal arrival radius (m) | 0.15 |
| `wp_tolerance_z` | Altitude tolerance (m) | 0.08 |
| `wp_tolerance_yaw` | Yaw tolerance (rad) | 0.1 |
| `wp_tolerance_vel` | Velocity tolerance (m/s) | 0.05 |
| `wp_hover_time_s` | Time to hover at each waypoint (s) | 3.0 |

---

## Validation Ranges

Parameters must be validated before applying. Reject values outside these ranges:

| Parameter | Min | Max | Notes |
|-----------|-----|-----|-------|
| `rate_kp` | 0.0 | 0.1 | Higher causes oscillation |
| `rate_ki` | 0.0 | 0.01 | Higher causes overshoot |
| `rate_kd` | 0.0 | 0.01 | Higher amplifies noise |
| `rate_imax` | 0.0 | 1.0 | |
| `rate_omax_*` | 0.0 | 0.5 | |
| `att_kp` | 0.0 | 5.0 | |
| `att_ki` | 0.0 | 1.0 | |
| `att_kd` | 0.0 | 0.5 | |
| `att_imax` | 0.0 | 1.0 | |
| `att_omax` | 0.0 | 10.0 | rad/s |
| `alt_kp` | 0.0 | 1.0 | |
| `alt_ki` | 0.0 | 0.1 | |
| `alt_kd` | 0.0 | 0.5 | |
| `alt_imax` | 0.0 | 0.5 | |
| `alt_omax` | 0.0 | 0.5 | |
| `vvel_damping` | 0.0 | 1.0 | |
| `pos_kp` | 0.0 | 0.5 | |
| `pos_kd` | 0.0 | 0.5 | |
| `max_tilt_angle` | 0.0 | 0.78 | ~45 deg max |
| `emergency_tilt_limit` | 0.1 | 1.57 | Up to 90 deg |
| `emergency_alt_max` | 0.5 | 10.0 | meters |
| `landing_descent_rate` | -0.5 | -0.05 | Must be negative |
| `landing_velocity_gain` | 0.0 | 2.0 | |
| `kf_q_*` | 1e-6 | 10.0 | |
| `kf_r_altitude` | 1e-6 | 1.0 | |
| `hvel_filter_alpha` | 0.0 | 1.0 | 0=no filter, 1=no update |
| `cf_alpha` | 0.9 | 0.999 | |
| `cf_mag_alpha` | 0.0 | 1.0 | |
| `cf_use_mag` | 0.0 | 1.0 | Treated as bool |
| `cf_accel_thresh_lo` | 0.5 | 0.95 | g |
| `cf_accel_thresh_hi` | 1.05 | 1.5 | g |
| `wp_tolerance_xy` | 0.05 | 1.0 | meters |
| `wp_tolerance_z` | 0.02 | 0.5 | meters |
| `wp_tolerance_yaw` | 0.01 | 0.5 | radians |
| `wp_tolerance_vel` | 0.01 | 0.5 | m/s |
| `wp_hover_time_s` | 0.0 | 60.0 | seconds |

---

## Radio Protocol

### Commands

| Command | Value | Direction | Description |
|---------|-------|-----------|-------------|
| `CMD_SET_PARAM` | 0x30 | Ground -> Drone | Set a parameter value |
| `CMD_GET_PARAM` | 0x31 | Ground -> Drone | Request a parameter value |
| `CMD_LIST_PARAMS` | 0x32 | Ground -> Drone | Request parameter list |
| `RESP_PARAM_ACK` | 0x33 | Drone -> Ground | Acknowledge set (status byte) |
| `RESP_PARAM_VALUE` | 0x34 | Drone -> Ground | Response with parameter value |
| `RESP_PARAM_LIST` | 0x35 | Drone -> Ground | Response with parameter list chunk |

**Note:** Values 0x10-0x12 are reserved for log download commands, 0x20 for GO command.

### Packet Format

All parameter packets use the same structure (7 bytes payload):

```c
typedef struct {
    uint8_t cmd;          // Command type
    uint16_t param_id;    // Parameter ID (enum value)
    float value;          // Parameter value
} __attribute__((packed)) param_packet_t;
```

### Example Exchange

**Set parameter:**
```
Ground -> Drone: [CMD_SET_PARAM][PARAM_ATT_KP][2.0f]
Drone -> Ground: [CMD_PARAM_VALUE][PARAM_ATT_KP][2.0f]  (ACK with current value)
```

**Get parameter:**
```
Ground -> Drone: [CMD_GET_PARAM][PARAM_ATT_KP][0.0f]  (value ignored)
Drone -> Ground: [CMD_PARAM_VALUE][PARAM_ATT_KP][1.8f]
```

---

## Ground Station Interface

### CLI Commands

```bash
# Set individual parameter
> set att_kp 2.0
OK: att_kp = 2.0

# Get parameter
> get att_kp
att_kp = 2.0

# List all parameters
> list
rate_kp = 0.020
rate_ki = 0.001
...
```

**Workflow:** Tune via radio until happy, then update `hal_config.h` with final
values and reflash. No runtime persistence needed.

---

## Implementation Status

All core functionality is implemented:

- [x] Create `tunable_params.h` with struct and enum
- [x] Create `tunable_params.c` with init/set/get functions
- [x] Update `pilot.c` to allocate and pass params pointer
- [x] Update actors to read from shared params (not hal_config.h constants)
- [x] Add `CMD_SET_PARAM` / `CMD_GET_PARAM` / `CMD_LIST_PARAMS` handling to `comms_actor.c`
- [x] Update `ground_station.py` with set/get/list commands
- [x] Add validation ranges per parameter
- [ ] Test with live tuning on hardware

**Implementation notes:**
- Typedef is `tunable_param_id_t` (not `param_id_t`)
- `tunable_params_set()` returns `hive_status_t` (not `bool`)
- 48 parameter slots: all used (0-47)
- Kalman filter Q/R and velocity filter alpha are now runtime-tunable
