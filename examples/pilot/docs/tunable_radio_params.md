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
| `hover_thrust` | 0.38 | 0.553 |
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
    // Rate PID (same Kp/Ki/Kd for all axes, per-axis output limits)
    PARAM_RATE_KP = 0,
    PARAM_RATE_KI,
    PARAM_RATE_KD,
    PARAM_RATE_IMAX,
    PARAM_RATE_OMAX_ROLL,
    PARAM_RATE_OMAX_PITCH,
    PARAM_RATE_OMAX_YAW,

    // Attitude PID
    PARAM_ATT_KP,
    PARAM_ATT_KI,
    PARAM_ATT_KD,
    PARAM_ATT_IMAX,
    PARAM_ATT_OMAX,

    // Altitude PID
    PARAM_ALT_KP,
    PARAM_ALT_KI,
    PARAM_ALT_KD,
    PARAM_ALT_IMAX,
    PARAM_ALT_OMAX,
    PARAM_VVEL_DAMPING,

    // Position PD
    PARAM_POS_KP,
    PARAM_POS_KD,
    PARAM_MAX_TILT,

    // Thrust
    PARAM_HOVER_THRUST,

    // Safety
    PARAM_EMERGENCY_TILT,
    PARAM_EMERGENCY_ALT_MAX,
    PARAM_MOTOR_DEADMAN_MS,
    PARAM_LANDED_ACTUAL_THRESH,

    // Landing / Takeoff
    PARAM_LANDING_DESCENT_RATE,
    PARAM_LANDING_VELOCITY_GAIN,
    PARAM_THRUST_RAMP_MS,

    // Altitude Kalman Filter
    PARAM_KF_Q_ALTITUDE,
    PARAM_KF_Q_VELOCITY,
    PARAM_KF_Q_BIAS,
    PARAM_KF_R_ALTITUDE,

    // Velocity Filter
    PARAM_HVEL_FILTER_ALPHA,

    // Complementary Filter (attitude)
    PARAM_CF_ALPHA,
    PARAM_CF_MAG_ALPHA,
    PARAM_CF_USE_MAG,
    PARAM_CF_ACCEL_THRESH_LO,
    PARAM_CF_ACCEL_THRESH_HI,

    // Waypoint
    PARAM_WP_TOLERANCE_XY,
    PARAM_WP_TOLERANCE_Z,
    PARAM_WP_TOLERANCE_YAW,
    PARAM_WP_TOLERANCE_VEL,
    PARAM_WP_HOVER_TIME_S,

    PARAM_COUNT  // Total number of parameters (45)
} tunable_param_id_t;

// NOTE: New parameters must be added before PARAM_COUNT, at the end of
// their category or in a new category. Do not insert in the middle of
// existing groups - this would break any stored parameter IDs.

// Shared tunable parameters - all floats for simplicity
typedef struct {
    // Rate PID (same Kp/Ki/Kd for all axes, per-axis output limits)
    float rate_kp;            // Proportional gain
    float rate_ki;            // Integral gain
    float rate_kd;            // Derivative gain
    float rate_imax;          // Integral limit
    float rate_omax_roll;     // Output limit roll
    float rate_omax_pitch;    // Output limit pitch
    float rate_omax_yaw;      // Output limit yaw

    // Attitude PID
    float att_kp;             // Proportional gain
    float att_ki;             // Integral gain
    float att_kd;             // Derivative gain
    float att_imax;           // Integral limit
    float att_omax;           // Max rate setpoint (rad/s)

    // Altitude PID
    float alt_kp;             // Proportional gain
    float alt_ki;             // Integral gain
    float alt_kd;             // Derivative gain
    float alt_imax;           // Integral limit
    float alt_omax;           // Output limit
    float vvel_damping;       // Velocity damping gain

    // Position PD
    float pos_kp;             // Proportional gain
    float pos_kd;             // Derivative gain
    float max_tilt;           // Max tilt angle (rad)

    // Thrust
    float hover_thrust;       // Base hover thrust (0-1)

    // Safety
    float emergency_tilt;     // Crash cutoff angle (rad)
    float emergency_alt_max;  // Max altitude cutoff (m)
    float motor_deadman_ms;   // Watchdog timeout (ms)
    float landed_actual_thresh; // Landing detection alt (m)

    // Landing / Takeoff
    float landing_descent_rate;   // Descent velocity (m/s)
    float landing_velocity_gain;  // Thrust per m/s error
    float thrust_ramp_ms;         // Takeoff ramp duration (ms)

    // Altitude Kalman Filter
    float kf_q_altitude;      // Process noise position (m^2)
    float kf_q_velocity;      // Process noise velocity (m^2/s^2)
    float kf_q_bias;          // Process noise bias
    float kf_r_altitude;      // Measurement noise (m^2)

    // Velocity Filter
    float hvel_filter_alpha;  // LPF coefficient (0-1)

    // Complementary Filter (attitude)
    float cf_alpha;           // Gyro trust (0-1)
    float cf_mag_alpha;       // Magnetometer trust (0-1)
    float cf_use_mag;         // Enable magnetometer (0 or 1)
    float cf_accel_thresh_lo; // Min valid accel magnitude (g)
    float cf_accel_thresh_hi; // Max valid accel magnitude (g)

    // Waypoint
    float wp_tolerance_xy;    // Horizontal arrival radius (m)
    float wp_tolerance_z;     // Altitude tolerance (m)
    float wp_tolerance_yaw;   // Yaw tolerance (rad)
    float wp_tolerance_vel;   // Velocity tolerance (m/s)
    float wp_hover_time_s;    // Hover duration at waypoint (s)
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

    while (1) {
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

Same Kp/Ki/Kd gains used for all axes; only output limits vary per-axis.

| Parameter | Description | Default |
|-----------|-------------|---------|
| `rate_kp` | Proportional gain (all axes) | 0.020 |
| `rate_ki` | Integral gain (all axes) | 0.001 |
| `rate_kd` | Derivative gain (all axes) | 0.0015 |
| `rate_imax` | Integral windup limit | 0.3 |
| `rate_omax_roll` | Output limit for roll | 0.12 |
| `rate_omax_pitch` | Output limit for pitch | 0.12 |
| `rate_omax_yaw` | Output limit for yaw | 0.15 |

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
| `max_tilt` | Max tilt angle for position control | 0.25 |

### Thrust
| Parameter | Description | Default |
|-----------|-------------|---------|
| `hover_thrust` | Base thrust for hover (0.0-1.0) | 0.38 |

### Safety Limits
| Parameter | Description | Default |
|-----------|-------------|---------|
| `emergency_tilt` | Max tilt before crash cutoff (rad) | 0.78 (~45 deg) |
| `emergency_alt_max` | Max altitude before cutoff (m) | 2.0 |
| `motor_deadman_ms` | Timeout before motor shutoff (ms) | 50 |
| `landed_actual_thresh` | Actual alt confirming landed (m) | 0.08 |

### Landing
| Parameter | Description | Default |
|-----------|-------------|---------|
| `landing_descent_rate` | Target descent velocity (m/s, negative) | -0.15 |
| `landing_velocity_gain` | Thrust adjustment per m/s velocity error | 0.5 |

### Takeoff
| Parameter | Description | Default |
|-----------|-------------|---------|
| `thrust_ramp_ms` | Thrust ramp duration for gentle takeoff (ms) | 500 |

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
| `max_tilt` | 0.0 | 0.78 | ~45 deg max |
| `hover_thrust` | 0.1 | 0.8 | Platform dependent |
| `emergency_tilt` | 0.1 | 1.57 | Up to 90 deg |
| `emergency_alt_max` | 0.5 | 10.0 | meters |
| `motor_deadman_ms` | 10 | 500 | milliseconds |
| `landed_actual_thresh` | 0.01 | 0.2 | meters |
| `landing_descent_rate` | -0.5 | -0.05 | Must be negative |
| `landing_velocity_gain` | 0.0 | 2.0 | |
| `thrust_ramp_ms` | 0 | 2000 | milliseconds |
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
- 45 parameter slots: all used (0-44)
- Kalman filter Q/R and velocity filter alpha are now runtime-tunable
