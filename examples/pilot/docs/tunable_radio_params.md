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
   loop iteration. Since actors run cooperatively (no preemption), this is safe.

4. **Individual parameter commands** - Parameters sent one at a time over radio (not
   bulk transfer). Fits easily in 30-byte ESB payload.

5. **Enum for param_id** - Type-safe parameter identification, not byte offsets.

### Header File Structure

```c
// tunable_params.h

#include <stdint.h>

// Parameter IDs - explicit enum for type safety
typedef enum {
    // Rate PID
    PARAM_RATE_KP_ROLL = 0,
    PARAM_RATE_KI_ROLL,
    PARAM_RATE_KD_ROLL,
    PARAM_RATE_KP_PITCH,
    PARAM_RATE_KI_PITCH,
    PARAM_RATE_KD_PITCH,
    PARAM_RATE_KP_YAW,
    PARAM_RATE_KI_YAW,
    PARAM_RATE_KD_YAW,
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

    // ... more params ...

    PARAM_COUNT  // Total number of parameters
} param_id_t;

// Shared tunable parameters - all floats for simplicity
typedef struct {
    // Rate PID
    float rate_kp_roll;
    float rate_ki_roll;
    float rate_kd_roll;
    float rate_kp_pitch;
    float rate_ki_pitch;
    float rate_kd_pitch;
    float rate_kp_yaw;
    float rate_ki_yaw;
    float rate_kd_yaw;
    float rate_imax;
    float rate_omax_roll;
    float rate_omax_pitch;
    float rate_omax_yaw;

    // Attitude PID
    float att_kp;
    float att_ki;
    float att_kd;
    float att_imax;
    float att_omax;

    // Altitude PID
    float alt_kp;
    float alt_ki;
    float alt_kd;
    float alt_imax;
    float alt_omax;
    float vvel_damping;

    // Position PD
    float pos_kp;
    float pos_kd;
    float max_tilt;

    // Thrust
    float hover_thrust;

    // Safety
    float emergency_tilt;
    float emergency_alt_max;
    float motor_deadman_ms;

    // ... more params ...
} tunable_params_t;

// Initialize with defaults from hal_config.h
void tunable_params_init(tunable_params_t *params);

// Set parameter by ID (returns false if invalid ID or value)
bool tunable_params_set(tunable_params_t *params, param_id_t id, float value);

// Get parameter by ID (returns NaN if invalid ID)
float tunable_params_get(const tunable_params_t *params, param_id_t id);
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
| Parameter | Description | Default |
|-----------|-------------|---------|
| `rate_kp_roll` | Proportional gain for roll rate | 0.020 |
| `rate_ki_roll` | Integral gain for roll rate | 0.001 |
| `rate_kd_roll` | Derivative gain for roll rate | 0.0015 |
| `rate_kp_pitch` | Proportional gain for pitch rate | 0.020 |
| `rate_ki_pitch` | Integral gain for pitch rate | 0.001 |
| `rate_kd_pitch` | Derivative gain for pitch rate | 0.0015 |
| `rate_kp_yaw` | Proportional gain for yaw rate | 0.020 |
| `rate_ki_yaw` | Integral gain for yaw rate | 0.001 |
| `rate_kd_yaw` | Derivative gain for yaw rate | 0.0015 |
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
| `landed_target_thresh` | Target alt indicating land (m) | 0.05 |
| `landed_actual_thresh` | Actual alt confirming landed (m) | 0.08 |

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

#### Sensor Calibration (live adjustment)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `gyro_bias_x` | Gyroscope X bias (rad/s) | auto |
| `gyro_bias_y` | Gyroscope Y bias (rad/s) | auto |
| `gyro_bias_z` | Gyroscope Z bias (rad/s) | auto |
| `accel_bias_z` | Accelerometer Z bias (m/s^2) | auto |

### Mission / Waypoint
| Parameter | Description | Default |
|-----------|-------------|---------|
| `wp_tolerance_xy` | Horizontal arrival radius (m) | 0.15 |
| `wp_tolerance_z` | Altitude tolerance (m) | 0.08 |
| `wp_tolerance_yaw` | Yaw tolerance (rad) | 0.1 |
| `wp_tolerance_vel` | Velocity tolerance (m/s) | 0.05 |

### Motor Mixing / Trim
| Parameter | Description | Default |
|-----------|-------------|---------|
| `motor_trim_1` | M1 thrust offset (-0.1 to 0.1) | 0.0 |
| `motor_trim_2` | M2 thrust offset | 0.0 |
| `motor_trim_3` | M3 thrust offset | 0.0 |
| `motor_trim_4` | M4 thrust offset | 0.0 |

### Timing
| Parameter | Description | Default |
|-----------|-------------|---------|
| `control_rate_hz` | Control loop frequency | 250 |
| `telemetry_rate_hz` | Telemetry transmit frequency | 50 |

### Debug / Override
| Parameter | Description | Default |
|-----------|-------------|---------|
| `log_level` | Log verbosity (0=TRACE, 4=ERROR) | 2 (INFO) |
| `disable_position` | Disable position control | false (0.0) |
| `disable_altitude` | Disable altitude control | false (0.0) |
| `thrust_override` | Manual thrust (0=auto, >0=manual) | 0.0 |

---

## Radio Protocol

### Commands

| Command | Value | Description |
|---------|-------|-------------|
| `CMD_SET_PARAM` | 0x10 | Set a parameter value |
| `CMD_GET_PARAM` | 0x11 | Request a parameter value |
| `CMD_PARAM_VALUE` | 0x12 | Response with parameter value |

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
rate_kp_roll = 0.020
rate_ki_roll = 0.001
...
```

**Workflow:** Tune via radio until happy, then update `hal_config.h` with final
values and reflash. No runtime persistence needed.

---

## Validation

Before applying any parameter:
1. Check `param_id` is valid (< PARAM_COUNT)
2. Check value is within valid range (defined per-parameter)
3. Log all parameter changes

---

## Implementation Checklist

- [ ] Create `tunable_params.h` with struct and enum
- [ ] Create `tunable_params.c` with init/set/get functions
- [ ] Update `pilot.c` to allocate and pass params pointer
- [ ] Update actors to read from shared params (not hal_config.h constants)
- [ ] Add `CMD_SET_PARAM` / `CMD_GET_PARAM` handling to `comms_actor.c`
- [ ] Update `ground_station.py` with set/get commands
- [ ] Add validation ranges per parameter
- [ ] Test with live tuning
