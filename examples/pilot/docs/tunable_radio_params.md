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
| `alt_ki` | 0.005 | 0.03 |
| `vvel_damping` | 0.55 | 0.15 |
| `pos_kd` | 0.10 | 0.06 |
| `max_tilt` | 0.25 | 0.20 |

### Header File

See `include/tunable_params.h` for the full enum, struct, and API.
55 parameters (`TUNABLE_PARAM_COUNT`), all stored as `float`.

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
| `alt_ki` | Integral gain | 0.005 |
| `alt_kd` | Derivative gain | 0.0 |
| `alt_imax` | Integral windup limit | 0.2 |
| `alt_omax` | Output limit | 0.15 |
| `vvel_damping` | Vertical velocity damping | 0.55 |

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
| `kf_p0_altitude` | Initial covariance - position (m^2) | 1.0 |
| `kf_p0_velocity` | Initial covariance - velocity (m^2/s^2) | 1.0 |
| `kf_p0_bias` | Initial covariance - accel bias (m^2/s^4) | 0.1 |

#### Horizontal Kalman Filter
| Parameter | Description | Default |
|-----------|-------------|---------|
| `hkf_q_position` | Process noise - position (m^2) | 0.0001 |
| `hkf_q_velocity` | Process noise - velocity (m^2/s^2) | 1.0 |
| `hkf_q_bias` | Process noise - accel bias | 0.0001 |
| `hkf_r_velocity` | Measurement noise - flow velocity | 0.01 |
| `hkf_p0_position` | Initial covariance - position (m^2) | 1.0 |
| `hkf_p0_velocity` | Initial covariance - velocity (m^2/s^2) | 1.0 |
| `hkf_p0_bias` | Initial covariance - accel bias (m^2/s^4) | 0.1 |

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

### Flight Manager Lifecycle
| Parameter | Description | Default (HW) | Default (Sim) |
|-----------|-------------|--------------|---------------|
| `armed_countdown_s` | Countdown before flight authorization (s) | 10.0 | 5.0 |
| `auto_go_delay_s` | Auto-GO delay after boot (s, 0=manual) | 0.0 | 2.0 |

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
| `kf_p0_altitude` | 0.01 | 10.0 | |
| `kf_p0_velocity` | 0.01 | 10.0 | |
| `kf_p0_bias` | 0.001 | 1.0 | |
| `hkf_p0_position` | 0.01 | 10.0 | |
| `hkf_p0_velocity` | 0.01 | 10.0 | |
| `hkf_p0_bias` | 0.001 | 1.0 | |
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
| `armed_countdown_s` | 5.0 | 300.0 | seconds |
| `auto_go_delay_s` | 0.0 | 60.0 | seconds (0=manual) |

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

**Note:** Values 0x10-0x12 are reserved for log download, 0x20-0x23 for flight commands (GO, ABORT, STATUS).

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

Fully implemented and tested on hardware (tuning sessions 3-10).
55 parameters, all runtime-tunable via radio.
