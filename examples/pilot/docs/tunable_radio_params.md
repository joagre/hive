# Tunable Radio Parameters

Runtime-configurable parameters for PID tuning and flight adjustment via ESB radio.
This eliminates the build-flash-wait-test cycle and enables live tuning during flight.

## Control Loops

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

## Safety Limits

| Parameter | Description | Default |
|-----------|-------------|---------|
| `emergency_tilt` | Max tilt before crash cutoff (rad) | 0.78 (~45 deg) |
| `emergency_alt_max` | Max altitude before cutoff (m) | 2.0 |
| `motor_deadman_ms` | Timeout before motor shutoff (ms) | 50 |
| `landed_target_thresh` | Target alt indicating land (m) | 0.05 |
| `landed_actual_thresh` | Actual alt confirming landed (m) | 0.08 |

## Estimator / Filters

### Altitude Kalman Filter
| Parameter | Description | Default |
|-----------|-------------|---------|
| `kf_q_altitude` | Process noise - position (m^2) | 0.0001 |
| `kf_q_velocity` | Process noise - velocity (m^2/s^2) | 1.0 |
| `kf_q_bias` | Process noise - accel bias | 0.0001 |
| `kf_r_altitude` | Measurement noise (m^2) | 0.001 |

### Velocity Filters
| Parameter | Description | Default |
|-----------|-------------|---------|
| `hvel_filter_alpha` | Horizontal velocity LPF (0-1) | 0.95 |

### Sensor Calibration (live adjustment)
| Parameter | Description | Default |
|-----------|-------------|---------|
| `gyro_bias_x` | Gyroscope X bias (rad/s) | auto |
| `gyro_bias_y` | Gyroscope Y bias (rad/s) | auto |
| `gyro_bias_z` | Gyroscope Z bias (rad/s) | auto |
| `accel_bias_z` | Accelerometer Z bias (m/s^2) | auto |

## Mission / Waypoint

| Parameter | Description | Default |
|-----------|-------------|---------|
| `wp_tolerance_xy` | Horizontal arrival radius (m) | 0.15 |
| `wp_tolerance_z` | Altitude tolerance (m) | 0.08 |
| `wp_tolerance_yaw` | Yaw tolerance (rad) | 0.1 |
| `wp_tolerance_vel` | Velocity tolerance (m/s) | 0.05 |

## Motor Mixing / Trim

| Parameter | Description | Default |
|-----------|-------------|---------|
| `motor_trim_1` | M1 thrust offset (-0.1 to 0.1) | 0.0 |
| `motor_trim_2` | M2 thrust offset | 0.0 |
| `motor_trim_3` | M3 thrust offset | 0.0 |
| `motor_trim_4` | M4 thrust offset | 0.0 |

## Timing

| Parameter | Description | Default |
|-----------|-------------|---------|
| `control_rate_hz` | Control loop frequency | 250 |
| `telemetry_rate_hz` | Telemetry transmit frequency | 50 |

## Debug / Override

| Parameter | Description | Default |
|-----------|-------------|---------|
| `log_level` | Log verbosity (0=TRACE, 4=ERROR) | 2 (INFO) |
| `disable_position` | Disable position control | false |
| `disable_altitude` | Disable altitude control | false |
| `thrust_override` | Manual thrust (0=auto, >0=manual) | 0.0 |

## Profiles

Pre-configured parameter sets for quick switching:

| Profile | Description |
|---------|-------------|
| `safe` | Low gains, tight limits - for testing |
| `normal` | Balanced performance |
| `aggressive` | High gains, fast response |
| `acro` | Rate mode only, no stabilization |

## Implementation Notes

### Protocol
- Use existing ESB radio link via comms_actor
- New packet type for parameter set/get commands
- Command format: `[CMD_PARAM_SET][param_id:u16][value:f32]`
- Response format: `[CMD_PARAM_ACK][param_id:u16][value:f32]`

### Ground Station
- Add CLI commands: `set rate_kp 0.025`, `get att_kd`, `profile aggressive`
- Optional: curses UI with sliders for live adjustment
- Log parameter changes with timestamps

### Persistence
- Optional save to flash: `save` command writes current params to `/config`
- Load on boot if config exists, otherwise use compiled defaults

### Safety
- Validate ranges before applying (e.g., thrust 0.0-1.0)
- Some params require disarm (e.g., motor trim)
- Log all parameter changes
