# hive_board_config.mk - Webots Crazyflie board-specific configuration
#
# Simulation-specific overrides for the Webots Crazyflie model.
# Include this file in Makefiles that build for this board.
#
# Usage:
#   include $(HAL_DIR)/hive_board_config.mk

# =============================================================================
# Sensor Configuration
# =============================================================================
# Webots GPS has no physical range limit (unlike VL53L1x at 1.3m).
# Override the rangefinder max so the estimator accepts altitude readings
# at any height the drone can reach.

HIVE_CFLAGS += -DRANGEFINDER_MAX_M=4.0f

# Webots GPS provides position at any height. Override the flow deck
# height limit so the sensor HAL continues providing altitude data
# above the real hardware's 1.3m ceiling.
HIVE_CFLAGS += -DFLOW_MAX_HEIGHT=4.0f

# =============================================================================
# Liftoff Detection
# =============================================================================
# Webots motor response has a first-order lag (MOTOR_TIME_CONSTANT_MS = 20ms).
# Thrust is proportional to omega^2, so commanded thrust != actual thrust.
# The hardware ramp rate (0.4/s) overshoots actual hover by the time the
# rangefinder detects liftoff, discovering thrust ~0.85 when real hover is ~0.55.
# Slow the ramp so discovered hover thrust is closer to the true value.
HIVE_CFLAGS += -DLIFTOFF_RAMP_RATE=0.10f
HIVE_CFLAGS += -DLIFTOFF_ALT_THRESHOLD=0.02f
