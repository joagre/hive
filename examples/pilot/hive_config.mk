# hive_config.mk - Shared Hive memory configuration for pilot
#
# These settings are determined by the pilot actor implementation,
# not the hardware platform. All platforms use the same values.
#
# Include this file in platform Makefiles:
#   include hive_config.mk
#
# Platform-specific settings (stack sizes, flash layout) remain
# in the individual Makefiles.
#
# ============================================================================
# RESOURCE AUDIT (keep in sync with actual usage)
# ============================================================================
#
# Actors (10-11 used, 13 configured):
#   Base (10): supervisor, sensor, estimator, waypoint, altitude,
#              position, attitude, rate, motor, flight_manager
#   Optional: telemetry (only on platforms with HAL_HAS_RADIO)
#
# Buses (7 used, 8 configured):
#   sensor_bus, state_bus, thrust_bus, position_target_bus,
#   attitude_setpoint_bus, rate_setpoint_bus, torque_bus
#
# Bus subscribers (max 6 per bus on state_bus, 7 configured):
#   sensor_bus:           estimator, [telemetry] (1-2)
#   state_bus:            altitude, attitude, position, rate, waypoint, [telemetry] (5-6)
#   thrust_bus:           rate, [telemetry] (1-2)
#   position_target_bus:  altitude, position (2)
#   attitude_setpoint_bus: attitude (1)
#   rate_setpoint_bus:    rate (1)
#   torque_bus:           motor (1)
#
# Timers (4-5 concurrent peak, 10 configured):
#   sensor: tick timer (1 periodic)
#   flight_manager: sync_timer (1 periodic) + flight_timer (1 one-shot)
#   waypoint: hover_timer (1 one-shot, on demand)
#   [telemetry]: tick timer (1 periodic, only with HAL_HAS_RADIO)
#
# Monitors (9 used, 12 configured):
#   supervisor monitors 9 children
#
# Links: 0 used, 8 configured
#
# Supervisors (1 used, 1 configured):
#   pipeline supervisor for all flight actors
#
# Supervisor children (10 used, 12 configured):
#   9 flight actors + optional telemetry
#
# Mailbox entries: ~7 peak, 16 configured
#   IPC: START, LANDING, LANDED, STOP notifications
#   Timer messages: up to 5 concurrent
#
# Message data pool: ~14 peak, 32 configured
#   7 bus entries + ~7 IPC messages in flight
#
# Message size: 64 bytes max (sensor_data_t), 128 configured
#
# ============================================================================

# Pilot uses 10-11 actors (9 children + 1 supervisor + optional telemetry), 7 buses
HIVE_CFLAGS += -DHIVE_MAX_ACTORS=13
HIVE_CFLAGS += -DHIVE_MAX_BUSES=8

# IPC pool sizes - sized for pilot's messaging patterns
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=32
HIVE_CFLAGS += -DHIVE_LINK_ENTRY_POOL_SIZE=8
HIVE_CFLAGS += -DHIVE_MONITOR_ENTRY_POOL_SIZE=12
HIVE_CFLAGS += -DHIVE_TIMER_ENTRY_POOL_SIZE=10

# Bus configuration (increased for telemetry subscriptions)
HIVE_CFLAGS += -DHIVE_MAX_BUS_SUBSCRIBERS=7
HIVE_CFLAGS += -DHIVE_MAX_BUS_ENTRIES=4

# Message size - enough for sensor_data_t (~64 bytes)
HIVE_CFLAGS += -DHIVE_MAX_MESSAGE_SIZE=128

# Supervisor configuration - pilot uses 1 supervisor with 10 children
# This significantly reduces RAM (default 8 supervisors × 16 children = ~25 KB)
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISORS=1
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISOR_CHILDREN=12
