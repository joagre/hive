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
# Actors (10 used, 12 configured):
#   supervisor, sensor, estimator, waypoint, altitude,
#   position, attitude, rate, motor, flight_manager
#
# Buses (7 used, 8 configured):
#   sensor_bus, state_bus, thrust_bus, position_target_bus,
#   attitude_setpoint_bus, rate_setpoint_bus, torque_bus
#
# Bus subscribers (max 5 per bus on state_bus, 6 configured):
#   sensor_bus:           estimator (1)
#   state_bus:            altitude, attitude, position, rate, waypoint (5)
#   thrust_bus:           rate (1)
#   position_target_bus:  altitude, position (2)
#   attitude_setpoint_bus: attitude (1)
#   rate_setpoint_bus:    rate (1)
#   torque_bus:           motor (1)
#
# Timers (4 concurrent peak, 8 configured):
#   sensor: tick timer (1 periodic)
#   flight_manager: sync_timer (1 periodic) + flight_timer (1 one-shot)
#   waypoint: hover_timer (1 one-shot, on demand)
#
# Monitors (9 used, 12 configured):
#   supervisor monitors 9 children
#
# Links: 0 used, 8 configured
#
# Mailbox entries: ~6 peak, 16 configured
#   IPC: START, LANDING, LANDED, STOP notifications
#   Timer messages: up to 4 concurrent
#
# Message data pool: ~13 peak, 32 configured
#   7 bus entries + ~6 IPC messages in flight
#
# Message size: 64 bytes max (sensor_data_t), 128 configured
#
# ============================================================================

# Pilot uses 10 actors (9 children + 1 supervisor), 7 buses
HIVE_CFLAGS += -DHIVE_MAX_ACTORS=12
HIVE_CFLAGS += -DHIVE_MAX_BUSES=8

# IPC pool sizes - sized for pilot's messaging patterns
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=32
HIVE_CFLAGS += -DHIVE_LINK_ENTRY_POOL_SIZE=8
HIVE_CFLAGS += -DHIVE_MONITOR_ENTRY_POOL_SIZE=12
HIVE_CFLAGS += -DHIVE_TIMER_ENTRY_POOL_SIZE=8

# Bus configuration
HIVE_CFLAGS += -DHIVE_MAX_BUS_SUBSCRIBERS=6
HIVE_CFLAGS += -DHIVE_MAX_BUS_ENTRIES=4

# Message size - enough for sensor_data_t (~64 bytes)
HIVE_CFLAGS += -DHIVE_MAX_MESSAGE_SIZE=128
