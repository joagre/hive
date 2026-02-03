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
#   Optional: comms (HAL_HAS_RADIO), telemetry_logger (SIMULATED_TIME)
#
# Buses (7 used, 8 configured):
#   sensor_bus, state_bus, thrust_bus, position_target_bus,
#   attitude_setpoint_bus, rate_setpoint_bus, torque_bus
#
# Bus subscribers (max 7 per bus on state_bus, 8 configured):
#   sensor_bus:           estimator, [comms], [tlog] (1-3)
#   state_bus:            altitude, attitude, position, rate, waypoint, [comms], [tlog] (5-7)
#   thrust_bus:           rate, [comms], [tlog] (1-3)
#   position_target_bus:  altitude, position, [tlog] (2-3)
#   attitude_setpoint_bus: attitude (1)
#   rate_setpoint_bus:    rate (1)
#   torque_bus:           motor (1)
#   [comms] = comms_actor (HAL_HAS_RADIO), [tlog] = telemetry_logger (always)
#
# Timers (4-6 concurrent peak, 10 configured):
#   sensor: tick timer (1 periodic)
#   flight_manager: sync_timer (1 periodic) + flight_timer (1 one-shot)
#   waypoint: hover_timer (1 one-shot, on demand)
#   [tlog]: tick timer (1 periodic, only with SIMULATED_TIME)
#
# Monitors (9-11 used, 12 configured):
#   supervisor monitors 9-11 children (depending on comms/tlog)
#
# Links: 0 used, 8 configured
#
# Supervisors (1 used, 1 configured):
#   pipeline supervisor for all flight actors
#
# Supervisor children (9-11 used, 12 configured):
#   9 flight actors + optional comms + optional tlog (all supervised)
#   comms: HAL_HAS_RADIO, tlog: SIMULATED_TIME (both TEMPORARY)
#
# Mailbox entries: ~13 peak, 32 configured
#   IPC: START, LANDING, LANDED, STOP notifications
#   Timer messages: up to 6 concurrent
#   Buffer for pool_block wake-up latency
#
# Message data pool: ~35 peak, 64 configured
#   7 buses x 4 entries = 28 max bus entries
#   Plus IPC messages and timer messages in flight
#
# Message size: 64 bytes max (sensor_data_t), 128 configured
#
# ============================================================================

# Pilot uses 11-12 actors (10-11 children + 1 supervisor), 7 buses
HIVE_CFLAGS += -DHIVE_MAX_ACTORS=13
HIVE_CFLAGS += -DHIVE_MAX_BUSES=8

# IPC pool sizes - sized for pilot's messaging patterns
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=32
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=64
HIVE_CFLAGS += -DHIVE_LINK_ENTRY_POOL_SIZE=8
HIVE_CFLAGS += -DHIVE_MONITOR_ENTRY_POOL_SIZE=12
HIVE_CFLAGS += -DHIVE_TIMER_ENTRY_POOL_SIZE=10

# Bus configuration (7 subscribers on state_bus when comms+tlog both active)
HIVE_CFLAGS += -DHIVE_MAX_BUS_SUBSCRIBERS=8
HIVE_CFLAGS += -DHIVE_MAX_BUS_ENTRIES=4

# Message size - enough for sensor_data_t (~64 bytes)
HIVE_CFLAGS += -DHIVE_MAX_MESSAGE_SIZE=128

# Supervisor configuration - pilot uses 1 supervisor with 10 children
# This significantly reduces RAM (default 8 supervisors × 16 children = ~25 KB)
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISORS=1
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISOR_CHILDREN=12
