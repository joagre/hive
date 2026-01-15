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

# Pilot uses 9 actors, 7 buses - add small headroom
HIVE_CFLAGS += -DHIVE_MAX_ACTORS=10
HIVE_CFLAGS += -DHIVE_MAX_BUSES=8

# IPC pool sizes - sized for pilot's messaging patterns
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=32
HIVE_CFLAGS += -DHIVE_LINK_ENTRY_POOL_SIZE=8
HIVE_CFLAGS += -DHIVE_MONITOR_ENTRY_POOL_SIZE=8
HIVE_CFLAGS += -DHIVE_TIMER_ENTRY_POOL_SIZE=4

# Bus configuration
HIVE_CFLAGS += -DHIVE_MAX_BUS_SUBSCRIBERS=6
HIVE_CFLAGS += -DHIVE_MAX_BUS_ENTRIES=4

# Message size - enough for sensor/state structs
HIVE_CFLAGS += -DHIVE_MAX_MESSAGE_SIZE=128
