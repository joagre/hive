# Makefile - Build pilot for Webots simulation
#
# Usage:
#   make
#   make FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST
#   make help
#
# Requirements:
#   WEBOTS_HOME environment variable set

# =============================================================================
# Build Configuration (override via command line)
# =============================================================================

FLIGHT_PROFILE := FLIGHT_PROFILE_FULL_3D
LOG_LEVEL := INFO
LOG_TO_STDOUT := 1
LOG_TO_FILE := 1
STACK_PROFILE := 0
SENSOR_NOISE := 3
MOTOR_LAG := 20

# =============================================================================
# Paths
# =============================================================================

WEBOTS_HOME ?= /usr/local/webots
HIVE_ROOT := ../..
HIVE_SRC_DIR := $(HIVE_ROOT)/src
HAL_DIR := hal/webots-crazyflie

BUILD_DIR := build
TARGET := pilot

# =============================================================================
# Toolchain
# =============================================================================

CC := gcc

# =============================================================================
# Hive Library Config (shared with Crazyflie build)
# =============================================================================

include hive_config.mk

# =============================================================================
# Webots Stack Config
# =============================================================================

# Smaller stacks sufficient for simulation
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=4096
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(52*1024)'

# =============================================================================
# Compiler Flags
# =============================================================================

# Base
CFLAGS := -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -MMD -MP

# Platform
CFLAGS += -DSIMULATED_TIME
CFLAGS += -DHIVE_PLATFORM_LINUX

# Includes
CFLAGS += -I$(WEBOTS_HOME)/include/controller/c
CFLAGS += -I$(HIVE_ROOT)/include -I$(HIVE_SRC_DIR) -I. -Iinclude -I$(HAL_DIR)
CFLAGS += -I$(HIVE_SRC_DIR)/hal/linux
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Pilot config
CFLAGS += -DFLIGHT_PROFILE=$(FLIGHT_PROFILE)
CFLAGS += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_$(LOG_LEVEL)
CFLAGS += -DHIVE_LOG_TO_STDOUT=$(LOG_TO_STDOUT)
CFLAGS += -DHIVE_LOG_TO_FILE=$(LOG_TO_FILE)

# Simulation config
CFLAGS += -DSENSOR_NOISE=$(SENSOR_NOISE)
CFLAGS += -DMOTOR_TIME_CONSTANT_MS=$(MOTOR_LAG)

# Hive library config
CFLAGS += $(HIVE_CFLAGS)

# Optional stack profiling
ifeq ($(STACK_PROFILE),1)
CFLAGS += -DHIVE_STACK_WATERMARK=1
endif

# =============================================================================
# Linker Flags
# =============================================================================

LDFLAGS := -L$(WEBOTS_HOME)/lib/controller -L$(BUILD_DIR)
LDLIBS := -lhive -lController -lm

# =============================================================================
# Sources
# =============================================================================

PILOT_SRCS := pilot.c pid.c stack_profile.c tunable_params.c \
	sensor_actor.c estimator_actor.c altitude_actor.c waypoint_actor.c \
	position_actor.c attitude_actor.c rate_actor.c motor_actor.c \
	flight_manager_actor.c telemetry_logger_actor.c \
	$(HAL_DIR)/hal_init.c \
	$(HAL_DIR)/hal_sensors.c \
	$(HAL_DIR)/hal_motors.c \
	$(HAL_DIR)/hal_time.c \
	$(HAL_DIR)/hal_led.c \
	$(HAL_DIR)/hal_debug.c \
	fusion/complementary_filter.c fusion/altitude_kf.c

PILOT_OBJS := $(PILOT_SRCS:.c=.o)
PILOT_DEPS := $(PILOT_SRCS:.c=.d)

HIVE_LIB := $(BUILD_DIR)/libhive.a
BOARD_MOUNTS_SRC := $(abspath $(HAL_DIR)/hive_mounts.c)

# =============================================================================
# Targets
# =============================================================================

.PHONY: all build install clean check-webots help FORCE

all: install

build: check-webots $(TARGET)

FORCE:

$(HIVE_LIB): FORCE
	@$(MAKE) -C $(HIVE_SRC_DIR) BUILD_DIR=$(abspath $(BUILD_DIR)) \
		BOARD_MOUNTS_SRC=$(BOARD_MOUNTS_SRC) \
		CFLAGS="$(CFLAGS)"

# Config files that affect all builds
CONFIG_DEPS := hive_config.mk

%.o: %.c $(CONFIG_DEPS)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(PILOT_OBJS) $(HIVE_LIB)
	$(CC) $(LDFLAGS) -o $@ $(PILOT_OBJS) $(LDLIBS)

install: check-webots $(TARGET)
	@mkdir -p controllers/pilot
	@cp $(TARGET) controllers/pilot/
	@echo "Installed to controllers/pilot/$(TARGET)"

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(PILOT_OBJS) $(PILOT_DEPS)

check-webots:
	@test -d "$(WEBOTS_HOME)" || { echo "Error: WEBOTS_HOME not set"; exit 1; }

help:
	@echo "Pilot Webots Controller"
	@echo ""
	@echo "Usage:"
	@echo "  make [options] [target]"
	@echo ""
	@echo "Targets:"
	@echo "  all       Build and install (default)"
	@echo "  build     Build only"
	@echo "  install   Install to controllers/pilot/"
	@echo "  clean     Remove build artifacts"
	@echo "  help      Show this help"
	@echo ""
	@echo "Options (current defaults shown):"
	@echo "  FLIGHT_PROFILE=$(FLIGHT_PROFILE)"
	@echo "      FLIGHT_PROFILE_GROUND_TEST  - Motors disabled"
	@echo "      FLIGHT_PROFILE_FIRST_TEST   - Hover 0.5m, land"
	@echo "      FLIGHT_PROFILE_ALTITUDE     - Altitude waypoints"
	@echo "      FLIGHT_PROFILE_FULL_3D      - Full 3D pattern"
	@echo ""
	@echo "  LOG_LEVEL=$(LOG_LEVEL)"
	@echo "      TRACE, DEBUG, INFO, WARN, ERROR, NONE"
	@echo ""
	@echo "  LOG_TO_STDOUT=$(LOG_TO_STDOUT)"
	@echo "  LOG_TO_FILE=$(LOG_TO_FILE)"
	@echo "  STACK_PROFILE=$(STACK_PROFILE)"
	@echo ""
	@echo "  SENSOR_NOISE=$(SENSOR_NOISE)  (0=clean, 3=realistic)"
	@echo "  MOTOR_LAG=$(MOTOR_LAG)        (ms response delay)"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST"
	@echo "  make SENSOR_NOISE=0 LOG_LEVEL=DEBUG"

-include $(PILOT_DEPS)
