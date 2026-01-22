# Makefile for pilot example (Webots controller)
#
# Usage:
#   make              - Build and install to Webots
#   make help         - Show all build options
#
# Requirements:
#   - WEBOTS_HOME environment variable set

# Paths
WEBOTS_HOME ?= /usr/local/webots
HIVE_ROOT := ../..
HIVE_SRC_DIR := $(HIVE_ROOT)/src

BUILD_DIR := build
TARGET := pilot

# Compiler
CC := gcc

# Pilot CFLAGS
CFLAGS := -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -I$(WEBOTS_HOME)/include/controller/c
CFLAGS += -I$(HIVE_ROOT)/include -I. -Iinclude -Ihal/webots-crazyflie
CFLAGS += -I$(HIVE_SRC_DIR)/hal/linux
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H
CFLAGS += -MMD -MP
CFLAGS += -DSIMULATED_TIME
CFLAGS += -DENABLE_TELEMETRY_LOG=1
CFLAGS += -DHIVE_PLATFORM_LINUX

# Flight profile (default: FULL_3D for simulation)
ifdef FLIGHT_PROFILE
CFLAGS += -DFLIGHT_PROFILE=$(FLIGHT_PROFILE)
endif

# Sensor noise level (default: 3 = flow deck realistic)
ifdef SENSOR_NOISE
CFLAGS += -DSENSOR_NOISE=$(SENSOR_NOISE)
endif

# Motor lag simulation (default: 20ms)
ifdef MOTOR_LAG
CFLAGS += -DMOTOR_TIME_CONSTANT_MS=$(MOTOR_LAG)
endif

# Flow deck height limits
ifdef FLOW_MIN_HEIGHT
CFLAGS += -DFLOW_MIN_HEIGHT=$(FLOW_MIN_HEIGHT)
endif
ifdef FLOW_MAX_HEIGHT
CFLAGS += -DFLOW_MAX_HEIGHT=$(FLOW_MAX_HEIGHT)
endif

# Log level (default: INFO)
LOG_LEVEL ?= INFO
CFLAGS += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_$(LOG_LEVEL)

# Stack profiling
ifdef STACK_PROFILE
CFLAGS += -DHIVE_STACK_WATERMARK=1
endif

# Hive memory configuration
include hive_config.mk
CFLAGS += $(HIVE_CFLAGS)
CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=4096
CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(52*1024)'

# Linker flags
LDFLAGS := -L$(WEBOTS_HOME)/lib/controller -L$(BUILD_DIR)
LDLIBS := -lhive -lController -lm

# Pilot sources
PILOT_SRCS := pilot.c pid.c stack_profile.c \
	sensor_actor.c estimator_actor.c altitude_actor.c waypoint_actor.c \
	position_actor.c attitude_actor.c rate_actor.c motor_actor.c \
	flight_manager_actor.c telemetry_logger_actor.c \
	hal/webots-crazyflie/hal_webots.c \
	fusion/complementary_filter.c fusion/altitude_kf.c

PILOT_OBJS := $(PILOT_SRCS:.c=.o)
PILOT_DEPS := $(PILOT_SRCS:.c=.d)

HIVE_LIB := $(BUILD_DIR)/libhive.a

# CFLAGS to pass to hive build (memory config only, not pilot-specific stuff)
HIVE_CFLAGS_EXPORT := $(HIVE_CFLAGS)
HIVE_CFLAGS_EXPORT += -DHIVE_DEFAULT_STACK_SIZE=4096
HIVE_CFLAGS_EXPORT += '-DHIVE_STACK_ARENA_SIZE=(52*1024)'
HIVE_CFLAGS_EXPORT += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_$(LOG_LEVEL)

# Targets
.PHONY: all build install clean check-webots

all: install

build: check-webots $(TARGET)

# Build hive library (delegate to src/Makefile)
$(HIVE_LIB):
	@$(MAKE) -C $(HIVE_SRC_DIR) BUILD_DIR=$(abspath $(BUILD_DIR)) \
		CFLAGS="$(HIVE_CFLAGS_EXPORT)"

# Compile pilot sources
%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# Link pilot
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

.PHONY: help
help:
	@echo "Pilot Webots Controller - Build System"
	@echo ""
	@echo "Targets:"
	@echo "  make              Build and install to controllers/pilot/"
	@echo "  make build        Build only (no install)"
	@echo "  make clean        Remove build artifacts"
	@echo "  make help         Show this help"
	@echo ""
	@echo "Build Options:"
	@echo "  FLIGHT_PROFILE=<profile>  Flight profile (default: FULL_3D in code)"
	@echo "      FLIGHT_PROFILE_FIRST_TEST   - Gentle 0.3m hover, no lateral"
	@echo "      FLIGHT_PROFILE_HOVER_TEST   - 0.5m hover with position hold"
	@echo "      FLIGHT_PROFILE_FULL_3D      - Full 3D flight with waypoints"
	@echo ""
	@echo "  SENSOR_NOISE=<0-3>        Sensor noise level (default: 3)"
	@echo "      0 = Clean (no noise)"
	@echo "      1 = Low noise"
	@echo "      2 = Moderate noise"
	@echo "      3 = Flow deck realistic"
	@echo ""
	@echo "  MOTOR_LAG=<ms>            Motor time constant (default: 20)"
	@echo "      Simulates motor response delay in milliseconds"
	@echo ""
	@echo "  FLOW_MIN_HEIGHT=<m>       Flow deck minimum height (default: 0.1)"
	@echo "  FLOW_MAX_HEIGHT=<m>       Flow deck maximum height (default: 4.0)"
	@echo "      Height range where optical flow is valid"
	@echo ""
	@echo "  LOG_LEVEL=<level>         Log verbosity (default: INFO)"
	@echo "      TRACE, DEBUG, INFO, WARN, ERROR, NONE"
	@echo ""
	@echo "  STACK_PROFILE=1           Enable stack watermark profiling"
	@echo ""
	@echo "Examples:"
	@echo "  make FLIGHT_PROFILE=FLIGHT_PROFILE_FIRST_TEST"
	@echo "  make SENSOR_NOISE=0 LOG_LEVEL=DEBUG"
	@echo "  make MOTOR_LAG=50 STACK_PROFILE=1"

-include $(PILOT_DEPS)
