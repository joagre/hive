# Makefile for pilot example (Webots controller)
#
# This builds a Webots controller that uses the hive actor runtime.
# Builds its own libhive.a with tight memory limits matching STM32 production.
#
# Usage:
#   make              - Build and install to Webots (always use this)
#   make build        - Build only (without install)
#   make clean        - Remove build artifacts
#
# NOTE: Always use 'make' (not 'make build') to ensure Webots runs the latest
# controller. Webots caches the controller binary and won't see changes unless
# you install to the controllers/ directory.
#
# Requirements:
#   - WEBOTS_HOME environment variable set

# Webots paths
WEBOTS_HOME ?= /usr/local/webots
WEBOTS_INCLUDE = $(WEBOTS_HOME)/include/controller/c
WEBOTS_LIB = $(WEBOTS_HOME)/lib/controller

# Hive runtime paths (relative to this directory)
HIVE_ROOT = ../..
HIVE_INCLUDE = $(HIVE_ROOT)/include
HIVE_SRC_DIR = $(HIVE_ROOT)/src

# Build directory
BUILD_DIR = build

# Compiler settings
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -I$(WEBOTS_INCLUDE) -I$(HIVE_INCLUDE) -I. -Iinclude -Ihal/webots-crazyflie
# Include Linux HAL for hive_hal_context_defs.h
CFLAGS += -I$(HIVE_SRC_DIR)/hal/linux
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H
CFLAGS += -MMD -MP

# Webots simulation uses simulated time
CFLAGS += -DSIMULATED_TIME

# Enable telemetry logging for Webots (disabled by default on Crazyflie)
CFLAGS += -DENABLE_TELEMETRY_LOG=1

# Flight profile selection (see waypoint_actor.c for details)
#   FLIGHT_PROFILE_FIRST_TEST (1) - First flight test: hover at 0.5m, land
#   FLIGHT_PROFILE_ALTITUDE   (2) - Altitude-only waypoints (no lateral movement)
#   FLIGHT_PROFILE_FULL_3D    (3) - Full 3D waypoint navigation (default)
# Example: make FLIGHT_PROFILE=1
ifdef FLIGHT_PROFILE
CFLAGS += -DFLIGHT_PROFILE=$(FLIGHT_PROFILE)
endif

# Platform identification (enables platform-specific defaults in hive_static_config.h)
CFLAGS += -DHIVE_PLATFORM_LINUX

# Log level: INFO for simulation (hides 250Hz debug spam that slows Webots)
# Use: make LOG_LEVEL=NONE to disable, make LOG_LEVEL=DEBUG for detailed output
LOG_LEVEL ?= INFO
CFLAGS += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_$(LOG_LEVEL)

# Stack profiling: enable watermarking to measure actual stack usage
# Use: make STACK_PROFILE=1 to enable stack usage reporting
ifdef STACK_PROFILE
CFLAGS += -DHIVE_STACK_WATERMARK=1
endif

# Hive memory configuration (shared pilot settings + platform stack sizes)
include hive_config.mk
CFLAGS += $(HIVE_CFLAGS)
# Stack: 4KB per actor, Arena: 52KB (11 actors × 4KB + block headers)
# Note: No HAL_HAS_RADIO on Webots, so comms actor not included
# Telemetry logger adds one more actor when ENABLE_TELEMETRY_LOG=1 (default in sim)
CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=4096
CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(52*1024)'

LDFLAGS = -L$(WEBOTS_LIB) -L$(BUILD_DIR)
LDLIBS = -lController -lm

# Target
TARGET = pilot

# ------------------------------------------------------------------------------
# Source Files
# ------------------------------------------------------------------------------

# Pilot application code
ACTOR_SRCS = sensor_actor.c estimator_actor.c altitude_actor.c waypoint_actor.c \
             position_actor.c attitude_actor.c rate_actor.c motor_actor.c \
             flight_manager_actor.c telemetry_logger_actor.c
HAL_SRCS = hal/webots-crazyflie/hal_webots.c
FUSION_SRCS = fusion/complementary_filter.c fusion/altitude_kf.c
UTIL_SRCS = stack_profile.c
PILOT_SRCS = pilot.c pid.c $(ACTOR_SRCS) $(HAL_SRCS) $(FUSION_SRCS) $(UTIL_SRCS)

# Hive runtime (Linux x86-64 platform)
HIVE_CORE_SRCS = hive_actor.c hive_bus.c hive_ipc.c \
                 hive_link.c hive_log.c hive_pool.c hive_runtime.c hive_select.c \
                 hive_supervisor.c hive_scheduler.c hive_timer_linux.c \
                 hive_net.c hive_file.c

# HAL source files (Linux platform)
HIVE_HAL_SRCS = hal/linux/hive_hal_linux.c hal/linux/hive_hal_context_linux.c

HIVE_ASM = hal/linux/hive_context_x86_64.S

# ------------------------------------------------------------------------------
# Object Files
# ------------------------------------------------------------------------------

PILOT_OBJS = $(PILOT_SRCS:.c=.o)
PILOT_DEPS = $(PILOT_SRCS:.c=.d)

HIVE_OBJS = $(HIVE_CORE_SRCS:%.c=$(BUILD_DIR)/hive/%.o)
HIVE_HAL_OBJS = $(HIVE_HAL_SRCS:%.c=$(BUILD_DIR)/hive/%.o)
HIVE_ASM_OBJS = $(HIVE_ASM:%.S=$(BUILD_DIR)/hive/%.o)
HIVE_DEPS = $(HIVE_OBJS:.o=.d)

# Lightweight printf library object
PRINTF_OBJ = $(BUILD_DIR)/hive/printf.o

LOCAL_HIVE_LIB = $(BUILD_DIR)/libhive.a

# Webots controller directory (for install)
CONTROLLER_DIR = controllers/pilot

# ------------------------------------------------------------------------------
# Build Rules
# ------------------------------------------------------------------------------

.PHONY: all build clean install check-webots clean-emacs

# Default target: build AND install (so Webots always runs latest code)
all: install

# Build only (use 'make' instead to ensure Webots gets updated)
build: check-webots $(TARGET)

# --- Hive library ---

$(BUILD_DIR)/hive:
	mkdir -p $@

$(BUILD_DIR)/hive/hal/linux:
	mkdir -p $@

$(BUILD_DIR)/hive/%.o: $(HIVE_SRC_DIR)/%.c | $(BUILD_DIR)/hive
	@echo "CC (hive) $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# HAL source files (need hal/linux subdirectory)
$(BUILD_DIR)/hive/hal/linux/%.o: $(HIVE_SRC_DIR)/hal/linux/%.c | $(BUILD_DIR)/hive/hal/linux
	@echo "CC (hal) $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/hive/%.o: $(HIVE_SRC_DIR)/%.S | $(BUILD_DIR)/hive
	@echo "AS (hive) $<"
	@$(CC) -g -c $< -o $@

# HAL assembly (in hal/linux subdirectory)
$(BUILD_DIR)/hive/hal/linux/%.o: $(HIVE_SRC_DIR)/hal/linux/%.S | $(BUILD_DIR)/hive/hal/linux
	@echo "AS (hal) $<"
	@$(CC) -g -c $< -o $@

# Compile lightweight printf library
$(PRINTF_OBJ): $(HIVE_ROOT)/lib/printf/printf.c | $(BUILD_DIR)/hive
	@echo "CC (printf) $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(LOCAL_HIVE_LIB): $(HIVE_OBJS) $(HIVE_HAL_OBJS) $(HIVE_ASM_OBJS) $(PRINTF_OBJ)
	@echo "AR $@"
	@ar rcs $@ $^

# --- Pilot application ---

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

$(TARGET): $(PILOT_OBJS) $(LOCAL_HIVE_LIB)
	$(CC) $(LDFLAGS) -o $@ $(PILOT_OBJS) $(LOCAL_HIVE_LIB) $(LDLIBS)

# ------------------------------------------------------------------------------
# Utility Targets
# ------------------------------------------------------------------------------

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(PILOT_OBJS) $(PILOT_DEPS)

clean-emacs:
	find . -name '*~' -delete

install: check-webots $(TARGET)
	@mkdir -p $(CONTROLLER_DIR)
	@cp $(TARGET) $(CONTROLLER_DIR)/
	@echo "Installed to $(CONTROLLER_DIR)/$(TARGET)"

check-webots:
	@if [ ! -d "$(WEBOTS_HOME)" ]; then \
		echo "Error: WEBOTS_HOME not set or invalid: $(WEBOTS_HOME)"; \
		echo "Set WEBOTS_HOME to your Webots installation directory"; \
		exit 1; \
	fi

# Auto-generated dependencies
-include $(PILOT_DEPS)
-include $(HIVE_DEPS)
