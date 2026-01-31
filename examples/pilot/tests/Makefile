# Pilot HAL Tests Makefile
#
# Builds tests for validating the pilot HAL and Hive file I/O.
#
# Usage:
#   make PLATFORM=crazyflie TEST=sensors_motors   # Build sensor/motor test
#   make PLATFORM=crazyflie TEST=flash            # Build flash storage test
#   make PLATFORM=crazyflie TEST=sd               # Build SD card test
#   make PLATFORM=crazyflie TEST=esb              # Build ESB radio communication test
#   make PLATFORM=crazyflie TEST=main             # Build combined test runner
#   make PLATFORM=crazyflie all-tests             # Build all tests
#   make PLATFORM=webots TEST=main                # Build for Webots (dummy pilot)
#   make flash-crazyflie TEST=sensors_motors      # Flash specific test
#   make clean                                    # Remove build artifacts
#
# Tests:
#   sensors_motors - HAL API validation (sensors, motors, calibration)
#   thrust         - Thrust calibration test (find HAL_BASE_THRUST)
#   flash          - Hive file API test with flash backend (/log)
#   sd             - Hive file API test with SD backend (/sd)
#   esb            - ESB radio communication test (DMA-based syslink)
#   main           - Combined test runner (all applicable tests)

# Default target shows help
# FORCE target - always out of date, triggers submake to check real deps
.PHONY: all help clean FORCE

all: help

help:
	@echo "Pilot HAL Tests"
	@echo ""
	@echo "Usage:"
	@echo "  make PLATFORM=crazyflie TEST=<test>  Build specific test"
	@echo "  make PLATFORM=crazyflie all-tests    Build all tests"
	@echo "  make PLATFORM=webots TEST=<test>     Build for Webots"
	@echo "  make install-webots                  Build and install test_main controller"
	@echo "  make flash-crazyflie TEST=<test>     Flash specific test"
	@echo "  make clean                           Remove all build artifacts"
	@echo ""
	@echo "Available tests:"
	@echo "  sensors_motors  - HAL API validation (sensors, motors)"
	@echo "  thrust          - Thrust calibration (find HAL_BASE_THRUST)"
	@echo "  flash           - Hive file API test (flash backend)"
	@echo "  sd              - Hive file API test (SD backend)"
	@echo "  esb             - ESB radio communication test"
	@echo "  main            - Combined test runner (Webots dummy pilot)"
	@echo ""
	@echo "Options:"
	@echo "  ENABLE_MOTOR_TEST=1     Enable motor test (disabled by default)"
	@echo ""
	@echo "Examples:"
	@echo "  make PLATFORM=crazyflie TEST=sensors_motors"
	@echo "  make PLATFORM=crazyflie TEST=sensors_motors ENABLE_MOTOR_TEST=1"
	@echo "  make PLATFORM=crazyflie TEST=thrust"
	@echo "  make PLATFORM=crazyflie TEST=thrust EXTRA_CFLAGS='-DTHRUST_TEST_VALUE=0.40f'"
	@echo "  make install-webots"
	@echo "  make flash-crazyflie TEST=thrust"

# Build directories
BUILD_DIR_CF = build_crazyflie
BUILD_DIR_WB = build_webots

# Pilot directory (parent)
PILOT_DIR = ..
HIVE_ROOT = ../../..
HIVE_SRC_DIR = $(HIVE_ROOT)/src

# Test selection
TEST ?= sensors_motors

# Motor test is disabled by default for safety (enable with ENABLE_MOTOR_TEST=1)
ENABLE_MOTOR_TEST ?= 0

# Tests that require libhive.a (use Hive file API or logging)
HIVE_TESTS = sensors_motors thrust flash sd esb main

# ==============================================================================
# Crazyflie 2.1+ Build
# ==============================================================================

ifeq ($(PLATFORM),crazyflie)

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AR = $(PREFIX)ar
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# HAL directory
HAL_DIR = $(PILOT_DIR)/hal/crazyflie-2.1plus

# MCU flags for STM32F405
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Compiler flags
CFLAGS = $(MCU)
CFLAGS += -std=c11
CFLAGS += -Wall -Wextra
CFLAGS += -Os
CFLAGS += -g3
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common
CFLAGS += -DSTM32F405xx
CFLAGS += -DHIVE_PLATFORM_STM32
CFLAGS += -DHIVE_LOG_TO_STDOUT=1
CFLAGS += -MMD -MP
CFLAGS += -I$(PILOT_DIR)
CFLAGS += -I$(PILOT_DIR)/include
CFLAGS += -I$(HAL_DIR)
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Include
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/STM32F4xx/Include

# Hive includes
CFLAGS += -I$(HIVE_ROOT)/include
CFLAGS += -I$(HIVE_SRC_DIR)/hal/stm32
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Extra CFLAGS from command line (e.g., EXTRA_CFLAGS='-DTHRUST_TEST_VALUE=0.40f')
CFLAGS += $(EXTRA_CFLAGS)

# Hive memory configuration (minimal for tests)
HIVE_CFLAGS = -DHIVE_MAX_ACTORS=4
HIVE_CFLAGS += -DHIVE_MAX_BUSES=2
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=2048
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(16*1024)'

# Board-specific configuration (flash, SD)
include $(HAL_DIR)/hive_board_config.mk

# Radio test needs HAL_HAS_RADIO and doesn't need SD
ifeq ($(TEST),esb)
CFLAGS += -DHAL_HAS_RADIO
ENABLE_SD = 0
endif

# Add Hive config to CFLAGS
CFLAGS += $(HIVE_CFLAGS)

# Motor test control
CFLAGS += -DENABLE_MOTOR_TEST=$(ENABLE_MOTOR_TEST)

# Linker flags
LDSCRIPT = $(HAL_DIR)/stm32f405_flash.ld
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm -lc

# Libraries
HAL_LIB = $(HAL_DIR)/build/libhal.a
HAL_STARTUP = $(HAL_DIR)/build/startup_stm32f405.o
HIVE_LIB = $(BUILD_DIR_CF)/libhive.a

# Target
TARGET_ELF = $(BUILD_DIR_CF)/test_$(TEST).elf
TARGET_BIN = $(BUILD_DIR_CF)/test_$(TEST).bin

# Check if this test needs libhive.a
NEEDS_HIVE = $(filter $(TEST),$(HIVE_TESTS))

.PHONY: crazyflie all-tests

# Main build target - handle test_main specially
ifeq ($(TEST),main)
# Combined test runner - needs all test objects + libhive.a
TEST_MAIN_OBJS = $(BUILD_DIR_CF)/test_sensors_motors.o \
                 $(BUILD_DIR_CF)/test_flash.o \
                 $(BUILD_DIR_CF)/test_sd.o

crazyflie: $(HAL_LIB) $(HIVE_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(BUILD_DIR_CF)/test_sensors_motors.o: test_sensors_motors.c | $(BUILD_DIR_CF)
	@echo "CC test_sensors_motors.c (object)"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(BUILD_DIR_CF)/test_flash.o: test_flash.c | $(BUILD_DIR_CF)
	@echo "CC test_flash.c (object)"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(BUILD_DIR_CF)/test_sd.o: test_sd.c | $(BUILD_DIR_CF)
	@echo "CC test_sd.c (object)"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(TARGET_ELF): test_main.c $(TEST_MAIN_OBJS) $(HAL_LIB) $(HIVE_LIB) | $(BUILD_DIR_CF)
	@echo "CC test_main.c (Crazyflie - combined)"
	@$(CC) $(CFLAGS) $< $(TEST_MAIN_OBJS) $(HAL_STARTUP) $(HIVE_LIB) $(HAL_LIB) $(LDFLAGS) -o $@

else ifeq ($(NEEDS_HIVE),)
# sensors_motors standalone: only needs HAL
crazyflie: $(HAL_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(TARGET_ELF): test_$(TEST).c $(HAL_LIB) | $(BUILD_DIR_CF)
	@echo "CC test_$(TEST).c (Crazyflie - HAL only)"
	@$(CC) $(CFLAGS) $< $(HAL_STARTUP) $(HAL_LIB) $(LDFLAGS) -o $@

else
# flash/sd standalone: needs HAL + libhive.a
crazyflie: $(HAL_LIB) $(HIVE_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(TARGET_ELF): test_$(TEST).c $(HAL_LIB) $(HIVE_LIB) | $(BUILD_DIR_CF)
	@echo "CC test_$(TEST).c (Crazyflie - HAL + Hive)"
	@$(CC) $(CFLAGS) $< $(HAL_STARTUP) $(HIVE_LIB) $(HAL_LIB) $(LDFLAGS) -o $@

endif

all-tests:
	@$(MAKE) PLATFORM=crazyflie TEST=sensors_motors
	@$(MAKE) PLATFORM=crazyflie TEST=thrust
	@$(MAKE) PLATFORM=crazyflie TEST=flash
	@$(MAKE) PLATFORM=crazyflie TEST=sd
	@$(MAKE) PLATFORM=crazyflie TEST=esb
	@$(MAKE) PLATFORM=crazyflie TEST=main

$(BUILD_DIR_CF):
	mkdir -p $@

# Build pilot HAL library - submake handles actual dependency checking
# If submake rebuilds, $(HAL_LIB) gets new timestamp and dependents rebuild
$(HAL_LIB): FORCE
	@$(MAKE) -C $(HAL_DIR) ENABLE_SD=$(ENABLE_SD)

# Board-specific mount configuration
BOARD_MOUNTS_SRC = $(abspath $(HAL_DIR)/hive_mounts.c)

# Build Hive library for STM32 - submake handles actual dependency checking
$(HIVE_LIB): FORCE | $(BUILD_DIR_CF)
	@$(MAKE) -C $(HIVE_SRC_DIR) PLATFORM=stm32 \
		BUILD_DIR=$(abspath $(BUILD_DIR_CF)) \
		CC=$(CC) \
		AR=$(AR) \
		BOARD_MOUNTS_SRC=$(BOARD_MOUNTS_SRC) \
		ENABLE_SD=$(ENABLE_SD) \
		CFLAGS="$(MCU) -std=c11 -Os -g3 -ffunction-sections -fdata-sections -DSTM32F405xx -DHIVE_PLATFORM_STM32 -DHIVE_LOG_TO_STDOUT=1 $(HIVE_CFLAGS) -I$(abspath $(HAL_DIR))/vendor/CMSIS/Include -I$(abspath $(HAL_DIR))/vendor/CMSIS/Device/ST/STM32F4xx/Include"

$(TARGET_BIN): $(TARGET_ELF)
	@echo "BIN test_$(TEST).bin"
	@$(CP) -O binary $< $@

# Include dependencies
-include $(BUILD_DIR_CF)/*.d

endif # PLATFORM=crazyflie

# ==============================================================================
# Webots Build
# ==============================================================================

ifeq ($(PLATFORM),webots)

# Webots paths
WEBOTS_HOME ?= /usr/local/webots
WEBOTS_INCLUDE = $(WEBOTS_HOME)/include/controller/c
WEBOTS_LIB = $(WEBOTS_HOME)/lib/controller

# HAL directory
HAL_DIR = $(PILOT_DIR)/hal/webots-crazyflie

# HAL source files
HAL_SRCS = $(HAL_DIR)/hal_init.c \
           $(HAL_DIR)/hal_sensors.c \
           $(HAL_DIR)/hal_motors.c \
           $(HAL_DIR)/hal_time.c \
           $(HAL_DIR)/hal_led.c \
           $(HAL_DIR)/hal_debug.c

# Compiler
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -DSIMULATED_TIME
CFLAGS += -DHIVE_PLATFORM_LINUX
CFLAGS += -MMD -MP
CFLAGS += -I$(WEBOTS_INCLUDE)
CFLAGS += -I$(PILOT_DIR)
CFLAGS += -I$(PILOT_DIR)/include
CFLAGS += -I$(HAL_DIR)

# Hive includes (for file tests)
CFLAGS += -I$(HIVE_ROOT)/include
CFLAGS += -I$(HIVE_SRC_DIR)/hal/linux
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Hive memory configuration
HIVE_CFLAGS = -DHIVE_MAX_ACTORS=4
HIVE_CFLAGS += -DHIVE_MAX_BUSES=2
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=4096
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(32*1024)'

CFLAGS += $(HIVE_CFLAGS)

# Motor test control
CFLAGS += -DENABLE_MOTOR_TEST=$(ENABLE_MOTOR_TEST)

LDFLAGS = -L$(WEBOTS_LIB)
LDLIBS = -lController -lm

# Libraries
HIVE_LIB = $(BUILD_DIR_WB)/libhive.a

# Target
TARGET = $(BUILD_DIR_WB)/test_$(TEST)

# Check if this test needs libhive.a
NEEDS_HIVE = $(filter $(TEST),$(HIVE_TESTS))

.PHONY: webots check-webots

check-webots:
	@if [ ! -d "$(WEBOTS_HOME)" ]; then \
		echo "Error: WEBOTS_HOME not set or invalid: $(WEBOTS_HOME)"; \
		echo "Set WEBOTS_HOME to your Webots installation directory"; \
		exit 1; \
	fi

$(BUILD_DIR_WB):
	mkdir -p $@

# Build Hive library for Linux - submake handles actual dependency checking
$(HIVE_LIB): FORCE | $(BUILD_DIR_WB)
	@$(MAKE) -C $(HIVE_SRC_DIR) \
		BUILD_DIR=$(abspath $(BUILD_DIR_WB)) \
		PLATFORM=linux \
		CFLAGS="$(HIVE_CFLAGS)"

# Main build target - handle test_main specially
ifeq ($(TEST),main)
# Combined test runner for Webots - only sensors_motors is applicable
TEST_MAIN_OBJS = $(BUILD_DIR_WB)/test_sensors_motors.o

webots: check-webots $(HIVE_LIB) $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Copy to controllers/test_main/ and run in Webots"

$(BUILD_DIR_WB)/test_sensors_motors.o: test_sensors_motors.c | $(BUILD_DIR_WB)
	@echo "CC test_sensors_motors.c (object)"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(TARGET): test_main.c $(TEST_MAIN_OBJS) $(HAL_SRCS) $(HIVE_LIB) | $(BUILD_DIR_WB)
	@echo "CC test_main.c (Webots - combined)"
	@$(CC) $(CFLAGS) test_main.c $(TEST_MAIN_OBJS) $(HAL_SRCS) $(HIVE_LIB) $(LDFLAGS) $(LDLIBS) -o $@

else ifeq ($(NEEDS_HIVE),)
# sensors_motors standalone
webots: check-webots $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Copy to controllers/test_$(TEST)/ and run in Webots"

$(TARGET): test_$(TEST).c $(HAL_SRCS) | $(BUILD_DIR_WB)
	@echo "CC test_$(TEST).c (Webots - HAL only)"
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LDLIBS) -o $@

else
# flash/sd standalone (not really applicable for Webots but included for completeness)
webots: check-webots $(HIVE_LIB) $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Note: flash and sd tests are Crazyflie-specific"

$(TARGET): test_$(TEST).c $(HAL_SRCS) $(HIVE_LIB) | $(BUILD_DIR_WB)
	@echo "CC test_$(TEST).c (Webots - HAL + Hive)"
	@$(CC) $(CFLAGS) test_$(TEST).c $(HAL_SRCS) $(HIVE_LIB) $(LDFLAGS) $(LDLIBS) -o $@

endif

# Include dependencies
-include $(BUILD_DIR_WB)/*.d

endif # PLATFORM=webots

# ==============================================================================
# Flash and Run Targets
# ==============================================================================

.PHONY: flash-crazyflie

flash-crazyflie: $(BUILD_DIR_CF)/test_$(TEST).bin
	@echo "Flashing test_$(TEST) to Crazyflie..."
	st-flash write $< 0x8000000

# ==============================================================================
# Webots Controller Install
# ==============================================================================

# Controller directory
CONTROLLERS_DIR = $(PILOT_DIR)/controllers

.PHONY: install-webots

install-webots: webots-main
	@mkdir -p $(CONTROLLERS_DIR)/test_main
	@cp $(BUILD_DIR_WB)/test_main $(CONTROLLERS_DIR)/test_main/
	@echo ""
	@echo "Installed: $(CONTROLLERS_DIR)/test_main/test_main"
	@echo "Ready to use in Webots simulation"

# Build test_main for webots (can be invoked without PLATFORM set)
.PHONY: webots-main
webots-main:
	$(MAKE) PLATFORM=webots TEST=main webots

# ==============================================================================
# Clean
# ==============================================================================

clean:
	rm -rf $(BUILD_DIR_CF) $(BUILD_DIR_WB)
