# Pilot HAL Tests Makefile
#
# Builds tests for validating the pilot HAL and Hive file I/O.
#
# Usage:
#   make PLATFORM=crazyflie TEST=sensors_motors   # Build sensor/motor test
#   make PLATFORM=crazyflie TEST=flash            # Build flash storage test
#   make PLATFORM=crazyflie TEST=sd               # Build SD card test
#   make PLATFORM=crazyflie all-tests             # Build all tests
#   make flash-crazyflie TEST=sensors_motors      # Flash specific test
#   make clean                                    # Remove build artifacts
#
# Tests:
#   sensors_motors - HAL API validation (sensors, motors, calibration)
#   flash          - Hive file API test with flash backend (/log)
#   sd             - Hive file API test with SD backend (/sd)

# Default target shows help
.PHONY: all help clean

all: help

help:
	@echo "Pilot HAL Tests"
	@echo ""
	@echo "Usage:"
	@echo "  make PLATFORM=crazyflie TEST=<test>  Build specific test"
	@echo "  make PLATFORM=crazyflie all-tests    Build all tests"
	@echo "  make PLATFORM=webots TEST=<test>     Build for Webots"
	@echo "  make flash-crazyflie TEST=<test>     Flash specific test"
	@echo "  make clean                           Remove all build artifacts"
	@echo ""
	@echo "Available tests:"
	@echo "  sensors_motors  - HAL API validation (sensors, motors)"
	@echo "  flash           - Hive file API test (flash backend)"
	@echo "  sd              - Hive file API test (SD backend)"
	@echo ""
	@echo "Examples:"
	@echo "  make PLATFORM=crazyflie TEST=sensors_motors"
	@echo "  make PLATFORM=crazyflie TEST=flash"
	@echo "  make flash-crazyflie TEST=flash"

# Build directories
BUILD_DIR_CF = build_crazyflie
BUILD_DIR_WB = build_webots

# Pilot directory (parent)
PILOT_DIR = ..
HIVE_ROOT = ../..
HIVE_SRC_DIR = $(HIVE_ROOT)/src

# Test selection
TEST ?= sensors_motors

# Tests that require libhive.a (use Hive file API)
HIVE_TESTS = flash sd

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
HAL_DIR = $(PILOT_DIR)/hal/crazyflie-2.1+

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
CFLAGS += -MMD -MP
CFLAGS += -I$(PILOT_DIR)
CFLAGS += -I$(HAL_DIR)
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Include
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Device/ST/STM32F4xx/Include

# Hive includes
CFLAGS += -I$(HIVE_ROOT)/include
CFLAGS += -I$(HIVE_SRC_DIR)/hal/stm32
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Hive memory configuration (minimal for tests)
HIVE_CFLAGS = -DHIVE_MAX_ACTORS=4
HIVE_CFLAGS += -DHIVE_MAX_BUSES=2
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=2048
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(16*1024)'

# STM32 flash file configuration
HIVE_CFLAGS += -DHIVE_VFILE_LOG_BASE=0x08080000
HIVE_CFLAGS += -DHIVE_VFILE_LOG_SIZE=131072
HIVE_CFLAGS += -DHIVE_VFILE_LOG_SECTOR=8
HIVE_CFLAGS += '-DHIVE_FILE_RING_SIZE=(8*1024)'

# Add Hive config to CFLAGS
CFLAGS += $(HIVE_CFLAGS)

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
HIVE_LIB = $(BUILD_DIR_CF)/libhive.a

# Test source file
TEST_SRC = test_$(TEST).c

# Target
TARGET_ELF = $(BUILD_DIR_CF)/test_$(TEST).elf
TARGET_BIN = $(BUILD_DIR_CF)/test_$(TEST).bin

# Check if this test needs libhive.a
NEEDS_HIVE = $(filter $(TEST),$(HIVE_TESTS))

.PHONY: crazyflie all-tests

# Main build target - conditionally include libhive.a
ifeq ($(NEEDS_HIVE),)
# sensors_motors: only needs HAL
crazyflie: $(HAL_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)
else
# flash/sd: needs HAL + libhive.a
crazyflie: $(HAL_LIB) $(HIVE_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)
endif

all-tests:
	@$(MAKE) PLATFORM=crazyflie TEST=sensors_motors
	@$(MAKE) PLATFORM=crazyflie TEST=flash
	@$(MAKE) PLATFORM=crazyflie TEST=sd

$(BUILD_DIR_CF):
	mkdir -p $@

# Build pilot HAL library
$(HAL_LIB):
	@echo "Building pilot HAL library..."
	@$(MAKE) -C $(HAL_DIR)

# Build Hive library for STM32
$(HIVE_LIB): | $(BUILD_DIR_CF)
	@echo "Building Hive library for STM32..."
	@$(MAKE) -C $(HIVE_SRC_DIR) \
		BUILD_DIR=$(abspath $(BUILD_DIR_CF)) \
		PLATFORM=stm32 \
		CC=$(CC) \
		AR=$(AR) \
		CFLAGS="$(MCU) -std=c11 -Os -g3 -ffunction-sections -fdata-sections -DSTM32F405xx -DHIVE_PLATFORM_STM32 $(HIVE_CFLAGS) -I$(abspath $(HAL_DIR))/vendor/CMSIS/Include -I$(abspath $(HAL_DIR))/vendor/CMSIS/Device/ST/STM32F4xx/Include"

# Build test binary - conditionally link against libhive.a
ifeq ($(NEEDS_HIVE),)
$(TARGET_ELF): $(TEST_SRC) | $(BUILD_DIR_CF)
	@echo "CC $(TEST_SRC) (Crazyflie - HAL only)"
	@$(CC) $(CFLAGS) $< $(HAL_LIB) $(LDFLAGS) -o $@
else
$(TARGET_ELF): $(TEST_SRC) | $(BUILD_DIR_CF)
	@echo "CC $(TEST_SRC) (Crazyflie - HAL + Hive)"
	@$(CC) $(CFLAGS) $< $(HIVE_LIB) $(HAL_LIB) $(LDFLAGS) -o $@
endif

$(TARGET_BIN): $(TARGET_ELF)
	@echo "BIN test_$(TEST).bin"
	@$(CP) -O binary $< $@

# Include dependencies
-include $(BUILD_DIR_CF)/test_$(TEST).d

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

# Compiler
CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -DSIMULATED_TIME
CFLAGS += -DHIVE_PLATFORM_LINUX
CFLAGS += -MMD -MP
CFLAGS += -I$(WEBOTS_INCLUDE)
CFLAGS += -I$(PILOT_DIR)
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

LDFLAGS = -L$(WEBOTS_LIB)
LDLIBS = -lController -lm

# Libraries
HIVE_LIB = $(BUILD_DIR_WB)/libhive.a

# Test source file
TEST_SRC = test_$(TEST).c

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

# Build Hive library for Linux
$(HIVE_LIB): | $(BUILD_DIR_WB)
	@echo "Building Hive library for Linux..."
	@$(MAKE) -C $(HIVE_SRC_DIR) \
		BUILD_DIR=$(abspath $(BUILD_DIR_WB)) \
		PLATFORM=linux \
		CFLAGS="$(HIVE_CFLAGS)"

# Main build target - conditionally include libhive.a
ifeq ($(NEEDS_HIVE),)
webots: check-webots $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Copy to controllers/test_$(TEST)/ and run in Webots"

$(TARGET): $(TEST_SRC) $(HAL_DIR)/hal_webots.c | $(BUILD_DIR_WB)
	@echo "CC $(TEST_SRC) (Webots - HAL only)"
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LDLIBS) -o $@
else
webots: check-webots $(HIVE_LIB) $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Note: flash and sd tests are Crazyflie-specific"

$(TARGET): $(TEST_SRC) $(HAL_DIR)/hal_webots.c $(HIVE_LIB) | $(BUILD_DIR_WB)
	@echo "CC $(TEST_SRC) (Webots - HAL + Hive)"
	@$(CC) $(CFLAGS) $(TEST_SRC) $(HAL_DIR)/hal_webots.c $(HIVE_LIB) $(LDFLAGS) $(LDLIBS) -o $@
endif

# Include dependencies
-include $(BUILD_DIR_WB)/test_$(TEST).d

endif # PLATFORM=webots

# ==============================================================================
# Flash and Run Targets
# ==============================================================================

.PHONY: flash-crazyflie

flash-crazyflie: $(BUILD_DIR_CF)/test_$(TEST).bin
	@echo "Flashing test_$(TEST) to Crazyflie..."
	st-flash write $< 0x8000000

# ==============================================================================
# Clean
# ==============================================================================

clean:
	rm -rf $(BUILD_DIR_CF) $(BUILD_DIR_WB)
