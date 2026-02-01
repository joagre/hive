# Pilot HAL Tests Makefile
#
# Usage:
#   make PLATFORM=crazyflie TEST=sensors_motors
#   make PLATFORM=crazyflie TEST=esb
#   make PLATFORM=crazyflie all-tests
#   make flash-crazyflie TEST=sensors_motors
#   make help

# =============================================================================
# Build Configuration (override via command line)
# =============================================================================

PLATFORM :=
TEST := sensors_motors
ENABLE_MOTOR_TEST := 0
ENABLE_SD := 1

# =============================================================================
# Paths
# =============================================================================

PILOT_DIR := ..
HIVE_ROOT := ../../..
HIVE_SRC_DIR := $(HIVE_ROOT)/src

BUILD_DIR_CF := build_crazyflie
BUILD_DIR_WB := build_webots

# =============================================================================
# Hive Test Config (minimal for tests)
# =============================================================================

HIVE_CFLAGS := -DHIVE_MAX_ACTORS=4
HIVE_CFLAGS += -DHIVE_MAX_BUSES=2
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=16

# Stack config - ESB test needs larger stacks for printf in HAL init
ifeq ($(TEST),esb)
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=6144
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(32*1024)'
else ifeq ($(PLATFORM),webots)
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=4096
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(32*1024)'
else
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=2048
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(16*1024)'
endif

# Tests that need libhive.a
HIVE_TESTS := sensors_motors thrust flash sd esb main

# =============================================================================
# Default Target
# =============================================================================

.PHONY: all help clean FORCE

ifeq ($(PLATFORM),crazyflie)
all: crazyflie
else ifeq ($(PLATFORM),webots)
all: webots
else
all: help
endif

help:
	@echo "Pilot HAL Tests"
	@echo ""
	@echo "Usage:"
	@echo "  make PLATFORM=crazyflie TEST=<test>"
	@echo "  make PLATFORM=crazyflie all-tests"
	@echo "  make PLATFORM=webots TEST=main"
	@echo "  make flash-crazyflie TEST=<test>"
	@echo ""
	@echo "Tests:"
	@echo "  sensors_motors  - HAL API validation"
	@echo "  thrust          - Thrust calibration"
	@echo "  flash           - Flash storage test"
	@echo "  sd              - SD card test"
	@echo "  esb             - ESB radio test"
	@echo "  main            - Combined test runner"
	@echo ""
	@echo "Options:"
	@echo "  ENABLE_MOTOR_TEST=1  Enable motor spin"
	@echo "  ENABLE_SD=1          Enable SD card"
	@echo ""
	@echo "Examples:"
	@echo "  make PLATFORM=crazyflie TEST=sensors_motors"
	@echo "  make PLATFORM=crazyflie TEST=esb"
	@echo "  make flash-crazyflie TEST=esb"

# =============================================================================
# Crazyflie Build
# =============================================================================

ifeq ($(PLATFORM),crazyflie)

PREFIX := arm-none-eabi-
CC := $(PREFIX)gcc
AR := $(PREFIX)ar
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size

HAL_DIR := $(PILOT_DIR)/hal/crazyflie-2.1plus
MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Base flags
CFLAGS := $(MCU)
CFLAGS += -std=c11 -Wall -Wextra -Os -g3
CFLAGS += -ffunction-sections -fdata-sections -fno-common
CFLAGS += -MMD -MP

# Platform
CFLAGS += -DSTM32F405xx -DHIVE_PLATFORM_STM32
CFLAGS += -DHIVE_LOG_TO_STDOUT=1
CFLAGS += -DENABLE_MOTOR_TEST=$(ENABLE_MOTOR_TEST)

# ESB test needs radio
ifeq ($(TEST),esb)
CFLAGS += -DHAL_HAS_RADIO
endif

# Includes
CFLAGS += -I$(PILOT_DIR) -I$(PILOT_DIR)/include -I$(HAL_DIR)
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Include
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/STM32F4xx/Include
CFLAGS += -I$(HIVE_ROOT)/include -I$(HIVE_SRC_DIR)/hal/stm32
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Board config (flash addresses)
include $(HAL_DIR)/hive_board_config.mk

# Hive config
CFLAGS += $(HIVE_CFLAGS)

# Extra flags from command line
CFLAGS += $(EXTRA_CFLAGS)

# Linker
LDSCRIPT := $(HAL_DIR)/stm32f405_flash.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs --specs=nosys.specs -lm -lc

# Libraries
HAL_LIB := $(HAL_DIR)/build/libhal.a
HAL_STARTUP := $(HAL_DIR)/build/startup_stm32f405.o
HIVE_LIB := $(BUILD_DIR_CF)/libhive.a
BOARD_MOUNTS_SRC := $(abspath $(HAL_DIR)/hive_mounts.c)

# Targets
TARGET_ELF := $(BUILD_DIR_CF)/test_$(TEST).elf
TARGET_BIN := $(BUILD_DIR_CF)/test_$(TEST).bin

NEEDS_HIVE := $(filter $(TEST),$(HIVE_TESTS))

.PHONY: crazyflie all-tests

# Build test_main (combined runner)
ifeq ($(TEST),main)
TEST_OBJS := $(BUILD_DIR_CF)/test_sensors_motors.o \
             $(BUILD_DIR_CF)/test_flash.o \
             $(BUILD_DIR_CF)/test_sd.o

crazyflie: $(HAL_LIB) $(HIVE_LIB) $(TARGET_BIN)
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(BUILD_DIR_CF)/test_%.o: test_%.c | $(BUILD_DIR_CF)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(TARGET_ELF): test_main.c $(TEST_OBJS) $(HAL_LIB) $(HIVE_LIB) | $(BUILD_DIR_CF)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $< $(TEST_OBJS) $(HAL_STARTUP) -Wl,--start-group $(HIVE_LIB) $(HAL_LIB) -Wl,--end-group $(LDFLAGS) -o $@

# Build standalone test with Hive
else ifneq ($(NEEDS_HIVE),)
crazyflie: $(HAL_LIB) $(HIVE_LIB) $(TARGET_BIN)
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(TARGET_ELF): test_$(TEST).c $(HAL_LIB) $(HIVE_LIB) | $(BUILD_DIR_CF)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $< $(HAL_STARTUP) -Wl,--start-group $(HIVE_LIB) $(HAL_LIB) -Wl,--end-group $(LDFLAGS) -o $@

# Build standalone test without Hive
else
crazyflie: $(HAL_LIB) $(TARGET_BIN)
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(TARGET_ELF): test_$(TEST).c $(HAL_LIB) | $(BUILD_DIR_CF)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $< $(HAL_STARTUP) $(HAL_LIB) $(LDFLAGS) -o $@
endif

all-tests:
	@$(MAKE) PLATFORM=crazyflie TEST=sensors_motors
	@$(MAKE) PLATFORM=crazyflie TEST=thrust
	@$(MAKE) PLATFORM=crazyflie TEST=flash
	@$(MAKE) PLATFORM=crazyflie TEST=sd
	@$(MAKE) PLATFORM=crazyflie TEST=esb
	@$(MAKE) PLATFORM=crazyflie TEST=main

$(BUILD_DIR_CF):
	@mkdir -p $@

$(HAL_LIB): FORCE
	@$(MAKE) -C $(HAL_DIR) ENABLE_SD=$(ENABLE_SD)

$(HIVE_LIB): FORCE | $(BUILD_DIR_CF)
	@$(MAKE) -C $(HIVE_SRC_DIR) PLATFORM=stm32 \
		BUILD_DIR=$(abspath $(BUILD_DIR_CF)) \
		CC=$(CC) AR=$(AR) \
		BOARD_MOUNTS_SRC=$(BOARD_MOUNTS_SRC) \
		ENABLE_SD=$(ENABLE_SD) \
		CFLAGS="$(CFLAGS)"

$(TARGET_BIN): $(TARGET_ELF)
	@$(CP) -O binary $< $@

-include $(BUILD_DIR_CF)/*.d

endif # crazyflie

# =============================================================================
# Webots Build
# =============================================================================

ifeq ($(PLATFORM),webots)

WEBOTS_HOME ?= /usr/local/webots
HAL_DIR := $(PILOT_DIR)/hal/webots-crazyflie

CC := gcc

# Base flags
CFLAGS := -std=c11 -Wall -Wextra -O2 -g
CFLAGS += -D_POSIX_C_SOURCE=200809L
CFLAGS += -MMD -MP

# Platform
CFLAGS += -DSIMULATED_TIME -DHIVE_PLATFORM_LINUX
CFLAGS += -DENABLE_MOTOR_TEST=$(ENABLE_MOTOR_TEST)

# Includes
CFLAGS += -I$(WEBOTS_HOME)/include/controller/c
CFLAGS += -I$(PILOT_DIR) -I$(PILOT_DIR)/include -I$(HAL_DIR)
CFLAGS += -I$(HIVE_ROOT)/include -I$(HIVE_SRC_DIR)/hal/linux
CFLAGS += -I$(HIVE_ROOT)/lib -I$(HIVE_ROOT)/lib/printf -DPRINTF_INCLUDE_CONFIG_H

# Hive config
CFLAGS += $(HIVE_CFLAGS)

# Linker
LDFLAGS := -L$(WEBOTS_HOME)/lib/controller
LDLIBS := -lController -lm

# HAL sources (compiled directly, no library)
HAL_SRCS := $(HAL_DIR)/hal_init.c \
            $(HAL_DIR)/hal_sensors.c \
            $(HAL_DIR)/hal_motors.c \
            $(HAL_DIR)/hal_time.c \
            $(HAL_DIR)/hal_led.c \
            $(HAL_DIR)/hal_debug.c

HIVE_LIB := $(BUILD_DIR_WB)/libhive.a
TARGET := $(BUILD_DIR_WB)/test_$(TEST)

NEEDS_HIVE := $(filter $(TEST),$(HIVE_TESTS))

.PHONY: webots check-webots install-webots webots-main

check-webots:
	@test -d "$(WEBOTS_HOME)" || { echo "Error: WEBOTS_HOME not set"; exit 1; }

$(BUILD_DIR_WB):
	@mkdir -p $@

$(HIVE_LIB): FORCE | $(BUILD_DIR_WB)
	@$(MAKE) -C $(HIVE_SRC_DIR) PLATFORM=linux \
		BUILD_DIR=$(abspath $(BUILD_DIR_WB)) \
		CFLAGS="$(CFLAGS)"

# Build test_main (combined runner)
ifeq ($(TEST),main)
TEST_OBJS := $(BUILD_DIR_WB)/test_sensors_motors.o

webots: check-webots $(HIVE_LIB) $(TARGET)
	@echo "Built: $(TARGET)"

$(BUILD_DIR_WB)/test_%.o: test_%.c | $(BUILD_DIR_WB)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -DTEST_MAIN_BUILD -c $< -o $@

$(TARGET): test_main.c $(TEST_OBJS) $(HAL_SRCS) $(HIVE_LIB) | $(BUILD_DIR_WB)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $< $(TEST_OBJS) $(HAL_SRCS) $(HIVE_LIB) $(LDFLAGS) $(LDLIBS) -o $@

# Build standalone test with Hive
else ifneq ($(NEEDS_HIVE),)
webots: check-webots $(HIVE_LIB) $(TARGET)
	@echo "Built: $(TARGET)"

$(TARGET): test_$(TEST).c $(HAL_SRCS) $(HIVE_LIB) | $(BUILD_DIR_WB)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $< $(HAL_SRCS) $(HIVE_LIB) $(LDFLAGS) $(LDLIBS) -o $@

# Build standalone test without Hive
else
webots: check-webots $(TARGET)
	@echo "Built: $(TARGET)"

$(TARGET): test_$(TEST).c $(HAL_SRCS) | $(BUILD_DIR_WB)
	@echo "LD $@"
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LDLIBS) -o $@
endif

install-webots: webots-main
	@mkdir -p $(PILOT_DIR)/controllers/test_main
	@cp $(BUILD_DIR_WB)/test_main $(PILOT_DIR)/controllers/test_main/
	@echo "Installed: $(PILOT_DIR)/controllers/test_main/test_main"

webots-main:
	@$(MAKE) PLATFORM=webots TEST=main

-include $(BUILD_DIR_WB)/*.d

endif # webots

# =============================================================================
# Flash Target
# =============================================================================

.PHONY: flash-crazyflie

flash-crazyflie: $(BUILD_DIR_CF)/test_$(TEST).bin
	st-flash write $< 0x8000000

# =============================================================================
# Clean
# =============================================================================

clean:
	rm -rf $(BUILD_DIR_CF) $(BUILD_DIR_WB)
