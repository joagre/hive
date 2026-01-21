# HAL Test Makefile
#
# Builds hal_test for different platforms.
#
# Usage:
#   make PLATFORM=crazyflie     # Build for Crazyflie 2.1+ hardware
#   make PLATFORM=webots        # Build for Webots simulation
#   make flash-crazyflie        # Flash to Crazyflie via ST-Link
#   make run-webots             # Run in Webots (must have simulation open)
#   make clean                  # Remove build artifacts
#
# The test validates the HAL API works correctly on the target platform.

# Default target shows help
.PHONY: all help clean

all: help

help:
	@echo "HAL Test - Platform-Independent HAL Validation"
	@echo ""
	@echo "Usage:"
	@echo "  make PLATFORM=crazyflie     Build for Crazyflie 2.1+ hardware"
	@echo "  make PLATFORM=webots        Build for Webots simulation"
	@echo "  make flash-crazyflie        Flash to Crazyflie via ST-Link"
	@echo "  make install-webots         Install to controllers/hal_test/"
	@echo "  make clean                  Remove all build artifacts"
	@echo ""
	@echo "Test sequence:"
	@echo "  1. hal_init()      - Initialize hardware"
	@echo "  2. hal_self_test() - Verify sensors respond"
	@echo "  3. hal_calibrate() - Calibrate sensors (keep still!)"
	@echo "  4. Sensor loop     - Read sensors for 5 seconds"
	@echo "  5. Motor test      - Brief motor spin (2 seconds)"
	@echo "  6. hal_cleanup()   - Cleanup"

# Build directories
BUILD_DIR_CF = build_crazyflie
BUILD_DIR_WB = build_webots

# Pilot directory (parent)
PILOT_DIR = ..

# ==============================================================================
# Crazyflie 2.1+ Build
# ==============================================================================

ifeq ($(PLATFORM),crazyflie)

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
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
CFLAGS += -MMD -MP
CFLAGS += -I$(PILOT_DIR)
CFLAGS += -I$(HAL_DIR)
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Include
CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Device/ST/STM32F4xx/Include

# Linker flags
LDSCRIPT = $(HAL_DIR)/stm32f405_flash.ld
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm -lc

# HAL library
HAL_LIB = $(HAL_DIR)/build/libhal.a

# Target
TARGET_ELF = $(BUILD_DIR_CF)/hal_test.elf
TARGET_BIN = $(BUILD_DIR_CF)/hal_test.bin

.PHONY: crazyflie

all: crazyflie

crazyflie: $(HAL_LIB) $(TARGET_BIN)
	@echo ""
	@echo "Built: $(TARGET_BIN)"
	@$(SZ) $(TARGET_ELF)

$(BUILD_DIR_CF):
	mkdir -p $@

$(HAL_LIB):
	@echo "Building HAL library..."
	@$(MAKE) -C $(HAL_DIR)

$(TARGET_ELF): hal_test.c | $(BUILD_DIR_CF)
	@echo "CC hal_test.c (Crazyflie)"
	@$(CC) $(CFLAGS) $< $(HAL_LIB) $(LDFLAGS) -o $@

$(TARGET_BIN): $(TARGET_ELF)
	@echo "BIN hal_test.bin"
	@$(CP) -O binary $< $@

# Include dependencies
-include $(BUILD_DIR_CF)/hal_test.d

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
CFLAGS += -MMD -MP
CFLAGS += -I$(WEBOTS_INCLUDE)
CFLAGS += -I$(PILOT_DIR)
CFLAGS += -I$(HAL_DIR)

LDFLAGS = -L$(WEBOTS_LIB)
LDLIBS = -lController -lm

# Target
TARGET = $(BUILD_DIR_WB)/hal_test

.PHONY: webots check-webots

all: webots

webots: check-webots $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"
	@echo "Copy to controllers/hal_test/ and run in Webots"

check-webots:
	@if [ ! -d "$(WEBOTS_HOME)" ]; then \
		echo "Error: WEBOTS_HOME not set or invalid: $(WEBOTS_HOME)"; \
		echo "Set WEBOTS_HOME to your Webots installation directory"; \
		exit 1; \
	fi

$(BUILD_DIR_WB):
	mkdir -p $@

$(TARGET): hal_test.c $(HAL_DIR)/hal_webots.c | $(BUILD_DIR_WB)
	@echo "CC hal_test.c (Webots)"
	@$(CC) $(CFLAGS) $^ $(LDFLAGS) $(LDLIBS) -o $@

# Include dependencies
-include $(BUILD_DIR_WB)/hal_test.d

endif # PLATFORM=webots

# ==============================================================================
# Flash and Run Targets
# ==============================================================================

.PHONY: flash-crazyflie run-webots

flash-crazyflie: $(BUILD_DIR_CF)/hal_test.bin
	@echo "Flashing hal_test to Crazyflie..."
	st-flash write $< 0x8000000

install-webots: $(BUILD_DIR_WB)/hal_test
	@mkdir -p $(PILOT_DIR)/controllers/hal_test
	@cp $< $(PILOT_DIR)/controllers/hal_test/
	@echo "Installed: $(PILOT_DIR)/controllers/hal_test/hal_test"
	@echo ""
	@echo "To run in Webots:"
	@echo "  1. Open worlds/hover_test.wbt"
	@echo "  2. Select Crazyflie robot in scene tree"
	@echo "  3. Change controller to 'hal_test'"
	@echo "  4. Run simulation"

# ==============================================================================
# Clean
# ==============================================================================

clean:
	rm -rf $(BUILD_DIR_CF) $(BUILD_DIR_WB)
