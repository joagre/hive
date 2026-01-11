# Makefile for STEVAL-DRONE01 HAL (STM32F401)
#
# Builds HAL drivers as a static library for use with pilot.
# For full firmware build, use: make -f Makefile.STEVAL-DRONE01 (from examples/pilot/)
#
# Requires: ARM GCC toolchain (arm-none-eabi-gcc)
# Install: apt install gcc-arm-none-eabi
#
# Usage:
#   make          - Build HAL library (libhal.a)
#   make clean    - Remove build artifacts

# ------------------------------------------------------------------------------
# Project Configuration
# ------------------------------------------------------------------------------

TARGET = libhal.a
BUILD_DIR = build

# ------------------------------------------------------------------------------
# Source Files
# ------------------------------------------------------------------------------

# HAL driver sources (no main.c - pilot.c is the entry point)
SRCS = \
	syscalls.c \
	system_config.c \
	gpio_config.c \
	spi1.c \
	i2c1.c \
	tim4.c \
	usart1.c \
	platform_stm32f4.c \
	attitude.c \
	lsm6dsl.c \
	lis2mdl.c \
	lps22hd.c \
	motors.c \
	hal_stm32.c

# Startup file
ASM_SRCS = startup_stm32f401.s

# STM32 HAL sources (add when integrating with CubeMX-generated code)
# SRCS += \
# 	../Core/Src/stm32f4xx_it.c \
# 	../Core/Src/stm32f4xx_hal_msp.c \
# 	../Core/Src/system_stm32f4xx.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
# 	../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c

# ------------------------------------------------------------------------------
# Include Paths
# ------------------------------------------------------------------------------

INCLUDES = \
	-I. \
	-I../..  # For types.h, config.h (via hal.h)

# STM32 HAL includes (add when integrating)
# INCLUDES += \
# 	-I../Core/Inc \
# 	-I../Drivers/STM32F4xx_HAL_Driver/Inc \
# 	-I../Drivers/CMSIS/Device/ST/STM32F4xx/Include \
# 	-I../Drivers/CMSIS/Include

# ------------------------------------------------------------------------------
# Toolchain
# ------------------------------------------------------------------------------

PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
AR = $(PREFIX)ar
SZ = $(PREFIX)size

# ------------------------------------------------------------------------------
# MCU Configuration (STM32F401CEU6)
# ------------------------------------------------------------------------------

CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------

# C standard and warnings
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic

# MCU flags
CFLAGS += $(MCU)

# Optimization (use -Og for debug, -O2 for release)
CFLAGS += -O2

# Generate debugging information
CFLAGS += -g3 -gdwarf-2

# Function and data sections for link-time optimization
CFLAGS += -ffunction-sections -fdata-sections

# STM32 defines
CFLAGS += -DSTM32F401xE
CFLAGS += -DUSE_HAL_DRIVER
CFLAGS += -DPLATFORM_STEVAL_DRONE01

# Math library
CFLAGS += -DARM_MATH_CM4

# Log level: NONE for production (all logging compiled out)
CFLAGS += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_NONE

# Include paths
CFLAGS += $(INCLUDES)

# Dependency generation
CFLAGS += -MMD -MP

# ------------------------------------------------------------------------------
# Assembler Flags
# ------------------------------------------------------------------------------

ASFLAGS = $(MCU) -g3

# ------------------------------------------------------------------------------
# Build Rules
# ------------------------------------------------------------------------------

# Object files
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)
ASM_OBJS = $(ASM_SRCS:%.s=$(BUILD_DIR)/%.o)

# Dependency files
DEPS = $(OBJS:.o=.d)

# Default target
all: $(BUILD_DIR)/$(TARGET)

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Compile C sources
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile assembly sources
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# Create static library
$(BUILD_DIR)/$(TARGET): $(OBJS) $(ASM_OBJS) | $(BUILD_DIR)
	@echo "AR $@"
	@$(AR) rcs $@ $^
	@echo ""
	@echo "Built: $@"

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Include dependencies
-include $(DEPS)

# Phony targets
.PHONY: all clean help

# ------------------------------------------------------------------------------
# Help
# ------------------------------------------------------------------------------

help:
	@echo "STEVAL-DRONE01 HAL Build System"
	@echo ""
	@echo "This builds HAL drivers as a static library (libhal.a)."
	@echo "For full firmware, use: make -f Makefile.STEVAL-DRONE01 (from examples/pilot/)"
	@echo ""
	@echo "Targets:"
	@echo "  all   - Build HAL library (default)"
	@echo "  clean - Remove build artifacts"
	@echo "  help  - Show this help"
	@echo ""
	@echo "Requirements:"
	@echo "  - ARM GCC toolchain: apt install gcc-arm-none-eabi"
