# Makefile for STEVAL-DRONE01 HAL (STM32F401)
#
# Builds HAL drivers as a static library for use with pilot.
# Uses STM32 HAL + BSP for sensor access (SPI).
#
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

# Path to vendor directory containing ST HAL + BSP drivers
VENDOR_DIR = vendor

# ------------------------------------------------------------------------------
# Source Files
# ------------------------------------------------------------------------------

# Platform-specific sources
SRCS = \
	syscalls.c \
	system_config.c \
	gpio_config.c \
	tim4.c \
	motors.c \
	usart1.c \
	platform_stm32f4.c \
	hal_stm32.c \
	$(VENDOR_DIR)/system_stm32f4xx.c \
	$(VENDOR_DIR)/stm32f4xx_it.c

# STM32 HAL Driver sources (minimal set for sensors)
# Note: TIM driver not included - we use tim4.c for motor PWM
SRCS += \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c

# BSP sources (board support package for STEVAL-FCU001V1)
SRCS += \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1.c \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_accelero.c \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_gyro.c \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_magneto.c \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_pressure.c \
	$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_temperature.c

# Sensor driver sources
SRCS += \
	$(VENDOR_DIR)/Drivers/BSP/lsm6dsl/LSM6DSL_ACC_GYRO_driver.c \
	$(VENDOR_DIR)/Drivers/BSP/lsm6dsl/LSM6DSL_ACC_GYRO_driver_HL.c \
	$(VENDOR_DIR)/Drivers/BSP/lis2mdl/LIS2MDL_MAG_driver.c \
	$(VENDOR_DIR)/Drivers/BSP/lis2mdl/LIS2MDL_MAG_driver_HL.c \
	$(VENDOR_DIR)/Drivers/BSP/lps22hb/LPS22HB_Driver.c \
	$(VENDOR_DIR)/Drivers/BSP/lps22hb/LPS22HB_Driver_HL.c \
	$(VENDOR_DIR)/Drivers/BSP/lsm303agr/LSM303AGR_ACC_driver.c \
	$(VENDOR_DIR)/Drivers/BSP/lsm303agr/LSM303AGR_ACC_driver_HL.c \
	$(VENDOR_DIR)/Drivers/BSP/lsm303agr/LSM303AGR_MAG_driver.c \
	$(VENDOR_DIR)/Drivers/BSP/lsm303agr/LSM303AGR_MAG_driver_HL.c

# Startup file
ASM_SRCS = startup_stm32f401.s

# ------------------------------------------------------------------------------
# Include Paths
# ------------------------------------------------------------------------------

INCLUDES = \
	-I. \
	-I../.. \
	-I$(VENDOR_DIR) \
	-I$(VENDOR_DIR)/Drivers/STM32F4xx_HAL_Driver/Inc \
	-I$(VENDOR_DIR)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	-I$(VENDOR_DIR)/Drivers/CMSIS/Include \
	-I$(VENDOR_DIR)/Drivers/BSP/STEVAL_FCU001_V1 \
	-I$(VENDOR_DIR)/Drivers/BSP/Common \
	-I$(VENDOR_DIR)/Drivers/BSP/lsm6dsl \
	-I$(VENDOR_DIR)/Drivers/BSP/lis2mdl \
	-I$(VENDOR_DIR)/Drivers/BSP/lps22hb \
	-I$(VENDOR_DIR)/Drivers/BSP/lsm303agr

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

# Log level: INFO for debugging (change to NONE for production)
CFLAGS += -DHIVE_LOG_LEVEL=HIVE_LOG_LEVEL_INFO

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

# Compile C sources (create output dir as needed)
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Compile assembly sources
$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

# Create static library
$(BUILD_DIR)/$(TARGET): $(OBJS) $(ASM_OBJS)
	@mkdir -p $(dir $@)
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
