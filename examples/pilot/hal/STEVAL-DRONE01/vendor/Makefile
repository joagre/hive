# Hardware validation tests for STEVAL-FCU001V1
#
# Standalone tests for sensors and motors (no pilot runtime).
# Uses STM32 HAL + BSP drivers.
#
# Usage:
#   make              - Build sensor_motor_test (default)
#   make TEST=main    - Build WHO_AM_I test (main.c)
#   make flash        - Flash to device (press reset after)
#   make clean        - Clean build

# Select test: sensor_motor_test (default) or main
TEST ?= sensor_motor_test

TARGET = $(TEST)
BUILD_DIR = build

# Common sources (no HAL TIM - motor test uses direct registers)
COMMON_SRCS = \
	system_stm32f4xx.c \
	stm32f4xx_it.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
	Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
	Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1.c \
	Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_accelero.c \
	Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_gyro.c \
	Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_magneto.c \
	Drivers/BSP/STEVAL_FCU001_V1/steval_fcu001_v1_pressure.c \
	Drivers/BSP/lsm6dsl/LSM6DSL_ACC_GYRO_driver.c \
	Drivers/BSP/lsm6dsl/LSM6DSL_ACC_GYRO_driver_HL.c \
	Drivers/BSP/lis2mdl/LIS2MDL_MAG_driver.c \
	Drivers/BSP/lis2mdl/LIS2MDL_MAG_driver_HL.c \
	Drivers/BSP/lps22hb/LPS22HB_Driver.c \
	Drivers/BSP/lps22hb/LPS22HB_Driver_HL.c \
	Drivers/BSP/lsm303agr/LSM303AGR_ACC_driver.c \
	Drivers/BSP/lsm303agr/LSM303AGR_ACC_driver_HL.c

# USART1 driver for debug output (from parent directory)
# usart1.c is self-contained, no other dependencies needed
DEBUG_SRCS = ../usart1.c

# Test-specific source
C_SRCS = $(TEST).c $(COMMON_SRCS) $(DEBUG_SRCS)

ASM_SRCS = ../startup_stm32f401.s

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
LD = $(PREFIX)gcc
SZ = $(PREFIX)size
OBJCOPY = $(PREFIX)objcopy

# MCU
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Include paths
INCLUDES = \
	-I. \
	-I.. \
	-IDrivers/STM32F4xx_HAL_Driver/Inc \
	-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
	-IDrivers/CMSIS/Include \
	-IDrivers/BSP/STEVAL_FCU001_V1 \
	-IDrivers/BSP/Common \
	-IDrivers/BSP/lsm6dsl \
	-IDrivers/BSP/lis2mdl \
	-IDrivers/BSP/lps22hb \
	-IDrivers/BSP/lsm303agr

# Compiler flags
CFLAGS = $(MCU) $(INCLUDES)
CFLAGS += -std=c11 -Wall
CFLAGS += -DSTM32F401xC -DUSE_HAL_DRIVER
CFLAGS += -O2 -g3
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -MMD -MP

# Assembler flags
ASFLAGS = $(MCU) -g3

# Linker script
LDSCRIPT = ../stm32f401_flash.ld

# Linker flags
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += --specs=nano.specs
LDFLAGS += -lc -lm -lnosys

# Object files
OBJS = $(C_SRCS:%.c=$(BUILD_DIR)/%.o)
ASM_OBJS = $(ASM_SRCS:%.s=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(ASM_OBJS)
	@echo "LD $@"
	@$(LD) $(OBJS) $(ASM_OBJS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "BIN $@"
	@$(OBJCOPY) -O binary $< $@

size: $(BUILD_DIR)/$(TARGET).elf
	@echo ""
	@$(SZ) $<

flash: $(BUILD_DIR)/$(TARGET).bin
	st-flash write $< 0x8000000

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)

.PHONY: all clean flash size
