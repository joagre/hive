# Makefile for STEVAL-DRONE01 (STM32F401)
#
# Requires: ARM GCC toolchain (arm-none-eabi-gcc)
# Install: apt install gcc-arm-none-eabi
#
# Usage:
#   make          - Build firmware
#   make flash    - Flash to device (requires ST-Link)
#   make debug    - Start GDB debug session
#   make clean    - Remove build artifacts
#   make size     - Show memory usage

# ------------------------------------------------------------------------------
# Project Configuration
# ------------------------------------------------------------------------------

TARGET = pilot
BUILD_DIR = build

# ------------------------------------------------------------------------------
# Source Files
# ------------------------------------------------------------------------------

# HAL driver sources
SRCS = \
	main.c \
	platform.c \
	attitude.c \
	lsm6dsl.c \
	lis2mdl.c \
	lps22hd.c \
	motors.c

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
	-I.

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
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
GDB = $(PREFIX)gdb

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

# Math library
CFLAGS += -DARM_MATH_CM4

# Include paths
CFLAGS += $(INCLUDES)

# Dependency generation
CFLAGS += -MMD -MP

# ------------------------------------------------------------------------------
# Assembler Flags
# ------------------------------------------------------------------------------

ASFLAGS = $(MCU) -g3

# ------------------------------------------------------------------------------
# Linker Flags
# ------------------------------------------------------------------------------

# Linker script (from STM32CubeMX)
# LDSCRIPT = ../STM32F401CEUX_FLASH.ld

# For now, use a placeholder
LDSCRIPT = stm32f401_flash.ld

LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm -lc

# ------------------------------------------------------------------------------
# Build Rules
# ------------------------------------------------------------------------------

# Object files
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)
ASM_OBJS = $(ASM_SRCS:%.s=$(BUILD_DIR)/%.o)

# Dependency files
DEPS = $(OBJS:.o=.d)

# Default target
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

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

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJS) $(ASM_OBJS) | $(BUILD_DIR)
	@echo "LD $@"
	@$(CC) $(OBJS) $(ASM_OBJS) $(LDFLAGS) -o $@

# Generate hex file
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo "HEX $@"
	@$(CP) -O ihex $< $@

# Generate binary file
$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo "BIN $@"
	@$(CP) -O binary -S $< $@

# ------------------------------------------------------------------------------
# Utility Targets
# ------------------------------------------------------------------------------

# Show memory usage
size: $(BUILD_DIR)/$(TARGET).elf
	@echo ""
	@echo "Memory usage:"
	@$(SZ) $<

# Flash to device using ST-Link
flash: $(BUILD_DIR)/$(TARGET).bin
	@echo "Flashing $(TARGET).bin..."
	st-flash write $< 0x8000000

# Alternative: Flash using OpenOCD
flash-openocd: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
		-c "program $< verify reset exit"

# Start GDB debug session
debug: $(BUILD_DIR)/$(TARGET).elf
	$(GDB) -ex "target extended-remote :3333" \
	       -ex "monitor reset halt" \
	       -ex "load" \
	       $<

# Start OpenOCD server (run in separate terminal)
openocd:
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# Clean build artifacts
clean:
	rm -rf $(BUILD_DIR)

# Generate linker script placeholder
$(LDSCRIPT):
	@echo "Creating placeholder linker script..."
	@echo "/* STM32F401CEU6 - 512KB Flash, 96KB RAM */" > $@
	@echo "/* Replace with STM32CubeMX generated script */" >> $@
	@echo "" >> $@
	@echo "MEMORY {" >> $@
	@echo "  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K" >> $@
	@echo "  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 96K" >> $@
	@echo "}" >> $@
	@echo "" >> $@
	@echo "ENTRY(Reset_Handler)" >> $@
	@echo "" >> $@
	@echo "SECTIONS {" >> $@
	@echo "  .text : { *(.text*) } > FLASH" >> $@
	@echo "  .rodata : { *(.rodata*) } > FLASH" >> $@
	@echo "  .data : { *(.data*) } > RAM AT > FLASH" >> $@
	@echo "  .bss : { *(.bss*) } > RAM" >> $@
	@echo "}" >> $@

# Include dependencies
-include $(DEPS)

# Phony targets
.PHONY: all clean flash flash-openocd debug openocd size

# ------------------------------------------------------------------------------
# Help
# ------------------------------------------------------------------------------

help:
	@echo "STEVAL-DRONE01 Build System"
	@echo ""
	@echo "Targets:"
	@echo "  all          - Build firmware (default)"
	@echo "  clean        - Remove build artifacts"
	@echo "  flash        - Flash using ST-Link"
	@echo "  flash-openocd - Flash using OpenOCD"
	@echo "  debug        - Start GDB debug session"
	@echo "  openocd      - Start OpenOCD server"
	@echo "  size         - Show memory usage"
	@echo "  help         - Show this help"
	@echo ""
	@echo "Requirements:"
	@echo "  - ARM GCC toolchain: apt install gcc-arm-none-eabi"
	@echo "  - ST-Link tools: apt install stlink-tools"
	@echo "  - OpenOCD (optional): apt install openocd"
	@echo ""
	@echo "Note: This Makefile is a skeleton. For full build:"
	@echo "  1. Generate STM32 HAL code with STM32CubeMX"
	@echo "  2. Uncomment HAL sources and includes above"
	@echo "  3. Copy linker script from CubeMX project"
