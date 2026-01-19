# Crazyflie 2.1+ HAL Library Makefile
#
# Builds libhal.a static library for the Crazyflie 2.1+ platform.
# Uses direct register access (no ST HAL library).

# Toolchain
CC      = arm-none-eabi-gcc
AS      = arm-none-eabi-as
AR      = arm-none-eabi-ar
OBJCOPY = arm-none-eabi-objcopy
SIZE    = arm-none-eabi-size

# Directories
BUILD_DIR = build

# Target
TARGET = $(BUILD_DIR)/libhal.a

# MCU flags for STM32F405
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# C compiler flags
CFLAGS  = $(MCU)
CFLAGS += -std=gnu11
CFLAGS += -Wall -Wextra -Wpedantic
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common
CFLAGS += -Os
CFLAGS += -g3
CFLAGS += -MMD -MP

# Processor defines
CFLAGS += -DSTM32F405xx
CFLAGS += -DUSE_FULL_ASSERT
CFLAGS += -DHAL_HAS_RADIO

# Include paths
CFLAGS += -I.
CFLAGS += -I../..
CFLAGS += -I../../include
CFLAGS += -Ivendor/CMSIS/Include
CFLAGS += -Ivendor/CMSIS/Device/ST/STM32F4xx/Include

# Assembler flags
ASFLAGS = $(MCU) -g

# Source files
C_SOURCES = \
    hal_crazyflie.c \
    hal_radio.c \
    platform_crazyflie.c \
    motors.c \
    bmi088.c \
    bmp388.c \
    pmw3901.c \
    vl53l1x.c \
    syscalls.c

ASM_SOURCES = \
    startup_stm32f405.s

# Object files
OBJECTS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/, $(ASM_SOURCES:.s=.o))

# Dependency files
DEPS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.d))

# Default target
all: $(TARGET)
	@echo ""
	@echo "Built: $(TARGET)"

# Library archive
$(TARGET): $(OBJECTS)
	@echo "AR $@"
	@$(AR) rcs $@ $^

# C compilation
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

# Assembly
$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) $< -o $@

# Create build directory
$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Phony targets
.PHONY: all clean

# Auto-generated dependencies
-include $(DEPS)
