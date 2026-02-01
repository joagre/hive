# Crazyflie 2.1+ HAL Library Makefile
#
# Builds libhal.a static library for the Crazyflie 2.1+ platform.
# Uses Bitcraze-derived DMA/interrupt I2C driver for reliable sensor access.

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

# Processor defines (matching Bitcraze firmware)
CFLAGS += -DSTM32F4XX
CFLAGS += -DSTM32F40_41xxx
CFLAGS += -DUSE_FULL_ASSERT
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -DHSE_VALUE=8000000
CFLAGS += -DHAL_HAS_RADIO

# Flight profile (passed from pilot Makefile)
ifdef FLIGHT_PROFILE
CFLAGS += -DFLIGHT_PROFILE=$(FLIGHT_PROFILE)
endif

# Include paths (Bitcraze vendor library structure + ARM CMSIS core)
CFLAGS += -I.
CFLAGS += -I../..
CFLAGS += -I../../include
CFLAGS += -I../../../../include
CFLAGS += -I../../../../src/hal/stm32
CFLAGS += -Ivendor/CMSIS/Include
CFLAGS += -Ivendor/CMSIS/STM32F4xx/Include
CFLAGS += -Ivendor/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -Ivendor/bosch
CFLAGS += -Ivendor/bitcraze/pmw3901
CFLAGS += -Ivendor/st/vl53l1x

# Assembler flags
ASFLAGS = $(MCU) -g

# Source files - platform (DMA I2C driver)
C_SOURCES = \
    debug_swo.c \
    hal_init.c \
    hal_sensors.c \
    hal_motors.c \
    hal_time.c \
    hal_led.c \
    hal_debug.c \
    hal_syslink.c \
    platform.c \
    i2c_drv.c \
    i2cdev.c \
    syscalls.c \
    system_stm32f4xx.c

# Source files - vendor drivers (Bosch sensor drivers - Bitcraze version)
C_SOURCES += \
    vendor/bosch/bmi088_accel.c \
    vendor/bosch/bmi088_gyro.c \
    vendor/bosch/bmp3.c

# Source files - flow deck drivers (PMW3901 optical flow, VL53L1x ToF)
C_SOURCES += \
    vendor/bitcraze/pmw3901/pmw3901.c \
    vendor/st/vl53l1x/vl53l1x_uld.c

# Source files - ST Standard Peripheral Library (Bitcraze version)
C_SOURCES += \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_misc.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
    vendor/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c

# Source files - SD card SPI low-level driver (optional)
ifeq ($(ENABLE_SD),1)
C_SOURCES += spi_ll_sd.c
CFLAGS += -DHIVE_ENABLE_SD=1
endif

# Startup assembly - built separately, not included in libhal.a
ASM_SOURCES = \
    startup_stm32f405.s

# Object files for libhal.a (C sources only)
OBJECTS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.o))

# Startup object (built but not in library)
STARTUP_OBJ = $(BUILD_DIR)/startup_stm32f405.o

# Dependency files
DEPS = $(addprefix $(BUILD_DIR)/, $(C_SOURCES:.c=.d))

# Default target - build library and startup object
all: $(TARGET) $(STARTUP_OBJ)
	@echo ""
	@echo "Built: $(TARGET)"

# Library archive (without startup - applications link startup separately)
$(TARGET): $(OBJECTS)
	@echo "AR $@"
	@$(AR) rcs $@ $^

# C compilation
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
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
