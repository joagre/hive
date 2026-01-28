# Crazyflie 2.1+ Bring-Up Test Makefile

# Toolchain
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Directories
BUILD_DIR = build
CMSIS_DIR = ../vendor/CMSIS

# Source files
SRCS = bringup_main.c \
       bringup_swo.c \
       bringup_i2c.c \
       bringup_sensors.c \
       bringup_motors.c \
       bringup_radio.c \
       bringup_flash.c \
       bringup_sd.c \
       bringup_eeprom.c \
       bringup_deck.c \
       bringup_leds.c

# Use startup from parent directory
STARTUP = ../startup_stm32f405.s
LDSCRIPT = ../stm32f405_flash.ld

# Object files
OBJS = $(SRCS:%.c=$(BUILD_DIR)/%.o)
OBJS += $(BUILD_DIR)/startup.o

# Target
TARGET = $(BUILD_DIR)/bringup

# Compiler flags
CFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic
CFLAGS += -O0 -g
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -I$(CMSIS_DIR)/Include
CFLAGS += -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include
CFLAGS += -DSTM32F405xx

# Assembler flags
ASFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g

# Linker flags
LDFLAGS = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -nostartfiles
LDFLAGS += -specs=nosys.specs
LDFLAGS += -lm

# Targets
.PHONY: all clean flash reset

all: $(TARGET).elf $(TARGET).hex $(TARGET).bin
	@echo ""
	@echo "Memory usage:"
	@$(SIZE) $(TARGET).elf
	@echo ""
	@echo "Built: $(TARGET).bin"

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@echo "CC $<"
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup.o: $(STARTUP) | $(BUILD_DIR)
	@echo "AS $<"
	@$(AS) $(ASFLAGS) $< -o $@

$(TARGET).elf: $(OBJS)
	@echo "LD $@"
	@$(LD) $(LDFLAGS) $(OBJS) -o $@

$(TARGET).hex: $(TARGET).elf
	@echo "HEX $@"
	@$(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
	@echo "BIN $@"
	@$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD_DIR)

flash: $(TARGET).bin
	st-flash write $(TARGET).bin 0x08000000

reset:
	st-flash reset

# Help
help:
	@echo "Crazyflie 2.1+ Bring-Up Test"
	@echo ""
	@echo "Targets:"
	@echo "  all     - Build bringup.bin"
	@echo "  clean   - Remove build artifacts"
	@echo "  flash   - Flash to device via ST-Link"
	@echo "  reset   - Reset device via ST-Link"
	@echo ""
	@echo "Usage:"
	@echo "  1. make"
	@echo "  2. make flash"
	@echo "  3. View SWO output: st-trace -c 168"
	@echo "  4. Press reset or: make reset"
