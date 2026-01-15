# Crazyflie 2.1+ Test Programs Makefile
#
# Standalone test programs for hardware verification.
# These are self-contained and don't require the full HAL or Hive runtime.
#
# Usage:
#   make thrust_test          # Build thrust calibration test
#   make sensor_motor_test    # Build sensor/motor diagnostic
#   make flash-thrust         # Flash thrust_test
#   make flash-sensor         # Flash sensor_motor_test
#   make clean                # Clean build artifacts

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# Build directory
BUILD_DIR = build

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

# Linker flags
LDSCRIPT = ../stm32f405_flash.ld
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm -lc

# Targets
.PHONY: all clean thrust_test sensor_motor_test flash-thrust flash-sensor

all: thrust_test sensor_motor_test

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Thrust test
thrust_test: $(BUILD_DIR)/thrust_test.bin
	@echo ""
	@echo "Built: $(BUILD_DIR)/thrust_test.bin"
	@$(SZ) $(BUILD_DIR)/thrust_test.elf

$(BUILD_DIR)/thrust_test.elf: thrust_test.c | $(BUILD_DIR)
	@echo "CC thrust_test.c"
	@$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

$(BUILD_DIR)/thrust_test.bin: $(BUILD_DIR)/thrust_test.elf
	@echo "BIN thrust_test.bin"
	@$(CP) -O binary $< $@

# Sensor motor test
sensor_motor_test: $(BUILD_DIR)/sensor_motor_test.bin
	@echo ""
	@echo "Built: $(BUILD_DIR)/sensor_motor_test.bin"
	@$(SZ) $(BUILD_DIR)/sensor_motor_test.elf

$(BUILD_DIR)/sensor_motor_test.elf: sensor_motor_test.c | $(BUILD_DIR)
	@echo "CC sensor_motor_test.c"
	@$(CC) $(CFLAGS) $< $(LDFLAGS) -o $@

$(BUILD_DIR)/sensor_motor_test.bin: $(BUILD_DIR)/sensor_motor_test.elf
	@echo "BIN sensor_motor_test.bin"
	@$(CP) -O binary $< $@

# Flash targets
flash-thrust: $(BUILD_DIR)/thrust_test.bin
	@echo "Flashing thrust_test..."
	st-flash write $< 0x8000000

flash-sensor: $(BUILD_DIR)/sensor_motor_test.bin
	@echo "Flashing sensor_motor_test..."
	st-flash write $< 0x8000000

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Help
help:
	@echo "Crazyflie 2.1+ Test Programs"
	@echo ""
	@echo "Build targets:"
	@echo "  thrust_test        - Build thrust calibration test"
	@echo "  sensor_motor_test  - Build sensor/motor diagnostic"
	@echo "  all                - Build both tests"
	@echo ""
	@echo "Flash targets:"
	@echo "  flash-thrust       - Flash thrust_test via ST-Link"
	@echo "  flash-sensor       - Flash sensor_motor_test via ST-Link"
	@echo ""
	@echo "Other:"
	@echo "  clean              - Remove build artifacts"
	@echo "  help               - Show this help"
	@echo ""
	@echo "LED feedback (blue LED on PC4):"
	@echo ""
	@echo "  thrust_test:"
	@echo "    2 slow blinks   = Starting (secure drone!)"
	@echo "    3 quick blinks  = Motors initialized"
	@echo "    Fast blink      = Motors running at TEST_THRUST"
	@echo "    5 slow blinks   = Test complete"
	@echo ""
	@echo "  sensor_motor_test:"
	@echo "    2 slow blinks   = Starting"
	@echo "    3 quick blinks  = Motors OK"
	@echo "    4 quick blinks  = Gyro OK"
	@echo "    N blinks        = Testing motor N (1-4)"
	@echo "    Fast blink      = Motor spinning"
	@echo "    After motor test:"
	@echo "      1 slow blink  = CCW detected"
	@echo "      2 slow blinks = CW detected"
	@echo "      3 slow blinks = Unclear/no rotation"
	@echo "    10 fast blinks  = All motors test"
	@echo "    After all motors:"
	@echo "      1 slow blink  = Balanced (good!)"
	@echo "      2 slow blinks = Net CCW rotation"
	@echo "      3 slow blinks = Net CW rotation"
	@echo "    Continuous slow = Test complete"
	@echo ""
	@echo "  Errors:"
	@echo "    Continuous fast blink  = Motor init failed"
	@echo "    Continuous medium blink = Gyro init failed"
