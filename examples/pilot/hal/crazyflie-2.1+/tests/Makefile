# Crazyflie 2.1+ Test Programs Makefile
#
# Test programs for hardware verification:
#   - thrust_test, sensor_motor_test: Standalone (direct register access)
#   - hal_test: Uses HAL library (tests actual HAL code path)
#
# Usage:
#   make thrust_test          # Build thrust calibration test (standalone)
#   make sensor_motor_test    # Build sensor/motor diagnostic (standalone)
#   make hal_test             # Build HAL validation test (uses libhal.a)
#   make flash-thrust         # Flash thrust_test
#   make flash-sensor         # Flash sensor_motor_test
#   make flash-hal            # Flash hal_test
#   make clean                # Clean build artifacts

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# Build directory
BUILD_DIR = build

# HAL directory (parent)
HAL_DIR = ..

# MCU flags for STM32F405
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# Compiler flags (standalone tests)
CFLAGS = $(MCU)
CFLAGS += -std=c11
CFLAGS += -Wall -Wextra
CFLAGS += -Os
CFLAGS += -g3
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -fno-common
CFLAGS += -DSTM32F405xx

# Additional flags for HAL test
HAL_CFLAGS = $(CFLAGS)
HAL_CFLAGS += -I$(HAL_DIR)
HAL_CFLAGS += -I$(HAL_DIR)/../..
HAL_CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Include
HAL_CFLAGS += -I$(HAL_DIR)/vendor/CMSIS/Device/ST/STM32F4xx/Include

# Linker flags
LDSCRIPT = ../stm32f405_flash.ld
LDFLAGS = $(MCU)
LDFLAGS += -T$(LDSCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += --specs=nano.specs
LDFLAGS += --specs=nosys.specs
LDFLAGS += -lm -lc

# Targets
.PHONY: all clean thrust_test sensor_motor_test hal_test flash-thrust flash-sensor flash-hal

all: thrust_test sensor_motor_test hal_test

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

# HAL test (links against libhal.a)
hal_test: $(HAL_DIR)/libhal.a $(BUILD_DIR)/hal_test.bin
	@echo ""
	@echo "Built: $(BUILD_DIR)/hal_test.bin"
	@$(SZ) $(BUILD_DIR)/hal_test.elf

$(HAL_DIR)/libhal.a:
	@echo "Building HAL library..."
	@$(MAKE) -C $(HAL_DIR) libhal.a

$(BUILD_DIR)/hal_test.elf: hal_test.c | $(BUILD_DIR)
	@echo "CC hal_test.c (with HAL)"
	@$(CC) $(HAL_CFLAGS) $< $(HAL_DIR)/libhal.a $(LDFLAGS) -o $@

$(BUILD_DIR)/hal_test.bin: $(BUILD_DIR)/hal_test.elf
	@echo "BIN hal_test.bin"
	@$(CP) -O binary $< $@

# Flash targets
flash-thrust: $(BUILD_DIR)/thrust_test.bin
	@echo "Flashing thrust_test..."
	st-flash write $< 0x8000000

flash-sensor: $(BUILD_DIR)/sensor_motor_test.bin
	@echo "Flashing sensor_motor_test..."
	st-flash write $< 0x8000000

flash-hal: $(BUILD_DIR)/hal_test.bin
	@echo "Flashing hal_test..."
	st-flash write $< 0x8000000

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Help
help:
	@echo "Crazyflie 2.1+ Test Programs"
	@echo ""
	@echo "Build targets:"
	@echo "  thrust_test        - Build thrust calibration test (standalone)"
	@echo "  sensor_motor_test  - Build sensor/motor diagnostic (standalone)"
	@echo "  hal_test           - Build HAL validation test (uses libhal.a)"
	@echo "  all                - Build all tests"
	@echo ""
	@echo "Flash targets:"
	@echo "  flash-thrust       - Flash thrust_test via ST-Link"
	@echo "  flash-sensor       - Flash sensor_motor_test via ST-Link"
	@echo "  flash-hal          - Flash hal_test via ST-Link"
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
	@echo "  hal_test:"
	@echo "    1-3 blinks      = hal_init() progress (from HAL)"
	@echo "    2 blinks        = hal_init() passed, starting self-test"
	@echo "    3 blinks        = hal_self_test() passed, starting calibration"
	@echo "    Slow blink      = Calibration in progress (from HAL)"
	@echo "    4 blinks        = Calibration done, starting sensor read"
	@echo "    Fast blink      = Sensor read loop (5 seconds)"
	@echo "    5 blinks        = Sensor test done, starting motor test"
	@echo "    Medium blink    = Motors running (2 seconds)"
	@echo "    6 blinks        = Motor test done"
	@echo "    LED on solid    = All tests passed!"
	@echo ""
	@echo "  Errors:"
	@echo "    Continuous fast blink  = Motor init failed"
	@echo "    Continuous medium blink = Gyro init failed"
	@echo "    3-5 fast blinks = hal_init() failed (from HAL)"
	@echo "    6-9 fast blinks = hal_self_test() failed (from HAL)"
	@echo "    10 fast blinks  = Sensor data out of range (hal_test)"
	@echo "    Continuous slow = Fatal error"
