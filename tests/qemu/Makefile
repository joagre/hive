# qemu/Makefile - Cross-compile for Cortex-M3 and run in QEMU
#
# Can be run standalone or from top-level Makefile.
#
# Requirements:
#   sudo apt install gcc-arm-none-eabi qemu-system-arm

# Directories
SRC_DIR ?= ../../src
TESTS_DIR ?= ../../tests
EXAMPLES_DIR ?= ../../examples
BUILD_DIR ?= ../../build/qemu

# ARM cross-compiler
ARM_CC := arm-none-eabi-gcc
ARM_AR := arm-none-eabi-ar
ARM_OBJCOPY := arm-none-eabi-objcopy
ARM_SIZE := arm-none-eabi-size

# MCU flags for Cortex-M3
MCU := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft

# ============================================================================
# QEMU Memory Configuration (smaller values for 64KB RAM)
# ============================================================================
HIVE_CFLAGS := -DHIVE_MAX_ACTORS=8
HIVE_CFLAGS += -DHIVE_MAX_BUSES=8
HIVE_CFLAGS += -DHIVE_MAILBOX_ENTRY_POOL_SIZE=32
HIVE_CFLAGS += -DHIVE_MESSAGE_DATA_POOL_SIZE=64
HIVE_CFLAGS += -DHIVE_LINK_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MONITOR_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_TIMER_ENTRY_POOL_SIZE=16
HIVE_CFLAGS += -DHIVE_MAX_BUS_SUBSCRIBERS=32
HIVE_CFLAGS += -DHIVE_MAX_BUS_ENTRIES=16
HIVE_CFLAGS += -DHIVE_MAX_MESSAGE_SIZE=256
HIVE_CFLAGS += -DHIVE_DEFAULT_STACK_SIZE=2048
HIVE_CFLAGS += '-DHIVE_STACK_ARENA_SIZE=(16*1024)'
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISORS=2
HIVE_CFLAGS += -DHIVE_MAX_SUPERVISOR_CHILDREN=4

# CFLAGS to pass to src/Makefile (PLATFORM=stm32)
HIVE_CFLAGS_EXPORT := $(MCU)
HIVE_CFLAGS_EXPORT += -I$(abspath ../../include)
HIVE_CFLAGS_EXPORT += -I$(abspath ../../lib) -I$(abspath ../../lib/printf)
HIVE_CFLAGS_EXPORT += $(HIVE_CFLAGS)

# ============================================================================
# Local Build Configuration
# ============================================================================

# ARM compiler flags for local sources (tests, examples)
ARM_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2 -g
ARM_CFLAGS += $(MCU)
ARM_CFLAGS += -ffunction-sections -fdata-sections -fno-common
ARM_CFLAGS += -ffreestanding -nostdlib
ARM_CFLAGS += -MMD -MP

# Include paths and defines for local sources
ARM_CPPFLAGS := -I../../include -I../../src -I../../lib -I../../lib/printf -DPRINTF_INCLUDE_CONFIG_H -I.
ARM_CPPFLAGS += -I../../src/hal/stm32
ARM_CPPFLAGS += -DHIVE_PLATFORM_STM32 -DHIVE_ENABLE_TCP=0 -DHIVE_ENABLE_FILE=0
ARM_CPPFLAGS += $(HIVE_CFLAGS)

ARM_LDFLAGS := -Tlm3s6965.ld
ARM_LDFLAGS += $(MCU)
ARM_LDFLAGS += -Wl,--gc-sections
ARM_LDFLAGS += -nostartfiles -specs=nosys.specs

# Hive library (built by delegating to src/Makefile)
HIVE_LIB := $(BUILD_DIR)/libhive.a

# Local QEMU sources
QEMU_LOCAL_SRCS := startup.S semihosting.c test_main.c
QEMU_LOCAL_OBJS := $(BUILD_DIR)/startup.o $(BUILD_DIR)/semihosting.o $(BUILD_DIR)/test_main.o

# Basic test binary
QEMU_ELF := $(BUILD_DIR)/test.elf
QEMU_BIN := $(BUILD_DIR)/test.bin

# Compatible tests (exclude tcp_test.c, file_test.c, logging_test.c which require disabled features)
QEMU_COMPAT_TESTS := actor_test ipc_test timer_test link_test \
                     monitor_test bus_test priority_test runtime_test \
                     timeout_test arena_test pool_exhaustion_test \
                     backoff_retry_test simple_backoff_test congestion_demo \
                     select_test supervisor_test registry_test \
                     sibling_test spawn_init_test

# Compatible examples (exclude echo.c, fileio.c, logging.c which require tcp/file features)
QEMU_COMPAT_EXAMPLES := timer pingpong priority link_demo supervisor bus request_reply select \
                        supervisor_manual registry

# ============================================================================
# Build Rules
# ============================================================================

.PHONY: all
all: $(QEMU_ELF)

# Create build directory
$(BUILD_DIR):
	mkdir -p $@

# Build hive library (delegate to src/Makefile)
$(HIVE_LIB): FORCE
	@$(MAKE) -C $(SRC_DIR) PLATFORM=stm32 \
		BUILD_DIR=$(abspath $(BUILD_DIR)) \
		CC=$(ARM_CC) AR=$(ARM_AR) \
		ENABLE_TCP=0 ENABLE_FILE=0 \
		CFLAGS="$(HIVE_CFLAGS_EXPORT)"

FORCE:

# Compile QEMU local sources
$(BUILD_DIR)/startup.o: startup.S | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/semihosting.o: semihosting.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/test_main.o: test_main.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/test_runner.o: test_runner.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Link basic test binary
$(QEMU_ELF): $(QEMU_LOCAL_OBJS) $(HIVE_LIB)
	$(ARM_CC) $(ARM_LDFLAGS) $(QEMU_LOCAL_OBJS) $(HIVE_LIB) -o $@
	$(ARM_SIZE) $@

# Create binary image
$(QEMU_BIN): $(QEMU_ELF)
	$(ARM_OBJCOPY) -O binary $< $@

# ============================================================================
# Test Suite
# ============================================================================

# Pattern rule: compile test source for ARM
$(BUILD_DIR)/qemu_%.o: $(TESTS_DIR)/%.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) \
		-include qemu_compat.h \
		-Dmain=test_main \
		-c $< -o $@

# Pattern rule: link test ELF
$(BUILD_DIR)/test_%.elf: $(BUILD_DIR)/qemu_%.o $(BUILD_DIR)/test_runner.o $(BUILD_DIR)/startup.o $(BUILD_DIR)/semihosting.o $(HIVE_LIB)
	$(ARM_CC) $(ARM_LDFLAGS) $(BUILD_DIR)/qemu_$*.o $(BUILD_DIR)/test_runner.o $(BUILD_DIR)/startup.o $(BUILD_DIR)/semihosting.o $(HIVE_LIB) -o $@
	$(ARM_SIZE) $@

# Preserve test ELF files
.PRECIOUS: $(BUILD_DIR)/test_%.elf $(BUILD_DIR)/qemu_%.o

# Timeout for QEMU runs (seconds) - prevents hangs from blocking examples/tests
QEMU_TIMEOUT ?= 10

# Run a single test: make run-actor_test
.PHONY: run-%
run-%: $(BUILD_DIR)/test_%.elf
	@echo "=== Running $* on QEMU ==="
	@timeout $(QEMU_TIMEOUT) qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $< 2>&1 | grep -v "Timer with period zero" || true
	@echo "=== $* completed ==="

# Run all compatible tests
.PHONY: test-suite
test-suite:
	@echo "Running QEMU test suite ($(words $(QEMU_COMPAT_TESTS)) tests)..."
	@failed=0; \
	for test in $(QEMU_COMPAT_TESTS); do \
		echo ""; \
		$(MAKE) --no-print-directory run-$$test || failed=1; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "All QEMU tests completed!"; \
	else \
		echo "Some QEMU tests failed!"; \
		exit 1; \
	fi

# ============================================================================
# Example Suite
# ============================================================================

# Pattern rule: compile example source for ARM
$(BUILD_DIR)/qemu_example_%.o: $(EXAMPLES_DIR)/%.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) \
		-include qemu_compat.h \
		-Dmain=test_main \
		-c $< -o $@

# Pattern rule: link example ELF
$(BUILD_DIR)/example_%.elf: $(BUILD_DIR)/qemu_example_%.o $(BUILD_DIR)/test_runner.o $(BUILD_DIR)/startup.o $(BUILD_DIR)/semihosting.o $(HIVE_LIB)
	$(ARM_CC) $(ARM_LDFLAGS) $(BUILD_DIR)/qemu_example_$*.o $(BUILD_DIR)/test_runner.o $(BUILD_DIR)/startup.o $(BUILD_DIR)/semihosting.o $(HIVE_LIB) -o $@
	$(ARM_SIZE) $@

# Preserve example ELF files
.PRECIOUS: $(BUILD_DIR)/example_%.elf $(BUILD_DIR)/qemu_example_%.o

# Run a single example: make example-pingpong
.PHONY: example-%
example-%: $(BUILD_DIR)/example_%.elf
	@echo "=== Running $* example on QEMU ==="
	@timeout $(QEMU_TIMEOUT) qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $< 2>&1 | grep -v "Timer with period zero" || true
	@echo "=== $* example completed ==="

# Run all compatible examples
.PHONY: example-suite
example-suite:
	@echo "Running QEMU example suite ($(words $(QEMU_COMPAT_EXAMPLES)) examples)..."
	@failed=0; \
	for example in $(QEMU_COMPAT_EXAMPLES); do \
		echo ""; \
		$(MAKE) --no-print-directory example-$$example || failed=1; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "All QEMU examples completed!"; \
	else \
		echo "Some QEMU examples failed!"; \
		exit 1; \
	fi

# ============================================================================
# Run Targets
# ============================================================================

# Build QEMU test
.PHONY: build
build: $(QEMU_ELF)
	@echo "QEMU test binary built: $(QEMU_ELF)"

# Run basic QEMU test
.PHONY: test
test: $(QEMU_ELF)
	@echo "Running QEMU test..."
	@qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $(QEMU_ELF) 2>&1 | grep -v "Timer with period zero" \
		|| true
	@echo "QEMU test completed"

# Run QEMU test with timeout (for CI)
.PHONY: test-ci
test-ci: $(QEMU_ELF)
	@echo "Running QEMU test with timeout..."
	@timeout $(QEMU_TIMEOUT) qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $(QEMU_ELF) 2>&1 | grep -v "Timer with period zero" \
		|| ([ $$? -eq 124 ] && echo "Test timed out" && exit 1)

# ============================================================================
# Clean and Help
# ============================================================================

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

.PHONY: help
help:
	@echo "qemu/Makefile targets:"
	@echo "  all           - Build basic QEMU test binary"
	@echo "  build         - Same as all"
	@echo "  test          - Run basic runtime test in QEMU"
	@echo "  test-ci       - Run with timeout (for CI)"
	@echo "  test-suite    - Run all compatible tests"
	@echo "  run-<test>    - Run specific test (e.g., run-actor_test)"
	@echo "  example-suite - Run all compatible examples"
	@echo "  example-<ex>  - Run specific example (e.g., example-pingpong)"
	@echo "  clean         - Remove build artifacts"
	@echo ""
	@echo "Requirements:"
	@echo "  sudo apt install gcc-arm-none-eabi qemu-system-arm"

# ============================================================================
# Auto-generated Dependencies
# ============================================================================

# Include all .d files in build directory (silently ignore if missing)
-include $(wildcard $(BUILD_DIR)/*.d)
