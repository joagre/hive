# qemu/Makefile - Cross-compile for Cortex-M3 and run in QEMU
#
# Can be run standalone or from top-level Makefile.
#
# Requirements:
#   sudo apt install gcc-arm-none-eabi qemu-system-arm

# Directories
SRC_DIR ?= ../src
TESTS_DIR ?= ../tests
EXAMPLES_DIR ?= ../examples
BUILD_DIR ?= ../build/qemu

# ARM cross-compiler
ARM_CC := arm-none-eabi-gcc
ARM_OBJCOPY := arm-none-eabi-objcopy
ARM_SIZE := arm-none-eabi-size

# ARM compiler flags for Cortex-M3
ARM_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2 -g \
              -mcpu=cortex-m3 -mthumb -mfloat-abi=soft \
              -ffunction-sections -fdata-sections -fno-common \
              -ffreestanding -nostdlib \
              -MMD -MP

# QEMU test uses smaller config values suitable for 64KB RAM
ARM_CPPFLAGS := -I../include -I. \
                -DHIVE_PLATFORM_STM32 -DHIVE_ENABLE_NET=0 -DHIVE_ENABLE_FILE=0 \
                -DHIVE_MAX_ACTORS=8 \
                -DHIVE_MAX_BUSES=4 \
                -DHIVE_MAILBOX_ENTRY_POOL_SIZE=32 \
                -DHIVE_MESSAGE_DATA_POOL_SIZE=32 \
                -DHIVE_LINK_ENTRY_POOL_SIZE=16 \
                -DHIVE_MONITOR_ENTRY_POOL_SIZE=16 \
                -DHIVE_TIMER_ENTRY_POOL_SIZE=16 \
                -DHIVE_MAX_BUS_SUBSCRIBERS=4 \
                -DHIVE_MAX_BUS_ENTRIES=8 \
                -DHIVE_MAX_MESSAGE_SIZE=128 \
                -DHIVE_DEFAULT_STACK_SIZE=2048 \
                '-DHIVE_STACK_ARENA_SIZE=(16*1024)' \
                -DHIVE_MAX_SUPERVISORS=2 \
                -DHIVE_MAX_SUPERVISOR_CHILDREN=4

ARM_LDFLAGS := -Tlm3s6965.ld \
               -mcpu=cortex-m3 -mthumb \
               -Wl,--gc-sections \
               -nostartfiles -specs=nosys.specs

# Runtime source files
QEMU_CORE_SRCS := hive_actor.c hive_bus.c hive_context.c hive_ipc.c \
                  hive_link.c hive_log.c hive_pool.c hive_runtime.c \
                  hive_select.c hive_supervisor.c hive_scheduler_stm32.c \
                  hive_timer_stm32.c

QEMU_SRCS := $(addprefix $(SRC_DIR)/,$(QEMU_CORE_SRCS))
QEMU_ASM := $(SRC_DIR)/hive_context_arm_cm.S
QEMU_TEST_SRCS := startup.S semihosting.c test_main.c

QEMU_OBJS := $(QEMU_SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o) \
             $(QEMU_ASM:$(SRC_DIR)/%.S=$(BUILD_DIR)/%.o) \
             $(BUILD_DIR)/startup.o \
             $(BUILD_DIR)/semihosting.o \
             $(BUILD_DIR)/test_main.o

# Basic test binary
QEMU_ELF := $(BUILD_DIR)/test.elf
QEMU_BIN := $(BUILD_DIR)/test.bin

# Runtime objects for test linking (exclude test_main.o which has its own main)
QEMU_RUNTIME_OBJS := $(filter-out $(BUILD_DIR)/test_main.o,$(QEMU_OBJS))

# Compatible tests (exclude net_test.c and file_test.c which require disabled features)
QEMU_COMPAT_TESTS := actor_test ipc_test timer_test link_test \
                     monitor_test bus_test priority_test runtime_test \
                     timeout_test arena_test pool_exhaustion_test \
                     backoff_retry_test simple_backoff_test congestion_demo

# Compatible examples (exclude echo.c and fileio.c which require net/file features)
QEMU_COMPAT_EXAMPLES := timer pingpong priority link_demo supervisor bus request_reply

# ============================================================================
# Build Rules
# ============================================================================

.PHONY: all
all: $(QEMU_ELF)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile runtime sources for ARM
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Compile ARM assembly from src/
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.S | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Compile QEMU test sources
$(BUILD_DIR)/startup.o: startup.S | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/semihosting.o: semihosting.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/test_main.o: test_main.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(BUILD_DIR)/test_runner.o: test_runner.c | $(BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Link basic test binary
$(QEMU_ELF): $(QEMU_OBJS)
	$(ARM_CC) $(ARM_LDFLAGS) $^ -o $@
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
$(BUILD_DIR)/test_%.elf: $(BUILD_DIR)/qemu_%.o $(BUILD_DIR)/test_runner.o $(QEMU_RUNTIME_OBJS)
	$(ARM_CC) $(ARM_LDFLAGS) $^ -o $@
	$(ARM_SIZE) $@

# Preserve test ELF files
.PRECIOUS: $(BUILD_DIR)/test_%.elf $(BUILD_DIR)/qemu_%.o

# Run a single test: make run-actor_test
.PHONY: run-%
run-%: $(BUILD_DIR)/test_%.elf
	@echo "=== Running $* on QEMU ==="
	@qemu-system-arm -M lm3s6965evb -nographic \
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
$(BUILD_DIR)/example_%.elf: $(BUILD_DIR)/qemu_example_%.o $(BUILD_DIR)/test_runner.o $(QEMU_RUNTIME_OBJS)
	$(ARM_CC) $(ARM_LDFLAGS) $^ -o $@
	$(ARM_SIZE) $@

# Preserve example ELF files
.PRECIOUS: $(BUILD_DIR)/example_%.elf $(BUILD_DIR)/qemu_example_%.o

# Run a single example: make example-pingpong
.PHONY: example-%
example-%: $(BUILD_DIR)/example_%.elf
	@echo "=== Running $* example on QEMU ==="
	@qemu-system-arm -M lm3s6965evb -nographic \
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
	@timeout 10 qemu-system-arm -M lm3s6965evb -nographic \
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
