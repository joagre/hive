# Compiler and flags
CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
CPPFLAGS := -Iinclude -D_POSIX_C_SOURCE=200809L
DEPFLAGS = -MMD -MP
LDFLAGS :=
LDLIBS :=

# Platform selection (linux or stm32)
PLATFORM ?= linux

# Platform-specific settings
ifeq ($(PLATFORM),linux)
  CPPFLAGS += -DHIVE_PLATFORM_LINUX
  PLATFORM_SRCS := hive_scheduler_linux.c hive_timer_linux.c
  PLATFORM_ASM := hive_context_x86_64.S
else ifeq ($(PLATFORM),stm32)
  CPPFLAGS += -DHIVE_PLATFORM_STM32
  PLATFORM_SRCS := hive_scheduler_stm32.c hive_timer_stm32.c
  PLATFORM_ASM := hive_context_arm_cm.S
  # STM32 defaults: disable net and file
  ENABLE_NET ?= 0
  ENABLE_FILE ?= 0
else
  $(error Unknown PLATFORM: $(PLATFORM). Use 'linux' or 'stm32')
endif

# Feature toggles (set to 0 to disable)
ENABLE_NET ?= 1
ENABLE_FILE ?= 1

# Add feature flags to compiler
ifeq ($(ENABLE_NET),1)
  CPPFLAGS += -DHIVE_ENABLE_NET=1
else
  CPPFLAGS += -DHIVE_ENABLE_NET=0
endif

ifeq ($(ENABLE_FILE),1)
  CPPFLAGS += -DHIVE_ENABLE_FILE=1
else
  CPPFLAGS += -DHIVE_ENABLE_FILE=0
endif

# Directories
SRC_DIR := src
INC_DIR := include
BUILD_DIR := build
EXAMPLES_DIR := examples
MAN_DIR := man

# Installation directories
PREFIX ?= /usr/local
MANPREFIX ?= $(PREFIX)/share/man

# Core source files (platform-independent)
CORE_SRCS := hive_actor.c hive_bus.c hive_context.c hive_ipc.c \
             hive_link.c hive_log.c hive_pool.c hive_runtime.c

# Feature-specific source files
FEATURE_SRCS :=
ifeq ($(ENABLE_NET),1)
  FEATURE_SRCS += hive_net.c
endif
ifeq ($(ENABLE_FILE),1)
  FEATURE_SRCS += hive_file.c
endif

# Combine all source files
SRCS := $(addprefix $(SRC_DIR)/,$(CORE_SRCS) $(PLATFORM_SRCS) $(FEATURE_SRCS))
ASM_SRCS := $(addprefix $(SRC_DIR)/,$(PLATFORM_ASM))

OBJS := $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.o) $(ASM_SRCS:$(SRC_DIR)/%.S=$(BUILD_DIR)/%.o)
DEPS := $(SRCS:$(SRC_DIR)/%.c=$(BUILD_DIR)/%.d)

# Library
LIB := $(BUILD_DIR)/libhive.a

# Examples
EXAMPLE_SRCS := $(wildcard $(EXAMPLES_DIR)/*.c)

# Exclude examples that depend on disabled features
ifneq ($(ENABLE_NET),1)
  EXAMPLE_SRCS := $(filter-out $(EXAMPLES_DIR)/echo.c,$(EXAMPLE_SRCS))
endif
ifneq ($(ENABLE_FILE),1)
  EXAMPLE_SRCS := $(filter-out $(EXAMPLES_DIR)/fileio.c,$(EXAMPLE_SRCS))
endif

EXAMPLES := $(EXAMPLE_SRCS:$(EXAMPLES_DIR)/%.c=$(BUILD_DIR)/%)

# Benchmarks
BENCHMARKS_DIR := benchmarks
BENCHMARK_SRCS := $(wildcard $(BENCHMARKS_DIR)/*.c)
BENCHMARKS := $(BENCHMARK_SRCS:$(BENCHMARKS_DIR)/%.c=$(BUILD_DIR)/%)

# Tests
TESTS_DIR := tests
TEST_SRCS := $(wildcard $(TESTS_DIR)/*.c)

# Exclude tests that depend on disabled features
ifneq ($(ENABLE_NET),1)
  TEST_SRCS := $(filter-out $(TESTS_DIR)/net_test.c,$(TEST_SRCS))
endif
ifneq ($(ENABLE_FILE),1)
  TEST_SRCS := $(filter-out $(TESTS_DIR)/file_test.c,$(TEST_SRCS))
endif

TESTS := $(TEST_SRCS:$(TESTS_DIR)/%.c=$(BUILD_DIR)/%)

# Default target
.PHONY: all
all: $(LIB) $(EXAMPLES) $(BENCHMARKS)

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files with dependency generation
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $< -o $@

# Compile assembly files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.S | $(BUILD_DIR)
	$(CC) $(CPPFLAGS) -c $< -o $@

# Create static library
$(LIB): $(OBJS)
	ar rcs $@ $^

# Build examples
$(BUILD_DIR)/%: $(EXAMPLES_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive $(LDLIBS)

# Build benchmarks
$(BUILD_DIR)/%: $(BENCHMARKS_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive $(LDLIBS)

# Build tests
$(BUILD_DIR)/%: $(TESTS_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive $(LDLIBS)

# Run benchmarks
.PHONY: bench
bench: $(BUILD_DIR)/bench
	./$(BUILD_DIR)/bench

# Run tests
.PHONY: test
test: $(TESTS)
	@echo "Running tests..."
	@for test in $(TESTS); do \
		echo ""; \
		echo "=== Running $$test ==="; \
		timeout 5 $$test || exit 1; \
	done
	@echo ""
	@echo "All tests passed!"

# Clean
.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)

# Clean Emacs backup/auto-save files recursively
.PHONY: clean-emacs
clean-emacs:
	@echo "Removing Emacs backup and auto-save files..."
	find . -name '*~' -delete
	find . -name '#*#' -delete
	find . -name '.#*' -delete

# Run ping-pong example
.PHONY: run-pingpong
run-pingpong: $(BUILD_DIR)/pingpong
	./$(BUILD_DIR)/pingpong

# Run file I/O example
.PHONY: run-fileio
run-fileio: $(BUILD_DIR)/fileio
	./$(BUILD_DIR)/fileio

# Run echo server/client example
.PHONY: run-echo
run-echo: $(BUILD_DIR)/echo
	./$(BUILD_DIR)/echo

# Install man pages
.PHONY: install-man
install-man:
	@echo "Installing man pages to $(MANPREFIX)/man3/"
	install -d $(MANPREFIX)/man3
	install -m 644 $(MAN_DIR)/man3/*.3 $(MANPREFIX)/man3/

# Uninstall man pages
.PHONY: uninstall-man
uninstall-man:
	@echo "Removing man pages from $(MANPREFIX)/man3/"
	rm -f $(MANPREFIX)/man3/hive_*.3

# ============================================================================
# QEMU Testing (Cross-compile for Cortex-M3 and run in QEMU)
# ============================================================================

QEMU_DIR := qemu
QEMU_BUILD_DIR := $(BUILD_DIR)/qemu

# ARM cross-compiler
ARM_CC := arm-none-eabi-gcc
ARM_OBJCOPY := arm-none-eabi-objcopy
ARM_SIZE := arm-none-eabi-size

# ARM compiler flags for Cortex-M3
ARM_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -O2 -g \
              -mcpu=cortex-m3 -mthumb -mfloat-abi=soft \
              -ffunction-sections -fdata-sections -fno-common \
              -ffreestanding -nostdlib

# QEMU test uses smaller config values suitable for 64KB RAM
# Override static config via -D flags (all values use #ifndef in hive_static_config.h)
ARM_CPPFLAGS := -Iinclude -I$(QEMU_DIR) \
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
                '-DHIVE_STACK_ARENA_SIZE=(16*1024)'

ARM_LDFLAGS := -T$(QEMU_DIR)/lm3s6965.ld \
               -mcpu=cortex-m3 -mthumb \
               -Wl,--gc-sections \
               -nostartfiles -specs=nosys.specs

# QEMU source files
QEMU_CORE_SRCS := hive_actor.c hive_bus.c hive_context.c hive_ipc.c \
                  hive_link.c hive_log.c hive_pool.c hive_runtime.c \
                  hive_scheduler_stm32.c hive_timer_stm32.c

QEMU_SRCS := $(addprefix $(SRC_DIR)/,$(QEMU_CORE_SRCS))
QEMU_ASM := $(SRC_DIR)/hive_context_arm_cm.S
QEMU_TEST_SRCS := $(QEMU_DIR)/startup.S $(QEMU_DIR)/semihosting.c $(QEMU_DIR)/test_main.c

QEMU_OBJS := $(QEMU_SRCS:$(SRC_DIR)/%.c=$(QEMU_BUILD_DIR)/%.o) \
             $(QEMU_ASM:$(SRC_DIR)/%.S=$(QEMU_BUILD_DIR)/%.o) \
             $(QEMU_BUILD_DIR)/startup.o \
             $(QEMU_BUILD_DIR)/semihosting.o \
             $(QEMU_BUILD_DIR)/test_main.o

# QEMU test binary
QEMU_ELF := $(QEMU_BUILD_DIR)/test.elf
QEMU_BIN := $(QEMU_BUILD_DIR)/test.bin

# Create QEMU build directory
$(QEMU_BUILD_DIR):
	mkdir -p $(QEMU_BUILD_DIR)

# Compile runtime sources for ARM
$(QEMU_BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Compile ARM assembly
$(QEMU_BUILD_DIR)/%.o: $(SRC_DIR)/%.S | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Compile QEMU test sources
$(QEMU_BUILD_DIR)/startup.o: $(QEMU_DIR)/startup.S | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(QEMU_BUILD_DIR)/semihosting.o: $(QEMU_DIR)/semihosting.c | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

$(QEMU_BUILD_DIR)/test_main.o: $(QEMU_DIR)/test_main.c | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Link QEMU test binary
$(QEMU_ELF): $(QEMU_OBJS)
	$(ARM_CC) $(ARM_LDFLAGS) $^ -o $@
	$(ARM_SIZE) $@

# Create binary image
$(QEMU_BIN): $(QEMU_ELF)
	$(ARM_OBJCOPY) -O binary $< $@

# Build QEMU test
.PHONY: qemu-build
qemu-build: $(QEMU_ELF)
	@echo "QEMU test binary built: $(QEMU_ELF)"

# Run QEMU test
# Note: Filter out "Timer with period zero" warning from unused LM3S6965 hardware timers
.PHONY: qemu-test
qemu-test: $(QEMU_ELF)
	@echo "Running QEMU test..."
	@qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $(QEMU_ELF) 2>&1 | grep -v "Timer with period zero" \
		|| true
	@echo "QEMU test completed"

# Run QEMU test with timeout
.PHONY: qemu-test-ci
qemu-test-ci: $(QEMU_ELF)
	@echo "Running QEMU test with timeout..."
	@timeout 10 qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $(QEMU_ELF) 2>&1 | grep -v "Timer with period zero" \
		|| ([ $$? -eq 124 ] && echo "Test timed out" && exit 1)

# ============================================================================
# QEMU Test Suite (run tests/*.c on ARM/QEMU)
# ============================================================================

# Compatible tests (exclude net_test.c and file_test.c which require disabled features)
QEMU_COMPAT_TESTS := actor_test ipc_test timer_test link_test \
                     monitor_test bus_test priority_test runtime_test \
                     timeout_test arena_test pool_exhaustion_test \
                     backoff_retry_test simple_backoff_test congestion_demo \
                     stack_overflow_test

# Runtime objects for test linking (exclude test_main.o which has its own main)
QEMU_RUNTIME_OBJS := $(filter-out $(QEMU_BUILD_DIR)/test_main.o,$(QEMU_OBJS))

# Test runner object (provides main with SysTick init)
$(QEMU_BUILD_DIR)/test_runner.o: $(QEMU_DIR)/test_runner.c | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) -c $< -o $@

# Pattern rule: compile test source for ARM
# -Dmain=test_main renames the test's main() so our runner provides main()
# -include qemu/qemu_compat.h adds printf/clock_gettime overrides
$(QEMU_BUILD_DIR)/qemu_%.o: $(TESTS_DIR)/%.c | $(QEMU_BUILD_DIR)
	$(ARM_CC) $(ARM_CFLAGS) $(ARM_CPPFLAGS) \
		-include $(QEMU_DIR)/qemu_compat.h \
		-Dmain=test_main \
		-c $< -o $@

# Pattern rule: link test ELF
$(QEMU_BUILD_DIR)/test_%.elf: $(QEMU_BUILD_DIR)/qemu_%.o $(QEMU_BUILD_DIR)/test_runner.o $(QEMU_RUNTIME_OBJS)
	$(ARM_CC) $(ARM_LDFLAGS) $^ -o $@
	$(ARM_SIZE) $@

# Preserve test ELF files (prevent make from deleting intermediates)
.PRECIOUS: $(QEMU_BUILD_DIR)/test_%.elf $(QEMU_BUILD_DIR)/qemu_%.o

# Run a single test: make qemu-run-actor_test
.PHONY: qemu-run-%
qemu-run-%: $(QEMU_BUILD_DIR)/test_%.elf
	@echo "=== Running $* on QEMU ==="
	@qemu-system-arm -M lm3s6965evb -nographic \
		-semihosting-config enable=on,target=native \
		-kernel $< 2>&1 | grep -v "Timer with period zero" || true
	@echo "=== $* completed ==="

# Run all compatible tests
.PHONY: qemu-test-suite
qemu-test-suite:
	@echo "Running QEMU test suite ($(words $(QEMU_COMPAT_TESTS)) tests)..."
	@failed=0; \
	for test in $(QEMU_COMPAT_TESTS); do \
		echo ""; \
		$(MAKE) --no-print-directory qemu-run-$$test || failed=1; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "All QEMU tests completed!"; \
	else \
		echo "Some QEMU tests failed!"; \
		exit 1; \
	fi

# ============================================================================
# Help
# ============================================================================

# Help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all               - Build library, examples, and benchmarks (default)"
	@echo "  clean             - Remove build artifacts"
	@echo "  clean-emacs       - Remove Emacs backup files (*~, #*#, .#*)"
	@echo "  test              - Build and run all tests"
	@echo "  bench             - Build and run benchmark suite"
	@echo "  install-man       - Install man pages to $(MANPREFIX)/man3"
	@echo "  uninstall-man     - Remove installed man pages"
	@echo "  run-pingpong      - Build and run ping-pong example"
	@echo "  run-fileio        - Build and run file I/O example"
	@echo "  run-echo          - Build and run echo server/client example"
	@echo "  qemu-build        - Cross-compile for Cortex-M3 (QEMU target)"
	@echo "  qemu-test         - Run runtime tests in QEMU emulator"
	@echo "  qemu-test-ci      - Run QEMU tests with timeout (for CI)"
	@echo "  qemu-run-<test>   - Run specific test on QEMU (e.g., qemu-run-actor_test)"
	@echo "  qemu-test-suite   - Run all compatible tests on QEMU"
	@echo "  help              - Show this help message"
	@echo ""
	@echo "Platform selection:"
	@echo "  PLATFORM=linux    - Linux x86-64 (default)"
	@echo "  PLATFORM=stm32    - STM32 ARM Cortex-M (requires cross-compiler)"
	@echo ""
	@echo "Feature toggles (set to 0 to disable):"
	@echo "  ENABLE_NET=1      - Network I/O subsystem (default: 1 on linux, 0 on stm32)"
	@echo "  ENABLE_FILE=1     - File I/O subsystem (default: 1 on linux, 0 on stm32)"
	@echo ""
	@echo "QEMU testing:"
	@echo "  Requires: arm-none-eabi-gcc, qemu-system-arm"
	@echo "  Install: sudo apt install gcc-arm-none-eabi qemu-system-arm"
	@echo ""
	@echo "Examples:"
	@echo "  make                              - Build for Linux with all features"
	@echo "  make ENABLE_NET=0 ENABLE_FILE=0   - Build for Linux without net/file"
	@echo "  make PLATFORM=stm32               - Build for STM32 (placeholder)"
	@echo "  make qemu-test                    - Test ARM code in QEMU emulator"

# Dependencies
.PHONY: deps
deps:
	@echo "No external dependencies required"

# Print variables for debugging
.PHONY: print-%
print-%:
	@echo '$*=$($*)'

# Include automatically generated dependencies
-include $(DEPS)
