# Compiler and flags
CC := gcc
CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
CPPFLAGS := -Iinclude -D_POSIX_C_SOURCE=200809L

# Platform selection (linux or stm32)
PLATFORM ?= linux

# STM32 defaults: disable net (no lwIP yet), enable file (flash-backed)
ifeq ($(PLATFORM),stm32)
  ENABLE_NET ?= 0
  ENABLE_FILE ?= 1
endif

# Feature toggles (set to 0 to disable)
ENABLE_NET ?= 1
ENABLE_FILE ?= 1

# Add feature flags to compiler (for benchmarks)
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
BUILD_DIR := build
MAN_DIR := man

# Installation directories
PREFIX ?= /usr/local
MANPREFIX ?= $(PREFIX)/share/man

# Library (built by src/Makefile)
LIB := $(BUILD_DIR)/libhive.a

# Benchmarks
BENCHMARKS_DIR := benchmarks
BENCHMARK_SRCS := $(wildcard $(BENCHMARKS_DIR)/*.c)
BENCHMARKS := $(BENCHMARK_SRCS:$(BENCHMARKS_DIR)/%.c=$(BUILD_DIR)/%)

# Variables to export to sub-Makefiles
export CC CFLAGS PLATFORM ENABLE_NET ENABLE_FILE

# ============================================================================
# Primary Targets
# ============================================================================

.PHONY: all
all: $(LIB) examples benchmarks

# Build the library (delegate to src/Makefile)
.PHONY: lib
lib: $(LIB)

$(LIB):
	$(MAKE) -C src

# Build benchmarks
$(BUILD_DIR)/%: $(BENCHMARKS_DIR)/%.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive

.PHONY: benchmarks
benchmarks: $(BENCHMARKS)

# ============================================================================
# Delegated Targets
# ============================================================================

.PHONY: examples
examples: $(LIB)
	$(MAKE) -C examples

.PHONY: test
test: $(LIB)
	$(MAKE) -C tests test

.PHONY: tests
tests: $(LIB)
	$(MAKE) -C tests

# Run targets (delegate to examples/)
.PHONY: run-pingpong run-fileio run-echo
run-pingpong run-fileio run-echo: $(LIB)
	$(MAKE) -C examples $@

# ============================================================================
# QEMU Targets (delegate to qemu/)
# ============================================================================

.PHONY: qemu-build
qemu-build:
	$(MAKE) -C qemu build

.PHONY: qemu-test
qemu-test:
	$(MAKE) -C qemu test

.PHONY: qemu-test-ci
qemu-test-ci:
	$(MAKE) -C qemu test-ci

.PHONY: qemu-test-suite
qemu-test-suite:
	$(MAKE) -C qemu test-suite

.PHONY: qemu-example-suite
qemu-example-suite:
	$(MAKE) -C qemu example-suite

# Pattern rules for QEMU targets
.PHONY: qemu-run-%
qemu-run-%:
	$(MAKE) -C qemu run-$*

.PHONY: qemu-example-%
qemu-example-%:
	$(MAKE) -C qemu example-$*

# ============================================================================
# Utility Targets
# ============================================================================

# Run benchmarks
.PHONY: bench
bench: $(BUILD_DIR)/bench
	./$(BUILD_DIR)/bench

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
# Help
# ============================================================================

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
	@echo ""
	@echo "QEMU targets (cross-compile for Cortex-M3):"
	@echo "  qemu-build        - Build runtime for QEMU"
	@echo "  qemu-test         - Run basic test in QEMU"
	@echo "  qemu-test-ci      - Run with timeout (for CI)"
	@echo "  qemu-test-suite   - Run all compatible tests"
	@echo "  qemu-run-<test>   - Run specific test (e.g., qemu-run-actor_test)"
	@echo "  qemu-example-suite - Run all compatible examples"
	@echo "  qemu-example-<ex> - Run specific example (e.g., qemu-example-pingpong)"
	@echo ""
	@echo "Platform selection:"
	@echo "  PLATFORM=linux    - Linux x86-64 (default)"
	@echo "  PLATFORM=stm32    - STM32 ARM Cortex-M (requires cross-compiler)"
	@echo ""
	@echo "Feature toggles (set to 0 to disable):"
	@echo "  ENABLE_NET=1      - Network I/O (default: 1 on linux, 0 on stm32)"
	@echo "  ENABLE_FILE=1     - File I/O (default: 1)"
	@echo ""
	@echo "Sub-Makefiles:"
	@echo "  make -C src       - Build library directly"
	@echo "  make -C tests     - Build/run tests directly"
	@echo "  make -C examples  - Build/run examples directly"
	@echo "  make -C qemu      - QEMU cross-compile directly"

# Dependencies
.PHONY: deps
deps:
	@echo "No external dependencies required"

# Print variables for debugging
.PHONY: print-%
print-%:
	@echo '$*=$($*)'
