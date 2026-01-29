# benchmarks/Makefile - Build and run benchmarks
#
# Can be run standalone or from top-level Makefile.
# Uses variables from top-level if available, otherwise sets defaults.

# Defaults (when run standalone)
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
ENABLE_TCP ?= 1
ENABLE_FILE ?= 1

# Build directory - always use ../build when in benchmarks/
BUILD_DIR := ../build
LIB := $(BUILD_DIR)/libhive.a

# Platform (default to linux)
PLATFORM ?= linux

# HAL include path based on platform
ifeq ($(PLATFORM),stm32)
  HAL_INCLUDE := -I../src/hal/stm32
else
  HAL_INCLUDE := -I../src/hal/linux
endif

# Include path - always relative to this directory
CPPFLAGS := -I../include $(HAL_INCLUDE) -D_POSIX_C_SOURCE=200809L -DHIVE_PLATFORM_LINUX
DEPFLAGS := -MMD -MP

# Feature flags
ifeq ($(ENABLE_TCP),1)
  CPPFLAGS += -DHIVE_ENABLE_TCP=1
else
  CPPFLAGS += -DHIVE_ENABLE_TCP=0
endif

ifeq ($(ENABLE_FILE),1)
  CPPFLAGS += -DHIVE_ENABLE_FILE=1
else
  CPPFLAGS += -DHIVE_ENABLE_FILE=0
endif

# Benchmark sources
BENCHMARK_SRCS := $(wildcard *.c)
BENCHMARKS := $(BENCHMARK_SRCS:%.c=$(BUILD_DIR)/%)
DEPS := $(BENCHMARK_SRCS:%.c=$(BUILD_DIR)/%.d)

# Default target
.PHONY: all
all: $(BENCHMARKS)

# Ensure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build benchmarks
$(BUILD_DIR)/%: %.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive

# Run the main benchmark
.PHONY: run
run: $(BUILD_DIR)/bench
	./$(BUILD_DIR)/bench

# Clean benchmark binaries and deps
.PHONY: clean
clean:
	rm -f $(BENCHMARKS) $(DEPS)

.PHONY: help
help:
	@echo "benchmarks/Makefile targets:"
	@echo "  all   - Build all benchmarks"
	@echo "  run   - Build and run main benchmark"
	@echo "  clean - Remove benchmark binaries"

# Auto-generated dependencies
-include $(DEPS)
