# tests/Makefile - Build and run tests
#
# Can be run standalone or from top-level Makefile.
# Uses variables from top-level if available, otherwise sets defaults.

# Defaults (when run standalone)
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
ENABLE_TCP ?= 1
ENABLE_FILE ?= 1

# Build directory - always use ../build when in tests/
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

# Test sources
TEST_SRCS := $(wildcard *.c)

# Exclude tests that depend on disabled features
ifneq ($(ENABLE_TCP),1)
  TEST_SRCS := $(filter-out tcp_test.c,$(TEST_SRCS))
endif
ifneq ($(ENABLE_FILE),1)
  TEST_SRCS := $(filter-out file_test.c,$(TEST_SRCS))
endif

TESTS := $(TEST_SRCS:%.c=$(BUILD_DIR)/%)
DEPS := $(TEST_SRCS:%.c=$(BUILD_DIR)/%.d)

# Default target
.PHONY: all
all: $(TESTS)

# Ensure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build tests
$(BUILD_DIR)/%: %.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive

# Run all tests
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

# Clean test binaries and deps
.PHONY: clean
clean:
	rm -f $(TESTS) $(DEPS)

.PHONY: help
help:
	@echo "tests/Makefile targets:"
	@echo "  all   - Build all tests"
	@echo "  test  - Build and run all tests"
	@echo "  clean - Remove test binaries"

# Auto-generated dependencies
-include $(DEPS)
