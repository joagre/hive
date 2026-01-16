# examples/Makefile - Build and run examples
#
# Can be run standalone or from top-level Makefile.
# Uses variables from top-level if available, otherwise sets defaults.
#
# Note: This builds the simple examples in examples/*.c.
# The pilot/ subdirectory has its own Makefiles.

# Defaults (when run standalone)
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
ENABLE_NET ?= 1
ENABLE_FILE ?= 1

# Build directory - always use ../build when in examples/
BUILD_DIR := ../build
LIB := $(BUILD_DIR)/libhive.a

# Include path - always relative to this directory
CPPFLAGS := -I../include -D_POSIX_C_SOURCE=200809L -DHIVE_PLATFORM_LINUX
DEPFLAGS := -MMD -MP

# Feature flags
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

# Example sources (exclude subdirectories)
EXAMPLE_SRCS := $(wildcard *.c)

# Exclude examples that depend on disabled features
ifneq ($(ENABLE_NET),1)
  EXAMPLE_SRCS := $(filter-out echo.c,$(EXAMPLE_SRCS))
endif
ifneq ($(ENABLE_FILE),1)
  EXAMPLE_SRCS := $(filter-out fileio.c,$(EXAMPLE_SRCS))
endif

EXAMPLES := $(EXAMPLE_SRCS:%.c=$(BUILD_DIR)/%)
DEPS := $(EXAMPLE_SRCS:%.c=$(BUILD_DIR)/%.d)

# Default target
.PHONY: all
all: $(EXAMPLES)

# Ensure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build examples
$(BUILD_DIR)/%: %.c $(LIB) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) $< -o $@ -L$(BUILD_DIR) -lhive

# Run targets
.PHONY: run-pingpong
run-pingpong: $(BUILD_DIR)/pingpong
	./$(BUILD_DIR)/pingpong

.PHONY: run-fileio
run-fileio: $(BUILD_DIR)/fileio
	./$(BUILD_DIR)/fileio

.PHONY: run-echo
run-echo: $(BUILD_DIR)/echo
	./$(BUILD_DIR)/echo

# Clean example binaries and deps
.PHONY: clean
clean:
	rm -f $(EXAMPLES) $(DEPS)

.PHONY: help
help:
	@echo "examples/Makefile targets:"
	@echo "  all          - Build all examples"
	@echo "  run-pingpong - Run ping-pong example"
	@echo "  run-fileio   - Run file I/O example"
	@echo "  run-echo     - Run echo server/client example"
	@echo "  clean        - Remove example binaries"

# Auto-generated dependencies
-include $(DEPS)
