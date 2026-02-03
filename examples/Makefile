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
ENABLE_TCP ?= 1
ENABLE_FILE ?= 1

# Build directory - always use ../build when in examples/
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

# Example sources (exclude subdirectories)
EXAMPLE_SRCS := $(wildcard *.c)

# Exclude examples that depend on disabled features
ifneq ($(ENABLE_TCP),1)
  EXAMPLE_SRCS := $(filter-out echo.c,$(EXAMPLE_SRCS))
endif
ifneq ($(ENABLE_FILE),1)
  EXAMPLE_SRCS := $(filter-out fileio.c logging.c,$(EXAMPLE_SRCS))
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

# Individual run targets
.PHONY: run-pingpong run-bus run-echo run-fileio run-hal_event run-link_demo
.PHONY: run-logging run-priority run-registry run-request_reply run-select
.PHONY: run-supervisor run-supervisor_manual run-timer

run-pingpong: $(BUILD_DIR)/pingpong
	@./$(BUILD_DIR)/pingpong

run-bus: $(BUILD_DIR)/bus
	@./$(BUILD_DIR)/bus

run-echo: $(BUILD_DIR)/echo
	@./$(BUILD_DIR)/echo

run-fileio: $(BUILD_DIR)/fileio
	@./$(BUILD_DIR)/fileio

run-hal_event: $(BUILD_DIR)/hal_event
	@./$(BUILD_DIR)/hal_event

run-link_demo: $(BUILD_DIR)/link_demo
	@./$(BUILD_DIR)/link_demo

run-logging: $(BUILD_DIR)/logging
	@./$(BUILD_DIR)/logging

run-priority: $(BUILD_DIR)/priority
	@./$(BUILD_DIR)/priority

run-registry: $(BUILD_DIR)/registry
	@./$(BUILD_DIR)/registry

run-request_reply: $(BUILD_DIR)/request_reply
	@./$(BUILD_DIR)/request_reply

run-select: $(BUILD_DIR)/select
	@./$(BUILD_DIR)/select

run-supervisor: $(BUILD_DIR)/supervisor
	@./$(BUILD_DIR)/supervisor

run-supervisor_manual: $(BUILD_DIR)/supervisor_manual
	@./$(BUILD_DIR)/supervisor_manual

run-timer: $(BUILD_DIR)/timer
	@./$(BUILD_DIR)/timer

# Run all examples
.PHONY: run-all
run-all: $(EXAMPLES)
	@echo "=== Running all examples ==="
	@failed=0; \
	for ex in $(EXAMPLES); do \
		name=$$(basename $$ex); \
		printf "%-20s " "$$name..."; \
		if $$ex >/dev/null 2>&1; then \
			echo "OK"; \
		else \
			echo "FAIL"; \
			failed=$$((failed + 1)); \
		fi; \
	done; \
	echo ""; \
	if [ $$failed -eq 0 ]; then \
		echo "All examples passed!"; \
	else \
		echo "$$failed example(s) failed"; \
		exit 1; \
	fi

# Clean example binaries and deps
.PHONY: clean
clean:
	rm -f $(EXAMPLES) $(DEPS)

.PHONY: help
help:
	@echo "examples/Makefile targets:"
	@echo "  all          - Build all examples"
	@echo "  run-all      - Run all examples (summary output)"
	@echo "  run-<name>   - Run specific example (e.g., run-pingpong)"
	@echo "  clean        - Remove example binaries"
	@echo ""
	@echo "Available examples:"
	@echo "  bus, echo, fileio, hal_event, link_demo, logging, pingpong,"
	@echo "  priority, registry, request_reply, select,"
	@echo "  supervisor, supervisor_manual, timer"

# Auto-generated dependencies
-include $(DEPS)
