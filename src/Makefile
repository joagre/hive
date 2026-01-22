# src/Makefile - Build the Hive runtime library
#
# Can be run standalone or from top-level Makefile.
# Uses variables from top-level if available, otherwise sets defaults.

# Defaults (when run standalone)
CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
PLATFORM ?= linux
ENABLE_NET ?= 1
ENABLE_FILE ?= 1

# Build directory - always use ../build when in src/
BUILD_DIR := ../build

# Include path - always relative to this directory
# lib/printf: lightweight printf for embedded (low stack usage)
# -I../lib enables <printf/printf.h> include style
CPPFLAGS := -I../include -I../lib -I../lib/printf -D_POSIX_C_SOURCE=200809L
CPPFLAGS += -DPRINTF_INCLUDE_CONFIG_H
DEPFLAGS = -MMD -MP

# Platform-specific settings
ifeq ($(PLATFORM),linux)
  CPPFLAGS += -DHIVE_PLATFORM_LINUX -Ihal/linux
  PLATFORM_SRCS := hive_scheduler_linux.c hive_timer_linux.c hal/linux/hive_hal_linux.c hal/linux/hive_hal_context_linux.c
  PLATFORM_ASM := hal/linux/hive_context_x86_64.S
else ifeq ($(PLATFORM),stm32)
  CPPFLAGS += -DHIVE_PLATFORM_STM32 -Ihal/stm32
  PLATFORM_SRCS := hive_scheduler_stm32.c hive_timer_stm32.c hal/stm32/hive_hal_stm32.c hal/stm32/hive_hal_context_stm32.c
  PLATFORM_ASM := hal/stm32/hive_context_arm_cm.S
else
  $(error Unknown PLATFORM: $(PLATFORM). Use 'linux' or 'stm32')
endif

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

# Core source files (platform-independent)
CORE_SRCS := hive_actor.c hive_bus.c hive_ipc.c \
             hive_link.c hive_log.c hive_pool.c hive_runtime.c \
             hive_select.c hive_supervisor.c

# Lightweight printf library (low stack usage for embedded)
LIB_SRCS := ../lib/printf/printf.c

# Feature-specific source files
FEATURE_SRCS :=
ifeq ($(ENABLE_NET),1)
  FEATURE_SRCS += hive_net.c
endif
ifeq ($(ENABLE_FILE),1)
  ifeq ($(PLATFORM),stm32)
    FEATURE_SRCS += hive_file_stm32.c
  else
    FEATURE_SRCS += hive_file_linux.c
  endif
endif

# Combine all source files
SRCS := $(CORE_SRCS) $(PLATFORM_SRCS) $(FEATURE_SRCS)
ASM_SRCS := $(PLATFORM_ASM)

# Objects: hive sources + library sources
OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o) $(ASM_SRCS:%.S=$(BUILD_DIR)/%.o)
LIB_OBJS := $(BUILD_DIR)/printf.o
DEPS := $(SRCS:%.c=$(BUILD_DIR)/%.d)

# Library
LIB := $(BUILD_DIR)/libhive.a

# Default target
.PHONY: all
all: $(LIB)

# Ensure build directory exists
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Compile source files with dependency generation
$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $< -o $@

# Compile assembly files
$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) -c $< -o $@

# Compile printf library (from lib/printf/)
$(BUILD_DIR)/printf.o: ../lib/printf/printf.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(DEPFLAGS) -c $< -o $@

# Create static library
$(LIB): $(OBJS) $(LIB_OBJS)
	ar rcs $@ $^

# Clean library objects only
.PHONY: clean
clean:
	rm -f $(OBJS) $(LIB_OBJS) $(DEPS) $(LIB)

.PHONY: help
help:
	@echo "src/Makefile targets:"
	@echo "  all   - Build libhive.a (default)"
	@echo "  clean - Remove library and object files"
	@echo ""
	@echo "Configuration (inherited from top-level or set manually):"
	@echo "  PLATFORM=linux|stm32  - Target platform"
	@echo "  ENABLE_NET=0|1        - Enable network I/O"
	@echo "  ENABLE_FILE=0|1       - Enable file I/O"

# Include automatically generated dependencies
-include $(DEPS)
