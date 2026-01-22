# src/Makefile - Build libhive.a for Linux (x86-64)
#
# Usage:
#   make                      # Build to ../build/
#   make BUILD_DIR=/some/path # Build to custom directory
#   make CFLAGS+="-DFOO"      # Add extra flags
#
# Called from top-level Makefile or from examples that need libhive.a.

# Toolchain
CC ?= gcc
AR ?= ar

# Build directory (caller can override)
BUILD_DIR ?= ../build

# Base flags
BASE_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
BASE_CFLAGS += -D_POSIX_C_SOURCE=200809L
BASE_CFLAGS += -DHIVE_PLATFORM_LINUX
BASE_CFLAGS += -DPRINTF_INCLUDE_CONFIG_H
BASE_CFLAGS += -I../include -I../lib -I../lib/printf -Ihal/linux
BASE_CFLAGS += -MMD -MP

# Combine with caller's CFLAGS (allows += from caller)
ALL_CFLAGS := $(BASE_CFLAGS) $(CFLAGS)

# Feature flags (default: all enabled for Linux)
ENABLE_NET ?= 1
ENABLE_FILE ?= 1

ifeq ($(ENABLE_NET),1)
  ALL_CFLAGS += -DHIVE_ENABLE_NET=1
else
  ALL_CFLAGS += -DHIVE_ENABLE_NET=0
endif

ifeq ($(ENABLE_FILE),1)
  ALL_CFLAGS += -DHIVE_ENABLE_FILE=1
else
  ALL_CFLAGS += -DHIVE_ENABLE_FILE=0
endif

# Source files (from shared definitions)
include hive_sources.mk

SRCS := $(HIVE_CORE_SRCS) $(HIVE_HAL_LINUX_SRCS)
ifeq ($(ENABLE_NET),1)
  SRCS += $(HIVE_NET_SRCS)
endif
ifeq ($(ENABLE_FILE),1)
  SRCS += $(HIVE_FILE_SRCS)
endif

ASM_SRCS := $(HIVE_HAL_LINUX_ASM)

# Object files
OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o) $(ASM_SRCS:%.S=$(BUILD_DIR)/%.o)
PRINTF_OBJ := $(BUILD_DIR)/printf.o
DEPS := $(OBJS:.o=.d)

# Output library
LIB := $(BUILD_DIR)/libhive.a

# Default target
.PHONY: all clean

all: $(LIB)

# Compile C sources
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) -c $< -o $@

# Compile assembly
$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) -c $< -o $@

# Compile printf library
$(PRINTF_OBJ): ../lib/printf/printf.c
	@mkdir -p $(dir $@)
	$(CC) $(ALL_CFLAGS) -c $< -o $@

# Create static library
$(LIB): $(OBJS) $(PRINTF_OBJ)
	@mkdir -p $(dir $@)
	$(AR) rcs $@ $^

clean:
	rm -f $(OBJS) $(PRINTF_OBJ) $(DEPS) $(LIB)

-include $(DEPS)
