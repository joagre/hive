# src/Makefile - Build libhive.a for Linux (x86-64)
#
# Usage: make [options]       (run 'make help' for details)
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

.PHONY: help
help:
	@echo "Hive Runtime Library - Linux Build"
	@echo ""
	@echo "Targets:"
	@echo "  make              Build libhive.a to ../build/"
	@echo "  make clean        Remove build artifacts"
	@echo "  make help         Show this help"
	@echo ""
	@echo "Build Options:"
	@echo "  BUILD_DIR=<path>      Output directory (default: ../build)"
	@echo "  ENABLE_NET=<0|1>      Network support (default: 1)"
	@echo "  ENABLE_FILE=<0|1>     File I/O support (default: 1)"
	@echo "  CFLAGS+=<flags>       Add extra compiler flags"
	@echo "  CC=<compiler>         C compiler (default: gcc)"
	@echo "  AR=<archiver>         Archiver (default: ar)"
	@echo ""
	@echo "Memory Configuration (via CFLAGS):"
	@echo "  -DHIVE_MAX_ACTORS=N              Max actors (default: 64)"
	@echo "  -DHIVE_MAX_BUSES=N               Max buses (default: 32)"
	@echo "  -DHIVE_MAILBOX_ENTRY_POOL_SIZE=N Mailbox pool (default: 256)"
	@echo "  -DHIVE_MESSAGE_DATA_POOL_SIZE=N  Message pool (default: 256)"
	@echo "  -DHIVE_DEFAULT_STACK_SIZE=N      Default stack (default: 65536)"
	@echo "  -DHIVE_STACK_ARENA_SIZE=N        Stack arena (default: 1MB)"
	@echo ""
	@echo "Examples:"
	@echo "  make"
	@echo "  make BUILD_DIR=/tmp/build"
	@echo "  make ENABLE_NET=0 ENABLE_FILE=0"
	@echo "  make CFLAGS+='-DHIVE_MAX_ACTORS=16'"

-include $(DEPS)
