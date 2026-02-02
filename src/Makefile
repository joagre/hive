# src/Makefile - Build libhive.a for Linux or STM32
#
# Usage:
#   make                              Build for Linux (default)
#   make PLATFORM=linux               Build for Linux
#   make PLATFORM=stm32 BUILD_DIR=... Build for STM32
#   make help                         Show full usage
#
# Called from top-level Makefile or from board-specific Makefiles.

# Platform selection (linux or stm32)
PLATFORM ?= linux

# ==============================================================================
# Platform-specific defaults
# ==============================================================================

ifeq ($(PLATFORM),linux)
  # Linux toolchain
  CC ?= gcc
  AR ?= ar

  # Build directory
  BUILD_DIR ?= ../build

  # Base flags for Linux
  BASE_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic -Werror -O2 -g
  BASE_CFLAGS += -D_POSIX_C_SOURCE=200809L
  BASE_CFLAGS += -DHIVE_PLATFORM_LINUX
  BASE_CFLAGS += -DPRINTF_INCLUDE_CONFIG_H
  BASE_CFLAGS += -I. -I../include -I../lib -I../lib/printf -Ihal/linux
  BASE_CFLAGS += -MMD -MP

  # Feature defaults for Linux
  ENABLE_TCP ?= 1
  ENABLE_FILE ?= 1

  # Board-specific mounts (optional)
  BOARD_MOUNTS_SRC ?=

else ifeq ($(PLATFORM),stm32)
  # STM32 toolchain
  CC ?= arm-none-eabi-gcc
  AR ?= arm-none-eabi-ar

  # Build directory (required for STM32)
  ifeq ($(MAKECMDGOALS),help)
    BUILD_DIR ?= /tmp/dummy
  else
    BUILD_DIR ?= $(error BUILD_DIR must be specified for PLATFORM=stm32)
  endif

  # Base flags for STM32 (caller adds MCU flags, includes, defines via CFLAGS)
  BASE_CFLAGS := -std=c11 -Wall -Wextra -Wpedantic
  BASE_CFLAGS += -O2 -g3 -gdwarf-2
  BASE_CFLAGS += -ffunction-sections -fdata-sections
  BASE_CFLAGS += -DHIVE_PLATFORM_STM32
  BASE_CFLAGS += -DPRINTF_INCLUDE_CONFIG_H
  BASE_CFLAGS += -I. -I../include -I../lib -I../lib/printf -Ihal/stm32
  BASE_CFLAGS += -MMD -MP

  # Feature defaults for STM32
  ENABLE_TCP ?= 0
  ENABLE_FILE ?= 1
  ENABLE_SD ?= 0

  # Board-specific mounts (optional)
  BOARD_MOUNTS_SRC ?=

else
  $(error Unknown PLATFORM=$(PLATFORM). Use 'linux' or 'stm32')
endif

# ==============================================================================
# Feature flags (shared logic)
# ==============================================================================

# Combine base flags with caller's CFLAGS
ALL_CFLAGS := $(BASE_CFLAGS) $(CFLAGS)

ifeq ($(ENABLE_TCP),1)
  ALL_CFLAGS += -DHIVE_ENABLE_TCP=1
else
  ALL_CFLAGS += -DHIVE_ENABLE_TCP=0
endif

ifeq ($(ENABLE_FILE),1)
  ALL_CFLAGS += -DHIVE_ENABLE_FILE=1
else
  ALL_CFLAGS += -DHIVE_ENABLE_FILE=0
endif

ifeq ($(PLATFORM),stm32)
  ifeq ($(ENABLE_SD),1)
    ALL_CFLAGS += -DHIVE_ENABLE_SD=1
    ALL_CFLAGS += -I../lib/fatfs
  else
    ALL_CFLAGS += -DHIVE_ENABLE_SD=0
  endif
endif

# ==============================================================================
# Source files
# ==============================================================================

include hive_sources.mk

ifeq ($(PLATFORM),linux)
  SRCS := $(HIVE_CORE_SRCS)
  ifneq ($(BOARD_MOUNTS_SRC),)
    # Use board-specific mounts, exclude default from HAL sources
    SRCS += $(filter-out hal/linux/hive_mounts.c,$(HIVE_HAL_LINUX_SRCS))
    SRCS += $(BOARD_MOUNTS_SRC)
  else
    SRCS += $(HIVE_HAL_LINUX_SRCS)
  endif
  ASM_SRCS := $(HIVE_HAL_LINUX_ASM)
  ifeq ($(ENABLE_TCP),1)
    SRCS += $(HIVE_TCP_SRCS) $(HIVE_HAL_LINUX_TCP_SRCS)
  endif
  ifeq ($(ENABLE_FILE),1)
    SRCS += $(HIVE_FILE_SRCS) $(HIVE_HAL_LINUX_FILE_SRCS)
  endif

else ifeq ($(PLATFORM),stm32)
  SRCS := $(HIVE_CORE_SRCS)
  ifneq ($(BOARD_MOUNTS_SRC),)
    # Use board-specific mounts, exclude default from HAL sources
    SRCS += $(filter-out hal/stm32/hive_mounts.c,$(HIVE_HAL_STM32_SRCS))
    SRCS += $(BOARD_MOUNTS_SRC)
  else
    SRCS += $(HIVE_HAL_STM32_SRCS)
  endif
  ifeq ($(ENABLE_TCP),1)
    SRCS += $(HIVE_TCP_SRCS) $(HIVE_HAL_STM32_TCP_SRCS)
  endif
  ifeq ($(ENABLE_FILE),1)
    SRCS += $(HIVE_FILE_SRCS) $(HIVE_HAL_STM32_FILE_SRCS)
  endif
  ifeq ($(ENABLE_SD),1)
    SRCS += $(HIVE_HAL_STM32_SD_SRCS) $(HIVE_FATFS_SRCS)
  endif
  ASM_SRCS := $(HIVE_HAL_STM32_ASM)
endif

# ==============================================================================
# Object files and output
# ==============================================================================

OBJS := $(SRCS:%.c=$(BUILD_DIR)/%.o) $(ASM_SRCS:%.S=$(BUILD_DIR)/%.o)
PRINTF_OBJ := $(BUILD_DIR)/printf.o
DEPS := $(OBJS:.o=.d)
LIB := $(BUILD_DIR)/libhive.a

# ==============================================================================
# Build rules
# ==============================================================================

.PHONY: all clean help

all: $(LIB)

# Compile C sources
$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
ifeq ($(PLATFORM),linux)
	$(CC) $(ALL_CFLAGS) -c $< -o $@
else
	@echo "CC $<"
	@$(CC) $(ALL_CFLAGS) -c $< -o $@
endif

# Compile assembly
$(BUILD_DIR)/%.o: %.S
	@mkdir -p $(dir $@)
ifeq ($(PLATFORM),linux)
	$(CC) $(ALL_CFLAGS) -c $< -o $@
else
	@echo "AS $<"
	@$(CC) $(ALL_CFLAGS) -c $< -o $@
endif

# Compile printf library
$(PRINTF_OBJ): ../lib/printf/printf.c
	@mkdir -p $(dir $@)
ifeq ($(PLATFORM),linux)
	$(CC) $(ALL_CFLAGS) -c $< -o $@
else
	@echo "CC $<"
	@$(CC) $(ALL_CFLAGS) -c $< -o $@
endif

# Create static library
$(LIB): $(OBJS) $(PRINTF_OBJ)
	@mkdir -p $(dir $@)
ifeq ($(PLATFORM),linux)
	$(AR) rcs $@ $^
else
	@echo "AR $@"
	@$(AR) rcs $@ $^
endif

clean:
	rm -f $(OBJS) $(PRINTF_OBJ) $(DEPS) $(LIB)

# ==============================================================================
# Help
# ==============================================================================

help:
	@echo "Hive Runtime Library"
	@echo ""
	@echo "Targets:"
	@echo "  make                    Build libhive.a for Linux"
	@echo "  make PLATFORM=stm32 ... Build libhive.a for STM32"
	@echo "  make clean              Remove build artifacts"
	@echo "  make help               Show this help"
	@echo ""
	@echo "Common Options:"
	@echo "  PLATFORM=<linux|stm32>  Target platform (default: linux)"
	@echo "  BUILD_DIR=<path>        Output directory (default: ../build for linux)"
	@echo "  ENABLE_TCP=<0|1>        TCP support (default: 1 linux, 0 stm32)"
	@echo "  ENABLE_FILE=<0|1>       File I/O support (default: 1)"
	@echo "  CFLAGS+=<flags>         Add extra compiler flags"
	@echo "  CC=<compiler>           C compiler"
	@echo "  AR=<archiver>           Archiver"
	@echo ""
	@echo "STM32-specific Options:"
	@echo "  BUILD_DIR=<path>        Output directory (required)"
	@echo "  ENABLE_SD=<0|1>         SD card support via FatFS (default: 0)"
	@echo "  BOARD_MOUNTS_SRC=<path> Board-specific hive_mounts.c"
	@echo ""
	@echo "Memory Configuration (via CFLAGS):"
	@echo "  -DHIVE_MAX_ACTORS=N              Max actors (default: 64)"
	@echo "  -DHIVE_MAX_BUSES=N               Max buses (default: 32)"
	@echo "  -DHIVE_MAILBOX_ENTRY_POOL_SIZE=N Mailbox pool (default: 256)"
	@echo "  -DHIVE_MESSAGE_DATA_POOL_SIZE=N  Message pool (default: 256)"
	@echo "  -DHIVE_DEFAULT_STACK_SIZE=N      Default stack (default: 65536)"
	@echo "  -DHIVE_STACK_ARENA_SIZE=N        Stack arena (default: 1MB)"
	@echo "  -DHIVE_MAX_SD_FILES=N            Max SD files (when ENABLE_SD=1)"
	@echo ""
	@echo "Examples:"
	@echo "  make                                    # Linux build"
	@echo "  make BUILD_DIR=/tmp/build              # Custom output dir"
	@echo "  make ENABLE_TCP=0 ENABLE_FILE=0        # Minimal build"
	@echo "  make PLATFORM=stm32 BUILD_DIR=./build \\"
	@echo "       CC=arm-none-eabi-gcc AR=arm-none-eabi-ar \\"
	@echo "       CFLAGS='-mcpu=cortex-m4 -mthumb'"

-include $(DEPS)
