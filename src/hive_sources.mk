# hive_sources.mk - Shared source file lists for hive runtime
#
# Include this file from platform-specific Makefiles.
# This is the single source of truth for hive source files.

# Core source files (platform-independent)
HIVE_CORE_SRCS := \
	hive_actor.c \
	hive_bus.c \
	hive_ipc.c \
	hive_link.c \
	hive_log.c \
	hive_pool.c \
	hive_runtime.c \
	hive_scheduler.c \
	hive_select.c \
	hive_supervisor.c \
	hive_timer.c

# Optional feature sources
HIVE_NET_SRCS := hive_net.c
HIVE_FILE_SRCS := hive_file.c

# Linux HAL sources
HIVE_HAL_LINUX_SRCS := \
	hal/linux/hive_hal_timer_linux.c \
	hal/linux/hive_hal_linux.c \
	hal/linux/hive_hal_context_linux.c

HIVE_HAL_LINUX_ASM := hal/linux/hive_context_x86_64.S

# STM32 HAL sources
HIVE_HAL_STM32_SRCS := \
	hal/stm32/hive_hal_timer_stm32.c \
	hal/stm32/hive_hal_stm32.c \
	hal/stm32/hive_hal_context_stm32.c

HIVE_HAL_STM32_FILE_SRCS := hal/stm32/hive_hal_file_stm32.c

HIVE_HAL_STM32_ASM := hal/stm32/hive_context_arm_cm.S
