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
HIVE_TCP_SRCS := hive_tcp.c
HIVE_FILE_SRCS := hive_file.c

# Linux HAL sources (required)
HIVE_HAL_LINUX_SRCS := \
	hal/linux/hive_hal_time.c \
	hal/linux/hive_hal_event.c \
	hal/linux/hive_hal_timer.c \
	hal/linux/hive_hal_context.c \
	hal/linux/hive_hal_log.c \
	hal/linux/hive_mounts.c

# Linux HAL optional sources
HIVE_HAL_LINUX_FILE_SRCS := hal/linux/hive_hal_file.c
HIVE_HAL_LINUX_TCP_SRCS := hal/linux/hive_hal_tcp.c

HIVE_HAL_LINUX_ASM := hal/linux/hive_context_x86_64.S

# STM32 HAL sources (required)
# Note: hive_hal_printf() must be provided by the application (no default)
HIVE_HAL_STM32_SRCS := \
	hal/stm32/hive_hal_time.c \
	hal/stm32/hive_hal_event.c \
	hal/stm32/hive_hal_timer.c \
	hal/stm32/hive_hal_context.c \
	hal/stm32/hive_mounts.c

# STM32 HAL optional sources
HIVE_HAL_STM32_FILE_SRCS := hal/stm32/hive_hal_file.c
HIVE_HAL_STM32_TCP_SRCS := hal/stm32/hive_hal_tcp.c

# SD card support sources (conditional on HIVE_ENABLE_SD)
HIVE_HAL_STM32_SD_SRCS := hal/stm32/spi_sd.c

# FatFS library sources (conditional on HIVE_ENABLE_SD)
# Note: ff.c must be fetched separately - see lib/fatfs/README.md
# Note: 8.3 filenames only (no LFN) to save ~2KB flash
HIVE_FATFS_SRCS := ../lib/fatfs/ff.c ../lib/fatfs/diskio.c

HIVE_HAL_STM32_ASM := hal/stm32/hive_context_arm_cm.S
