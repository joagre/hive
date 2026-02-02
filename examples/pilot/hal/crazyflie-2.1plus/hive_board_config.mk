# hive_board_config.mk - Crazyflie 2.1+ board-specific Hive configuration
#
# Flash and SD card configuration for the Crazyflie 2.1+ board.
# Include this file in Makefiles that build for this board.
#
# Usage:
#   include $(HAL_DIR)/hive_board_config.mk
#
# This file adds -D flags to HIVE_CFLAGS. Ensure HIVE_CFLAGS is used
# when building libhive.a.

# =============================================================================
# STM32 Flash File Configuration
# =============================================================================
# The Crazyflie 2.1+ uses STM32F405 with 1MB flash.
# Sector 8 (128KB at 0x08080000) can be used for log storage.
#
# Flash logging disabled by default: flash erase blocks scheduler 1-4 seconds.
# Use SD card for logging instead (ENABLE_SD=1, LOG_FILE_PATH=/sd/hive.log).
# Enable flash with ENABLE_FLASH_LOG=1 if needed for debugging without SD card.

ENABLE_FLASH_LOG ?= 0
ifeq ($(ENABLE_FLASH_LOG),1)
  HIVE_CFLAGS += -DHIVE_VFILE_LOG_BASE=0x08080000
  HIVE_CFLAGS += -DHIVE_VFILE_LOG_SIZE=131072
  HIVE_CFLAGS += -DHIVE_VFILE_LOG_SECTOR=8
  HIVE_CFLAGS += '-DHIVE_FILE_RING_SIZE=(8*1024)'
endif

# =============================================================================
# SD Card Configuration
# =============================================================================
# The Micro SD Card Deck connects via SPI3 with CS on PB6.
# Enable with ENABLE_SD=1

ENABLE_SD ?= 0
ifeq ($(ENABLE_SD),1)
  HIVE_CFLAGS += -DHIVE_ENABLE_SD=1
  HIVE_CFLAGS += -DHIVE_MAX_SD_FILES=4
endif
