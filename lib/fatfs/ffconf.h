// FatFS Configuration for Hive Runtime
//
// Minimal configuration for SD card telemetry logging.
// See http://elm-chan.org/fsw/ff/doc/config.html for all options.

#ifndef FFCONF_DEF
#define FFCONF_DEF 80286 // Revision ID (R0.15)

// ---------------------------------------------------------------------------
// Function Configuration
// ---------------------------------------------------------------------------

#define FF_FS_READONLY 0  // 0: Read/Write, 1: Read-only
#define FF_FS_MINIMIZE 0  // 0: Full function, 1-3: Reduced function levels
#define FF_USE_FIND 0     // 0: Disable f_findfirst/f_findnext
#define FF_USE_MKFS 0     // 0: Disable f_mkfs (we don't format cards)
#define FF_USE_FASTSEEK 0 // 0: Disable fast seek (saves RAM)
#define FF_USE_EXPAND 0   // 0: Disable f_expand
#define FF_USE_CHMOD 0    // 0: Disable f_chmod/f_utime
#define FF_USE_LABEL 0    // 0: Disable f_getlabel/f_setlabel
#define FF_USE_FORWARD 0  // 0: Disable f_forward

// ---------------------------------------------------------------------------
// Strings and Paths
// ---------------------------------------------------------------------------

#define FF_USE_STRFUNC 0 // 0: Disable f_gets/f_putc/f_puts/f_printf
#define FF_PRINT_LLI 0   // 0: No long long in f_printf
#define FF_PRINT_FLOAT 0 // 0: No float in f_printf
#define FF_STRF_ENCODE 0 // 0: ANSI encoding for strings

// ---------------------------------------------------------------------------
// Locale and Code Page
// ---------------------------------------------------------------------------

#define FF_CODE_PAGE 437 // US English (ASCII subset)

// Long File Name (LFN) support is disabled to save ~2KB flash.
// Filenames must follow 8.3 format: up to 8 chars name + 3 chars extension.
// Examples: "data.bin", "log00001.txt", "config.ini"
// To enable LFN: set FF_USE_LFN=2 and add ffunicode.c to build.
#define FF_USE_LFN 0   // 0: Disable LFN (8.3 only), 2: Enable with stack buffer
#define FF_MAX_LFN 255 // Max LFN length (ignored when LFN disabled)
#define FF_LFN_UNICODE 0 // 0: ANSI/OEM encoding
#define FF_LFN_BUF 255   // LFN buffer size
#define FF_SFN_BUF 12    // SFN buffer size
#define FF_FS_RPATH 0    // 0: Disable relative path

// ---------------------------------------------------------------------------
// Drive/Volume Configuration
// ---------------------------------------------------------------------------

#define FF_VOLUMES 1       // Number of logical drives (just SD)
#define FF_STR_VOLUME_ID 0 // 0: Numeric volume ID only
#define FF_VOLUME_STRS "SD"
#define FF_MULTI_PARTITION 0 // 0: Single partition per drive
#define FF_MIN_SS 512        // Minimum sector size
#define FF_MAX_SS 512        // Maximum sector size (fixed 512 for SD)

// ---------------------------------------------------------------------------
// System Configuration
// ---------------------------------------------------------------------------

#define FF_LBA64 0            // 0: 32-bit LBA (up to 2TB)
#define FF_MIN_GPT 0x10000000 // GPT threshold (ignored with LBA32)
#define FF_USE_TRIM 0         // 0: Disable TRIM command

// ---------------------------------------------------------------------------
// Memory and Threading
// ---------------------------------------------------------------------------

#define FF_FS_TINY 0       // 0: Normal mode (separate buffer per file)
#define FF_FS_EXFAT 0      // 0: Disable exFAT (FAT12/16/32 only)
#define FF_FS_NORTC 1      // 1: No RTC - use fixed timestamp
#define FF_NORTC_MON 1     // Default month
#define FF_NORTC_MDAY 1    // Default day
#define FF_NORTC_YEAR 2025 // Default year
#define FF_FS_NOFSINFO 0   // 0: Use FSINFO if available
#define FF_FS_LOCK 0       // 0: Disable file locking (single-threaded)
#define FF_FS_REENTRANT 0  // 0: Not reentrant (single-threaded runtime)
#define FF_FS_TIMEOUT 1000 // Timeout in ticks (if reentrant enabled)

// ---------------------------------------------------------------------------
// Debug
// ---------------------------------------------------------------------------

#ifdef DEBUG
#define FF_DBG 1
#else
#define FF_DBG 0
#endif

#endif // FFCONF_DEF
