/**
 * QEMU Compatibility Header for Test Suite
 *
 * This header provides compatibility shims for running the Linux test suite
 * on STM32/ARM Cortex-M via QEMU emulation.
 *
 * Include via: -include qemu/qemu_compat.h
 *
 * This header is included BEFORE any test code, so we need to be careful
 * about macro definitions that might conflict with standard headers.
 */

#ifndef QEMU_COMPAT_H
#define QEMU_COMPAT_H

/* Include standard headers first to avoid macro conflicts */
#include <stdio.h>
#include <stdint.h>

/* Define POSIX clock constants for bare-metal */
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC 1
#endif

/* Define clockid_t and timespec if not present */
#ifndef __clockid_t_defined
typedef int clockid_t;
#define __clockid_t_defined
#endif

#include <time.h>

/* Now include semihosting after stdio.h */
#include "semihosting.h"

/* Override printf/fprintf after stdio.h has been included */
#undef printf
#define printf semihosting_printf

/* Redirect fprintf to semihosting (ignore stream argument) */
#define fprintf(stream, ...) semihosting_printf(__VA_ARGS__)

/* Stub out fflush - semihosting has no buffering */
static inline int qemu_fflush(void *stream) {
    (void)stream;
    return 0;
}
#undef fflush
#define fflush qemu_fflush

/*
 * Override test stack sizes for QEMU's limited RAM (64KB total).
 * Tests typically request 128KB stacks which won't fit.
 *
 * TEST_STACK_SIZE(x) caps stack size to QEMU limit when building for QEMU.
 * On native Linux, it passes through unchanged.
 */
#define QEMU_TEST_STACK_SIZE 2048
#define TEST_STACK_SIZE(x) (((x) > QEMU_TEST_STACK_SIZE) ? QEMU_TEST_STACK_SIZE : (x))

/*
 * Replace clock_gettime with SysTick-based implementation.
 * Tests use a time_ms() helper that calls clock_gettime(CLOCK_MONOTONIC, &ts).
 * We redirect to hive_timer_get_ticks() which returns milliseconds.
 */
#include "hive_timer.h"

static inline int qemu_clock_gettime(clockid_t clk_id, struct timespec *tp) {
    (void)clk_id;
    uint32_t ticks = hive_timer_get_ticks();
    tp->tv_sec = ticks / 1000;
    tp->tv_nsec = (ticks % 1000) * 1000000;
    return 0;
}
#undef clock_gettime
#define clock_gettime qemu_clock_gettime

#endif /* QEMU_COMPAT_H */
