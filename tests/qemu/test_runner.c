/**
 * QEMU Test Runner
 *
 * Provides the main() entry point for running test suite tests on QEMU.
 * The actual test's main() is renamed to test_main via -Dmain=test_main.
 *
 * This runner:
 * 1. Initializes SysTick for timer support
 * 2. Calls the test's entry point
 * 3. Reports exit status via semihosting
 */

#include "semihosting.h"
#include <stdint.h>

/* SysTick registers (ARM Cortex-M) */
#define SYST_CSR (*(volatile uint32_t *)0xE000E010) /* Control and Status */
#define SYST_RVR (*(volatile uint32_t *)0xE000E014) /* Reload Value */
#define SYST_CVR (*(volatile uint32_t *)0xE000E018) /* Current Value */

/* SysTick control bits */
#define SYST_CSR_ENABLE (1 << 0)
#define SYST_CSR_TICKINT (1 << 1)
#define SYST_CSR_CLKSOURCE (1 << 2)

/* LM3S6965 runs at 12 MHz in QEMU */
#define CPU_CLOCK_HZ 12000000
#define TICK_RATE_HZ 1000
#define SYSTICK_RELOAD ((CPU_CLOCK_HZ / TICK_RATE_HZ) - 1)

/* Initialize SysTick for 1ms timer ticks */
static void systick_init(void) {
    SYST_CVR = 0;                  /* Clear current value */
    SYST_RVR = SYSTICK_RELOAD;     /* Set reload value */
    SYST_CSR = SYST_CSR_ENABLE |   /* Enable SysTick */
               SYST_CSR_TICKINT |  /* Enable interrupt */
               SYST_CSR_CLKSOURCE; /* Use processor clock */
}

/* Test's main() is renamed to test_main via -Dmain=test_main */
extern int test_main(void);

int main(void) {
    systick_init();

    int result = test_main();

    semihosting_exit(result);
    return result; /* Never reached */
}
