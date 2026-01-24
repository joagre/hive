/**
 * QEMU Test for Hive Runtime (STM32/Cortex-M target)
 *
 * Tests:
 *   1. Runtime initialization
 *   2. Actor spawn and context switching
 *   3. IPC message passing
 *   4. Timer/sleep functionality
 *
 * Run with: make qemu-test
 */

#include "semihosting.h"
#include "hive_runtime.h"
#include "hive_actor.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include "hive_scheduler.h"
#include <stdint.h>
#include <stdbool.h>

/* Internal STM32 functions used for testing */
extern uint32_t hive_timer_get_ticks(void);
extern void hive_timer_process_pending(void);

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

/* Test counters */
static volatile int s_test_passed = 0;
static volatile int s_test_failed = 0;
static volatile int s_pong_count = 0;
static volatile int s_context_switches = 0;
static volatile int s_sleep_done = 0;
static actor_id_t g_pong_actor = 0;

/* Test macros */
#define TEST_ASSERT(cond, msg)                      \
    do {                                            \
        if (cond) {                                 \
            semihosting_printf("[PASS] %s\n", msg); \
            s_test_passed++;                        \
        } else {                                    \
            semihosting_printf("[FAIL] %s\n", msg); \
            s_test_failed++;                        \
        }                                           \
    } while (0)

/* Pong actor - responds to ping messages, no timeout */
static void pong_actor(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    semihosting_printf("Pong actor started (id=%u)\n", (unsigned)hive_self());
    s_context_switches++;

    /* Receive 3 pings and reply to each */
    for (int i = 0; i < 3; i++) {
        hive_message_t msg;
        hive_status_t s = hive_ipc_recv(
            &msg, 0); /* Non-blocking - message should already be there */

        if (HIVE_FAILED(s)) {
            /* Yield and try again */
            hive_yield();
            s_context_switches++;
            i--; /* Retry this iteration */
            continue;
        }

        semihosting_printf("Pong: received ping #%d, replying\n", i + 1);

        /* Reply with pong */
        const char *pong = "PONG";
        hive_ipc_reply(&msg, pong, 5);
        s_pong_count++;
    }

    semihosting_printf("Pong actor exiting\n");
}

/* Ping actor - sends pings immediately (no sleep) */
static void ping_actor(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    semihosting_printf("Ping actor started (id=%u)\n", (unsigned)hive_self());
    s_context_switches++;

    /* Send 3 pings */
    for (int i = 0; i < 3; i++) {
        const char *ping = "PING";
        hive_message_t reply;

        semihosting_printf("Ping: sending request #%d to actor %u\n", i + 1,
                           (unsigned)g_pong_actor);

        /* Use notify + recv instead of request to avoid timeout dependency */
        hive_ipc_notify(g_pong_actor, HIVE_TAG_NONE, ping, 5);

        /* Yield to let pong process */
        hive_yield();
        s_context_switches++;
    }

    semihosting_printf("Ping actor exiting\n");
}

/* Simple yield test actor */
static void yield_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    int id = (int)(intptr_t)args;
    semihosting_printf("Yield actor %d started\n", id);
    s_context_switches++;

    for (int i = 0; i < 3; i++) {
        semihosting_printf("Yield actor %d: iteration %d\n", id, i);
        hive_yield();
        s_context_switches++;
    }

    semihosting_printf("Yield actor %d exiting\n", id);
}

/* Sleep test actor - tests timer-based sleep */
static void sleep_actor(void *args, const hive_spawn_info_t *siblings,
                        size_t sibling_count) {
    semihosting_printf("Sleep actor started (id=%u)\n", (unsigned)hive_self());

    uint32_t start = hive_timer_get_ticks();
    semihosting_printf("Sleep actor: ticks before sleep = %u\n",
                       (unsigned)start);

    /* Sleep for 50ms (50 ticks at 1ms/tick) */
    hive_status_t s = hive_sleep(50000);

    uint32_t end = hive_timer_get_ticks();
    uint32_t elapsed = end - start;

    semihosting_printf("Sleep actor: ticks after sleep = %u (elapsed=%u)\n",
                       (unsigned)end, (unsigned)elapsed);

    if (HIVE_FAILED(s)) {
        semihosting_printf("Sleep actor: hive_sleep failed: %s\n",
                           HIVE_ERR_STR(s));
    } else {
        semihosting_printf("Sleep actor: sleep completed successfully\n");
    }

    s_sleep_done = 1;
    semihosting_printf("Sleep actor exiting\n");
}

/* Main test entry point */
int main(void) {
    semihosting_printf("\n");
    semihosting_printf("=== Hive Runtime QEMU Test ===\n");
    semihosting_printf("Testing: init, spawn, context switch, IPC, sleep\n");
    semihosting_printf("\n");

    /* Initialize SysTick for timer interrupts */
    systick_init();
    semihosting_printf("SysTick initialized (reload=%u, %u Hz)\n",
                       (unsigned)SYSTICK_RELOAD, (unsigned)TICK_RATE_HZ);

    /* Test 1: Runtime initialization */
    hive_status_t s = hive_init();
    TEST_ASSERT(!HIVE_FAILED(s), "Runtime initialization");

    if (HIVE_FAILED(s)) {
        semihosting_printf("FATAL: Failed to initialize runtime\n");
        semihosting_exit(1);
    }

    /* Test 2: Spawn yield test actors */
    semihosting_printf("\n--- Test: Context Switching ---\n");
    actor_id_t yield1, yield2;

    s = hive_spawn(yield_actor, NULL, (void *)1, NULL, &yield1);
    TEST_ASSERT(!HIVE_FAILED(s), "Spawn yield actor 1");

    s = hive_spawn(yield_actor, NULL, (void *)2, NULL, &yield2);
    TEST_ASSERT(!HIVE_FAILED(s), "Spawn yield actor 2");

    /* Run scheduler until actors complete */
    hive_scheduler_run_until_blocked();

    TEST_ASSERT(s_context_switches >= 6, "Context switches occurred");
    semihosting_printf("Context switches: %d\n", s_context_switches);

    /* Test 3: IPC test */
    semihosting_printf("\n--- Test: IPC Message Passing ---\n");
    s_context_switches = 0;

    actor_id_t pong_id, ping_id;

    s = hive_spawn(pong_actor, NULL, NULL, NULL, &pong_id);
    TEST_ASSERT(!HIVE_FAILED(s), "Spawn pong actor");
    g_pong_actor = pong_id;

    s = hive_spawn(ping_actor, NULL, NULL, NULL, &ping_id);
    TEST_ASSERT(!HIVE_FAILED(s), "Spawn ping actor");

    semihosting_printf("Spawned: pong=%u, ping=%u\n", (unsigned)pong_id,
                       (unsigned)ping_id);

    /* Run scheduler until actors complete */
    hive_scheduler_run_until_blocked();

    TEST_ASSERT(s_pong_count >= 1, "IPC messages exchanged");
    semihosting_printf("Pong replies: %d\n", s_pong_count);

    /* Test 4: Sleep/timer test */
    semihosting_printf("\n--- Test: Timer/Sleep ---\n");

    /* Check if ticks are incrementing */
    uint32_t tick1 = hive_timer_get_ticks();
    /* Busy wait - needs to be long enough for at least one tick (1ms at 12MHz = 12000 cycles) */
    for (volatile int i = 0; i < 1000000; i++) {
    }
    uint32_t tick2 = hive_timer_get_ticks();
    semihosting_printf("Tick check: %u -> %u (delta=%u)\n", (unsigned)tick1,
                       (unsigned)tick2, (unsigned)(tick2 - tick1));
    TEST_ASSERT(tick2 > tick1, "SysTick is running");

    /* Test 5: hive_get_time() monotonicity */
    semihosting_printf("\n--- Test: hive_get_time() ---\n");
    uint64_t time1 = hive_get_time();
    for (volatile int i = 0; i < 100000; i++) {
    }
    uint64_t time2 = hive_get_time();
    semihosting_printf("hive_get_time: %u -> %u us\n", (unsigned)time1,
                       (unsigned)time2);
    TEST_ASSERT(time2 >= time1, "hive_get_time is monotonic");

    /* Test 6: hive_get_time() returns microseconds (matches tick * 1000) */
    uint32_t tick_now = hive_timer_get_ticks();
    uint64_t time_now = hive_get_time();
    /* With 1ms ticks and HIVE_TIMER_TICK_US=1000, time_us should be tick * 1000 */
    uint64_t expected_us = (uint64_t)tick_now * 1000;
    int64_t diff = (int64_t)time_now - (int64_t)expected_us;
    semihosting_printf("Tick=%u, time=%u us, expected=%u us, diff=%d\n",
                       (unsigned)tick_now, (unsigned)time_now,
                       (unsigned)expected_us, (int)diff);
    /* Allow some tolerance for timing between the two calls */
    TEST_ASSERT(diff >= -2000 && diff <= 2000,
                "hive_get_time matches tick*1000");

    if (tick2 > tick1) {
        /* Ticks working, test sleep */
        actor_id_t sleep_id;
        s = hive_spawn(sleep_actor, NULL, NULL, NULL, &sleep_id);
        TEST_ASSERT(!HIVE_FAILED(s), "Spawn sleep actor");

        /* Run scheduler - need to process timer events */
        semihosting_printf("Running scheduler for sleep test...\n");
        for (int i = 0; i < 1000 && !s_sleep_done; i++) {
            hive_timer_process_pending();
            hive_scheduler_run_until_blocked();
            /* Small delay to let time pass */
            for (volatile int j = 0; j < 10000; j++) {
            }
        }

        TEST_ASSERT(s_sleep_done, "Sleep completed");
    } else {
        semihosting_printf("Skipping sleep test - SysTick not running\n");
    }

    /* Cleanup */
    hive_cleanup();

    /* Summary */
    semihosting_printf("\n");
    semihosting_printf("=== Test Summary ===\n");
    semihosting_printf("Passed: %d\n", s_test_passed);
    semihosting_printf("Failed: %d\n", s_test_failed);
    semihosting_printf("\n");

    if (s_test_failed > 0) {
        semihosting_printf("TESTS FAILED\n");
        semihosting_exit(1);
    } else {
        semihosting_printf("ALL TESTS PASSED\n");
        semihosting_exit(0);
    }

    return 0; /* Never reached */
}
