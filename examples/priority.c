/*
 * Priority Scheduling Example
 *
 * Demonstrates the 4-level priority scheduler for real-time systems.
 * Higher priority actors (lower number) run before lower priority actors.
 *
 * PRIORITY LEVELS:
 *   RT_PRIO_CRITICAL (0) - Safety-critical tasks (flight control, emergency stop)
 *   RT_PRIO_HIGH     (1) - Time-sensitive tasks (sensor fusion, control loops)
 *   RT_PRIO_NORMAL   (2) - Standard tasks (telemetry, logging)
 *   RT_PRIO_LOW      (3) - Background tasks (diagnostics, housekeeping)
 *
 * SCHEDULING RULES:
 * - Scheduler always picks highest priority (lowest number) runnable actor
 * - Round-robin within same priority level
 * - Lower priority actors only run when no higher priority actors are runnable
 * - Actors must yield cooperatively (no preemption mid-execution)
 *
 * USE CASES:
 * - Drone autopilot: CRITICAL=flight control, HIGH=sensors, NORMAL=telemetry
 * - Industrial control: CRITICAL=safety interlock, HIGH=PID loops, LOW=logging
 */

#include "rt_runtime.h"
#include "rt_ipc.h"
#include <stdio.h>

// Shared state to track execution order
static int execution_order[20];
static int execution_count = 0;

// Record when an actor runs
static void record_execution(const char *name, int id) {
    if (execution_count < 20) {
        execution_order[execution_count++] = id;
    }
    printf("  [%d] %s actor running\n", execution_count, name);
}

// Critical priority actor - runs first, safety-critical
static void critical_actor(void *arg) {
    int id = (int)(uintptr_t)arg;

    for (int i = 0; i < 3; i++) {
        record_execution("CRITICAL", id);
        rt_yield();  // Give scheduler a chance to pick next actor
    }

    printf("  CRITICAL actor %d done\n", id);
    rt_exit();
}

// High priority actor - runs after critical, time-sensitive
static void high_actor(void *arg) {
    int id = (int)(uintptr_t)arg;

    for (int i = 0; i < 3; i++) {
        record_execution("HIGH", id);
        rt_yield();
    }

    printf("  HIGH actor %d done\n", id);
    rt_exit();
}

// Normal priority actor - standard processing
static void normal_actor(void *arg) {
    int id = (int)(uintptr_t)arg;

    for (int i = 0; i < 3; i++) {
        record_execution("NORMAL", id);
        rt_yield();
    }

    printf("  NORMAL actor %d done\n", id);
    rt_exit();
}

// Low priority actor - background tasks
static void low_actor(void *arg) {
    int id = (int)(uintptr_t)arg;

    for (int i = 0; i < 3; i++) {
        record_execution("LOW", id);
        rt_yield();
    }

    printf("  LOW actor %d done\n", id);
    rt_exit();
}

// High priority actor that runs for a while without yielding (for starvation demo)
static void busy_high_actor(void *arg) {
    (void)arg;
    printf("  BUSY_HIGH: Starting long computation (no yield)...\n");

    // Simulate long computation without yielding
    volatile int sum = 0;
    for (int i = 0; i < 50000000; i++) {
        sum += i;
    }
    (void)sum;

    printf("  BUSY_HIGH: Done with computation\n");
    rt_exit();
}

// Low priority actor that should run after high finishes (for starvation demo)
static void waiting_low_actor(void *arg) {
    (void)arg;
    printf("  WAITING_LOW: Finally got to run!\n");
    rt_exit();
}

// Demonstrate starvation scenario
static void starving_demo(void) {
    printf("\n--- Starvation Demo ---\n");
    printf("A high-priority actor that never yields starves lower priorities.\n\n");

    // Spawn low priority first (but it won't run until high is done)
    actor_config low_cfg = RT_ACTOR_CONFIG_DEFAULT;
    low_cfg.priority = RT_PRIO_LOW;
    rt_spawn_ex(waiting_low_actor, NULL, &low_cfg);

    // Spawn high priority - it will run first and block low
    actor_config high_cfg = RT_ACTOR_CONFIG_DEFAULT;
    high_cfg.priority = RT_PRIO_HIGH;
    rt_spawn_ex(busy_high_actor, NULL, &high_cfg);

    printf("  Spawned: BUSY_HIGH and WAITING_LOW\n");
    printf("  LOW will be starved until HIGH finishes or yields.\n\n");
}

int main(void) {
    printf("=== Priority Scheduling Example ===\n\n");

    printf("Priority levels (lower number = higher priority):\n");
    printf("  RT_PRIO_CRITICAL = %d (highest)\n", RT_PRIO_CRITICAL);
    printf("  RT_PRIO_HIGH     = %d\n", RT_PRIO_HIGH);
    printf("  RT_PRIO_NORMAL   = %d\n", RT_PRIO_NORMAL);
    printf("  RT_PRIO_LOW      = %d (lowest)\n\n", RT_PRIO_LOW);

    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to init: %s\n", status.msg);
        return 1;
    }

    // --- Demo 1: Priority ordering ---
    printf("--- Demo 1: Priority Ordering ---\n");
    printf("Spawning actors in reverse priority order (LOW first).\n");
    printf("Expected: CRITICAL runs first, then HIGH, NORMAL, LOW.\n\n");

    // Spawn in reverse order to show priority matters, not spawn order
    {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.priority = RT_PRIO_LOW;
        rt_spawn_ex(low_actor, (void *)(uintptr_t)4, &cfg);
    }
    {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.priority = RT_PRIO_NORMAL;
        rt_spawn_ex(normal_actor, (void *)(uintptr_t)3, &cfg);
    }
    {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.priority = RT_PRIO_HIGH;
        rt_spawn_ex(high_actor, (void *)(uintptr_t)2, &cfg);
    }
    {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.priority = RT_PRIO_CRITICAL;
        rt_spawn_ex(critical_actor, (void *)(uintptr_t)1, &cfg);
    }

    printf("Spawned 4 actors (LOW, NORMAL, HIGH, CRITICAL)\n");
    printf("Running scheduler...\n\n");

    // --- Demo 2: Round-robin within priority ---
    printf("--- Demo 2: Round-Robin Within Priority ---\n");
    printf("Spawning 2 NORMAL actors - they alternate.\n\n");

    {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.priority = RT_PRIO_NORMAL;
        rt_spawn_ex(normal_actor, (void *)(uintptr_t)5, &cfg);
        rt_spawn_ex(normal_actor, (void *)(uintptr_t)6, &cfg);
    }

    // --- Demo 3: Starvation ---
    starving_demo();

    // Run all demos
    rt_run();

    printf("\nScheduler finished\n");
    rt_cleanup();

    // Print execution summary
    printf("\n=== Execution Order Summary ===\n");
    printf("Actors ran in this order (by ID): ");
    for (int i = 0; i < execution_count && i < 20; i++) {
        printf("%d ", execution_order[i]);
    }
    printf("\n");

    printf("\n=== Key Takeaways ===\n");
    printf("1. Higher priority actors always run before lower priority\n");
    printf("2. Round-robin scheduling within same priority level\n");
    printf("3. Lower priority actors starve if higher priority never yields\n");
    printf("4. Cooperative: actors must yield voluntarily (rt_yield, I/O, exit)\n");

    return 0;
}
