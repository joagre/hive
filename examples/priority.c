/*
 * Priority Scheduling Example
 *
 * Demonstrates the 4-level priority scheduler for real-time systems.
 * Higher priority actors (lower number) run before lower priority actors.
 *
 * PRIORITY LEVELS:
 *   HIVE_PRIORITY_CRITICAL (0) - Safety-critical tasks (flight control, emergency stop)
 *   HIVE_PRIORITY_HIGH     (1) - Time-sensitive tasks (sensor fusion, control loops)
 *   HIVE_PRIORITY_NORMAL   (2) - Standard tasks (telemetry, logging)
 *   HIVE_PRIORITY_LOW      (3) - Background tasks (diagnostics, housekeeping)
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

#include "hive_runtime.h"
#include "hive_ipc.h"
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
static void critical_actor(void *args, const hive_spawn_info_t *siblings,
                           size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = (int)(uintptr_t)args;

    for (int i = 0; i < 3; i++) {
        record_execution("CRITICAL", id);
        hive_yield(); // Give scheduler a chance to pick next actor
    }

    printf("  CRITICAL actor %d done\n", id);
    hive_exit();
}

// High priority actor - runs after critical, time-sensitive
static void high_actor(void *args, const hive_spawn_info_t *siblings,
                       size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = (int)(uintptr_t)args;

    for (int i = 0; i < 3; i++) {
        record_execution("HIGH", id);
        hive_yield();
    }

    printf("  HIGH actor %d done\n", id);
    hive_exit();
}

// Normal priority actor - standard processing
static void normal_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = (int)(uintptr_t)args;

    for (int i = 0; i < 3; i++) {
        record_execution("NORMAL", id);
        hive_yield();
    }

    printf("  NORMAL actor %d done\n", id);
    hive_exit();
}

// Low priority actor - background tasks
static void low_actor(void *args, const hive_spawn_info_t *siblings,
                      size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int id = (int)(uintptr_t)args;

    for (int i = 0; i < 3; i++) {
        record_execution("LOW", id);
        hive_yield();
    }

    printf("  LOW actor %d done\n", id);
    hive_exit();
}

// High priority actor that runs for a while without yielding (for starvation demo)
static void busy_high_actor(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("  BUSY_HIGH: Starting long computation (no yield)...\n");

    // Simulate long computation without yielding
    volatile int sum = 0;
    for (int i = 0; i < 50000000; i++) {
        sum += i;
    }
    (void)sum;

    printf("  BUSY_HIGH: Done with computation\n");
    hive_exit();
}

// Low priority actor that should run after high finishes (for starvation demo)
static void waiting_low_actor(void *args, const hive_spawn_info_t *siblings,
                              size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;
    printf("  WAITING_LOW: Finally got to run!\n");
    hive_exit();
}

// Demonstrate starvation scenario
static void starving_demo(void) {
    printf("\n--- Starvation Demo ---\n");
    printf("A high-priority actor that never yields starves lower "
           "priorities.\n\n");

    // Spawn low priority first (but it won't run until high is done)
    actor_config_t low_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    low_cfg.priority = HIVE_PRIORITY_LOW;
    actor_id_t low_id;
    hive_spawn(waiting_low_actor, NULL, NULL, &low_cfg, &low_id);

    // Spawn high priority - it will run first and block low
    actor_config_t high_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    high_cfg.priority = HIVE_PRIORITY_HIGH;
    actor_id_t high_id;
    hive_spawn(busy_high_actor, NULL, NULL, &high_cfg, &high_id);

    printf("  Spawned: BUSY_HIGH and WAITING_LOW\n");
    printf("  LOW will be starved until HIGH finishes or yields.\n\n");
}

int main(void) {
    printf("=== Priority Scheduling Example ===\n\n");

    printf("Priority levels (lower number = higher priority):\n");
    printf("  HIVE_PRIORITY_CRITICAL = %d (highest)\n", HIVE_PRIORITY_CRITICAL);
    printf("  HIVE_PRIORITY_HIGH     = %d\n", HIVE_PRIORITY_HIGH);
    printf("  HIVE_PRIORITY_NORMAL   = %d\n", HIVE_PRIORITY_NORMAL);
    printf("  HIVE_PRIORITY_LOW      = %d (lowest)\n\n", HIVE_PRIORITY_LOW);

    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to init: %s\n", HIVE_ERR_STR(status));
        return 1;
    }

    // --- Demo 1: Priority ordering ---
    printf("--- Demo 1: Priority Ordering ---\n");
    printf("Spawning actors in reverse priority order (LOW first).\n");
    printf("Expected: CRITICAL runs first, then HIGH, NORMAL, LOW.\n\n");

    // Spawn in reverse order to show priority matters, not spawn order
    {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = HIVE_PRIORITY_LOW;
        actor_id_t id;
        hive_spawn(low_actor, NULL, (void *)(uintptr_t)4, &cfg, &id);
    }
    {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = HIVE_PRIORITY_NORMAL;
        actor_id_t id;
        hive_spawn(normal_actor, NULL, (void *)(uintptr_t)3, &cfg, &id);
    }
    {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = HIVE_PRIORITY_HIGH;
        actor_id_t id;
        hive_spawn(high_actor, NULL, (void *)(uintptr_t)2, &cfg, &id);
    }
    {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = HIVE_PRIORITY_CRITICAL;
        actor_id_t id;
        hive_spawn(critical_actor, NULL, (void *)(uintptr_t)1, &cfg, &id);
    }

    printf("Spawned 4 actors (LOW, NORMAL, HIGH, CRITICAL)\n");
    printf("Running scheduler...\n\n");

    // --- Demo 2: Round-robin within priority ---
    printf("--- Demo 2: Round-Robin Within Priority ---\n");
    printf("Spawning 2 NORMAL actors - they alternate.\n\n");

    {
        actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.priority = HIVE_PRIORITY_NORMAL;
        actor_id_t id5, id6;
        hive_spawn(normal_actor, NULL, (void *)(uintptr_t)5, &cfg, &id5);
        hive_spawn(normal_actor, NULL, (void *)(uintptr_t)6, &cfg, &id6);
    }

    // --- Demo 3: Starvation ---
    starving_demo();

    // Run all demos
    hive_run();

    printf("\nScheduler finished\n");
    hive_cleanup();

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
    printf("4. Cooperative: actors must yield voluntarily (hive_yield, I/O, "
           "exit)\n");

    return 0;
}
