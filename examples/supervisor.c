// Supervisor Example - Using the hive_supervisor library
//
// Demonstrates Erlang-style supervision with automatic restart policies.
// Spawns worker actors that periodically crash, showing how the supervisor
// automatically restarts them according to the configured strategy.

#include "hive_runtime.h"
#include "hive_supervisor.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Worker states for demonstration
static volatile int g_worker_iterations[3] = {0};

// Worker actor - does some work, occasionally crashes
static void worker_actor(void *arg) {
    int worker_id = *(int *)arg;

    printf("Worker %d started (Actor ID: %u)\n", worker_id, hive_self());

    // Do some work iterations
    for (int i = 0; i < 5; i++) {
        g_worker_iterations[worker_id]++;

        // Simulate work with a short delay
        timer_id t;
        hive_timer_after(100000, &t); // 100ms
        hive_message msg;
        hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, t, &msg, -1);

        printf("Worker %d: iteration %d (total: %d)\n", worker_id, i + 1,
               g_worker_iterations[worker_id]);

        // Randomly crash (1 in 3 chance per iteration)
        if (rand() % 3 == 0) {
            printf("Worker %d: CRASHING!\n", worker_id);
            return; // Crash (return without hive_exit)
        }
    }

    printf("Worker %d: Completed all work, exiting normally\n", worker_id);
    hive_exit();
}

// Callback when supervisor shuts down
static void on_supervisor_shutdown(void *ctx) {
    (void)ctx;
    printf("\n=== Supervisor shutting down ===\n");
    printf("Final worker iterations: [%d, %d, %d]\n", g_worker_iterations[0],
           g_worker_iterations[1], g_worker_iterations[2]);
}

// Main orchestrator actor
static void orchestrator_actor(void *arg) {
    (void)arg;

    printf("Orchestrator started\n\n");

    // Seed random number generator
    srand(time(NULL));

    // Define child specifications
    static int worker_ids[3] = {0, 1, 2};
    hive_child_spec children[3] = {
        {
            .id = "worker-0",
            .fn = worker_actor,
            .arg = &worker_ids[0],
            .arg_size = sizeof(int),
            .restart = HIVE_CHILD_PERMANENT,
            .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
        },
        {
            .id = "worker-1",
            .fn = worker_actor,
            .arg = &worker_ids[1],
            .arg_size = sizeof(int),
            .restart = HIVE_CHILD_PERMANENT,
            .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
        },
        {
            .id = "worker-2",
            .fn = worker_actor,
            .arg = &worker_ids[2],
            .arg_size = sizeof(int),
            .restart = HIVE_CHILD_TRANSIENT, // Only restart on crash
            .actor_cfg = HIVE_ACTOR_CONFIG_DEFAULT,
        },
    };

    // Configure supervisor
    hive_supervisor_config sup_config = {
        .strategy = HIVE_STRATEGY_ONE_FOR_ONE, // Restart only failed child
        .max_restarts = 10,                    // Max 10 restarts...
        .restart_period_ms = 10000,            // ...within 10 seconds
        .children = children,
        .num_children = 3,
        .on_shutdown = on_supervisor_shutdown,
        .shutdown_ctx = NULL,
    };

    printf("Starting supervisor with strategy: %s\n",
           hive_restart_strategy_str(sup_config.strategy));
    printf("Max restarts: %u in %u ms\n", sup_config.max_restarts,
           sup_config.restart_period_ms);
    printf("Children:\n");
    for (size_t i = 0; i < sup_config.num_children; i++) {
        printf("  [%zu] %s - restart: %s\n", i, children[i].id,
               hive_child_restart_str(children[i].restart));
    }
    printf("\n");

    // Start supervisor
    actor_id supervisor;
    hive_status status = hive_supervisor_start(&sup_config, NULL, &supervisor);
    if (HIVE_FAILED(status)) {
        printf("Failed to start supervisor: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Supervisor started (Actor ID: %u)\n\n", supervisor);

    // Monitor supervisor to know when it exits
    uint32_t mon_ref;
    hive_monitor(supervisor, &mon_ref);

    // Let the workers run for a while
    printf("=== Running for 3 seconds... ===\n\n");

    timer_id run_timer;
    hive_timer_after(3000000, &run_timer); // 3 seconds

    // Wait for either timer or supervisor exit
    hive_message msg;
    while (1) {
        status = hive_ipc_recv(&msg, -1);
        if (HIVE_FAILED(status)) {
            break;
        }

        if (msg.class == HIVE_MSG_TIMER && msg.tag == run_timer) {
            // Time's up
            printf("\n=== Time limit reached, stopping supervisor ===\n");
            hive_supervisor_stop(supervisor);
        } else if (msg.class == HIVE_MSG_EXIT) {
            // Supervisor exited (either stopped or intensity exceeded)
            hive_exit_msg exit_info;
            hive_decode_exit(&msg, &exit_info);
            if (exit_info.actor == supervisor) {
                printf("Supervisor exited (reason: %s)\n",
                       hive_exit_reason_str(exit_info.reason));
                break;
            }
        }
    }

    printf("\n=== Demo completed ===\n");
    hive_exit();
}

int main(void) {
    printf("=== Hive Supervisor Library Demo ===\n\n");

    // Initialize runtime
    hive_status status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn orchestrator with larger stack
    actor_config cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "orchestrator";
    cfg.stack_size = 128 * 1024;

    actor_id orchestrator;
    if (HIVE_FAILED(
            hive_spawn_ex(orchestrator_actor, NULL, &cfg, &orchestrator))) {
        fprintf(stderr, "Failed to spawn orchestrator\n");
        hive_cleanup();
        return 1;
    }

    // Run scheduler
    hive_run();

    // Cleanup
    hive_cleanup();

    return 0;
}
