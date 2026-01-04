#include "rt_runtime.h"
#include "rt_link.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Worker actor - does some work then exits
static void worker_actor(void *arg) {
    int worker_id = *(int *)arg;

    printf("Worker %d started (Actor ID: %u)\n", worker_id, rt_self());

    // Simulate some work with random duration
    srand(time(NULL) + worker_id);
    uint64_t work_time = 200000 + (rand() % 400000);  // 200-600ms

    timer_id work_timer;
    rt_timer_after(work_time, &work_timer);

    rt_message msg;
    rt_ipc_recv(&msg, -1);

    printf("Worker %d: Completed work, exiting normally\n", worker_id);
    rt_exit();
}

// Supervisor actor - monitors workers and reports when they exit
static void supervisor_actor(void *arg) {
    (void)arg;

    printf("Supervisor started (Actor ID: %u)\n", rt_self());

    // Spawn 3 workers
    #define NUM_WORKERS 3
    uint32_t monitor_refs[NUM_WORKERS];
    int worker_ids[NUM_WORKERS] = {1, 2, 3};

    printf("Supervisor: Spawning %d workers...\n", NUM_WORKERS);

    for (int i = 0; i < NUM_WORKERS; i++) {
        actor_config worker_cfg = RT_ACTOR_CONFIG_DEFAULT;
        worker_cfg.name = "worker";

        actor_id worker = rt_spawn_ex(worker_actor, &worker_ids[i], &worker_cfg);
        if (worker == ACTOR_ID_INVALID) {
            printf("Supervisor: Failed to spawn worker %d\n", i + 1);
            continue;
        }

        // Monitor the worker
        rt_status status = rt_monitor(worker, &monitor_refs[i]);
        if (RT_FAILED(status)) {
            printf("Supervisor: Failed to monitor worker %d: %s\n", i + 1,
                   status.msg ? status.msg : "unknown error");
        } else {
            printf("Supervisor: Monitoring worker %d (Actor ID: %u, ref: %u)\n",
                   i + 1, worker, monitor_refs[i]);
        }
    }

    // Wait for workers to exit
    int workers_completed = 0;
    printf("\nSupervisor: Waiting for workers to complete...\n");

    while (workers_completed < NUM_WORKERS) {
        rt_message msg;
        rt_status status = rt_ipc_recv(&msg, -1);

        if (RT_FAILED(status)) {
            printf("Supervisor: Failed to receive message\n");
            break;
        }

        if (rt_is_exit_msg(&msg)) {
            rt_exit_msg exit_info;
            rt_decode_exit(&msg, &exit_info);

            printf("Supervisor: Worker died (Actor ID: %u, reason: %s)\n",
                   exit_info.actor,
                   exit_info.reason == RT_EXIT_NORMAL ? "NORMAL" :
                   exit_info.reason == RT_EXIT_CRASH ? "CRASH" : "KILLED");

            workers_completed++;
        } else {
            printf("Supervisor: Received unexpected message from %u\n", msg.sender);
        }
    }

    printf("\nSupervisor: All workers completed, exiting\n");
    rt_exit();
}

int main(void) {
    printf("=== Actor Runtime Supervisor Demo ===\n\n");

    // Initialize runtime
    rt_config cfg = RT_CONFIG_DEFAULT;
    cfg.max_actors = 10;

    rt_status status = rt_init(&cfg);
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Spawn supervisor
    actor_config sup_cfg = RT_ACTOR_CONFIG_DEFAULT;
    sup_cfg.name = "supervisor";

    actor_id supervisor = rt_spawn_ex(supervisor_actor, NULL, &sup_cfg);
    if (supervisor == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn supervisor\n");
        rt_cleanup();
        return 1;
    }

    printf("Spawned supervisor (Actor ID: %u)\n\n", supervisor);

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    rt_cleanup();

    printf("\n=== Demo completed ===\n");

    return 0;
}
