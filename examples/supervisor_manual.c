#include "hive_runtime.h"
#include "hive_link.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Worker actor - does some work then exits
static void worker_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    int worker_id = *(int *)args;

    printf("Worker %d started (Actor ID: %u)\n", worker_id, hive_self());

    // Simulate some work with random duration
    srand(time(NULL) + worker_id);
    uint64_t work_time = 200000 + (rand() % 400000); // 200-600ms

    timer_id_t work_timer;
    hive_timer_after(work_time, &work_timer);

    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, work_timer, &msg, -1);

    printf("Worker %d: Completed work, exiting normally\n", worker_id);
    hive_exit();
}

// Supervisor actor - monitors workers and reports when they exit
static void supervisor_actor(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Supervisor started (Actor ID: %u)\n", hive_self());

// Spawn 3 workers
#define NUM_WORKERS 3
    uint32_t monitor_ids[NUM_WORKERS];
    int worker_ids[NUM_WORKERS] = {1, 2, 3};

    printf("Supervisor: Spawning %d workers...\n", NUM_WORKERS);

    for (int i = 0; i < NUM_WORKERS; i++) {
        actor_config_t worker_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        worker_cfg.name = "worker";

        actor_id_t worker;
        if (HIVE_FAILED(hive_spawn(worker_actor, NULL, &worker_ids[i],
                                   &worker_cfg, &worker))) {
            printf("Supervisor: Failed to spawn worker %d\n", i + 1);
            continue;
        }

        // Monitor the worker
        hive_status_t status = hive_monitor(worker, &monitor_ids[i]);
        if (HIVE_FAILED(status)) {
            printf("Supervisor: Failed to monitor worker %d: %s\n", i + 1,
                   HIVE_ERR_STR(status));
        } else {
            printf("Supervisor: Monitoring worker %d (Actor ID: %u, id: %u)\n",
                   i + 1, worker, monitor_ids[i]);
        }
    }

    // Wait for workers to exit
    int workers_completed = 0;
    printf("\nSupervisor: Waiting for workers to complete...\n");

    while (workers_completed < NUM_WORKERS) {
        hive_message_t msg;
        hive_status_t status = hive_ipc_recv(&msg, -1);

        if (HIVE_FAILED(status)) {
            printf("Supervisor: Failed to receive message\n");
            break;
        }

        if (msg.class == HIVE_MSG_EXIT) {
            hive_exit_msg_t *exit_info = (hive_exit_msg_t *)msg.data;

            printf("Supervisor: Worker died (Actor ID: %u, reason: %s)\n",
                   exit_info->actor, hive_exit_reason_str(exit_info->reason));

            workers_completed++;
        } else {
            printf("Supervisor: Received unexpected message from %u\n",
                   msg.sender);
        }
    }

    printf("\nSupervisor: All workers completed, exiting\n");
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Supervisor Demo ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn supervisor with larger stack (needs space for arrays and nested spawns)
    actor_config_t sup_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    sup_cfg.name = "supervisor";
#ifdef QEMU_TEST_STACK_SIZE
    sup_cfg.stack_size = 2048; // Reduced for QEMU
#else
    sup_cfg.stack_size = 128 * 1024; // 128KB stack
#endif

    actor_id_t supervisor;
    if (HIVE_FAILED(
            hive_spawn(supervisor_actor, NULL, NULL, &sup_cfg, &supervisor))) {
        fprintf(stderr, "Failed to spawn supervisor\n");
        hive_cleanup();
        return 1;
    }

    printf("Spawned supervisor (Actor ID: %u)\n\n", supervisor);

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Demo completed ===\n");

    return 0;
}
