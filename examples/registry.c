// Registry example - Demonstrates actor name registry for service discovery
//
// Shows:
// - hive_register() - register an actor with a symbolic name
// - hive_whereis() - look up an actor ID by name
// - hive_unregister() - remove a name registration
// - Auto-cleanup when actor exits
//
// Use case: Service discovery pattern
// - Service actors register themselves with well-known names
// - Client actors find services by name (no hardcoded actor IDs)
// - Names auto-cleaned when actors exit (handles crashes gracefully)
//
// Note: For supervised actors, prefer auto_register in actor_config_t instead
// of manual hive_register() calls. This example shows the manual API.

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_timer.h"
#include <stdio.h>

#define SERVICE_NAME "calculator"

// Calculator service - registers itself and handles requests
static void calculator_service(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    // Register with a well-known name so clients can find us
    hive_status_t status = hive_register(SERVICE_NAME);
    if (HIVE_FAILED(status)) {
        printf("[SERVICE] Failed to register: %s\n", HIVE_ERR_STR(status));
        fflush(stdout);
        hive_exit();
    }
    printf("[SERVICE] Registered as '%s' (ID: %u)\n", SERVICE_NAME,
           hive_self());
    fflush(stdout);

    // Handle requests
    int requests_handled = 0;
    while (requests_handled < 3) {
        hive_message_t msg;
        status = hive_ipc_recv(&msg, 2000); // 2 second timeout
        if (HIVE_FAILED(status)) {
            printf("[SERVICE] Timeout waiting for requests, shutting down\n");
            fflush(stdout);
            break;
        }

        if (msg.class == HIVE_MSG_REQUEST) {
            // Parse request: two integers
            int *operands = (int *)msg.data;
            int a = operands[0];
            int b = operands[1];
            int result = a + b;

            printf("[SERVICE] Received %d + %d, sending result %d\n", a, b,
                   result);
            fflush(stdout);

            // Send reply
            hive_ipc_reply(&msg, &result, sizeof(result));
            requests_handled++;
        }
    }

    // Explicit unregister (optional - would auto-cleanup on exit anyway)
    printf("[SERVICE] Unregistering and shutting down\n");
    fflush(stdout);
    hive_unregister(SERVICE_NAME);

    hive_exit();
}

// Client - looks up service by name and sends requests
static void client_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("[CLIENT] Started, looking for '%s' service...\n", SERVICE_NAME);
    fflush(stdout);

    // Give service time to register
    timer_id_t timer;
    hive_timer_after(100000, &timer); // 100ms
    hive_message_t msg;
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    // Look up service by name
    actor_id_t service;
    hive_status_t status = hive_whereis(SERVICE_NAME, &service);
    if (HIVE_FAILED(status)) {
        printf("[CLIENT] Service '%s' not found!\n", SERVICE_NAME);
        fflush(stdout);
        hive_exit();
    }
    printf("[CLIENT] Found '%s' at actor ID %u\n", SERVICE_NAME, service);
    fflush(stdout);

    // Send some requests
    int test_cases[][2] = {{10, 20}, {42, 58}, {100, 200}};

    for (int i = 0; i < 3; i++) {
        int operands[2] = {test_cases[i][0], test_cases[i][1]};
        hive_message_t reply;

        printf("[CLIENT] Requesting %d + %d...\n", operands[0], operands[1]);
        fflush(stdout);

        status =
            hive_ipc_request(service, operands, sizeof(operands), &reply, 1000);
        if (HIVE_SUCCEEDED(status)) {
            int result = *(int *)reply.data;
            printf("[CLIENT] Got result: %d\n", result);
        } else {
            printf("[CLIENT] Request failed: %s\n", HIVE_ERR_STR(status));
        }
        fflush(stdout);
    }

    // Demonstrate whereis after service exits
    printf("[CLIENT] Waiting for service to exit...\n");
    fflush(stdout);
    hive_timer_after(3000000, &timer); // Wait for service to timeout and exit
    hive_ipc_recv_match(HIVE_SENDER_ANY, HIVE_MSG_TIMER, timer, &msg, -1);

    status = hive_whereis(SERVICE_NAME, &service);
    if (HIVE_FAILED(status)) {
        printf("[CLIENT] Service '%s' no longer registered (auto-cleanup "
               "worked)\n",
               SERVICE_NAME);
    }
    fflush(stdout);

    printf("[CLIENT] Done\n");
    fflush(stdout);
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Registry Example ===\n\n");
    printf("Demonstrates service discovery using name registry:\n");
    printf("- Service registers with hive_register(\"%s\")\n", SERVICE_NAME);
    printf("- Client finds service with hive_whereis(\"%s\")\n", SERVICE_NAME);
    printf("- Names auto-cleanup when actors exit\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn service first so it can register
    actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    cfg.name = "calc_service";

    actor_id_t service_id;
    if (HIVE_FAILED(
            hive_spawn(calculator_service, NULL, NULL, &cfg, &service_id))) {
        fprintf(stderr, "Failed to spawn service\n");
        hive_cleanup();
        return 1;
    }

    // Spawn client
    cfg.name = "client";
    actor_id_t client_id;
    if (HIVE_FAILED(hive_spawn(client_actor, NULL, NULL, &cfg, &client_id))) {
        fprintf(stderr, "Failed to spawn client\n");
        hive_cleanup();
        return 1;
    }

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
