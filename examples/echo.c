#include "hive_runtime.h"
#include "hive_net.h"
#include "hive_ipc.h"
#include "hive_log.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define ECHO_PORT 8080

// IPC message types for coordination
#define MSG_SERVER_READY 1
#define MSG_CLIENT_DONE 2

typedef struct {
    int type;
} coord_msg_t;

// Echo server actor
static void server_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    actor_id_t client_id = (actor_id_t)(uintptr_t)args;

    HIVE_LOG_DEBUG("Server actor starting (ID: %u, client_id: %u)", hive_self(),
                   client_id);
    printf("Server actor started (ID: %u)\n", hive_self());

    // Listen on port
    HIVE_LOG_DEBUG("Server: About to call hive_net_listen");
    int listen_fd;
    hive_status_t status = hive_net_listen(ECHO_PORT, &listen_fd);
    HIVE_LOG_DEBUG("Server: hive_net_listen returned, status=%d", status.code);
    if (HIVE_FAILED(status)) {
        printf("Server: Failed to listen: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Server: Listening on port %d (fd=%d)\n", ECHO_PORT, listen_fd);

    // Notify client that server is ready via IPC
    coord_msg_t ready_msg = {.type = MSG_SERVER_READY};
    status = hive_ipc_notify(client_id, MSG_SERVER_READY, &ready_msg,
                             sizeof(ready_msg));
    if (HIVE_FAILED(status)) {
        printf("Server: Failed to send ready message: %s\n",
               HIVE_ERR_STR(status));
        hive_net_close(listen_fd);
        hive_exit();
    }
    printf("Server: Sent ready notification to client via IPC\n");

    // Accept connection (block forever)
    int conn_fd;
    status = hive_net_accept(listen_fd, &conn_fd, -1);
    if (HIVE_FAILED(status)) {
        printf("Server: Failed to accept: %s\n", HIVE_ERR_STR(status));
        hive_net_close(listen_fd);
        hive_exit();
    }

    printf("Server: Accepted connection (fd=%d)\n", conn_fd);

    // Echo loop
    char buffer[256];
    for (int i = 0; i < 3; i++) {
        size_t received;
        status =
            hive_net_recv(conn_fd, buffer, sizeof(buffer) - 1, &received, -1);
        if (HIVE_FAILED(status)) {
            printf("Server: Failed to receive: %s\n", HIVE_ERR_STR(status));
            break;
        }

        if (received == 0) {
            printf("Server: Client disconnected\n");
            break;
        }

        buffer[received] = '\0';
        printf("Server: Received %zu bytes: \"%s\"\n", received, buffer);

        // Echo back
        size_t sent;
        status = hive_net_send(conn_fd, buffer, received, &sent, -1);
        if (HIVE_FAILED(status)) {
            printf("Server: Failed to send: %s\n", HIVE_ERR_STR(status));
            break;
        }

        printf("Server: Echoed %zu bytes\n", sent);
    }

    // Close connection
    hive_net_close(conn_fd);
    hive_net_close(listen_fd);

    // Wait for client done notification via IPC
    hive_message_t done_msg;
    status = hive_ipc_recv(&done_msg, 5000); // 5 second timeout
    if (HIVE_FAILED(status)) {
        printf("Server: Timeout waiting for client done message\n");
    } else {
        coord_msg_t *coord = (coord_msg_t *)done_msg.data;
        if (coord->type == MSG_CLIENT_DONE) {
            printf("Server: Received done notification from client via IPC\n");
        }
    }

    printf("Server: Done!\n");
    hive_exit();
}

// Messages to send (static to avoid stack issues)
static const char *messages[] = {"Hello, Server!", "How are you?", "Goodbye!"};

// Echo client actor
static void client_actor(void *args, const hive_spawn_info_t *siblings,
                         size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    HIVE_LOG_DEBUG("Client actor starting (ID: %u)", hive_self());
    printf("Client actor started (ID: %u)\n", hive_self());

    // Wait for server to be ready via IPC
    HIVE_LOG_DEBUG("Client: About to wait for server ready notification");
    printf("Client: Waiting for server ready notification...\n");
    hive_message_t msg;
    hive_status_t status =
        hive_ipc_recv(&msg, -1); // Block until message received
    if (HIVE_FAILED(status)) {
        printf("Client: Failed to receive ready message: %s\n",
               HIVE_ERR_STR(status));
        hive_exit();
    }

    actor_id_t server_id = msg.sender;
    coord_msg_t *coord = (coord_msg_t *)msg.data;
    if (coord->type == MSG_SERVER_READY) {
        printf("Client: Received server ready notification via IPC (from actor "
               "%u)\n",
               server_id);
    }

    printf("Client: Connecting to server...\n");

    // Connect to server (with timeout)
    int conn_fd;
    status = hive_net_connect("127.0.0.1", ECHO_PORT, &conn_fd,
                              5000); // 5 second timeout
    if (HIVE_FAILED(status)) {
        printf("Client: Failed to connect: %s\n", HIVE_ERR_STR(status));
        hive_exit();
    }

    printf("Client: Connected (fd=%d)\n", conn_fd);

    for (int i = 0; i < 3; i++) {
        // Send message
        size_t sent;
        status =
            hive_net_send(conn_fd, messages[i], strlen(messages[i]), &sent, -1);
        if (HIVE_FAILED(status)) {
            printf("Client: Failed to send: %s\n", HIVE_ERR_STR(status));
            break;
        }

        printf("Client: Sent %zu bytes: \"%s\"\n", sent, messages[i]);

        // Receive echo
        char buffer[256];
        size_t received;
        status =
            hive_net_recv(conn_fd, buffer, sizeof(buffer) - 1, &received, -1);
        if (HIVE_FAILED(status)) {
            printf("Client: Failed to receive: %s\n", HIVE_ERR_STR(status));
            break;
        }

        buffer[received] = '\0';
        printf("Client: Received echo: \"%s\"\n", buffer);
    }

    // Close connection
    hive_net_close(conn_fd);

    // Notify server that client is done via IPC
    coord_msg_t done_msg = {.type = MSG_CLIENT_DONE};
    status = hive_ipc_notify(server_id, MSG_CLIENT_DONE, &done_msg,
                             sizeof(done_msg));
    if (HIVE_FAILED(status)) {
        printf("Client: Failed to send done message: %s\n",
               HIVE_ERR_STR(status));
    } else {
        printf("Client: Sent done notification to server via IPC\n");
    }

    printf("Client: Done!\n");
    hive_exit();
}

int main(void) {
    printf("=== Actor Runtime Echo Server/Client Example ===\n\n");

    // Initialize runtime
    hive_status_t status = hive_init();
    if (HIVE_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                HIVE_ERR_STR(status));
        return 1;
    }

    // Spawn client actor first (so we know its ID to pass to server)
    actor_config_t client_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    client_cfg.name = "client";
    client_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id_t client_id;
    if (HIVE_FAILED(
            hive_spawn(client_actor, NULL, NULL, &client_cfg, &client_id))) {
        fprintf(stderr, "Failed to spawn client actor\n");
        hive_cleanup();
        return 1;
    }

    // Spawn server actor with client ID so it can send ready notification
    actor_config_t server_cfg = HIVE_ACTOR_CONFIG_DEFAULT;
    server_cfg.name = "server";
    server_cfg.priority = HIVE_PRIORITY_NORMAL;

    actor_id_t server_id;
    if (HIVE_FAILED(hive_spawn(server_actor, NULL, (void *)(uintptr_t)client_id,
                               &server_cfg, &server_id))) {
        fprintf(stderr, "Failed to spawn server actor\n");
        hive_cleanup();
        return 1;
    }

    (void)server_id; // Server ID not needed in main

    // Run scheduler
    hive_run();

    printf("\nScheduler finished\n");

    // Cleanup
    hive_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
