#include "rt_runtime.h"
#include "rt_net.h"
#include "rt_ipc.h"
#include "rt_log.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define ECHO_PORT 8080

// IPC message types for coordination
#define MSG_SERVER_READY 1
#define MSG_CLIENT_DONE  2

typedef struct {
    int type;
} coord_msg;

// Echo server actor
static void server_actor(void *arg) {
    actor_id client_id = (actor_id)(uintptr_t)arg;

    RT_LOG_DEBUG("Server actor starting (ID: %u, client_id: %u)", rt_self(), client_id);
    printf("Server actor started (ID: %u)\n", rt_self());

    // Listen on port
    RT_LOG_DEBUG("Server: About to call rt_net_listen");
    int listen_fd;
    rt_status status = rt_net_listen(ECHO_PORT, &listen_fd);
    RT_LOG_DEBUG("Server: rt_net_listen returned, status=%d", status.code);
    if (RT_FAILED(status)) {
        printf("Server: Failed to listen: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    printf("Server: Listening on port %d (fd=%d)\n", ECHO_PORT, listen_fd);

    // Notify client that server is ready via IPC
    coord_msg ready_msg = { .type = MSG_SERVER_READY };
    status = rt_ipc_send(client_id, &ready_msg, sizeof(ready_msg), IPC_ASYNC);
    if (RT_FAILED(status)) {
        printf("Server: Failed to send ready message: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_net_close(listen_fd);
        rt_exit();
    }
    printf("Server: Sent ready notification to client via IPC\n");

    // Accept connection (block forever)
    int conn_fd;
    status = rt_net_accept(listen_fd, &conn_fd, -1);
    if (RT_FAILED(status)) {
        printf("Server: Failed to accept: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_net_close(listen_fd);
        rt_exit();
    }

    printf("Server: Accepted connection (fd=%d)\n", conn_fd);

    // Echo loop
    char buffer[256];
    for (int i = 0; i < 3; i++) {
        size_t received;
        status = rt_net_recv(conn_fd, buffer, sizeof(buffer) - 1, &received, -1);
        if (RT_FAILED(status)) {
            printf("Server: Failed to receive: %s\n",
                   status.msg ? status.msg : "unknown error");
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
        status = rt_net_send(conn_fd, buffer, received, &sent, -1);
        if (RT_FAILED(status)) {
            printf("Server: Failed to send: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Server: Echoed %zu bytes\n", sent);
    }

    // Close connection
    rt_net_close(conn_fd);
    rt_net_close(listen_fd);

    // Wait for client done notification via IPC
    rt_message done_msg;
    status = rt_ipc_recv(&done_msg, 5000);  // 5 second timeout
    if (RT_FAILED(status)) {
        printf("Server: Timeout waiting for client done message\n");
    } else {
        coord_msg *coord = (coord_msg *)done_msg.data;
        if (coord->type == MSG_CLIENT_DONE) {
            printf("Server: Received done notification from client via IPC\n");
        }
    }

    printf("Server: Done!\n");
    rt_exit();
}

// Messages to send (static to avoid stack issues)
static const char *messages[] = {
    "Hello, Server!",
    "How are you?",
    "Goodbye!"
};

// Echo client actor
static void client_actor(void *arg) {
    (void)arg;

    RT_LOG_DEBUG("Client actor starting (ID: %u)", rt_self());
    printf("Client actor started (ID: %u)\n", rt_self());

    // Wait for server to be ready via IPC
    RT_LOG_DEBUG("Client: About to wait for server ready notification");
    printf("Client: Waiting for server ready notification...\n");
    rt_message msg;
    rt_status status = rt_ipc_recv(&msg, -1);  // Block until message received
    if (RT_FAILED(status)) {
        printf("Client: Failed to receive ready message: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    // Get server ID from message sender
    actor_id server_id = msg.sender;
    coord_msg *coord = (coord_msg *)msg.data;
    if (coord->type == MSG_SERVER_READY) {
        printf("Client: Received server ready notification via IPC (from actor %u)\n", server_id);
    }

    printf("Client: Connecting to server...\n");

    // Connect to server (with timeout)
    int conn_fd;
    status = rt_net_connect("127.0.0.1", ECHO_PORT, &conn_fd, 5000); // 5 second timeout
    if (RT_FAILED(status)) {
        printf("Client: Failed to connect: %s\n",
               status.msg ? status.msg : "unknown error");
        rt_exit();
    }

    printf("Client: Connected (fd=%d)\n", conn_fd);

    for (int i = 0; i < 3; i++) {
        // Send message
        size_t sent;
        status = rt_net_send(conn_fd, messages[i], strlen(messages[i]), &sent, -1);
        if (RT_FAILED(status)) {
            printf("Client: Failed to send: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        printf("Client: Sent %zu bytes: \"%s\"\n", sent, messages[i]);

        // Receive echo
        char buffer[256];
        size_t received;
        status = rt_net_recv(conn_fd, buffer, sizeof(buffer) - 1, &received, -1);
        if (RT_FAILED(status)) {
            printf("Client: Failed to receive: %s\n",
                   status.msg ? status.msg : "unknown error");
            break;
        }

        buffer[received] = '\0';
        printf("Client: Received echo: \"%s\"\n", buffer);
    }

    // Close connection
    rt_net_close(conn_fd);

    // Notify server that client is done via IPC
    coord_msg done_msg = { .type = MSG_CLIENT_DONE };
    status = rt_ipc_send(server_id, &done_msg, sizeof(done_msg), IPC_ASYNC);
    if (RT_FAILED(status)) {
        printf("Client: Failed to send done message: %s\n",
               status.msg ? status.msg : "unknown error");
    } else {
        printf("Client: Sent done notification to server via IPC\n");
    }

    printf("Client: Done!\n");
    rt_exit();
}

int main(void) {
    printf("=== Actor Runtime Echo Server/Client Example ===\n\n");

    // Initialize runtime
    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Spawn client actor first (so we know its ID to pass to server)
    actor_config client_cfg = RT_ACTOR_CONFIG_DEFAULT;
    client_cfg.name = "client";
    client_cfg.priority = RT_PRIO_NORMAL;

    actor_id client_id = rt_spawn_ex(client_actor, NULL, &client_cfg);
    if (client_id == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn client actor\n");
        rt_cleanup();
        return 1;
    }

    // Spawn server actor with client ID so it can send ready notification
    actor_config server_cfg = RT_ACTOR_CONFIG_DEFAULT;
    server_cfg.name = "server";
    server_cfg.priority = RT_PRIO_NORMAL;

    actor_id server_id = rt_spawn_ex(server_actor, (void *)(uintptr_t)client_id, &server_cfg);
    if (server_id == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn server actor\n");
        rt_cleanup();
        return 1;
    }

    (void)server_id;  // Server ID not needed in main

    // Run scheduler
    rt_run();

    printf("\nScheduler finished\n");

    // Cleanup
    rt_cleanup();

    printf("\n=== Example completed ===\n");

    return 0;
}
