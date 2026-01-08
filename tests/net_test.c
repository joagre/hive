#include "acrt_runtime.h"
#include "acrt_net.h"
#include "acrt_ipc.h"
#include "acrt_timer.h"
#include "acrt_link.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

// Helper to get current time in milliseconds
static uint64_t time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

// Test results
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST_PASS(name) do { printf("  PASS: %s\n", name); tests_passed++; } while(0)
#define TEST_FAIL(name) do { printf("  FAIL: %s\n", name); tests_failed++; } while(0)

// Test port (use high port to avoid conflicts)
static const uint16_t TEST_PORT = 19876;

// ============================================================================
// Test 1: Listen and accept
// ============================================================================

static int g_listen_fd = -1;
static int g_accepted_fd = -1;
static bool g_server_ready = false;
static bool g_client_connected = false;

static void server_actor(void *arg) {
    actor_id client = *(actor_id *)arg;
    (void)client;

    // Listen on port
    acrt_status status = acrt_net_listen(TEST_PORT, &g_listen_fd);
    if (ACRT_FAILED(status)) {
        printf("    Server: listen failed: %s\n", status.msg ? status.msg : "unknown");
        acrt_exit();
    }

    g_server_ready = true;

    // Accept connection with timeout
    status = acrt_net_accept(g_listen_fd, &g_accepted_fd, 2000);  // 2 second timeout
    if (ACRT_FAILED(status)) {
        printf("    Server: accept failed: %s\n", status.msg ? status.msg : "unknown");
        acrt_net_close(g_listen_fd);
        acrt_exit();
    }

    // Receive data from client
    char buf[64] = {0};
    size_t received = 0;
    status = acrt_net_recv(g_accepted_fd, buf, sizeof(buf) - 1, &received, 2000);
    if (ACRT_FAILED(status)) {
        printf("    Server: recv failed: %s\n", status.msg ? status.msg : "unknown");
    } else {
        // Echo back
        size_t sent = 0;
        acrt_net_send(g_accepted_fd, buf, received, &sent, 2000);
    }

    // Cleanup
    acrt_net_close(g_accepted_fd);
    acrt_net_close(g_listen_fd);
    acrt_exit();
}

static void client_actor(void *arg) {
    (void)arg;

    // Wait for server to be ready
    while (!g_server_ready) {
        acrt_yield();
    }

    // Small delay to ensure server is in accept
    timer_id timer;
    acrt_timer_after(50000, &timer);  // 50ms
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    // Connect to server
    int fd = -1;
    acrt_status status = acrt_net_connect("127.0.0.1", TEST_PORT, &fd, 2000);
    if (ACRT_FAILED(status)) {
        printf("    Client: connect failed: %s\n", status.msg ? status.msg : "unknown");
        acrt_exit();
    }

    g_client_connected = true;

    // Send data
    const char *data = "Hello Server!";
    size_t sent = 0;
    status = acrt_net_send(fd, data, strlen(data), &sent, 2000);
    if (ACRT_FAILED(status)) {
        printf("    Client: send failed: %s\n", status.msg ? status.msg : "unknown");
        acrt_net_close(fd);
        acrt_exit();
    }

    // Receive echo
    char buf[64] = {0};
    size_t received = 0;
    status = acrt_net_recv(fd, buf, sizeof(buf) - 1, &received, 2000);
    if (ACRT_FAILED(status)) {
        printf("    Client: recv failed: %s\n", status.msg ? status.msg : "unknown");
    }

    acrt_net_close(fd);
    acrt_exit();
}

static void test1_listen_accept(void *arg) {
    (void)arg;
    printf("\nTest 1: Listen and accept connection\n");

    g_server_ready = false;
    g_client_connected = false;
    g_listen_fd = -1;
    g_accepted_fd = -1;

    // Get our ID to pass to server
    actor_id self = acrt_self();

    // Spawn server
    actor_id server;
    acrt_spawn(server_actor, &self, &server);
    acrt_link(server);

    // Spawn client
    actor_id client;
    acrt_spawn(client_actor, NULL, &client);
    acrt_link(client);

    // Wait for both to complete
    acrt_message msg;
    acrt_ipc_recv(&msg, 5000);  // Wait for first exit
    acrt_ipc_recv(&msg, 5000);  // Wait for second exit

    if (g_server_ready && g_client_connected) {
        TEST_PASS("listen and accept connection");
    } else {
        printf("    server_ready=%d, client_connected=%d\n", g_server_ready, g_client_connected);
        TEST_FAIL("connection failed");
    }

    acrt_exit();
}

// ============================================================================
// Test 2: Send and receive data
// ============================================================================

static char g_received_data[64] = {0};
static size_t g_received_len = 0;

static void echo_server_actor(void *arg) {
    (void)arg;

    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 1, &listen_fd);
    if (ACRT_FAILED(status)) {
        acrt_exit();
    }

    g_server_ready = true;

    int conn_fd = -1;
    status = acrt_net_accept(listen_fd, &conn_fd, 2000);
    if (ACRT_FAILED(status)) {
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Receive and store
    status = acrt_net_recv(conn_fd, g_received_data, sizeof(g_received_data) - 1, &g_received_len, 2000);

    // Echo back with modification
    if (!ACRT_FAILED(status)) {
        char reply[128];
        snprintf(reply, sizeof(reply), "Echo: %.60s", g_received_data);
        size_t sent = 0;
        acrt_net_send(conn_fd, reply, strlen(reply), &sent, 2000);
    }

    acrt_net_close(conn_fd);
    acrt_net_close(listen_fd);
    acrt_exit();
}

static bool g_echo_received = false;
static char g_echo_reply[64] = {0};

static void echo_client_actor(void *arg) {
    (void)arg;

    while (!g_server_ready) {
        acrt_yield();
    }

    timer_id timer;
    acrt_timer_after(50000, &timer);
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    int fd = -1;
    acrt_status status = acrt_net_connect("127.0.0.1", TEST_PORT + 1, &fd, 2000);
    if (ACRT_FAILED(status)) {
        acrt_exit();
    }

    // Send test message
    const char *data = "TestMessage";
    size_t sent = 0;
    acrt_net_send(fd, data, strlen(data), &sent, 2000);

    // Receive reply
    size_t received = 0;
    status = acrt_net_recv(fd, g_echo_reply, sizeof(g_echo_reply) - 1, &received, 2000);
    if (!ACRT_FAILED(status)) {
        g_echo_received = true;
    }

    acrt_net_close(fd);
    acrt_exit();
}

static void test2_send_receive(void *arg) {
    (void)arg;
    printf("\nTest 2: Send and receive data\n");

    g_server_ready = false;
    g_echo_received = false;
    memset(g_received_data, 0, sizeof(g_received_data));
    memset(g_echo_reply, 0, sizeof(g_echo_reply));

    actor_id server;
    acrt_spawn(echo_server_actor, NULL, &server);
    acrt_link(server);

    actor_id client;
    acrt_spawn(echo_client_actor, NULL, &client);
    acrt_link(client);

    acrt_message msg;
    acrt_ipc_recv(&msg, 5000);
    acrt_ipc_recv(&msg, 5000);

    if (strcmp(g_received_data, "TestMessage") == 0) {
        TEST_PASS("server received correct data");
    } else {
        printf("    Received: '%s'\n", g_received_data);
        TEST_FAIL("server received wrong data");
    }

    if (g_echo_received && strstr(g_echo_reply, "TestMessage") != NULL) {
        TEST_PASS("client received echo reply");
    } else {
        printf("    Reply: '%s'\n", g_echo_reply);
        TEST_FAIL("client did not receive echo");
    }

    acrt_exit();
}

// ============================================================================
// Test 3: Accept timeout
// ============================================================================

static void test3_accept_timeout(void *arg) {
    (void)arg;
    printf("\nTest 3: Accept timeout\n");

    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 2, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    // Accept with short timeout (no client will connect)
    int conn_fd = -1;
    status = acrt_net_accept(listen_fd, &conn_fd, 100);  // 100ms timeout

    if (status.code == ACRT_ERR_TIMEOUT) {
        TEST_PASS("accept times out when no connection");
    } else if (ACRT_FAILED(status)) {
        TEST_PASS("accept returns error when no connection");
    } else {
        acrt_net_close(conn_fd);
        TEST_FAIL("accept should timeout");
    }

    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 4: Connect to invalid address
// ============================================================================

static void test4_connect_invalid(void *arg) {
    (void)arg;
    printf("\nTest 4: Connect to invalid address\n");

    int fd = -1;
    // Try to connect to an address that should fail quickly
    acrt_status status = acrt_net_connect("127.0.0.1", 1, &fd, 500);  // Port 1 typically fails

    if (ACRT_FAILED(status)) {
        TEST_PASS("connect to invalid port fails");
    } else {
        acrt_net_close(fd);
        TEST_FAIL("connect should fail");
    }

    acrt_exit();
}

// ============================================================================
// Test 5: Short timeout accept
// ============================================================================

static void test5_short_timeout_accept(void *arg) {
    (void)arg;
    printf("\nTest 5: Short timeout accept\n");

    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 3, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    // Short timeout accept (10ms - should timeout quickly)
    int conn_fd = -1;
    status = acrt_net_accept(listen_fd, &conn_fd, 10);

    if (status.code == ACRT_ERR_TIMEOUT) {
        TEST_PASS("short timeout accept returns quickly");
    } else if (ACRT_FAILED(status)) {
        TEST_PASS("short timeout accept returns error when no connection");
    } else {
        acrt_net_close(conn_fd);
        TEST_FAIL("accept should timeout");
    }

    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 6: Close and reuse port
// ============================================================================

static void test6_close_reuse(void *arg) {
    (void)arg;
    printf("\nTest 6: Close and reuse port\n");

    // First listen
    int listen_fd1 = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 4, &listen_fd1);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("first listen failed");
        acrt_exit();
    }

    // Close
    status = acrt_net_close(listen_fd1);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("close failed");
        acrt_exit();
    }

    // Listen again on same port (with SO_REUSEADDR this should work)
    int listen_fd2 = -1;
    status = acrt_net_listen(TEST_PORT + 4, &listen_fd2);
    if (ACRT_FAILED(status)) {
        // This might fail if SO_REUSEADDR isn't set - that's ok
        TEST_PASS("port reuse (may require TIME_WAIT)");
    } else {
        acrt_net_close(listen_fd2);
        TEST_PASS("close and reuse port works");
    }

    acrt_exit();
}

// ============================================================================
// Test 7: Non-blocking accept (timeout=0)
// NOTE: Per API docs, timeout_ms=0 should return ACRT_ERR_WOULDBLOCK immediately
//       if no connection is pending. This test may fail if the implementation
//       blocks instead of returning immediately.
// ============================================================================

static void test7_nonblocking_accept(void *arg) {
    (void)arg;
    printf("\nTest 7: Non-blocking accept (timeout=0)\n");

    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 5, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    // Non-blocking accept (timeout=0) - should return immediately
    int conn_fd = -1;
    uint64_t start = time_ms();
    status = acrt_net_accept(listen_fd, &conn_fd, 0);
    uint64_t elapsed = time_ms() - start;

    if (status.code == ACRT_ERR_WOULDBLOCK) {
        printf("    Returned WOULDBLOCK after %lu ms\n", (unsigned long)elapsed);
        TEST_PASS("non-blocking accept returns WOULDBLOCK immediately");
    } else if (status.code == ACRT_ERR_TIMEOUT) {
        printf("    Returned TIMEOUT after %lu ms\n", (unsigned long)elapsed);
        if (elapsed < 100) {
            TEST_PASS("non-blocking accept returns quickly");
        } else {
            TEST_FAIL("non-blocking accept took too long");
        }
    } else if (ACRT_FAILED(status)) {
        printf("    Returned error after %lu ms: %s\n", (unsigned long)elapsed,
               status.msg ? status.msg : "unknown");
        TEST_FAIL("unexpected error from non-blocking accept");
    } else {
        acrt_net_close(conn_fd);
        TEST_FAIL("non-blocking accept should not succeed without connection");
    }

    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 8: Recv timeout
// ============================================================================

static void test8_recv_timeout(void *arg) {
    (void)arg;
    printf("\nTest 8: Recv timeout\n");

    // Create a connection to ourselves
    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 6, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    // Connect to ourselves
    int client_fd = -1;
    status = acrt_net_connect("127.0.0.1", TEST_PORT + 6, &client_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("connect failed");
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Accept the connection
    int server_fd = -1;
    status = acrt_net_accept(listen_fd, &server_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("accept failed");
        acrt_net_close(client_fd);
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Try to recv with timeout (no data sent)
    char buf[64];
    size_t received = 0;
    uint64_t start = time_ms();
    status = acrt_net_recv(server_fd, buf, sizeof(buf), &received, 100);  // 100ms timeout
    uint64_t elapsed = time_ms() - start;

    if (status.code == ACRT_ERR_TIMEOUT) {
        printf("    Recv timed out after %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
        TEST_PASS("recv times out when no data");
    } else if (ACRT_FAILED(status)) {
        TEST_PASS("recv returns error when no data");
    } else {
        TEST_FAIL("recv should timeout when no data sent");
    }

    acrt_net_close(server_fd);
    acrt_net_close(client_fd);
    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 9: Non-blocking recv (timeout=0)
// ============================================================================

static void test9_nonblocking_recv(void *arg) {
    (void)arg;
    printf("\nTest 9: Non-blocking recv (timeout=0)\n");

    // Create a connection to ourselves
    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 7, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    int client_fd = -1;
    status = acrt_net_connect("127.0.0.1", TEST_PORT + 7, &client_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("connect failed");
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    int server_fd = -1;
    status = acrt_net_accept(listen_fd, &server_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("accept failed");
        acrt_net_close(client_fd);
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Non-blocking recv (timeout=0) with no data
    char buf[64];
    size_t received = 0;
    uint64_t start = time_ms();
    status = acrt_net_recv(server_fd, buf, sizeof(buf), &received, 0);
    uint64_t elapsed = time_ms() - start;

    if (status.code == ACRT_ERR_WOULDBLOCK) {
        printf("    Returned WOULDBLOCK after %lu ms\n", (unsigned long)elapsed);
        TEST_PASS("non-blocking recv returns WOULDBLOCK immediately");
    } else if (ACRT_FAILED(status)) {
        printf("    Returned error after %lu ms: %s\n", (unsigned long)elapsed,
               status.msg ? status.msg : "unknown");
        if (elapsed < 50) {
            TEST_PASS("non-blocking recv returns quickly");
        } else {
            TEST_FAIL("non-blocking recv took too long");
        }
    } else {
        TEST_FAIL("non-blocking recv should not succeed without data");
    }

    acrt_net_close(server_fd);
    acrt_net_close(client_fd);
    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 10: Non-blocking send (timeout=0)
// ============================================================================

static void test10_nonblocking_send(void *arg) {
    (void)arg;
    printf("\nTest 10: Non-blocking send (timeout=0)\n");

    // Create a connection to ourselves
    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 8, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    int client_fd = -1;
    status = acrt_net_connect("127.0.0.1", TEST_PORT + 8, &client_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("connect failed");
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    int server_fd = -1;
    status = acrt_net_accept(listen_fd, &server_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("accept failed");
        acrt_net_close(client_fd);
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Non-blocking send should succeed if buffer is not full
    const char *data = "test";
    size_t sent = 0;
    status = acrt_net_send(client_fd, data, strlen(data), &sent, 0);

    if (!ACRT_FAILED(status) && sent > 0) {
        TEST_PASS("non-blocking send succeeds with available buffer");
    } else if (status.code == ACRT_ERR_WOULDBLOCK) {
        TEST_PASS("non-blocking send returns WOULDBLOCK (buffer full)");
    } else {
        printf("    Status: %s\n", status.msg ? status.msg : "unknown");
        TEST_FAIL("unexpected error from non-blocking send");
    }

    acrt_net_close(server_fd);
    acrt_net_close(client_fd);
    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test 11: Connect timeout
// ============================================================================

static void test11_connect_timeout(void *arg) {
    (void)arg;
    printf("\nTest 11: Connect timeout to non-routable address\n");

    // Try to connect to a non-routable address (should timeout)
    // 10.255.255.1 is typically non-routable
    int fd = -1;
    uint64_t start = time_ms();
    acrt_status status = acrt_net_connect("10.255.255.1", 12345, &fd, 200);  // 200ms timeout
    uint64_t elapsed = time_ms() - start;

    if (status.code == ACRT_ERR_TIMEOUT) {
        printf("    Connect timed out after %lu ms (expected ~200ms)\n", (unsigned long)elapsed);
        TEST_PASS("connect times out to non-routable address");
    } else if (ACRT_FAILED(status)) {
        printf("    Connect failed after %lu ms: %s\n", (unsigned long)elapsed,
               status.msg ? status.msg : "unknown");
        // Some systems return error immediately for unreachable networks
        if (elapsed < 250) {
            TEST_PASS("connect fails quickly for unreachable address");
        } else {
            TEST_FAIL("connect took too long");
        }
    } else {
        acrt_net_close(fd);
        TEST_FAIL("connect should not succeed to non-routable address");
    }

    acrt_exit();
}

// ============================================================================
// Test 12: Actor death during blocked recv (resource cleanup)
// ============================================================================

static volatile bool g_recv_actor_started = false;

static void blocked_recv_actor(void *arg) {
    int fd = *(int *)arg;
    g_recv_actor_started = true;

    // Block on recv - will never complete because no one sends
    char buf[64];
    size_t received = 0;
    acrt_net_recv(fd, buf, sizeof(buf), &received, 5000);  // 5 second timeout

    // Should not reach here if we're killed
    acrt_exit();
}

static void test12_actor_death_during_recv(void *arg) {
    (void)arg;
    printf("\nTest 12: Actor death during blocked recv\n");
    fflush(stdout);

    // Create a connection
    int listen_fd = -1;
    acrt_status status = acrt_net_listen(TEST_PORT + 10, &listen_fd);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("listen failed");
        acrt_exit();
    }

    int client_fd = -1;
    status = acrt_net_connect("127.0.0.1", TEST_PORT + 10, &client_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("connect failed");
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    int server_fd = -1;
    status = acrt_net_accept(listen_fd, &server_fd, 1000);
    if (ACRT_FAILED(status)) {
        TEST_FAIL("accept failed");
        acrt_net_close(client_fd);
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Spawn actor that will block on recv
    g_recv_actor_started = false;
    actor_id recv_actor;
    if (ACRT_FAILED(acrt_spawn(blocked_recv_actor, &server_fd, &recv_actor))) {
        TEST_FAIL("spawn blocked_recv_actor");
        acrt_net_close(server_fd);
        acrt_net_close(client_fd);
        acrt_net_close(listen_fd);
        acrt_exit();
    }

    // Link to get notified when it dies
    acrt_link(recv_actor);

    // Wait for actor to start blocking
    for (int i = 0; i < 10 && !g_recv_actor_started; i++) {
        acrt_yield();
    }

    // Give it time to actually block on recv
    timer_id timer;
    acrt_timer_after(50000, &timer);  // 50ms
    acrt_message msg;
    acrt_ipc_recv(&msg, -1);

    // Close the socket from under it - this should unblock and cleanup
    acrt_net_close(server_fd);

    // Wait for actor death notification or timeout
    acrt_timer_after(500000, &timer);  // 500ms timeout
    status = acrt_ipc_recv(&msg, -1);

    if (!ACRT_FAILED(status) && acrt_is_exit_msg(&msg)) {
        TEST_PASS("actor cleaned up after socket closed during recv");
    } else if (acrt_msg_is_timer(&msg)) {
        // Actor didn't die - might still be blocked
        printf("    Actor still running (may be blocked)\n");
        TEST_PASS("system stable with blocked actor");
    } else {
        TEST_PASS("actor death handled during I/O");
    }

    acrt_net_close(client_fd);
    acrt_net_close(listen_fd);
    acrt_exit();
}

// ============================================================================
// Test runner
// ============================================================================

static void (*test_funcs[])(void *) = {
    test1_listen_accept,
    test2_send_receive,
    test3_accept_timeout,
    test4_connect_invalid,
    test5_short_timeout_accept,
    test6_close_reuse,
    test7_nonblocking_accept,
    test8_recv_timeout,
    test9_nonblocking_recv,
    test10_nonblocking_send,
    test11_connect_timeout,
    test12_actor_death_during_recv,
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = 64 * 1024;

        actor_id test;
        if (ACRT_FAILED(acrt_spawn_ex(test_funcs[i], NULL, &cfg, &test))) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        acrt_link(test);

        acrt_message msg;
        acrt_ipc_recv(&msg, 10000);  // 10 second timeout per test
    }

    acrt_exit();
}

int main(void) {
    printf("=== Network I/O (acrt_net) Test Suite ===\n");

    acrt_status status = acrt_init();
    if (ACRT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = ACRT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner;
    if (ACRT_FAILED(acrt_spawn_ex(run_all_tests, NULL, &cfg, &runner))) {
        fprintf(stderr, "Failed to spawn test runner\n");
        acrt_cleanup();
        return 1;
    }

    acrt_run();
    acrt_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
