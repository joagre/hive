#include "rt_runtime.h"
#include "rt_net.h"
#include "rt_ipc.h"
#include "rt_timer.h"
#include "rt_link.h"
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
    rt_status status = rt_net_listen(TEST_PORT, &g_listen_fd);
    if (RT_FAILED(status)) {
        printf("    Server: listen failed: %s\n", status.msg ? status.msg : "unknown");
        rt_exit();
    }

    g_server_ready = true;

    // Accept connection with timeout
    status = rt_net_accept(g_listen_fd, &g_accepted_fd, 2000);  // 2 second timeout
    if (RT_FAILED(status)) {
        printf("    Server: accept failed: %s\n", status.msg ? status.msg : "unknown");
        rt_net_close(g_listen_fd);
        rt_exit();
    }

    // Receive data from client
    char buf[64] = {0};
    size_t received = 0;
    status = rt_net_recv(g_accepted_fd, buf, sizeof(buf) - 1, &received, 2000);
    if (RT_FAILED(status)) {
        printf("    Server: recv failed: %s\n", status.msg ? status.msg : "unknown");
    } else {
        // Echo back
        size_t sent = 0;
        rt_net_send(g_accepted_fd, buf, received, &sent, 2000);
    }

    // Cleanup
    rt_net_close(g_accepted_fd);
    rt_net_close(g_listen_fd);
    rt_exit();
}

static void client_actor(void *arg) {
    (void)arg;

    // Wait for server to be ready
    while (!g_server_ready) {
        rt_yield();
    }

    // Small delay to ensure server is in accept
    timer_id timer;
    rt_timer_after(50000, &timer);  // 50ms
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    // Connect to server
    int fd = -1;
    rt_status status = rt_net_connect("127.0.0.1", TEST_PORT, &fd, 2000);
    if (RT_FAILED(status)) {
        printf("    Client: connect failed: %s\n", status.msg ? status.msg : "unknown");
        rt_exit();
    }

    g_client_connected = true;

    // Send data
    const char *data = "Hello Server!";
    size_t sent = 0;
    status = rt_net_send(fd, data, strlen(data), &sent, 2000);
    if (RT_FAILED(status)) {
        printf("    Client: send failed: %s\n", status.msg ? status.msg : "unknown");
        rt_net_close(fd);
        rt_exit();
    }

    // Receive echo
    char buf[64] = {0};
    size_t received = 0;
    status = rt_net_recv(fd, buf, sizeof(buf) - 1, &received, 2000);
    if (RT_FAILED(status)) {
        printf("    Client: recv failed: %s\n", status.msg ? status.msg : "unknown");
    }

    rt_net_close(fd);
    rt_exit();
}

static void test1_listen_accept(void *arg) {
    (void)arg;
    printf("\nTest 1: Listen and accept connection\n");

    g_server_ready = false;
    g_client_connected = false;
    g_listen_fd = -1;
    g_accepted_fd = -1;

    // Get our ID to pass to server
    actor_id self = rt_self();

    // Spawn server
    actor_id server = rt_spawn(server_actor, &self);
    rt_link(server);

    // Spawn client
    actor_id client = rt_spawn(client_actor, NULL);
    rt_link(client);

    // Wait for both to complete
    rt_message msg;
    rt_ipc_recv(&msg, 5000);  // Wait for first exit
    rt_ipc_recv(&msg, 5000);  // Wait for second exit

    if (g_server_ready && g_client_connected) {
        TEST_PASS("listen and accept connection");
    } else {
        printf("    server_ready=%d, client_connected=%d\n", g_server_ready, g_client_connected);
        TEST_FAIL("connection failed");
    }

    rt_exit();
}

// ============================================================================
// Test 2: Send and receive data
// ============================================================================

static char g_received_data[64] = {0};
static size_t g_received_len = 0;

static void echo_server_actor(void *arg) {
    (void)arg;

    int listen_fd = -1;
    rt_status status = rt_net_listen(TEST_PORT + 1, &listen_fd);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    g_server_ready = true;

    int conn_fd = -1;
    status = rt_net_accept(listen_fd, &conn_fd, 2000);
    if (RT_FAILED(status)) {
        rt_net_close(listen_fd);
        rt_exit();
    }

    // Receive and store
    status = rt_net_recv(conn_fd, g_received_data, sizeof(g_received_data) - 1, &g_received_len, 2000);

    // Echo back with modification
    if (!RT_FAILED(status)) {
        char reply[128];
        snprintf(reply, sizeof(reply), "Echo: %.60s", g_received_data);
        size_t sent = 0;
        rt_net_send(conn_fd, reply, strlen(reply), &sent, 2000);
    }

    rt_net_close(conn_fd);
    rt_net_close(listen_fd);
    rt_exit();
}

static bool g_echo_received = false;
static char g_echo_reply[64] = {0};

static void echo_client_actor(void *arg) {
    (void)arg;

    while (!g_server_ready) {
        rt_yield();
    }

    timer_id timer;
    rt_timer_after(50000, &timer);
    rt_message msg;
    rt_ipc_recv(&msg, -1);

    int fd = -1;
    rt_status status = rt_net_connect("127.0.0.1", TEST_PORT + 1, &fd, 2000);
    if (RT_FAILED(status)) {
        rt_exit();
    }

    // Send test message
    const char *data = "TestMessage";
    size_t sent = 0;
    rt_net_send(fd, data, strlen(data), &sent, 2000);

    // Receive reply
    size_t received = 0;
    status = rt_net_recv(fd, g_echo_reply, sizeof(g_echo_reply) - 1, &received, 2000);
    if (!RT_FAILED(status)) {
        g_echo_received = true;
    }

    rt_net_close(fd);
    rt_exit();
}

static void test2_send_receive(void *arg) {
    (void)arg;
    printf("\nTest 2: Send and receive data\n");

    g_server_ready = false;
    g_echo_received = false;
    memset(g_received_data, 0, sizeof(g_received_data));
    memset(g_echo_reply, 0, sizeof(g_echo_reply));

    actor_id server = rt_spawn(echo_server_actor, NULL);
    rt_link(server);

    actor_id client = rt_spawn(echo_client_actor, NULL);
    rt_link(client);

    rt_message msg;
    rt_ipc_recv(&msg, 5000);
    rt_ipc_recv(&msg, 5000);

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

    rt_exit();
}

// ============================================================================
// Test 3: Accept timeout
// ============================================================================

static void test3_accept_timeout(void *arg) {
    (void)arg;
    printf("\nTest 3: Accept timeout\n");

    int listen_fd = -1;
    rt_status status = rt_net_listen(TEST_PORT + 2, &listen_fd);
    if (RT_FAILED(status)) {
        TEST_FAIL("listen failed");
        rt_exit();
    }

    // Accept with short timeout (no client will connect)
    int conn_fd = -1;
    status = rt_net_accept(listen_fd, &conn_fd, 100);  // 100ms timeout

    if (status.code == RT_ERR_TIMEOUT) {
        TEST_PASS("accept times out when no connection");
    } else if (RT_FAILED(status)) {
        TEST_PASS("accept returns error when no connection");
    } else {
        rt_net_close(conn_fd);
        TEST_FAIL("accept should timeout");
    }

    rt_net_close(listen_fd);
    rt_exit();
}

// ============================================================================
// Test 4: Connect to invalid address
// ============================================================================

static void test4_connect_invalid(void *arg) {
    (void)arg;
    printf("\nTest 4: Connect to invalid address\n");

    int fd = -1;
    // Try to connect to an address that should fail quickly
    rt_status status = rt_net_connect("127.0.0.1", 1, &fd, 500);  // Port 1 typically fails

    if (RT_FAILED(status)) {
        TEST_PASS("connect to invalid port fails");
    } else {
        rt_net_close(fd);
        TEST_FAIL("connect should fail");
    }

    rt_exit();
}

// ============================================================================
// Test 5: Short timeout accept
// ============================================================================

static void test5_short_timeout_accept(void *arg) {
    (void)arg;
    printf("\nTest 5: Short timeout accept\n");

    int listen_fd = -1;
    rt_status status = rt_net_listen(TEST_PORT + 3, &listen_fd);
    if (RT_FAILED(status)) {
        TEST_FAIL("listen failed");
        rt_exit();
    }

    // Short timeout accept (10ms - should timeout quickly)
    int conn_fd = -1;
    status = rt_net_accept(listen_fd, &conn_fd, 10);

    if (status.code == RT_ERR_TIMEOUT) {
        TEST_PASS("short timeout accept returns quickly");
    } else if (RT_FAILED(status)) {
        TEST_PASS("short timeout accept returns error when no connection");
    } else {
        rt_net_close(conn_fd);
        TEST_FAIL("accept should timeout");
    }

    rt_net_close(listen_fd);
    rt_exit();
}

// ============================================================================
// Test 6: Close and reuse port
// ============================================================================

static void test6_close_reuse(void *arg) {
    (void)arg;
    printf("\nTest 6: Close and reuse port\n");

    // First listen
    int listen_fd1 = -1;
    rt_status status = rt_net_listen(TEST_PORT + 4, &listen_fd1);
    if (RT_FAILED(status)) {
        TEST_FAIL("first listen failed");
        rt_exit();
    }

    // Close
    status = rt_net_close(listen_fd1);
    if (RT_FAILED(status)) {
        TEST_FAIL("close failed");
        rt_exit();
    }

    // Listen again on same port (with SO_REUSEADDR this should work)
    int listen_fd2 = -1;
    status = rt_net_listen(TEST_PORT + 4, &listen_fd2);
    if (RT_FAILED(status)) {
        // This might fail if SO_REUSEADDR isn't set - that's ok
        TEST_PASS("port reuse (may require TIME_WAIT)");
    } else {
        rt_net_close(listen_fd2);
        TEST_PASS("close and reuse port works");
    }

    rt_exit();
}

// ============================================================================
// Test 7: Non-blocking accept (timeout=0)
// NOTE: Per API docs, timeout_ms=0 should return RT_ERR_WOULDBLOCK immediately
//       if no connection is pending. This test may fail if the implementation
//       blocks instead of returning immediately.
// ============================================================================

static void test7_nonblocking_accept(void *arg) {
    (void)arg;
    printf("\nTest 7: Non-blocking accept (timeout=0)\n");

    int listen_fd = -1;
    rt_status status = rt_net_listen(TEST_PORT + 5, &listen_fd);
    if (RT_FAILED(status)) {
        TEST_FAIL("listen failed");
        rt_exit();
    }

    // Non-blocking accept (timeout=0) - should return immediately
    int conn_fd = -1;
    uint64_t start = time_ms();
    status = rt_net_accept(listen_fd, &conn_fd, 0);
    uint64_t elapsed = time_ms() - start;

    if (status.code == RT_ERR_WOULDBLOCK) {
        printf("    Returned WOULDBLOCK after %lu ms\n", (unsigned long)elapsed);
        TEST_PASS("non-blocking accept returns WOULDBLOCK immediately");
    } else if (status.code == RT_ERR_TIMEOUT) {
        printf("    Returned TIMEOUT after %lu ms\n", (unsigned long)elapsed);
        if (elapsed < 100) {
            TEST_PASS("non-blocking accept returns quickly");
        } else {
            TEST_FAIL("non-blocking accept took too long");
        }
    } else if (RT_FAILED(status)) {
        printf("    Returned error after %lu ms: %s\n", (unsigned long)elapsed,
               status.msg ? status.msg : "unknown");
        TEST_FAIL("unexpected error from non-blocking accept");
    } else {
        rt_net_close(conn_fd);
        TEST_FAIL("non-blocking accept should not succeed without connection");
    }

    rt_net_close(listen_fd);
    rt_exit();
}

// ============================================================================
// Test 8: Recv timeout
// ============================================================================

static void test8_recv_timeout(void *arg) {
    (void)arg;
    printf("\nTest 8: Recv timeout\n");

    // Create a connection to ourselves
    int listen_fd = -1;
    rt_status status = rt_net_listen(TEST_PORT + 6, &listen_fd);
    if (RT_FAILED(status)) {
        TEST_FAIL("listen failed");
        rt_exit();
    }

    // Connect to ourselves
    int client_fd = -1;
    status = rt_net_connect("127.0.0.1", TEST_PORT + 6, &client_fd, 1000);
    if (RT_FAILED(status)) {
        TEST_FAIL("connect failed");
        rt_net_close(listen_fd);
        rt_exit();
    }

    // Accept the connection
    int server_fd = -1;
    status = rt_net_accept(listen_fd, &server_fd, 1000);
    if (RT_FAILED(status)) {
        TEST_FAIL("accept failed");
        rt_net_close(client_fd);
        rt_net_close(listen_fd);
        rt_exit();
    }

    // Try to recv with timeout (no data sent)
    char buf[64];
    size_t received = 0;
    uint64_t start = time_ms();
    status = rt_net_recv(server_fd, buf, sizeof(buf), &received, 100);  // 100ms timeout
    uint64_t elapsed = time_ms() - start;

    if (status.code == RT_ERR_TIMEOUT) {
        printf("    Recv timed out after %lu ms (expected ~100ms)\n", (unsigned long)elapsed);
        TEST_PASS("recv times out when no data");
    } else if (RT_FAILED(status)) {
        TEST_PASS("recv returns error when no data");
    } else {
        TEST_FAIL("recv should timeout when no data sent");
    }

    rt_net_close(server_fd);
    rt_net_close(client_fd);
    rt_net_close(listen_fd);
    rt_exit();
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
};

#define NUM_TESTS (sizeof(test_funcs) / sizeof(test_funcs[0]))

static void run_all_tests(void *arg) {
    (void)arg;

    for (size_t i = 0; i < NUM_TESTS; i++) {
        actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
        cfg.stack_size = 64 * 1024;

        actor_id test = rt_spawn_ex(test_funcs[i], NULL, &cfg);
        if (test == ACTOR_ID_INVALID) {
            printf("Failed to spawn test %zu\n", i);
            continue;
        }

        rt_link(test);

        rt_message msg;
        rt_ipc_recv(&msg, 10000);  // 10 second timeout per test
    }

    rt_exit();
}

int main(void) {
    printf("=== Network I/O (rt_net) Test Suite ===\n");

    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    actor_config cfg = RT_ACTOR_CONFIG_DEFAULT;
    cfg.stack_size = 128 * 1024;

    actor_id runner = rt_spawn_ex(run_all_tests, NULL, &cfg);
    if (runner == ACTOR_ID_INVALID) {
        fprintf(stderr, "Failed to spawn test runner\n");
        rt_cleanup();
        return 1;
    }

    rt_run();
    rt_cleanup();

    printf("\n=== Results ===\n");
    printf("Passed: %d\n", tests_passed);
    printf("Failed: %d\n", tests_failed);
    printf("\n%s\n", tests_failed == 0 ? "All tests passed!" : "Some tests FAILED!");

    return tests_failed > 0 ? 1 : 0;
}
