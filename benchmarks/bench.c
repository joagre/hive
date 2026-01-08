#include "rt_runtime.h"
#include "rt_ipc.h"
#include "rt_pool.h"
#include "rt_bus.h"
#include "rt_static_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

// Timing utilities
#define BILLION 1000000000UL

static inline uint64_t get_nanos(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * BILLION + ts.tv_nsec;
}

// Benchmark parameters
#define ITERATIONS 10000
#define WARMUP_ITERATIONS 100

// ============================================================================
// 1. Context Switch Benchmark
// ============================================================================

typedef struct {
    actor_id partner;
    uint64_t count;
    uint64_t max_count;
    uint64_t start_time;
    uint64_t end_time;
} switch_ctx;

static void switch_actor_a(void *arg) {
    switch_ctx *ctx = (switch_ctx *)arg;

    while (ctx->count < ctx->max_count) {
        // Send ping to B
        int msg = 1;
        rt_ipc_send(ctx->partner, &msg, sizeof(msg));

        // Wait for pong from B
        rt_message reply;
        rt_ipc_recv(&reply, -1);

        ctx->count++;
    }

    ctx->end_time = get_nanos();
    rt_exit();
}

static void switch_actor_b(void *arg) {
    switch_ctx *ctx = (switch_ctx *)arg;

    while (ctx->count < ctx->max_count) {
        // Wait for ping from A
        rt_message msg;
        rt_ipc_recv(&msg, -1);

        // Send pong back to A
        int reply = 2;
        rt_ipc_send(ctx->partner, &reply, sizeof(reply));

        ctx->count++;
    }

    rt_exit();
}

static void bench_context_switch(void) {
    printf("Context Switch Benchmark\n");
    printf("-------------------------\n");

    // Warmup
    switch_ctx *ctx_a_warmup = calloc(1, sizeof(switch_ctx));
    switch_ctx *ctx_b_warmup = calloc(1, sizeof(switch_ctx));

    ctx_a_warmup->max_count = WARMUP_ITERATIONS;
    ctx_b_warmup->max_count = WARMUP_ITERATIONS;

    actor_id b = rt_spawn(switch_actor_b, ctx_b_warmup);
    actor_id a = rt_spawn(switch_actor_a, ctx_a_warmup);
    ctx_a_warmup->partner = b;
    ctx_b_warmup->partner = a;

    rt_run();

    free(ctx_a_warmup);
    free(ctx_b_warmup);

    // Actual benchmark
    switch_ctx *ctx_a = calloc(1, sizeof(switch_ctx));
    switch_ctx *ctx_b = calloc(1, sizeof(switch_ctx));

    ctx_a->max_count = ITERATIONS;
    ctx_b->max_count = ITERATIONS;

    ctx_a->start_time = get_nanos();

    b = rt_spawn(switch_actor_b, ctx_b);
    a = rt_spawn(switch_actor_a, ctx_a);
    ctx_a->partner = b;
    ctx_b->partner = a;

    rt_run();

    uint64_t elapsed = ctx_a->end_time - ctx_a->start_time;
    uint64_t total_switches = ITERATIONS * 2; // A->B and B->A per iteration
    uint64_t ns_per_switch = elapsed / total_switches;
    double switches_per_sec = (double)total_switches / ((double)elapsed / BILLION);

    printf("  Iterations:           %d round-trips\n", ITERATIONS);
    printf("  Total switches:       %lu\n", total_switches);
    printf("  Total time:           %lu ns (%.3f ms)\n", elapsed, elapsed / 1000000.0);
    printf("  Latency per switch:   %lu ns\n", ns_per_switch);
    printf("  Throughput:           %.2f M switches/sec\n", switches_per_sec / 1000000.0);
    printf("\n");

    free(ctx_a);
    free(ctx_b);
}

// ============================================================================
// 2. IPC Performance Benchmark
// ============================================================================

typedef struct {
    actor_id partner;
    uint64_t count;
    uint64_t max_count;
    size_t msg_size;
    uint64_t start_time;
    uint64_t end_time;
} ipc_ctx;

static void ipc_sender(void *arg) {
    ipc_ctx *ctx = (ipc_ctx *)arg;
    uint8_t buffer[256];
    memset(buffer, 0xAA, sizeof(buffer));

    ctx->start_time = get_nanos();

    for (uint64_t i = 0; i < ctx->max_count; i++) {
        rt_ipc_send(ctx->partner, buffer, ctx->msg_size);

        // Wait for ack
        rt_message ack;
        rt_ipc_recv(&ack, -1);
    }

    ctx->end_time = get_nanos();
    rt_exit();
}

static void ipc_receiver(void *arg) {
    ipc_ctx *ctx = (ipc_ctx *)arg;
    uint8_t ack = 1;

    for (uint64_t i = 0; i < ctx->max_count; i++) {
        rt_message msg;
        rt_ipc_recv(&msg, -1);

        // Send ack
        rt_ipc_send(ctx->partner, &ack, sizeof(ack));
    }

    rt_exit();
}

static void bench_ipc_copy(size_t msg_size, const char *label) {
    // Warmup
    ipc_ctx *ctx_send_warmup = calloc(1, sizeof(ipc_ctx));
    ipc_ctx *ctx_recv_warmup = calloc(1, sizeof(ipc_ctx));

    ctx_send_warmup->max_count = WARMUP_ITERATIONS;
    ctx_send_warmup->msg_size = msg_size;
    ctx_recv_warmup->max_count = WARMUP_ITERATIONS;
    ctx_recv_warmup->msg_size = msg_size;

    actor_id recv = rt_spawn(ipc_receiver, ctx_recv_warmup);
    actor_id send = rt_spawn(ipc_sender, ctx_send_warmup);
    ctx_send_warmup->partner = recv;
    ctx_recv_warmup->partner = send;

    rt_run();

    free(ctx_send_warmup);
    free(ctx_recv_warmup);

    // Actual benchmark
    ipc_ctx *ctx_send = calloc(1, sizeof(ipc_ctx));
    ipc_ctx *ctx_recv = calloc(1, sizeof(ipc_ctx));

    ctx_send->max_count = ITERATIONS;
    ctx_send->msg_size = msg_size;
    ctx_recv->max_count = ITERATIONS;
    ctx_recv->msg_size = msg_size;

    recv = rt_spawn(ipc_receiver, ctx_recv);
    send = rt_spawn(ipc_sender, ctx_send);
    ctx_send->partner = recv;
    ctx_recv->partner = send;

    rt_run();

    uint64_t elapsed = ctx_send->end_time - ctx_send->start_time;
    uint64_t ns_per_msg = elapsed / ITERATIONS;
    double msgs_per_sec = (double)ITERATIONS / ((double)elapsed / BILLION);

    printf("  %-20s %6lu ns/msg  (%.2f M msgs/sec)\n",
           label, ns_per_msg, msgs_per_sec / 1000000.0);

    free(ctx_send);
    free(ctx_recv);
}

static void bench_ipc(void) __attribute__((unused));
static void bench_ipc(void) {
    printf("IPC Performance (COPY mode)\n");
    printf("---------------------------\n");

    bench_ipc_copy(8, "8 bytes:");
    bench_ipc_copy(64, "64 bytes:");
    bench_ipc_copy(256, "256 bytes:");

    printf("\n");
}

// ============================================================================
// 3. Pool Allocation Benchmark
// ============================================================================

static void bench_pool_allocation(void) __attribute__((unused));
static void bench_pool_allocation(void) {
    printf("Pool Allocation Performance\n");
    printf("---------------------------\n");

    // Use more iterations for this micro-benchmark
    const int POOL_ITERATIONS = ITERATIONS * 100;  // 1,000,000 iterations

    // Create a test pool
    #define POOL_SIZE 1024
    static uint8_t pool_buffer[POOL_SIZE * 64];
    static bool pool_used[POOL_SIZE];
    rt_pool pool_mgr;

    rt_pool_init(&pool_mgr, pool_buffer, pool_used, 64, POOL_SIZE);

    // Warmup
    void *ptrs[100];
    for (int i = 0; i < 100; i++) {
        ptrs[i] = rt_pool_alloc(&pool_mgr);
    }
    for (int i = 0; i < 100; i++) {
        rt_pool_free(&pool_mgr, ptrs[i]);
    }

    // Benchmark allocation
    // Write to the allocated memory to prevent compiler from optimizing away the calls
    uint64_t start = get_nanos();
    volatile uint64_t pool_sum = 0;
    for (int i = 0; i < POOL_ITERATIONS; i++) {
        void *p = rt_pool_alloc(&pool_mgr);
        if (p) {
            *(uint64_t *)p = i;  // Write to force actual allocation
            pool_sum += *(uint64_t *)p;  // Read to prevent dead code elimination
            rt_pool_free(&pool_mgr, p);
        }
    }
    uint64_t elapsed = get_nanos() - start;
    (void)pool_sum;  // Prevent sum from being optimized away

    uint64_t ns_per_op = elapsed / POOL_ITERATIONS;
    double ops_per_sec = (double)POOL_ITERATIONS / ((double)elapsed / BILLION);

    printf("  Pool alloc+free:      %lu ns/op  (%.2f M ops/sec)  [elapsed: %lu ns]\n",
           ns_per_op, ops_per_sec / 1000000.0, elapsed);

    // Compare to malloc/free
    // Write to the allocated memory to prevent compiler from optimizing away the calls
    start = get_nanos();
    volatile uint64_t sum = 0;
    for (int i = 0; i < POOL_ITERATIONS; i++) {
        void *p = malloc(64);
        if (p) {
            *(uint64_t *)p = i;  // Write to force actual allocation
            sum += *(uint64_t *)p;  // Read to prevent dead code elimination
            free(p);
        }
    }
    elapsed = get_nanos() - start;
    (void)sum;  // Prevent sum from being optimized away

    uint64_t malloc_ns_per_op = elapsed / POOL_ITERATIONS;
    double malloc_ops_per_sec = (double)POOL_ITERATIONS / ((double)elapsed / BILLION);

    printf("  malloc+free (64B):    %lu ns/op  (%.2f M ops/sec)  [elapsed: %lu ns]\n",
           malloc_ns_per_op, malloc_ops_per_sec / 1000000.0, elapsed);
    printf("  Speedup:              %.1fx faster than malloc\n",
           (double)malloc_ns_per_op / ns_per_op);

    printf("\n");
}

// ============================================================================
// 4. Actor Spawn Benchmark
// ============================================================================

static void dummy_actor(void *arg) {
    (void)arg;
    rt_exit();
}

static void bench_actor_spawn(void) __attribute__((unused));
static void bench_actor_spawn(void) {
    printf("Actor Spawn Performance\n");
    printf("-----------------------\n");

    // Warmup
    for (int i = 0; i < 10; i++) {
        rt_spawn(dummy_actor, NULL);
    }
    rt_run();

    // Benchmark
    uint64_t start = get_nanos();
    for (int i = 0; i < 100; i++) {
        rt_spawn(dummy_actor, NULL);
    }
    rt_run();
    uint64_t elapsed = get_nanos() - start;

    uint64_t ns_per_spawn = elapsed / 100;
    double spawns_per_sec = 100.0 / ((double)elapsed / BILLION);

    printf("  Spawn time:           %lu ns/actor\n", ns_per_spawn);
    printf("  Throughput:           %.0f actors/sec\n", spawns_per_sec);
    printf("  Note: Includes stack allocation (arena)\n");

    printf("\n");
}

// ============================================================================
// 5. Bus Performance Benchmark
// ============================================================================

typedef struct {
    bus_id bus;
    uint64_t count;
    uint64_t max_count;
    uint64_t start_time;
    uint64_t end_time;
} bus_ctx;

static void bus_publisher(void *arg) {
    bus_ctx *ctx = (bus_ctx *)arg;
    uint8_t data[64];
    memset(data, 0xBB, sizeof(data));

    ctx->start_time = get_nanos();

    for (uint64_t i = 0; i < ctx->max_count; i++) {
        rt_bus_publish(ctx->bus, data, sizeof(data));

        // Yield periodically to let subscriber consume messages
        // This is realistic cooperative behavior
        if (i % 10 == 0) {
            rt_yield();
        }
    }

    ctx->end_time = get_nanos();
    rt_exit();
}

static void bus_subscriber(void *arg) {
    bus_ctx *ctx = (bus_ctx *)arg;

    rt_bus_subscribe(ctx->bus);

    uint8_t buffer[256];
    for (uint64_t i = 0; i < ctx->max_count; i++) {
        size_t len;
        rt_status status;
        // Wait for message to be available
        while (RT_FAILED(status = rt_bus_read(ctx->bus, buffer, sizeof(buffer), &len))) {
            rt_yield();
        }
    }

    rt_exit();
}

static void bench_bus(void) __attribute__((unused));
static void bench_bus(void) {
    printf("Bus Performance\n");
    printf("---------------\n");

    // Create bus with enough capacity for benchmark messages
    rt_bus_config cfg = {
        .max_entries = 64,  // Max allowed by RT_MAX_BUS_ENTRIES
        .max_entry_size = 256,  // Max size of each message
        .max_subscribers = 8,
        .max_readers = 1,  // Remove entries after 1 reader reads them
        .max_age_ms = 0
    };
    bus_id bus;
    rt_bus_create(&cfg, &bus);

    // Warmup
    bus_ctx *ctx_pub_warmup = calloc(1, sizeof(bus_ctx));
    bus_ctx *ctx_sub_warmup = calloc(1, sizeof(bus_ctx));

    ctx_pub_warmup->bus = bus;
    ctx_pub_warmup->max_count = 100;
    ctx_sub_warmup->bus = bus;
    ctx_sub_warmup->max_count = 100;

    rt_spawn(bus_subscriber, ctx_sub_warmup);
    rt_spawn(bus_publisher, ctx_pub_warmup);
    rt_run();

    free(ctx_pub_warmup);
    free(ctx_sub_warmup);

    // Benchmark publish latency
    // Use moderate iterations - publisher yields every 10 messages to cooperate
    const uint64_t BUS_ITERATIONS = 1000;

    bus_ctx *ctx_pub = calloc(1, sizeof(bus_ctx));
    bus_ctx *ctx_sub = calloc(1, sizeof(bus_ctx));

    ctx_pub->bus = bus;
    ctx_pub->max_count = BUS_ITERATIONS;
    ctx_sub->bus = bus;
    ctx_sub->max_count = BUS_ITERATIONS;

    rt_spawn(bus_subscriber, ctx_sub);
    rt_spawn(bus_publisher, ctx_pub);
    rt_run();

    uint64_t elapsed = ctx_pub->end_time - ctx_pub->start_time;
    uint64_t ns_per_pub = elapsed / BUS_ITERATIONS;
    double pubs_per_sec = (double)BUS_ITERATIONS / ((double)elapsed / BILLION);

    printf("  Publish latency:      %lu ns/msg\n", ns_per_pub);
    printf("  Throughput:           %.2f M msgs/sec\n", pubs_per_sec / 1000000.0);

    free(ctx_pub);
    free(ctx_sub);

    rt_bus_destroy(bus);
    printf("\n");
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    printf("\n");
    printf("=================================================\n");
    printf("  Actor Runtime Benchmark Suite\n");
    printf("=================================================\n");
    printf("\n");
    printf("Configuration:\n");
    printf("  RT_MAX_ACTORS:               %d\n", RT_MAX_ACTORS);
    printf("  RT_MAILBOX_ENTRY_POOL_SIZE:  %d\n", RT_MAILBOX_ENTRY_POOL_SIZE);
    printf("  RT_MESSAGE_DATA_POOL_SIZE:   %d\n", RT_MESSAGE_DATA_POOL_SIZE);
    printf("  RT_DEFAULT_STACK_SIZE:       %d\n", RT_DEFAULT_STACK_SIZE);
    printf("  Iterations:                  %d\n", ITERATIONS);
    printf("\n");

    printf("Initializing runtime...\n");
    fflush(stdout);

    // Initialize runtime
    rt_status status = rt_init();
    if (RT_FAILED(status)) {
        fprintf(stderr, "Failed to initialize runtime: %s\n",
                status.msg ? status.msg : "unknown error");
        return 1;
    }

    // Run benchmarks (all in single runtime session)
    printf("Starting context switch benchmark...\n");
    fflush(stdout);
    bench_context_switch();

    printf("Starting IPC benchmark...\n");
    fflush(stdout);
    bench_ipc();

    printf("Starting pool allocation benchmark...\n");
    fflush(stdout);
    bench_pool_allocation();

    printf("Starting actor spawn benchmark...\n");
    fflush(stdout);
    bench_actor_spawn();

    printf("Starting bus benchmark...\n");
    fflush(stdout);
    bench_bus();

    rt_cleanup();

    printf("=================================================\n");
    printf("  Benchmark Complete\n");
    printf("=================================================\n");
    printf("\n");

    return 0;
}
