/**
 * Pool Exhaustion Example
 *
 * This example demonstrates handling pool exhaustion in the Hive actor runtime.
 * It shows both approaches:
 * 1. Default behavior: HIVE_ERR_NOMEM returned (caller handles retry/backoff)
 * 2. Blocking behavior: pool_block=true (notify operations yield until pool space available)
 *
 * The example spawns a fast sender and slow receiver to intentionally exhaust
 * the message pool, then demonstrates both handling strategies.
 */

#include "hive_runtime.h"
#include "hive_ipc.h"
#include "hive_static_config.h"
#include <stdio.h>
#include <stdbool.h>

// Counters for tracking progress
static int messages_sent_nonblocking = 0;
static int messages_sent_blocking = 0;
static int messages_received = 0;

// =============================================================================
// Demo 1: Default behavior (pool_block=false) - returns HIVE_ERR_NOMEM
// =============================================================================

static void slow_receiver(void *args, const hive_spawn_info_t *siblings,
                          size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Slow receiver started (ID: %u)\n", hive_self());

    // Process messages slowly, allowing pool to fill up
    // Note: We don't use timers here because the pool may be exhausted
    // and timer messages can't be delivered. Instead, we just yield.
    int empty_count = 0;
    while (empty_count < 10) {
        hive_message_t msg;
        hive_status_t s = hive_ipc_recv(&msg, 0); // Non-blocking
        if (HIVE_SUCCEEDED(s)) {
            messages_received++;
            empty_count = 0;
            // Simulate slow processing by yielding multiple times
            for (int i = 0; i < 5; i++) {
                hive_yield();
            }
        } else {
            empty_count++;
            hive_yield();
        }
    }

    printf("Slow receiver processed %d messages\n", messages_received);
    return;
}

static void nonblocking_sender(void *args, const hive_spawn_info_t *siblings,
                               size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t receiver = *(hive_actor_id_t *)args;

    printf("Non-blocking sender started (ID: %u)\n", hive_self());
    printf("Demonstrating default behavior: HIVE_ERR_NOMEM on pool "
           "exhaustion\n\n");

    int retries = 0;
    int max_retries = 3;

    // Notify messages until pool exhausted, then demonstrate retry pattern
    for (int i = 0; i < 300; i++) {
        int data = i;
        hive_status_t s =
            hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));
        if (HIVE_FAILED(s)) {
            if (s.code == HIVE_ERR_NOMEM) {
                printf("  [%d] Pool exhausted! Implementing backoff-retry...\n",
                       i);

                // Backoff-retry pattern: wait for incoming messages, then retry
                bool sent = false;
                for (int r = 0; r < max_retries && !sent; r++) {
                    retries++;
                    hive_message_t msg;
                    hive_status_t recv_s =
                        hive_ipc_recv(&msg, 50); // 50ms backoff
                    if (HIVE_SUCCEEDED(recv_s)) {
                        printf("  [%d] Received message during backoff\n", i);
                    }

                    // Retry notify
                    s = hive_ipc_notify(receiver, HIVE_TAG_NONE, &data,
                                        sizeof(data));
                    if (HIVE_SUCCEEDED(s)) {
                        messages_sent_nonblocking++;
                        sent = true;
                    } else {
                        printf("  [%d] Retry %d failed\n", i, r + 1);
                    }
                }

                if (!sent) {
                    printf("  [%d] Giving up after %d retries\n", i,
                           max_retries);
                    break;
                }
            } else {
                printf("  [%d] Unexpected error: %s\n", i, HIVE_ERR_STR(s));
                break;
            }
        } else {
            messages_sent_nonblocking++;
        }
    }

    printf("\nNon-blocking sender: sent %d messages with %d retries\n",
           messages_sent_nonblocking, retries);
    return;
}

// =============================================================================
// Demo 2: Blocking behavior (pool_block=true) - yields until pool available
// =============================================================================

static void consumer(void *args, const hive_spawn_info_t *siblings,
                     size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Consumer started (ID: %u)\n", hive_self());

    // Continuously consume messages to free pool space
    int consumed = 0;
    int empty_count = 0;

    while (empty_count < 5) {
        hive_message_t msg;
        hive_status_t s = hive_ipc_recv(&msg, 0); // Non-blocking
        if (HIVE_SUCCEEDED(s)) {
            consumed++;
            empty_count = 0;
        } else {
            empty_count++;
        }
        hive_yield(); // Let sender run
    }

    printf("Consumer: consumed %d messages\n", consumed);
    return;
}

static void blocking_sender(void *args, const hive_spawn_info_t *siblings,
                            size_t sibling_count) {
    (void)siblings;
    (void)sibling_count;
    hive_actor_id_t receiver = *(hive_actor_id_t *)args;

    printf("Blocking sender started (ID: %u)\n", hive_self());
    printf("Demonstrating pool_block=true: yields until pool available\n");
    printf("Current pool_block setting: %s\n\n",
           hive_pool_get_block() ? "true" : "false");

    // Notify many messages - will block when pool exhausted, resume when space
    // available
    int target = HIVE_MESSAGE_DATA_POOL_SIZE + 50; // More than pool size

    for (int i = 0; i < target; i++) {
        int data = i;
        hive_status_t s =
            hive_ipc_notify(receiver, HIVE_TAG_NONE, &data, sizeof(data));
        if (HIVE_FAILED(s)) {
            // Should not happen with pool_block=true (unless other error)
            printf("  Unexpected error: %s\n", HIVE_ERR_STR(s));
            break;
        }
        messages_sent_blocking++;

        // Progress indicator
        if (messages_sent_blocking % 100 == 0) {
            printf("  Sent %d messages...\n", messages_sent_blocking);
        }
    }

    printf("\nBlocking sender: sent %d messages (no retries needed!)\n",
           messages_sent_blocking);
    return;
}

// =============================================================================
// Demo 3: Runtime API to change pool_block setting
// =============================================================================

static void runtime_api_demo(void *args, const hive_spawn_info_t *siblings,
                             size_t sibling_count) {
    (void)args;
    (void)siblings;
    (void)sibling_count;

    printf("Runtime API demo started\n");

    // Check initial setting (should be false from default config)
    printf("  Initial pool_block: %s\n",
           hive_pool_get_block() ? "true" : "false");

    // Enable blocking
    hive_pool_set_block(HIVE_POOL_BLOCK);
    printf("  After HIVE_POOL_BLOCK: %s\n",
           hive_pool_get_block() ? "true" : "false");

    // Disable blocking
    hive_pool_set_block(HIVE_POOL_NO_BLOCK);
    printf("  After HIVE_POOL_NO_BLOCK: %s\n",
           hive_pool_get_block() ? "true" : "false");

    // Restore default
    hive_pool_set_block(HIVE_POOL_BLOCK);
    hive_pool_set_block(HIVE_POOL_DEFAULT);
    printf("  After HIVE_POOL_DEFAULT: %s (restored to spawn default)\n",
           hive_pool_get_block() ? "true" : "false");

    printf("Runtime API demo complete\n");
    return;
}

// =============================================================================
// Main
// =============================================================================

int main(void) {
    printf("=== Pool Exhaustion Handling Example ===\n\n");
    printf("Pool sizes: MAILBOX_ENTRY=%d, MESSAGE_DATA=%d\n\n",
           HIVE_MAILBOX_ENTRY_POOL_SIZE, HIVE_MESSAGE_DATA_POOL_SIZE);

    // Demo 1: Default non-blocking behavior
    printf("--- Demo 1: Default behavior (pool_block=false) ---\n\n");
    hive_init();
    {
        hive_actor_id_t receiver;
        hive_spawn(slow_receiver, NULL, NULL, NULL, &receiver);

        hive_actor_id_t sender;
        hive_spawn(nonblocking_sender, NULL, &receiver, NULL, &sender);

        hive_run();
    }
    hive_cleanup();

    // Reset counters
    messages_received = 0;

    printf("\n--- Demo 2: Blocking behavior (pool_block=true) ---\n\n");
    hive_init();
    {
        hive_actor_id_t receiver;
        hive_spawn(consumer, NULL, NULL, NULL, &receiver);

        // Spawn sender with pool_block=true
        hive_actor_config_t cfg = HIVE_ACTOR_CONFIG_DEFAULT;
        cfg.pool_block = true;

        hive_actor_id_t sender;
        hive_spawn(blocking_sender, NULL, &receiver, &cfg, &sender);

        hive_run();
    }
    hive_cleanup();

    printf("\n--- Demo 3: Runtime API ---\n\n");
    hive_init();
    {
        hive_actor_id_t demo;
        hive_spawn(runtime_api_demo, NULL, NULL, NULL, &demo);
        hive_run();
    }
    hive_cleanup();

    printf("\n=== Example completed ===\n");
    printf("Summary:\n");
    printf(
        "  Non-blocking approach: Sent %d messages with explicit retry logic\n",
        messages_sent_nonblocking);
    printf("  Blocking approach: Sent %d messages with automatic yield\n",
           messages_sent_blocking);
    printf("\nSee hive_ipc(3) and spec/design.md for more details.\n");

    return 0;
}
