// Stack profiling implementation
//
// All stack measurement code is contained here.
// Compiles to no-op functions when HIVE_STACK_WATERMARK=0.

#include "stack_profile.h"
#include "hive_static_config.h"

#if HIVE_STACK_WATERMARK

#include "hive_actor.h"
#include "hive_log.h"
#include "hive_runtime.h"
#include <string.h>

#ifdef HIVE_PLATFORM_STM32
// On STM32, use HIVE_LOG for output
#define PROFILE_PRINT(...) HIVE_LOG_INFO(__VA_ARGS__)
#else
// On Linux/Webots, use stderr
#include <stdio.h>
#define PROFILE_PRINT(...) fprintf(stderr, __VA_ARGS__)
#endif

// Storage for actors that exit before report (e.g., flight_manager)
#define MAX_CAPTURED 4
static struct {
    const char *name;
    size_t stack_size;
    size_t used;
} s_captured[MAX_CAPTURED];
static size_t s_captured_count = 0;

static volatile bool s_report_requested = false;

static void print_actor_stack(hive_actor_id_t id, const char *name,
                              size_t stack_size, size_t used) {
    (void)id;
    const char *n = name ? name : "(unnamed)";
    float pct =
        stack_size > 0 ? 100.0f * (float)used / (float)stack_size : 0.0f;
    PROFILE_PRINT("%-15s %8zu %8zu %5.1f%%\n", n, stack_size, used, pct);
}

void stack_profile_capture(const char *name) {
    if (s_captured_count >= MAX_CAPTURED) {
        return;
    }
    hive_actor_id_t self = hive_self();
    size_t used = hive_actor_stack_usage(self);

    // Get stack size from actor (need to access it before actor exits)
    actor *a = hive_actor_get(self);
    size_t stack_size = a ? a->stack_size : 0;

    s_captured[s_captured_count].name = name;
    s_captured[s_captured_count].stack_size = stack_size;
    s_captured[s_captured_count].used = used;
    s_captured_count++;
}

void stack_profile_request(void) {
    s_report_requested = true;
}

bool stack_profile_check(void) {
    if (!s_report_requested) {
        return false;
    }

    PROFILE_PRINT("\n=== Stack Usage Report ===\n");
    PROFILE_PRINT("%-15s %8s %8s %6s\n", "Actor", "Size", "Used", "Usage");
    PROFILE_PRINT("%-15s %8s %8s %6s\n", "---------------", "--------",
                  "--------", "------");

    // Print live actors
    hive_actor_stack_usage_all(print_actor_stack);

    // Print captured (exited) actors
    for (size_t i = 0; i < s_captured_count; i++) {
        print_actor_stack(0, s_captured[i].name, s_captured[i].stack_size,
                          s_captured[i].used);
    }

    PROFILE_PRINT("==========================\n\n");

    s_report_requested = false;
    return true;
}

#else

void stack_profile_capture(const char *name) {
    (void)name;
}

void stack_profile_request(void) {
}

bool stack_profile_check(void) {
    return false;
}

#endif
