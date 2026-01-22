// Stack profiling for pilot example
//
// Reports actual stack usage for all actors when HIVE_STACK_WATERMARK=1.
// Build with: make STACK_PROFILE=1
//
// Usage:
//   - Actor calls stack_profile_capture() before exiting to save its usage
//   - Actor calls stack_profile_request() when ready (e.g., flight complete)
//   - Main loop calls stack_profile_check() to print report if requested

#ifndef STACK_PROFILE_H
#define STACK_PROFILE_H

#include <stdbool.h>

// Capture current actor's stack usage before it exits (stores for later report)
void stack_profile_capture(const char *name);

// Request a stack profile report (called by actor when flight complete)
void stack_profile_request(void);

// Check if report requested, print it and return true (for main loop to break)
// Returns false if no report requested or watermarking disabled
bool stack_profile_check(void);

#endif // STACK_PROFILE_H
