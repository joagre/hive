// Flight profiles - Waypoint definitions for different flight modes
//
// FLIGHT_PROFILE is set in config.h

#ifndef FLIGHT_PROFILES_H
#define FLIGHT_PROFILES_H

#include "config.h"
#include "math_utils.h"

// Waypoint definition
typedef struct {
    float x, y, z; // Position (meters, world frame)
    float yaw;     // Heading (radians)
} waypoint_t;

#if FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
// First flight test: hover at low altitude until flight manager initiates
// landing. Safe profile for initial hardware validation (tethered recommended)
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.5f, 0.0f}, // Hover at 0.5m
};
#define WAYPOINT_HOVER_TIME_US (6 * 1000000) // 6 seconds hover
#define FLIGHT_PROFILE_NAME "FIRST_TEST"

#elif FLIGHT_PROFILE == FLIGHT_PROFILE_ALTITUDE
// Altitude-only waypoints (no GPS, x/y fixed at origin)
// Position actor sees zero error, drone hovers in place
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.5f, 0.0f}, // 0.5m - start low
    {0.0f, 0.0f, 1.0f, 0.0f}, // 1.0m
    {0.0f, 0.0f, 1.5f, 0.0f}, // 1.5m - max height
    {0.0f, 0.0f, 1.0f, 0.0f}, // 1.0m - descend
};
#define WAYPOINT_HOVER_TIME_US (5 * 1000000) // 5 seconds hover
#define FLIGHT_PROFILE_NAME "ALTITUDE"

#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FULL_3D
// Full 3D waypoint navigation demo
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 1.0f, 0.0f}, // Start: origin, 1.0m
    {1.0f, 0.0f, 1.2f, 0.0f}, // Waypoint 1: +X, rise to 1.2m
    {1.0f, 1.0f, 1.4f,
     M_PI_F / 2.0f},            // Waypoint 2: corner, rise to 1.4m, face east
    {0.0f, 1.0f, 1.2f, M_PI_F}, // Waypoint 3: -X, drop to 1.2m, face south
    {0.0f, 0.0f, 1.0f, 0.0f},   // Return: origin, 1.0m, face north
};
#define WAYPOINT_HOVER_TIME_US (2 * 1000000) // 2 seconds hover
#define FLIGHT_PROFILE_NAME "FULL_3D"

#else
#error "Unknown FLIGHT_PROFILE"
#endif

#define NUM_WAYPOINTS (sizeof(waypoints) / sizeof(waypoints[0]))

#endif // FLIGHT_PROFILES_H
