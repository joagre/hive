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

#if FLIGHT_PROFILE == FLIGHT_PROFILE_GROUND_TEST
// Ground test: motors disabled, for testing ESB telemetry without flight.
// Drone sits on floor, control loops run, telemetry streams to ground station.
// Build with: make FLIGHT_PROFILE=FLIGHT_PROFILE_GROUND_TEST
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.0f, 0.0f}, // Stay on ground
};
#define WAYPOINT_HOVER_TIME_US (60 * 1000000) // 60 seconds
#define FLIGHT_PROFILE_NAME "GROUND_TEST"
#define MOTORS_DISABLED 1

#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FIRST_TEST
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
// Max altitude 1.2m to stay within flow deck range (VL53L1x: 1.3m)
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.5f, 0.0f}, // 0.5m - start low
    {0.0f, 0.0f, 0.8f, 0.0f}, // 0.8m
    {0.0f, 0.0f, 1.2f, 0.0f}, // 1.2m - max height (within flow deck range)
    {0.0f, 0.0f, 0.8f, 0.0f}, // 0.8m - descend
};
#define WAYPOINT_HOVER_TIME_US (5 * 1000000) // 5 seconds hover
#define FLIGHT_PROFILE_NAME "ALTITUDE"

#elif FLIGHT_PROFILE == FLIGHT_PROFILE_FULL_3D
// Full 3D waypoint navigation demo
// Altitudes limited to 1.2m max to stay within flow deck range (VL53L1x: 1.3m)
static const waypoint_t waypoints[] = {
    {0.0f, 0.0f, 0.8f, 0.0f}, // Start: origin, 0.8m
    {1.0f, 0.0f, 1.0f, 0.0f}, // Waypoint 1: +X, rise to 1.0m
    {1.0f, 1.0f, 1.2f,
     M_PI_F / 2.0f},            // Waypoint 2: corner, rise to 1.2m, face east
    {0.0f, 1.0f, 1.0f, M_PI_F}, // Waypoint 3: -X, drop to 1.0m, face south
    {0.0f, 0.0f, 0.8f, 0.0f},   // Return: origin, 0.8m, face north
};
#define WAYPOINT_HOVER_TIME_US (2 * 1000000) // 2 seconds hover
#define FLIGHT_PROFILE_NAME "FULL_3D"

#else
#error "Unknown FLIGHT_PROFILE"
#endif

#define NUM_WAYPOINTS (sizeof(waypoints) / sizeof(waypoints[0]))

#endif // FLIGHT_PROFILES_H
