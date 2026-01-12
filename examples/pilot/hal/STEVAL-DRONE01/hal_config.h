// STEVAL-DRONE01 HAL Configuration
//
// Platform-specific constants for the STEVAL-DRONE01 hardware.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover - calibrated via thrust_test.c
// 0.29 = just below liftoff, 0.30 = liftoff
#define HAL_BASE_THRUST  0.29f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Conservative output limit for slow, controlled climbs
#define HAL_ALT_PID_OMAX  0.08f

// ----------------------------------------------------------------------------
// Waypoint Navigation
// ----------------------------------------------------------------------------

// Time to hover at each waypoint before advancing (ticks at 250Hz)
// 1250 = 5 seconds - conservative for hardware testing
#define HAL_WAYPOINT_HOVER_TICKS  1250

#endif // HAL_CONFIG_H
