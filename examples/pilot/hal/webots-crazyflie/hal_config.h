// Webots Crazyflie HAL Configuration
//
// Platform-specific constants for the Webots Crazyflie simulation.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// ----------------------------------------------------------------------------
// Thrust
// ----------------------------------------------------------------------------

// Base thrust for hover (tuned for Webots Crazyflie simulation)
#define HAL_BASE_THRUST  0.553f

// ----------------------------------------------------------------------------
// Altitude Control
// ----------------------------------------------------------------------------

// Output limit for altitude PID
#define HAL_ALT_PID_OMAX  0.15f

// ----------------------------------------------------------------------------
// Waypoint Navigation
// ----------------------------------------------------------------------------

// Time to hover at each waypoint before advancing (ticks at 250Hz)
// 50 = 200ms - fast for simulation testing
#define HAL_WAYPOINT_HOVER_TICKS  50

#endif // HAL_CONFIG_H
