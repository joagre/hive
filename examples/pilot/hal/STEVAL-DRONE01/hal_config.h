// STEVAL-DRONE01 HAL Configuration
//
// Platform-specific constants for the STEVAL-DRONE01 hardware.

#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

// Base thrust for hover - NEEDS CALIBRATION for your specific drone
// Tune by gradually increasing until drone lifts off, then use ~90% of that value
// Use hal/STEVAL-DRONE01/vendor/thrust_test.c to find the liftoff threshold
#define HAL_BASE_THRUST  0.29f

#endif // HAL_CONFIG_H
