// Math utilities
//
// Common math macros and constants.

#ifndef PILOT_MATH_UTILS_H
#define PILOT_MATH_UTILS_H

#include <math.h>

// Clamp value to range [lo, hi]
#define CLAMPF(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))

// Low-pass filter: LPF(state, new_sample, alpha)
// alpha=0: instant response, alpha=1: no response
#define LPF(state, sample, alpha) \
    ((alpha) * (state) + (1.0f - (alpha)) * (sample))

// Angle conversions
#define RAD_TO_DEG 57.2957795f // 180/pi
#define DEG_TO_RAD 0.0174533f  // pi/180
#define M_PI_F 3.14159265f     // pi as float

// Normalize angle to [-pi, pi] range
// Uses fmodf for bounded execution (no infinite loop on corrupted input)
static inline float normalize_angle(float a) {
    // Handle NaN/Inf - return 0 as safe default
    if (!isfinite(a)) {
        return 0.0f;
    }
    // fmodf is bounded, unlike while-loop approach
    a = fmodf(a + M_PI_F, 2.0f * M_PI_F);
    if (a < 0.0f) {
        a += 2.0f * M_PI_F;
    }
    return a - M_PI_F;
}

#endif // PILOT_MATH_UTILS_H
