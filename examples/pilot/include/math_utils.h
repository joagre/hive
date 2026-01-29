// Math utilities
//
// Common math macros and constants.

#ifndef PILOT_MATH_UTILS_H
#define PILOT_MATH_UTILS_H

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
static inline float normalize_angle(float a) {
    while (a > M_PI_F)
        a -= 2.0f * M_PI_F;
    while (a < -M_PI_F)
        a += 2.0f * M_PI_F;
    return a;
}

#endif // PILOT_MATH_UTILS_H
