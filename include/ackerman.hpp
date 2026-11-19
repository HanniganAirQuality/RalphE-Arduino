#pragma once

#include <math.h>
#include <stdint.h>

#include "constants.hpp"

namespace ralphee {

/**
 * Calculate signed wheel headings in radians: FL, FR, RL, RR.
 * Zero is straight, positive input turns left in ROS coordinates.
 * referenceAngleRad is explicitly supplied: the front-inner heading at |input|=1.
 * Returns zero on success, a negative value on error. Never clamps input.
 * This function has no Arduino, serial or Servo dependencies.
 */
inline int8_t getWheelAngles(double input, double referenceAngleRad, double* outAnglesRad) {
    if (outAnglesRad == nullptr || !isfinite(input) || input < -1.0 || input > 1.0) {
        return -1;
    }
    if (!isfinite(referenceAngleRad) || referenceAngleRad <= 0.0
            || referenceAngleRad >= HALF_PI_RAD) {
        return -2;
    }
    if (input == 0.0) {
        for (size_t i = 0; i < WHEEL_COUNT; ++i) outAnglesRad[i] = 0.0;
        return 0;
    }

    const double angle = input * referenceAngleRad;
    const double sine = sin(angle);
    const double denominator = FRONT_TO_MIDDLE_M * cos(angle) + HALF_TRACK_M * fabs(sine);
    if (!isfinite(denominator) || denominator <= 0.0) return -3;
    const double curvature = sine / denominator;
    if (!isfinite(curvature) || curvature == 0.0
            || fabs(curvature) >= 1.0 / HALF_TRACK_M) {
        return -3;
    }

    const double left = 1.0 - curvature * HALF_TRACK_M;
    const double right = 1.0 + curvature * HALF_TRACK_M;
    if (left <= 0.0 || right <= 0.0) return -3;
    const double front = curvature * FRONT_TO_MIDDLE_M;
    const double rear = -curvature * MIDDLE_TO_REAR_M;
    const double angles[WHEEL_COUNT] = {
        atan2(front, left), atan2(front, right),
        atan2(rear, left), atan2(rear, right)
    };
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        if (!isfinite(angles[i])) return -3;
    }
    // Publish the complete result only after every heading passes validation.
    for (size_t i = 0; i < WHEEL_COUNT; ++i) outAnglesRad[i] = angles[i];
    return 0;
}

} // namespace ralphee
