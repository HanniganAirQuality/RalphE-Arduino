#pragma once

#include <Servo.h>
#include <math.h>
#include <stdint.h>

#include "constants.hpp"

namespace ralphee {

// User-provided mapping, in the same FL, FR, RL, RR order as the geometry.
static constexpr int32_t g_servoPins[WHEEL_COUNT] = {
    3, // front left
    5, // front right
    6, // rear left ("bottom left")
    9  // rear right ("bottom right")
};
typedef int16_t IntAngle;

// All trims reset at the user's request. Nominal neutral is 90 degrees.
static constexpr IntAngle g_servoOffsets[WHEEL_COUNT] = {0, 0, 0, 0};

// Unverified: replace each 0 with the measured +1 or -1 physical direction.
static constexpr int8_t g_servoDirections[WHEEL_COUNT] = {1, 1, 1, 1};
// Unverified: servo-degree change per wheel-heading degree; 0 means unset.
static constexpr double g_servoDegreesPerWheelDegree[WHEEL_COUNT] = {0.6667, 1, 1, 1};

static Servo g_servos[WHEEL_COUNT];
static bool g_servosInitialized = false;

inline bool servoConfigurationReady() {
    if (!STEERING_OUTPUT_ENABLED || !isfinite(STEERING_REFERENCE_ANGLE_RAD)
            || STEERING_REFERENCE_ANGLE_RAD <= 0.0
            || STEERING_REFERENCE_ANGLE_RAD >= HALF_PI_RAD) {
        return false;
    }
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        if ((g_servoDirections[i] != 1 && g_servoDirections[i] != -1)
                || !isfinite(g_servoDegreesPerWheelDegree[i])
                || g_servoDegreesPerWheelDegree[i] <= 0.0) {
            return false;
        }
    }
    return true;
}

inline int8_t initServos() {
    if (!servoConfigurationReady()) return -1;
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        const IntAngle neutral = START_ANGLE + g_servoOffsets[i];
        if (neutral < MIN_ANGLE || neutral > MAX_ANGLE) return -2;
    }
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        g_servos[i].attach(g_servoPins[i]);
        g_servos[i].write(START_ANGLE + g_servoOffsets[i]);
    }
    g_servosInitialized = true;
    return 0;
}

/** Convert four wheel headings to servo degrees, then write all four servos. */
inline int8_t writeToServos(const double* wheelAnglesRad) {
    if (!g_servosInitialized || !servoConfigurationReady() || wheelAnglesRad == nullptr) {
        return -1;
    }
    IntAngle commands[WHEEL_COUNT];
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        if (!isfinite(wheelAnglesRad[i])) return -2;
        const double wheelDegrees = wheelAnglesRad[i] * (90.0 / HALF_PI_RAD);
        const double command = START_ANGLE + g_servoOffsets[i]
            + g_servoDirections[i] * g_servoDegreesPerWheelDegree[i] * wheelDegrees;
        // Check after calibration and trims, before rounding or any servo write.
        if (!isfinite(command) || command < MIN_ANGLE || command > MAX_ANGLE) return -2;
        commands[i] = static_cast<IntAngle>(lround(command));
    }
    for (size_t i = 0; i < WHEEL_COUNT; ++i) g_servos[i].write(commands[i]);
    return 0;
}

} // namespace ralphee
