#pragma once

#include <stddef.h>

namespace ralphee {

// Measured wheel-center geometry in metres, relative to the middle axle.
static constexpr double FRONT_TO_MIDDLE_M = 0.350;
static constexpr double MIDDLE_TO_REAR_M = 0.330;
static constexpr double TRACK_WIDTH_M = 0.580;
static constexpr double HALF_TRACK_M = TRACK_WIDTH_M / 2.0;

// Wheel order for geometry, pins, trims and direction/calibration arrays.
enum Wheel { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };
static constexpr size_t WHEEL_COUNT = 4;
static constexpr double HALF_PI_RAD = 1.57079632679489661923;

// Unconfigured: supply the same verified front-inner reference as the host.
// Zero is an invalid reference, not a physical steering limit.
static constexpr double STEERING_REFERENCE_ANGLE_RAD = 0.261799;

// Keep both servo attachment and writes disabled until calibration is verified.
static constexpr bool STEERING_OUTPUT_ENABLED = true;

} // namespace ralphee

// Legacy servo command bounds in degrees; these are not measured wheel limits.
#define MAX_ANGLE 135.0
#define MIN_ANGLE 45.0
#define START_ANGLE 90
#define BUFFER_SIZE 32
#define MIN_CHARS_IN_MESSAGE 2
