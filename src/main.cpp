#include <stdlib.h>

// Enables logging
// #define LOGGING

#include "util.hpp"
#include "ackerman.hpp"
#include "char_buffer.hpp"
#include "servo.hpp"
#include "constants.hpp"

void setup() {
    Serial.begin(9600);
    if (ralphee::initServos() < 0) {
        LOGLN("Steering output disabled: reference/directions/calibration need verification.");
    }
    LOG("Enter a normalized steering command [-1,1], followed by carriage return:\n");
}

void loop() {
    if (Serial.available() >= MIN_CHARS_IN_MESSAGE) {
        char buffer[BUFFER_SIZE];
        if (ralphee::read(buffer) < 0) return;

        char* end = nullptr;
        const double input = strtod(buffer, &end);
        if (end == buffer || *end != '\0') return;

        double wheelAnglesRad[ralphee::WHEEL_COUNT];
        if (ralphee::getWheelAngles(
                input, ralphee::STEERING_REFERENCE_ANGLE_RAD, wheelAnglesRad) < 0) {
            LOGLN("Invalid steering command or unconfigured reference angle.");
            return;
        }
        if (ralphee::writeToServos(wheelAnglesRad) < 0) {
            LOGLN("Steering output disabled or servo command outside configured bounds.");
        }
    }
}
