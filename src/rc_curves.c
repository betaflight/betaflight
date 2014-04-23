#include <stdbool.h>
#include <stdint.h>

#include "rc_controls.h"
#include "escservo.h"

#include "rc_curves.h"

int16_t lookupPitchRollRC[PITCH_LOOKUP_LENGTH];     // lookup table for expo & RC rate PITCH+ROLL
int16_t lookupThrottleRC[THROTTLE_LOOKUP_LENGTH];   // lookup table for expo & mid THROTTLE


void generatePitchCurve(controlRateConfig_t *controlRateConfig)
{
    uint8_t i;

    for (i = 0; i < PITCH_LOOKUP_LENGTH; i++)
        lookupPitchRollRC[i] = (2500 + controlRateConfig->rcExpo8 * (i * i - 25)) * i * (int32_t) controlRateConfig->rcRate8 / 2500;
}

void generateThrottleCurve(controlRateConfig_t *controlRateConfig, escAndServoConfig_t *escAndServoConfig)
{
    uint8_t i;

    for (i = 0; i < THROTTLE_LOOKUP_LENGTH; i++) {
        int16_t tmp = 10 * i - controlRateConfig->thrMid8;
        uint8_t y = 1;
        if (tmp > 0)
            y = 100 - controlRateConfig->thrMid8;
        if (tmp < 0)
            y = controlRateConfig->thrMid8;
        lookupThrottleRC[i] = 10 * controlRateConfig->thrMid8 + tmp * (100 - controlRateConfig->thrExpo8 + (int32_t) controlRateConfig->thrExpo8 * (tmp * tmp) / (y * y)) / 10;
        lookupThrottleRC[i] = escAndServoConfig->minthrottle + (int32_t) (escAndServoConfig->maxthrottle - escAndServoConfig->minthrottle) * lookupThrottleRC[i] / 1000; // [MINTHROTTLE;MAXTHROTTLE]
    }
}
