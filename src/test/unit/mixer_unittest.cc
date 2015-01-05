/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>

#include <limits.h>

extern "C" {
    #include "flight/mixer.h" 
    #include "rx/rx.h"
    #include "io/gimbal.h"
    #include "io/escservo.h"
    extern void mixerUseConfigs(servoParam_t *servoConfToUse, flight3DConfig_t *flight3DConfigToUse, escAndServoConfig_t *escAndServoConfigToUse, mixerConfig_t *mixerConfigToUse, airplaneConfig_t *airplaneConfigToUse, rxConfig_t *rxConfigToUse, gimbalConfig_t *gimbalConfigToUse);
}
uint32_t debug[4];

#include "unittest_macros.h"
#include "gtest/gtest.h"

TEST(MixerTest, ServoLowpassFilter)
{
    int16_t servoCmds[3000];
    int16_t expectedOut[3000];
    const int16_t margin = 0;
    uint8_t servoIdx;
    uint16_t sampleIdx;
    static mixerConfig_t mixerConfig;
    static servoParam_t servoConfig[MAX_SUPPORTED_SERVOS];


    uint16_t sampleCount = sizeof(servoCmds) / sizeof(int16_t);

    // generate inputs and expecteds
    for (sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
        if (sampleIdx < 1000) {
            servoCmds[sampleIdx] = 501;
        } else if (sampleIdx >= 1000 && sampleIdx < 2000) {
            servoCmds[sampleIdx] = 2500;
        } else {
            servoCmds[sampleIdx] = 1500;
        }

        if ((sampleIdx >= 900 && sampleIdx < 1000) ||
            (sampleIdx >= 1900 && sampleIdx < 2000)||
            (sampleIdx >= 2900 && sampleIdx < 3000)) {
            expectedOut[sampleIdx] = servoCmds[sampleIdx];
        } else {
            expectedOut[sampleIdx] = -1;
        }

    }

    // Set mixer configuration
    for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
        servoConfig[servoIdx].min = 0;
        servoConfig[servoIdx].max = 2500;
    }

    mixerConfig.servo_lowpass_enable = 1;
    mixerConfig.servo_lowpass_freq_idx = 400; // 10 to 400
    mixerUseConfigs(servoConfig, NULL, NULL, &mixerConfig, NULL, NULL, NULL);

    // Run tests
    for (sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            servo[servoIdx] = servoCmds[sampleIdx];
        }

        filterServos();

        if (expectedOut[sampleIdx] >= 0) {
            for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
                EXPECT_LE(servo[servoIdx], expectedOut[sampleIdx] + margin);
                EXPECT_GE(servo[servoIdx], expectedOut[sampleIdx] - margin);
            }
        }
    }
}

// STUBS

extern "C" {

void delay(uint32_t ms)
{
    UNUSED(ms);
    return;
}

int constrain(int amt, int low, int high)
{
    return (amt > high ? high : (amt < low ? low : amt));
}

uint32_t micros()
{
    return 0;
}
}


