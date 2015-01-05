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
    extern void generate_lowpass_coeffs2(int16_t freq, lowpass_t *filter);
}

uint32_t debug[4];
static int16_t servoRef[MAX_SUPPORTED_SERVOS];
static int16_t referenceOut[MAX_SUPPORTED_SERVOS];
static uint16_t freq;
static lowpass_t lowpassFilters[MAX_SUPPORTED_SERVOS];
static servoParam_t servoConfig[MAX_SUPPORTED_SERVOS];

#include "unittest_macros.h"
#include "gtest/gtest.h"

static float lowpass_ref(lowpass_t *filter, float in, int16_t freq)
{
    int16_t coefIdx;
    float out;

    // Check to see if cutoff frequency changed
    if (freq != filter->freq) {
        filter->init = false;
    }

    // Initialize if needed
    if (!filter->init) {
        generate_lowpass_coeffs2(freq, filter);
        for (coefIdx = 0; coefIdx < LOWPASS_NUM_COEF; coefIdx++) {
            filter->xf[coefIdx] = in;
            filter->yf[coefIdx] = in;
        }
        filter->init = true;
    }

    // Delays
    for (coefIdx = LOWPASS_NUM_COEF-1; coefIdx > 0; coefIdx--) {
        filter->xf[coefIdx] = filter->xf[coefIdx-1];
        filter->yf[coefIdx] = filter->yf[coefIdx-1];
    }
    filter->xf[0] = in;

    // Accumulate result
    out = filter->xf[0] * filter->bf[0];
    for (coefIdx = 1; coefIdx < LOWPASS_NUM_COEF; coefIdx++) {
        out += filter->xf[coefIdx] * filter->bf[coefIdx];
        out -= filter->yf[coefIdx] * filter->af[coefIdx];
    }
    filter->yf[0] = out;

    return out;
}

static void filterServosReference(void)
{
    int16_t servoIdx;

    for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
        // Round to nearest
        referenceOut[servoIdx] = (int16_t)(lowpass_ref(&lowpassFilters[servoIdx], (float)servoRef[servoIdx], freq) + 0.5f);
    }
}


TEST(MixerTest, ServoLowpassFilter)
{
    int16_t servoCmds[3000];
    int16_t expectedOut[3000];
    uint8_t servoIdx;
    uint16_t sampleIdx;
    static mixerConfig_t mixerConfig;

    uint16_t sampleCount = sizeof(servoCmds) / sizeof(int16_t);

    // generate inputs and expecteds
    for (sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
        if (sampleIdx < 1000) {
            servoCmds[sampleIdx] = 500;
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
        servoConfig[servoIdx].max = 3000;
    }

    // Test all frequencies
    for (freq = 10; freq <= 400; freq++)
    {
        printf("*** Testing freq: %d (%f)\n", freq, ((float)freq * 0.001f));

        mixerConfig.servo_lowpass_enable = 1;
        mixerConfig.servo_lowpass_freq_idx = freq;
        mixerUseConfigs(servoConfig, NULL, NULL, &mixerConfig, NULL, NULL, NULL);

        // Run tests
        for (sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) {
            for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
                servo[servoIdx] = servoCmds[sampleIdx];
                servoRef[servoIdx] = servoCmds[sampleIdx];
            }

            filterServos();
            filterServosReference();

            for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
                if (expectedOut[sampleIdx] >= 0) {
                    EXPECT_EQ(servo[servoIdx], expectedOut[sampleIdx]);
                }
                EXPECT_LE(servo[servoIdx], referenceOut[servoIdx] + 1);
                EXPECT_GE(servo[servoIdx], referenceOut[servoIdx] - 1);
            }
        } // for each sample
    } // for each freq
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


