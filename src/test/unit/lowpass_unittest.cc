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

//#define DEBUG_LOWPASS

extern "C" {
    #include "flight/lowpass.h" 
}

static lowpass_t lowpassFilterReference;
static lowpass_t lowpassFilter;

#include "unittest_macros.h"
#include "gtest/gtest.h"

static float lowpassRef(lowpass_t *filter, float in, int16_t freq)
{
    int16_t coefIdx;
    float out;

    // Check to see if cutoff frequency changed
    if (freq != filter->freq) {
        filter->init = false;
    }

    // Initialize if needed
    if (!filter->init) {
        generateLowpassCoeffs2(freq, filter);
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

TEST(LowpassTest, Lowpass)
{
    int16_t servoCmds[3000];
    int16_t expectedOut[3000];
    int16_t referenceOut;
    int16_t filterOut;
    uint16_t sampleIdx;
    int16_t freq;

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

    // Test all frequencies
    for (freq = 10; freq <= 400; freq++) {
#ifdef DEBUG_LOWPASS
        printf("*** Testing freq: %d (%f)\n", freq, ((float)freq * 0.001f));
#endif
        // Run tests
        for (sampleIdx = 0; sampleIdx < sampleCount; sampleIdx++) 
        {
            // Filter under test
            filterOut = (int16_t)lowpassFixed(&lowpassFilter, servoCmds[sampleIdx], freq);

            // Floating-point reference
            referenceOut = (int16_t)(lowpassRef(&lowpassFilterReference, (float)servoCmds[sampleIdx], freq) + 0.5f);

            if (expectedOut[sampleIdx] >= 0) {
                EXPECT_EQ(filterOut, expectedOut[sampleIdx]);
            }
            // Some tolerance
            // TODO adjust precision to remove the need for this?
            EXPECT_LE(filterOut, referenceOut + 1);
            EXPECT_GE(filterOut, referenceOut - 1);
        } // for each sample
    } // for each freq
}

// STUBS

extern "C" {

}


