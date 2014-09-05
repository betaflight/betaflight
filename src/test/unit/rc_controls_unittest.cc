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

#include "common/axis.h"
#include "flight/flight.h"

#include "rx/rx.h"
#include "io/rc_controls.h"

#include "unittest_macros.h"
#include "gtest/gtest.h"


#define NON_AUX_CHANNEL_COUNT 4

TEST(RcControlsTest, updateRcOptionsWithAllInputsAtMidde)
{
    // given
    uint32_t activate[CHECKBOX_ITEM_COUNT];
    memset(&activate, 0, sizeof(activate));

    uint8_t index;

    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
        rcOptions[index] = 0;
    }

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.auxChannelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }


    // and
    uint32_t expectedRcOptions[CHECKBOX_ITEM_COUNT];
    memset(&expectedRcOptions, 0, sizeof(expectedRcOptions));

    // when
    updateRcOptions(activate);

    // then
    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
        printf("iteration: %d\n", index);
        EXPECT_EQ(expectedRcOptions[index], rcOptions[index]);
    }
}

TEST(RcControlsTest, updateRcOptionsUsingValidAuxConfigurationAndRXValues)
{
    // given
    uint32_t activate[CHECKBOX_ITEM_COUNT];
    memset(&activate, 0, sizeof(activate));
    activate[0] = 0b000000000100UL;
    activate[1] = 0b000000010000UL;
    activate[2] = 0b000001000000UL;
    activate[3] = 0b111000000000UL;

    activate[4] = 0b000000000001UL << 16;
    activate[5] = 0b000000010000UL << 16;
    activate[6] = 0b000100000000UL << 16;
    activate[7] = 0b111000000000UL << 16;

    uint8_t index;

    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
        rcOptions[index] = 0;
    }

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.auxChannelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MIDDLE;
    rcData[AUX3] = PWM_RANGE_MIN;
    rcData[AUX4] = PWM_RANGE_MAX;
    rcData[AUX5] = PWM_RANGE_MIN;
    rcData[AUX6] = PWM_RANGE_MIDDLE;
    rcData[AUX7] = PWM_RANGE_MAX;
    rcData[AUX8] = PWM_RANGE_MIN;


    // and
    uint32_t expectedRcOptions[CHECKBOX_ITEM_COUNT];
    memset(&expectedRcOptions, 0, sizeof(expectedRcOptions));
    expectedRcOptions[0] = 1;
    expectedRcOptions[1] = 1;
    expectedRcOptions[2] = 1;
    expectedRcOptions[3] = 1;
    expectedRcOptions[4] = 1;
    expectedRcOptions[5] = 1;
    expectedRcOptions[6] = 1;
    expectedRcOptions[7] = 1;

    // when
    updateRcOptions(activate);

    // then
    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
        printf("iteration: %d\n", index);
        EXPECT_EQ(expectedRcOptions[index], rcOptions[index]);
    }
}

void changeProfile(uint8_t profileIndex) {}
void accSetCalibrationCycles(uint16_t) {}
void gyroSetCalibrationCycles(uint16_t) {}
void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
void mwArm(void) {}
void feature(uint32_t) {}
void sensors(uint32_t) {}
void mwDisarm(void) {}

uint8_t armingFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxRuntimeConfig_t rxRuntimeConfig;
