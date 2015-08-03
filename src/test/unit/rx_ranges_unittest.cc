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
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"

    #include "rx/rx.h"
    #include "io/rc_controls.h"
    #include "common/maths.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
extern uint16_t applyRxChannelRangeConfiguraton(int sample, rxChannelRangeConfiguration_t range);
}

#define RANGE_CONFIGURATION(min, max) (rxChannelRangeConfiguration_t) {min, max}

TEST(RxChannelRangeTest, TestRxChannelRanges)
{
    // No signal, special condition
    EXPECT_EQ(applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1000, 2000)), 0);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(1300, 1700)), 0);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(0, RANGE_CONFIGURATION(900, 2100)), 0);

    // Exact mapping
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1000, RANGE_CONFIGURATION(1000, 2000)), 1000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1000, 2000)), 1500);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2000, RANGE_CONFIGURATION(1000, 2000)), 2000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1000, 2000)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1000, 2000)), 2250);

    // Shifted range
    EXPECT_EQ(applyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 1900)), 1000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1400, RANGE_CONFIGURATION(900, 1900)), 1500);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1900, RANGE_CONFIGURATION(900, 1900)), 2000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 1900)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(900, 1900)), 2250);
    
    // Narrower range than expected
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1300, RANGE_CONFIGURATION(1300, 1700)), 1000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(1300, 1700)), 1500);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1700, RANGE_CONFIGURATION(1300, 1700)), 2000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(700, RANGE_CONFIGURATION(1300, 1700)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2500, RANGE_CONFIGURATION(1300, 1700)), 2250);

    // Wider range than expected
    EXPECT_EQ(applyRxChannelRangeConfiguraton(900, RANGE_CONFIGURATION(900, 2100)), 1000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1500, RANGE_CONFIGURATION(900, 2100)), 1500);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2100, RANGE_CONFIGURATION(900, 2100)), 2000);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(600, RANGE_CONFIGURATION(900, 2100)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(2700, RANGE_CONFIGURATION(900, 2100)), 2250);
    
    // extreme out of range
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1000, 2000)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(1300, 1700)), 750);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(1, RANGE_CONFIGURATION(900, 2100)), 750);

    EXPECT_EQ(applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1000, 2000)), 2250);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(1300, 1700)), 2250);
    EXPECT_EQ(applyRxChannelRangeConfiguraton(10000, RANGE_CONFIGURATION(900, 2100)), 2250);
}


// stubs
extern "C" {

void rxPwmInit(rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
}

bool sbusInit(rxConfig_t *initialRxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(initialRxConfig);
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
    return true;
}

bool spektrumInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
    return true;
}

bool sumdInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
    return true;
}

bool sumhInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
    return true;
}

bool rxMspInit(rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig, rcReadRawDataPtr *callback)
{
    UNUSED(rxConfig);
    UNUSED(rxRuntimeConfig);
    UNUSED(callback);
    return true;
}

void updateActivatedModes(modeActivationCondition_t *modeActivationConditions)
{
    UNUSED(modeActivationConditions);
}

void configureAdjustment(uint8_t index, uint8_t auxChannelIndex, const adjustmentConfig_t *adjustmentConfig)
{
    UNUSED(index);
    UNUSED(auxChannelIndex);
    UNUSED(adjustmentConfig);
}

void feature(uint32_t)
{
}

bool rxMspFrameComplete(void)
{
    return false;
}

void failsafeReset(void)
{
}

bool isPPMDataBeingReceived(void)
{
    return false;
}

void resetPPMDataReceivedState(void)
{
}

void failsafeOnRxCycle(void)
{
}

void failsafeOnValidDataReceived(void)
{
}

void failsafeOnRxCycleStarted(void)
{
}

void failsafeCheckPulse(uint8_t channel, uint16_t pulseDuration)
{
    UNUSED(channel);
    UNUSED(pulseDuration);
}

}

