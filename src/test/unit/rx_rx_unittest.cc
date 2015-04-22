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

    void rxInit(rxConfig_t *rxConfig);
    void rxCheckPulse(uint8_t channel, uint16_t pulseDuration);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

enum {
    COUNTER_FAILSAFE_ON_VALID_DATA_RECEIVED = 0,
};
#define CALL_COUNT_ITEM_COUNT 1

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

typedef struct testData_s {
    bool isPPMDataBeingReceived;
} testData_t;

static testData_t testData;

TEST(RxTest, TestFailsafeInformedOfValidData)
{
    // given
    resetCallCounters();
    memset(&testData, 0, sizeof(testData));

    // and
    rxConfig_t rxConfig;

    memset(&rxConfig, 0, sizeof(rxConfig));
    rxConfig.rx_min_usec = 1000;
    rxConfig.rx_max_usec = 2000;

    // when
    rxInit(&rxConfig);

    for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
        rxCheckPulse(channelIndex, 1500);
    }

    // then
    EXPECT_EQ(1, CALL_COUNTER(COUNTER_FAILSAFE_ON_VALID_DATA_RECEIVED));
}

TEST(RxTest, TestFailsafeNotInformedOfValidDataWhenStickChannelsAreBad)
{
    // given
    memset(&testData, 0, sizeof(testData));

    // and
    rxConfig_t rxConfig;

    memset(&rxConfig, 0, sizeof(rxConfig));
    rxConfig.rx_min_usec = 1000;
    rxConfig.rx_max_usec = 2000;

    // and
    uint16_t channelPulses[MAX_SUPPORTED_RC_CHANNEL_COUNT];
    memset(&channelPulses, 1500, sizeof(channelPulses));

    // and
    rxInit(&rxConfig);

    // and

    for (uint8_t stickChannelIndex = 0; stickChannelIndex < STICK_CHANNEL_COUNT; stickChannelIndex++) {

        // given
        resetCallCounters();

        for (uint8_t channelIndex = 0; channelIndex < STICK_CHANNEL_COUNT; channelIndex++) {
            channelPulses[stickChannelIndex] = rxConfig.rx_min_usec;
        }
        channelPulses[stickChannelIndex] = rxConfig.rx_min_usec - 1;

        // when
        for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
            rxCheckPulse(channelIndex, channelPulses[channelIndex]);
        }

        // then
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_FAILSAFE_ON_VALID_DATA_RECEIVED));

        // given
        resetCallCounters();

        for (uint8_t channelIndex = 0; channelIndex < STICK_CHANNEL_COUNT; channelIndex++) {
            channelPulses[stickChannelIndex] = rxConfig.rx_max_usec;
        }
        channelPulses[stickChannelIndex] = rxConfig.rx_max_usec + 1;

        // when
        for (uint8_t channelIndex = 0; channelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT; channelIndex++) {
            rxCheckPulse(channelIndex, channelPulses[channelIndex]);
        }

        // then
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_FAILSAFE_ON_VALID_DATA_RECEIVED));
    }
}


// STUBS

extern "C" {
    void failsafeOnRxCycleStarted() {}
    void failsafeOnValidDataReceived() {
        callCounts[COUNTER_FAILSAFE_ON_VALID_DATA_RECEIVED]++;
    }

    bool feature(uint32_t mask) {
        UNUSED(mask);
        return false;
    }

    bool isPPMDataBeingReceived(void) {
        return testData.isPPMDataBeingReceived;
    }

    void resetPPMDataReceivedState(void) {}

    bool rxMspFrameComplete(void) { return false; }

    void rxMspInit(rxConfig_t *, rxRuntimeConfig_t *, rcReadRawDataPtr *) {}

    void rxPwmInit(rxRuntimeConfig_t *, rcReadRawDataPtr *) {}


}
