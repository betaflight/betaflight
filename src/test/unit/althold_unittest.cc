/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

extern "C" {

    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/alt_hold.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/position.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 0);

    extern altHoldState_t altHoldState;
    void altHoldReset(void);
    void altHoldProcessTransitions(void);
    void altHoldInit(void);
    void updateAltHoldState(timeUs_t);
    bool failsafeIsActive(void) { return false; }
    timeUs_t currentTimeUs = 0;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

TEST(AltholdUnittest, altHoldTransitionsTest)
{
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(altHoldState.isAltHoldActive, true);

    flightModeFlags ^= ALT_HOLD_MODE;
    millisRW = 56;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 64;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(altHoldState.isAltHoldActive, true);
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit();
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(altHoldState.isAltHoldActive, true);
}

// STUBS

extern "C" {
    acc_t acc;

    void pt2FilterInit(pt2Filter_t *altHoldDeltaLpf, float) {
        UNUSED(altHoldDeltaLpf);
    }
    float pt2FilterGain(float, float) {
        return 0.0f;
    }
    float pt2FilterApply(pt2Filter_t *altHoldDeltaLpf, float) {
        UNUSED(altHoldDeltaLpf);
        return 0.0f;
    }

    bool isAltitudeAvailable(void) { return true; }
    float getAltitude(void) { return 0.0f; }
    bool isAltitudeLow(void) { return true; }
    float getCosTiltAngle(void) { return 0.0f; }
    float rcCommand[4];

    float getRcDeflection(int)
    {
        return 0;
    }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig)
    {
        UNUSED(input);
        UNUSED(rxConfig);
    }

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    uint8_t armingFlags = 0;
    uint8_t stateFlags = 0;
    uint16_t flightModeFlags = 0;
}
