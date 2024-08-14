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

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/alt_hold.h"
    #include "flight/position.h"
    #include "flight/imu.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(altholdConfig_t, altholdConfig, PG_ALTHOLD_CONFIG, 0);

    extern altHoldState_t altHoldState;
    void altHoldReset(void);
    void altHoldProcessTransitions(void);
    void altHoldInit(void);
    void altHoldUpdate(void);
    // end why ??
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t millisRW;
uint32_t millis() {
    return millisRW;
}

TEST(AltholdUnittest, altHoldTransitionsTest)
{
    altHoldInit();
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.isAltHoldActive, true);

    millisRW = 56;
    flightModeFlags ^= ALTHOLD_MODE;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 64;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.isAltHoldActive, true);

//     millisRW = 64 + 0.3f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 64 + 0.9f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 64 + 1.4f * ALTHOLD_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 10042;
//     flightModeFlags ^= ALTHOLD_MODE;
//     altHoldUpdate();
// 
//     millisRW = 10042 + 0.3f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     millisRW = 10042 + 0.5f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldState.throttleOut = 0.5f;
//     altHoldUpdate();
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit();
    EXPECT_EQ(altHoldState.isAltHoldActive, false);

    flightModeFlags |= ALTHOLD_MODE;
    millisRW = 42;
    altHoldUpdate();
    EXPECT_EQ(altHoldState.isAltHoldActive, true);
//     EXPECT_EQ(altHoldState.enterTime, 42);
//     EXPECT_EQ(altHoldState.exitTime, 0);
// 
//     millisRW += ALTHOLD_ENTER_PERIOD * 2;
//     altHoldUpdate();
// 
//     flightModeFlags ^= ALTHOLD_MODE;
//     altHoldUpdate();
//     millisRW += 0.5f * ALTHOLD_MAX_ENTER_PERIOD;
//     altHoldUpdate();
// 
//     flightModeFlags |= ALTHOLD_MODE;
//     millisRW += 1;
//     altHoldUpdate();
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


    float getAltitude(void) { return 0.0f; }
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