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

    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "flight/alt_hold.h"
    #include "flight/autopilot.h"
    #include "flight/failsafe.h"
    #include "flight/imu.h"
    #include "flight/pid.h"
    #include "flight/position.h"

    #include "io/gps.h"

    #include "rx/rx.h"

    #include "sensors/acceleration.h"
    #include "sensors/gyro.h"

    PG_REGISTER(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 0);
    PG_REGISTER(altHoldConfig_t, altHoldConfig, PG_ALTHOLD_CONFIG, 0);
    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 0);
    PG_REGISTER(positionConfig_t, positionConfig, PG_POSITION, 0);
    PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

    bool failsafeIsActive(void) { return false; }
    timeUs_t currentTimeUs = 0;
    bool isAltHoldActive();
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
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);

    flightModeFlags ^= ALT_HOLD_MODE;
    millisRW = 56;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 64;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);
}

TEST(AltholdUnittest, altHoldTransitionsTestUnfinishedExitEnter)
{
    altHoldInit();
    EXPECT_EQ(isAltHoldActive(), false);

    flightModeFlags |= ALT_HOLD_MODE;
    millisRW = 42;
    updateAltHoldState(currentTimeUs);
    EXPECT_EQ(isAltHoldActive(), true);
}

// STUBS

extern "C" {
    uint8_t armingFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;

    acc_t acc;
    attitudeEulerAngles_t attitude;
    gpsSolutionData_t gpsSol;

    float getAltitudeCm(void) {return 0.0f;}
    float getAltitudeDerivative(void) {return 0.0f;}
    float getCosTiltAngle(void) { return 0.0f; }
    float getGpsDataIntervalSeconds(void) { return 0.01f; }//    gpsSolutionData_t gpsSol;
    float rcCommand[4];

    bool gpsHasNewData(uint16_t* gpsStamp) {
        UNUSED(*gpsStamp);
        return true;
    }

    float vector2Norm(const vector2_t *v) {
       UNUSED(*v);
       return 0.0f;
    }

void GPS_latLongVectors(const gpsLocation_t *from, const gpsLocation_t *to, float *pEWDist, float *pNSDist)
    {
       UNUSED(from);
       UNUSED(to);
       UNUSED(pEWDist);
       UNUSED(pNSDist);
    }

    void parseRcChannels(const char *input, rxConfig_t *rxConfig)
    {
        UNUSED(input);
        UNUSED(rxConfig);
    }

    float pt1FilterGain(float f_cut, float dT)
    {
        UNUSED(f_cut);
        UNUSED(dT);
        return 0.0;
    }

    void pt1FilterInit(pt1Filter_t *filter, float k)
    {
        UNUSED(filter);
        UNUSED(k);
    }

    void pt1FilterUpdateCutoff(pt1Filter_t *filter, float k)
    {
        UNUSED(filter);
        UNUSED(k);
    }

    float pt1FilterApply(pt1Filter_t *filter, float input)
    {
        UNUSED(filter);
        UNUSED(input);
        return 0.0;
    }

    float pt3FilterGain(float f_cut, float dT)
    {
        UNUSED(f_cut);
        UNUSED(dT);
        return 0.0;
    }
    void pt3FilterInit(pt3Filter_t *filter, float k)
    {
        UNUSED(filter);
        UNUSED(k);
    }
    void pt3FilterUpdateCutoff(pt3Filter_t *filter, float k)
    {
        UNUSED(filter);
        UNUSED(k);
    }
    float pt3FilterApply(pt3Filter_t *filter, float input)
    {
        UNUSED(filter);
        UNUSED(input);
        return 0.0;
    }
}
