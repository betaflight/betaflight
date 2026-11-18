/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "common/maths.h"
    #include "common/vector.h"

    #include "fc/core.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/autopilot.h"
    #include "flight/failsafe.h"
    #include "flight/gps_rescue.h"
    #include "flight/imu.h"
    #include "flight/position.h"
    #include "flight/position_estimator.h"

    #include "io/beeper.h"
    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/gps_rescue.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    gpsSolutionData_t gpsSol;
    attitudeEulerAngles_t attitude;

    PG_REGISTER(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);
    PG_REGISTER(failsafeConfig_t, failsafeConfig, PG_FAILSAFE_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static timeUs_t simulatedTimeUs = 1000000;
static float simulatedAltitudeCm;
static bool simulatedRxSignal;
static bool simulatedGpsHealthy;
static int disarmCount;
static flightLogDisarmReason_e lastDisarmReason;
static positionEstimate3d_t simulatedEstimate;
static int landingCalls;
static landingProfile_t lastLandingProfile;
static float landingTargetCm;

extern "C" {
    timeUs_t micros(void) { return simulatedTimeUs; }
    timeMs_t millis(void) { return simulatedTimeUs / 1000; }

    void disarm(flightLogDisarmReason_e reason) { disarmCount++; lastDisarmReason = reason; }
    bool crashRecoveryModeActive(void) { return false; }
    bool isRxReceivingSignal(void) { return simulatedRxSignal; }

    float getAltitudeCm(void) { return simulatedAltitudeCm; }
    float getAltitudeCmControl(void) { return simulatedAltitudeCm; }
    bool isAltitudeAvailable(void) { return true; }
    bool isBelowLandingAltitude(void) { return simulatedAltitudeCm < 100.0f * autopilotConfig()->landingAltitudeM; }

    bool gpsHasNewData(uint16_t *) { return false; }
    bool gpsIsHealthy(void) { return simulatedGpsHealthy; }
    bool imuIsHeadingValid(void) { return true; }
    bool positionEstimatorIsValidXY(void) { return true; }
    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &simulatedEstimate; }

    void autopilotSetYawTarget(float) { }
    void setTargetVelocity(const vector2_t *, bool) { }
    void altitudeControl(float, timeUs_t, float, float) { }
    void autopilotLand(const landingProfile_t *profile, float startAltitudeCm, timeUs_t taskIntervalUs)
    {
        landingCalls++;
        lastLandingProfile = *profile;
        landingTargetCm = startAltitudeCm - profile->touchdownRateCmS * US_TO_INTERVAL(taskIntervalUs);
    }
    float autopilotLandingTargetAltitudeCm(void) { return landingTargetCm; }
    void resetAltitudeControl(void) { }
    void pitchForwardOverride(bool) { }
    void initPositionHold(void) { }
    bool isModeActivationConditionPresent(boxId_e) { return false; }
    void beeperConfirmationBeeps(uint8_t) { }
}

static void runRescueSeconds(float seconds)
{
    const int ticks = lrintf(seconds * TASK_GPS_RESCUE_RATE_HZ);
    for (int i = 0; i < ticks; i++) {
        simulatedTimeUs += HZ_TO_INTERVAL_US(TASK_GPS_RESCUE_RATE_HZ);
        gpsRescueUpdate();
    }
}

class GpsRescueEmergencyDescent : public ::testing::Test {
protected:
    void SetUp() override
    {
        pgResetAll();
        simulatedTimeUs += 60000000;
        simulatedAltitudeCm = 2000.0f;
        simulatedRxSignal = false;
        simulatedGpsHealthy = false;
        disarmCount = 0;
        landingCalls = 0;
        stateFlags = 0;
        memset(&simulatedEstimate, 0, sizeof(simulatedEstimate));
        simulatedEstimate.position.v[EF_NORTH] = 20000.0f;
        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
    }

    void TearDown() override
    {
        DISABLE_FLIGHT_MODE(GPS_RESCUE_MODE);
        gpsRescueUpdate();
        DISABLE_ARMING_FLAG(ARMED);
    }
};

TEST_F(GpsRescueEmergencyDescent, DisarmsAfterSettlingOnTheGround)
{
    // no home point on a hard failsafe: emergency descent, no XY control
    runRescueSeconds(1.0f);
    EXPECT_GT(landingCalls, 0);
    EXPECT_EQ(0, disarmCount);

    // the craft comes to rest and the altitude stops changing
    simulatedAltitudeCm = 0.0f;
    runRescueSeconds(10.0f);
    EXPECT_EQ(0, disarmCount);
    runRescueSeconds(10.0f);
    EXPECT_EQ(1, disarmCount);
    EXPECT_EQ(DISARM_REASON_FAILSAFE, lastDisarmReason);
}

TEST_F(GpsRescueEmergencyDescent, DisarmsWhenStalledAboveLandingAltitudeWithoutRadio)
{
    runRescueSeconds(1.0f);

    // altitude stops changing well above the landing altitude, and the pilot cannot disarm
    runRescueSeconds(10.0f);
    EXPECT_EQ(0, disarmCount);
    runRescueSeconds(10.0f);
    EXPECT_EQ(1, disarmCount);
}

TEST_F(GpsRescueEmergencyDescent, KeepsFlyingWhenStalledAboveLandingAltitudeWithRadio)
{
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_ON;
    simulatedRxSignal = true;
    runRescueSeconds(1.0f);
    EXPECT_GT(landingCalls, 0);

    // altitude stops changing well above the landing altitude, left to the pilot's disarm switch
    runRescueSeconds(30.0f);
    EXPECT_EQ(0, disarmCount);
}

TEST_F(GpsRescueEmergencyDescent, StallWithRadioDoesNotCountTowardsDisarmAfterRadioLoss)
{
    gpsRescueConfigMutable()->sanityChecks = RESCUE_SANITY_ON;
    simulatedRxSignal = true;
    runRescueSeconds(30.0f);

    // the radio drops out with the craft still stalled above the landing altitude
    simulatedRxSignal = false;
    runRescueSeconds(5.0f);
    EXPECT_EQ(0, disarmCount);
    runRescueSeconds(12.0f);
    EXPECT_EQ(1, disarmCount);
}

class GpsRescueLanding : public GpsRescueEmergencyDescent {
protected:
    void SetUp() override
    {
        GpsRescueEmergencyDescent::SetUp();
        gpsRescueInit();
        simulatedGpsHealthy = true;
        gpsSol.numSat = GPS_MIN_SAT_COUNT + 4;
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_HOME);
        simulatedEstimate.position.v[EF_NORTH] = 1000.0f;
    }
};

TEST_F(GpsRescueLanding, DisarmsAfterDescentStallsAboveLandingAltitude)
{
    // climb to the return altitude, then descend as already within the descent distance
    runRescueSeconds(1.0f);
    EXPECT_EQ(0, landingCalls);
    simulatedAltitudeCm = 3100.0f;
    runRescueSeconds(1.0f);

    // the descent is flown as a landing, at the rescue descent rate near the ground and faster high up
    EXPECT_GT(landingCalls, 0);
    EXPECT_FLOAT_EQ(0.6f * gpsRescueConfig()->descendRate, lastLandingProfile.touchdownRateCmS);
    EXPECT_FLOAT_EQ(3.0f * gpsRescueConfig()->descendRate, lastLandingProfile.maxRateCmS);

    // on the ground on terrain higher than home: the descent stalls above the landing altitude
    simulatedAltitudeCm = 1000.0f;
    runRescueSeconds(20.0f);
    EXPECT_EQ(0, disarmCount);
    runRescueSeconds(15.0f);
    EXPECT_EQ(1, disarmCount);
    EXPECT_EQ(DISARM_REASON_FAILSAFE, lastDisarmReason);
}
