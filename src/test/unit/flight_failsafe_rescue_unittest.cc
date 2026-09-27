/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

// Flag-on companion to flight_failsafe_unittest.cc: ENABLE_RESCUE_PLAN=1,
// exercising the GPS-RESCUE failsafe procedure's rescue-as-mission staging
// and the FAILSAFE_AUTOPILOT engage-grace/fallback logic it adds.
// flight_failsafe_unittest.cc stays the flag-off regression guard.

#include <stdint.h>
#include <stdbool.h>

#include <limits.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/bitarray.h"

    #include "fc/runtime_config.h"
    #include "fc/rc_modes.h"
    #include "fc/rc_controls.h"
    #include "fc/core.h"

    #include "flight/failsafe.h"
    #include "flight/flight_plan_nav.h"

    #include "io/beeper.h"

    #include "drivers/io.h"
    #include "rx/rx.h"

    #include "pg/autopilot.h"

    extern boxBitmask_t rcModeActivationMask;
    extern uint16_t flightModeFlags;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

uint32_t testFeatureMask = 0;
throttleStatus_e throttleStatus = THROTTLE_HIGH;

enum {
    COUNTER_MW_DISARM = 0,
};
#define CALL_COUNT_ITEM_COUNT 1

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

void resetCallCounters(void)
{
    memset(&callCounts, 0, sizeof(callCounts));
}

uint32_t sysTickUptime;

void configureFailsafe(void)
{
    rxConfigMutable()->midrc = 1495;
    rxConfigMutable()->mincheck = 1100;

    failsafeConfigMutable()->failsafe_delay = 10; // 1 second
    failsafeConfigMutable()->failsafe_landing_time = 1; // 1.0 seconds
    failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1;
    failsafeConfigMutable()->failsafe_throttle = 1200;
    failsafeConfigMutable()->failsafe_throttle_low_delay = 100; // 10 seconds
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    sysTickUptime = 0;
}

//
// rx-loss policy / GPS-RESCUE-as-mission tests
//

extern "C" {
    extern bool testSticksActive;
    extern bool testFlightPlanActive;
    extern flightPlanNavState_e testFlightPlanState;
    extern bool testStageRescuePlanResult;
    extern int testStageRescuePlanCalls;
}

class FlightFailsafeRescueTest : public ::testing::Test {
protected:
    void SetUp() override {
        resetCallCounters();
        configureFailsafe();
        failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;

        flightModeFlags = 0;
        testSticksActive = false;
        testFlightPlanActive = false;
        testFlightPlanState = FP_NAV_IDLE;
        testStageRescuePlanResult = true;
        testStageRescuePlanCalls = 0;
        throttleStatus = THROTTLE_HIGH;

        failsafeInit();
        failsafeReset(); // tests in this fixture are self-contained, not sequential
        ENABLE_ARMING_FLAG(ARMED);
        failsafeStartMonitoring();

        sysTickUptime = 0;
        failsafeOnValidDataReceived();
    }

    void TearDown() override {
        flightModeFlags = 0;
        DISABLE_ARMING_FLAG(ARMED);
        testSticksActive = false;
        testFlightPlanActive = false;
    }

    void startMission() {
        ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
        testFlightPlanActive = true;
        testFlightPlanState = FP_NAV_TARGETING;
    }

    void loseRxIntoStage2() {
        sysTickUptime += (failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND) + 1;
        failsafeOnValidDataFailed();
        failsafeUpdateState();
        ASSERT_TRUE(failsafeIsActive());
    }

    void tickLinkStillDown(uint32_t ms) {
        sysTickUptime += ms;
        failsafeOnValidDataFailed();
        failsafeUpdateState();
    }
};

TEST_F(FlightFailsafeRescueTest, GpsRescueProcedureStagesAndEntersAutopilotPhase)
{
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE;
    testStageRescuePlanResult = true;

    loseRxIntoStage2();

    EXPECT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());
    EXPECT_TRUE(FLIGHT_MODE(FAILSAFE_MODE));
    EXPECT_FALSE(FLIGHT_MODE(GPS_RESCUE_MODE));
    EXPECT_EQ(1, testStageRescuePlanCalls);
}

TEST_F(FlightFailsafeRescueTest, StageFailureFallsBackToAutoLanding)
{
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE;
    testStageRescuePlanResult = false;

    loseRxIntoStage2();

    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_FALSE(FLIGHT_MODE(GPS_RESCUE_MODE));
    EXPECT_EQ(1, testStageRescuePlanCalls);
}

TEST_F(FlightFailsafeRescueTest, EngageGraceHonoured)
{
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE;
    testStageRescuePlanResult = true;

    loseRxIntoStage2();
    ASSERT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    // core.c has not engaged AUTOPILOT_MODE yet; within the 1000 ms grace
    // window the phase must not degrade.
    tickLinkStillDown(900);
    EXPECT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    // Past the grace deadline: degrade to auto-landing (GPS_RESCUE remapped).
    tickLinkStillDown(200);
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
}

TEST_F(FlightFailsafeRescueTest, AbortFallsBackToAutoLandingNotRescue)
{
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE;
    testStageRescuePlanResult = true;

    loseRxIntoStage2();
    ASSERT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    // core.c engaged the staged mission, which then aborted mid-flight.
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    testFlightPlanActive = true;
    testFlightPlanState = FP_NAV_ABORTED;
    const int stageCallsBefore = testStageRescuePlanCalls;

    tickLinkStillDown(100);

    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_FALSE(FLIGHT_MODE(GPS_RESCUE_MODE));
    // The failed mission must not be re-staged.
    EXPECT_EQ(stageCallsBefore, testStageRescuePlanCalls);
}

TEST_F(FlightFailsafeRescueTest, MissionCompleteFallsBackToConfiguredProcedure)
{
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_CONTINUE;
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    startMission();

    loseRxIntoStage2();
    ASSERT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    testFlightPlanState = FP_NAV_COMPLETE;
    tickLinkStillDown(100);

    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    // GPS_RESCUE staging is never involved when the configured procedure isn't GPS_RESCUE.
    EXPECT_EQ(0, testStageRescuePlanCalls);
}

// STUBS

extern "C" {
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float rcCommand[4];
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;
bool isUsingSticksToArm = true;

PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}

uint32_t micros(void)
{
    return millis() * 1000;
}

throttleStatus_e calculateThrottleStatus()
{
    return throttleStatus;
}

void delay(uint32_t) {}

bool featureIsEnabled(uint32_t mask)
{
    return (mask & testFeatureMask);
}

void disarm(flightLogDisarmReason_e)
{
    callCounts[COUNTER_MW_DISARM]++;
}

void beeper(beeperMode_e mode)
{
    UNUSED(mode);
}

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}

bool testSticksActive = false;
bool areSticksActive(uint8_t stickPercentLimit)
{
    UNUSED(stickPercentLimit);
    return testSticksActive;
}

bool testFlightPlanActive = false;
flightPlanNavState_e testFlightPlanState = FP_NAV_IDLE;

bool flightPlanNavIsActive(void)
{
    return testFlightPlanActive;
}

flightPlanNavState_e flightPlanNavGetState(void)
{
    return testFlightPlanState;
}

bool testStageRescuePlanResult = true;
int testStageRescuePlanCalls = 0;

bool flightPlanNavStageRescuePlan(void)
{
    testStageRescuePlanCalls++;
    return testStageRescuePlanResult;
}

void beeperConfirmationBeeps(uint8_t beepCount) { UNUSED(beepCount); }

bool crashRecoveryModeActive(void) { return false; }
void pinioBoxTaskControl(void) {}

bool usbCableIsInserted(void)
{
    return false;
}

bool mspSerialIsConfiguratorActive(void)
{
    return false;
}
}
