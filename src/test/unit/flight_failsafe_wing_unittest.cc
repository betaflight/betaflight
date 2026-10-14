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
#include <string.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "common/bitarray.h"

    #include "fc/core.h"
    #include "fc/rc_controls.h"
    #include "fc/rc_modes.h"
    #include "fc/runtime_config.h"

    #include "flight/failsafe.h"
    #include "flight/flight_plan_nav.h"

    #include "io/beeper.h"

    #include "pg/autopilot.h"

    #include "rx/rx.h"

    extern uint16_t flightModeFlags;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static throttleStatus_e throttleStatus = THROTTLE_HIGH;
static uint32_t sysTickUptime;
static int disarmCalls;
static bool testAutopilotThrottleValid;
static bool testFlightPlanActive;
static flightPlanNavState_e testFlightPlanState;
static bool testRescueActive;
static bool testRescueStages;
static int rescueStagings;
static bool testAwaitingThrow;
static bool testAltHoldActive;
static bool testGroundContact;

class FlightFailsafeWingTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        rxConfigMutable()->midrc = 1495;
        rxConfigMutable()->mincheck = 1100;
        failsafeConfigMutable()->failsafe_delay = 10;
        failsafeConfigMutable()->failsafe_landing_time = 1;
        failsafeConfigMutable()->failsafe_switch_mode = FAILSAFE_SWITCH_MODE_STAGE1;
        failsafeConfigMutable()->failsafe_throttle_low_delay = 100;
        failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;

        disarmCalls = 0;
        flightModeFlags = 0;
        testAutopilotThrottleValid = false;
        testFlightPlanActive = false;
        testFlightPlanState = FP_NAV_TARGETING;
        testRescueActive = false;
        testRescueStages = true;
        rescueStagings = 0;
        testAwaitingThrow = false;
        testAltHoldActive = false;
        testGroundContact = false;
        throttleStatus = THROTTLE_HIGH;
        sysTickUptime = 0;

        failsafeInit();
        failsafeReset();
        ENABLE_ARMING_FLAG(ARMED);
        failsafeStartMonitoring();
        failsafeOnValidDataReceived();
    }

    void TearDown() override
    {
        DISABLE_ARMING_FLAG(ARMED);
        flightModeFlags = 0;
    }

    // The pilot leaves the throttle stick low long enough for a disarm on rx loss, then the link goes.
    void loseRxWithTheThrottleLow()
    {
        loseRx(THROTTLE_LOW);
    }

    void loseRx(throttleStatus_e throttle)
    {
        sysTickUptime++;
        failsafeOnValidDataFailed();
        sysTickUptime += PERIOD_RXDATA_RECOVERY + 1;
        failsafeOnValidDataReceived();
        failsafeUpdateState();
        throttleStatus = THROTTLE_HIGH;
        failsafeUpdateState();
        throttleStatus = throttle;
        sysTickUptime += 13000;
        failsafeOnValidDataReceived();
        failsafeUpdateState();
        ASSERT_EQ(FAILSAFE_IDLE, failsafePhase());

        sysTickUptime += (failsafeConfig()->failsafe_delay * MILLIS_PER_TENTH_SECOND) + 1;
        failsafeOnValidDataFailed();
        failsafeUpdateState();
        ASSERT_TRUE(failsafeIsActive());
    }
};

TEST_F(FlightFailsafeWingTest, AltitudeHoldFliesItsOwnThrottleSoALowStickIsNoReasonToDisarm)
{
    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    testAutopilotThrottleValid = true;
    loseRxWithTheThrottleLow();
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(0, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, WithoutAltitudeHoldALowStickStillDisarms)
{
    loseRxWithTheThrottleLow();
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, AMissionCarriesOnUnderTheContinuePolicy)
{
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_CONTINUE;
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    testAutopilotThrottleValid = true;
    testFlightPlanActive = true;
    loseRxWithTheThrottleLow();
    EXPECT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());
    EXPECT_EQ(0, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, AMissionThatEndsUnderTheContinuePolicyFliesHomeToLand)
{
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_CONTINUE;
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    testAutopilotThrottleValid = true;
    testFlightPlanActive = true;
    loseRxWithTheThrottleLow();
    ASSERT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    testFlightPlanState = FP_NAV_COMPLETE;
    sysTickUptime += 2000;
    failsafeUpdateState();
    EXPECT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());
    EXPECT_EQ(1, rescueStagings);

    // the rescue flying, it is left to land
    testRescueActive = true;
    testFlightPlanState = FP_NAV_TARGETING;
    for (int i = 0; i < 100; i++) {
        sysTickUptime += 10;
        failsafeUpdateState();
    }
    EXPECT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());
    EXPECT_EQ(1, rescueStagings);
    EXPECT_EQ(0, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, WithNoWayHomeAnEndedMissionLandsWhereItIs)
{
    autopilotConfigMutable()->rxLossPolicy = AP_RX_LOSS_CONTINUE;
    ENABLE_FLIGHT_MODE(AUTOPILOT_MODE);
    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    testAutopilotThrottleValid = true;
    testFlightPlanActive = true;
    loseRxWithTheThrottleLow();
    ASSERT_EQ(FAILSAFE_AUTOPILOT, failsafePhase());

    testRescueStages = false;
    testFlightPlanState = FP_NAV_COMPLETE;
    sysTickUptime += 2000;
    failsafeUpdateState();
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
}

TEST_F(FlightFailsafeWingTest, ALinkLostWithTheWingStillInTheHandDisarmsWhateverTheThrottle)
{
    ENABLE_FLIGHT_MODE(LAUNCH_MODE);
    testAwaitingThrow = true;
    loseRx(THROTTLE_HIGH);
    EXPECT_EQ(FAILSAFE_RX_LOSS_MONITORING, failsafePhase());
    EXPECT_EQ(1, disarmCalls);

    SetUp();
    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_GPS_RESCUE;
    testAwaitingThrow = true;
    loseRx(THROTTLE_HIGH);
    EXPECT_EQ(1, disarmCalls);
    EXPECT_EQ(0, rescueStagings);

    // once thrown it is flying
    SetUp();
    loseRx(THROTTLE_HIGH);
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(0, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, ALandingUnderAltitudeHoldOnlyTimesOutOnTheGround)
{
    ENABLE_FLIGHT_MODE(ALT_HOLD_MODE);
    testAutopilotThrottleValid = true;
    testAltHoldActive = true;
    loseRx(THROTTLE_HIGH);
    ASSERT_EQ(FAILSAFE_LANDING, failsafePhase());

    // in the air, however long it takes
    for (int i = 0; i < 1000; i++) {
        sysTickUptime += 100;
        failsafeUpdateState();
    }
    EXPECT_EQ(FAILSAFE_LANDING, failsafePhase());
    EXPECT_EQ(0, disarmCalls);

    // on the ground for the landing time
    testGroundContact = true;
    for (int i = 0; i < 9; i++) {
        sysTickUptime += 100;
        failsafeUpdateState();
    }
    EXPECT_EQ(0, disarmCalls);
    sysTickUptime += 200;
    failsafeUpdateState();
    EXPECT_EQ(1, disarmCalls);
}

TEST_F(FlightFailsafeWingTest, WithoutAltitudeHoldTheLandingTimeRunsAsEver)
{
    loseRx(THROTTLE_HIGH);
    ASSERT_EQ(FAILSAFE_LANDING, failsafePhase());
    sysTickUptime += 1100;
    failsafeUpdateState();
    EXPECT_EQ(1, disarmCalls);
}

// STUBS

extern "C" {
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float rcCommand[4];
int16_t debug[DEBUG16_VALUE_COUNT];
uint8_t debugMode = 0;

PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

uint32_t millis(void) { return sysTickUptime; }
uint32_t micros(void) { return millis() * 1000; }
throttleStatus_e calculateThrottleStatus() { return throttleStatus; }
bool gpsRescueIsConfigured(void) { return false; }
void delay(uint32_t) {}
bool featureIsEnabled(uint32_t) { return false; }
void disarm(flightLogDisarmReason_e) { disarmCalls++; }
void beeper(beeperMode_e) {}
bool isUsingSticksForArming(void) { return true; }
bool areSticksActive(uint8_t) { return false; }
bool flightPlanNavIsActive(void) { return testFlightPlanActive; }
flightPlanNavState_e flightPlanNavGetState(void) { return testFlightPlanState; }
bool flightPlanNavIsRescuePlanActive(void) { return testRescueActive; }
bool flightPlanNavStageRescuePlan(void)
{
    rescueStagings++;
    return testRescueStages;
}
void beeperConfirmationBeeps(uint8_t) {}
bool crashRecoveryModeActive(void) { return false; }
void pinioBoxTaskControl(void) {}
bool usbCableIsInserted(void) { return false; }
bool mspSerialIsConfiguratorActive(void) { return false; }
bool isFixedWing(void) { return true; }
bool autopilotThrottleValid(void) { return testAutopilotThrottleValid; }
bool launchWingAwaitingThrow(void) { return testAwaitingThrow; }
bool isAltHoldActive(void) { return testAltHoldActive; }
bool altHoldGroundContact(void) { return testGroundContact; }
}
