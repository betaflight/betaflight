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

extern "C" {
    #include "platform.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"
    #include "pg/rx.h"

    #include "io/beeper.h"
    #include "io/vtx.h"

    #include "rx/rx.h"

    #include "flight/pid.h"

    #include "config/config.h"
    #include "fc/rc_modes.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"
    #include "fc/core.h"

    #include "scheduler/scheduler.h"

    float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

    timeDelta_t getTaskDeltaTimeUs(taskId_e) { return 100000; }

    bool rxIsReceivingSignal() { return true; }

    uint8_t cliMode = 0;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

#define DEFAULT_MIN_CHECK 1100
#define BELOW_MIN (DEFAULT_MIN_CHECK-1)
#define DEFAULT_MAX_CHECK 1900
#define ABOVE_MAX (DEFAULT_MAX_CHECK+1)
#define CENTERED ((DEFAULT_MAX_CHECK+DEFAULT_MIN_CHECK)/2)

extern "C" {
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
}

class StickCommandsTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        PG_RESET(rxConfig);
        rxConfigMutable()->mincheck = DEFAULT_MIN_CHECK;
        rxConfigMutable()->maxcheck = DEFAULT_MAX_CHECK;
        rxConfigMutable()->midrc = CENTERED;
    }
};

enum {
    COUNTER_STICK_COMMAND_CONFIRMED,
    COUNTER_CHANGE_CONTROL_RATE_PROFILE,
    COUNTER_CHANGE_PID_PROFILE,
    COUNTER_VTX_CONTROL
};
#define CALL_COUNT_ITEM_COUNT 4

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

extern "C" {
void beeper(beeperMode_e mode)
{
    if (mode == BEEPER_CONFIRM_STICK_COMMANDS) {
        callCounts[COUNTER_STICK_COMMAND_CONFIRMED]++;
    }
}

void changeControlRateProfile(uint8_t)
{
    callCounts[COUNTER_CHANGE_CONTROL_RATE_PROFILE]++;
}

void changePidProfile(uint8_t)
{
    callCounts[COUNTER_CHANGE_PID_PROFILE]++;
}

void vtxIncrementBand()
{
    callCounts[COUNTER_VTX_CONTROL]++;
}

void vtxDecrementBand()
{
    callCounts[COUNTER_VTX_CONTROL]++;
}

void vtxIncrementChannel()
{
    callCounts[COUNTER_VTX_CONTROL]++;
}

void vtxDecrementChannel()
{
    callCounts[COUNTER_VTX_CONTROL]++;
}

}

void resetCallCounters(void)
{
    memset(&callCounts, 0, sizeof(callCounts));
}

uint32_t fixedMillis;

extern "C" {
uint32_t millis(void)
{
    return fixedMillis;
}

uint32_t micros(void)
{
    return fixedMillis * 1000;
}
}

void resetMillis(void)
{
    fixedMillis = 0;
}

TEST_F(StickCommandsTest, testStickCommandConfirmationSticksInMiddle)
{
    // given
    resetCallCounters();
    resetMillis();

    // and
    rcData[ROLL] = CENTERED;
    rcData[PITCH] = CENTERED;
    rcData[YAW] = CENTERED;
    rcData[THROTTLE] = CENTERED;

    // when
    processRcStickPositions();

    // user doesn't move sticks for more than STICK_DELAY_MS
    processRcStickPositions();

    // then
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
    EXPECT_EQ(0, CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE));
}

TEST_F(StickCommandsTest, testStickCommandConfirmationPidProfileChange)
{
    float change_pid_stick_commands[][4] = {
        //     ROLL      PITCH        YAW   THROTTLE
        { BELOW_MIN,  CENTERED, BELOW_MIN, BELOW_MIN },
        {  CENTERED, ABOVE_MAX, BELOW_MIN, BELOW_MIN },
        { ABOVE_MAX,  CENTERED, BELOW_MIN, BELOW_MIN },
    };
    const int num_commands = sizeof(change_pid_stick_commands)/sizeof(change_pid_stick_commands[0]);

    for (int i = 0; i < num_commands; ++i) {
        // given
        resetCallCounters();
        resetMillis();

        // and
        rcData[ROLL] = change_pid_stick_commands[i][0];
        rcData[PITCH] = change_pid_stick_commands[i][1];
        rcData[YAW] = change_pid_stick_commands[i][2];
        rcData[THROTTLE] = change_pid_stick_commands[i][3];

        // when
        processRcStickPositions();

        // then
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_CHANGE_PID_PROFILE));

        // after another task delay, longer than STICK_DELAY_MS
        processRcStickPositions();

        // then
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_CHANGE_PID_PROFILE));
    }
}

TEST_F(StickCommandsTest, testStickCommandConfirmationRateProfileChange)
{
    float change_rate_profile_stick_commands[][4] = {
        //     ROLL      PITCH        YAW   THROTTLE
        {  CENTERED, ABOVE_MAX, CENTERED, ABOVE_MAX },
        {  CENTERED, BELOW_MIN, CENTERED, ABOVE_MAX },
        { ABOVE_MAX,  CENTERED, CENTERED, ABOVE_MAX },
        { BELOW_MIN,  CENTERED, CENTERED, ABOVE_MAX },
    };
    const int num_commands = sizeof(change_rate_profile_stick_commands)/sizeof(change_rate_profile_stick_commands[0]);

    for (int i = 0; i < num_commands; ++i) {
        // given
        resetCallCounters();
        resetMillis();

        // and
        rcData[ROLL] = change_rate_profile_stick_commands[i][0];
        rcData[PITCH] = change_rate_profile_stick_commands[i][1];
        rcData[YAW] = change_rate_profile_stick_commands[i][2];
        rcData[THROTTLE] = change_rate_profile_stick_commands[i][3];

        // when
        processRcStickPositions();

        // then
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE));

        // after another task delay, longer than STICK_DELAY_MS
        processRcStickPositions();

        // then
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE));
    }
}

TEST_F(StickCommandsTest, testStickCommandConfirmationVtxControl)
{
    float vtx_control_commands[][4] = {
        //     ROLL      PITCH        YAW   THROTTLE
        { ABOVE_MAX, CENTERED, BELOW_MIN, ABOVE_MAX },
        { BELOW_MIN, CENTERED, BELOW_MIN, ABOVE_MAX },
        { ABOVE_MAX, CENTERED, ABOVE_MAX, ABOVE_MAX },
        { BELOW_MIN, CENTERED, ABOVE_MAX, ABOVE_MAX },
    };
    const int num_commands = sizeof(vtx_control_commands)/sizeof(vtx_control_commands[0]);

    for (int i = 0; i < num_commands; ++i) {
        // given
        resetCallCounters();
        resetMillis();

        // and
        rcData[ROLL] = vtx_control_commands[i][0];
        rcData[PITCH] = vtx_control_commands[i][1];
        rcData[YAW] = vtx_control_commands[i][2];
        rcData[THROTTLE] = vtx_control_commands[i][3];

        // when
        processRcStickPositions();

        // then
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(0, CALL_COUNTER(COUNTER_VTX_CONTROL));

        // after another task delay, longer than STICK_DELAY_MS
        processRcStickPositions();

        // then
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_STICK_COMMAND_CONFIRMED));
        EXPECT_EQ(1, CALL_COUNTER(COUNTER_VTX_CONTROL));
    }
}

extern "C" {
void vtxControlInputPoll(void) {}
void saveConfigAndNotify(void) {}
void accStartCalibration(void) {}
void gyroStartCalibration(bool isFirstArmingCalibration)
{
    UNUSED(isFirstArmingCalibration);
}
void applyAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
bool featureIsEnabled(uint32_t) { return false;}
bool sensors(uint32_t) { return false;}
void tryArm(void) {}
void disarm(flightLogDisarmReason_e) {}
void unsetArmingDisabled(armingDisableFlags_e flag) { UNUSED(flag); }
void dashboardDisablePageCycling() {}
void dashboardEnablePageCycling() {}

void beeperConfirmationBeeps(uint8_t) {}

bool failsafeIsActive() { return false; }
bool failsafeIsReceivingRxData() { return true; }

void GPS_reset_home_position(void) {}
void baroSetGroundLevel(void) {}

uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
pidProfile_t *currentPidProfile;
rxRuntimeState_t rxRuntimeState;
PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
void resetArmingDisabled(void) {}
armingDisableFlags_e getArmingDisableFlags(void)
{
    return (armingDisableFlags_e) 0;
}
bool isTryingToArm(void) { return false; }
void resetTryingToArm(void) {}
void compassStartCalibration(void) {}
}
