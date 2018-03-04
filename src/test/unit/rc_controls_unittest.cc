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

//#define DEBUG_RC_CONTROLS

extern "C" {
    #include "platform.h"

    #include "common/maths.h"
    #include "common/axis.h"
    #include "common/bitarray.h"

    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    #include "blackbox/blackbox.h"
    #include "blackbox/blackbox_fielddefs.h"

    #include "drivers/sensor.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "io/beeper.h"

    #include "rx/rx.h"

    #include "flight/pid.h"

    #include "fc/config.h"
    #include "fc/controlrate_profile.h"
    #include "fc/rc_modes.h"
    #include "fc/rc_adjustments.h"

    #include "fc/rc_controls.h"
    #include "fc/runtime_config.h"

    #include "scheduler/scheduler.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

void unsetArmingDisabled(armingDisableFlags_e flag) {
  UNUSED(flag);
}

class RcControlsModesTest : public ::testing::Test {
protected:
    virtual void SetUp() {
    }
};

TEST_F(RcControlsModesTest, updateActivatedModesWithAllInputsAtMidde)
{
    // given
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // when
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(false, IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

TEST_F(RcControlsModesTest, updateActivatedModesUsingValidAuxConfigurationAndRXValues)
{
    // given
    modeActivationConditionsMutable(0)->modeId = (boxId_e)0;
    modeActivationConditionsMutable(0)->auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(1)->modeId = (boxId_e)1;
    modeActivationConditionsMutable(1)->auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(1)->range.endStep = CHANNEL_VALUE_TO_STEP(1700);

    modeActivationConditionsMutable(2)->modeId = (boxId_e)2;
    modeActivationConditionsMutable(2)->auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(2)->range.endStep = CHANNEL_VALUE_TO_STEP(1200);

    modeActivationConditionsMutable(3)->modeId = (boxId_e)3;
    modeActivationConditionsMutable(3)->auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(3)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(3)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditionsMutable(4)->modeId = (boxId_e)4;
    modeActivationConditionsMutable(4)->auxChannelIndex = AUX5 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(4)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(4)->range.endStep = CHANNEL_VALUE_TO_STEP(925);

    EXPECT_EQ(0, modeActivationConditions(4)->range.startStep);
    EXPECT_EQ(1, modeActivationConditions(4)->range.endStep);

    modeActivationConditionsMutable(5)->modeId = (boxId_e)5;
    modeActivationConditionsMutable(5)->auxChannelIndex = AUX6 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(5)->range.startStep = CHANNEL_VALUE_TO_STEP(2075);
    modeActivationConditionsMutable(5)->range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    EXPECT_EQ(47, modeActivationConditions(5)->range.startStep);
    EXPECT_EQ(48, modeActivationConditions(5)->range.endStep);

    modeActivationConditionsMutable(6)->modeId = (boxId_e)6;
    modeActivationConditionsMutable(6)->auxChannelIndex = AUX7 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditionsMutable(6)->range.startStep = CHANNEL_VALUE_TO_STEP(925);
    modeActivationConditionsMutable(6)->range.endStep = CHANNEL_VALUE_TO_STEP(950);

    EXPECT_EQ(1, modeActivationConditions(6)->range.startStep);
    EXPECT_EQ(2, modeActivationConditions(6)->range.endStep);

    // and
    boxBitmask_t mask;
    memset(&mask, 0, sizeof(mask));
    rcModeUpdate(&mask);

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.channelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MIDDLE;
    rcData[AUX3] = PWM_RANGE_MIN;
    rcData[AUX4] = PWM_RANGE_MAX;
    rcData[AUX5] = 899; // value lower that range minimum should be treated the same as the lowest range value
    rcData[AUX6] = 2101; // value higher than the range maximum should be treated the same as the highest range value
    rcData[AUX7] = 950; // value equal to range step upper boundary should not activate the mode

    // and
    boxBitmask_t activeBoxIds;
    memset(&activeBoxIds, 0, sizeof(boxBitmask_t));
    bitArraySet(&activeBoxIds, 0);
    bitArraySet(&activeBoxIds, 1);
    bitArraySet(&activeBoxIds, 2);
    bitArraySet(&activeBoxIds, 3);
    bitArraySet(&activeBoxIds, 4);
    bitArraySet(&activeBoxIds, 5);

    // when
    updateActivatedModes();

    // then
    for (int index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d, %d\n", index, (bool)(bitArrayGet(&activeBoxIds, index)));
#endif
        EXPECT_EQ((bool)(bitArrayGet(&activeBoxIds, index)), IS_RC_MODE_ACTIVE((boxId_e)index));
    }
}

enum {
    COUNTER_QUEUE_CONFIRMATION_BEEP,
    COUNTER_CHANGE_CONTROL_RATE_PROFILE
};
#define CALL_COUNT_ITEM_COUNT 2

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

extern "C" {
void beeperConfirmationBeeps(uint8_t) {
    callCounts[COUNTER_QUEUE_CONFIRMATION_BEEP]++;
}

void beeper(beeperMode_e mode) {
    UNUSED(mode);
}

void changeControlRateProfile(uint8_t) {
    callCounts[COUNTER_CHANGE_CONTROL_RATE_PROFILE]++;
}

}

void resetCallCounters(void) {
    memset(&callCounts, 0, sizeof(callCounts));
}

uint32_t fixedMillis;

extern "C" {
uint32_t millis(void) {
    return fixedMillis;
}
}

void resetMillis(void) {
    fixedMillis = 0;
}

#define DEFAULT_MIN_CHECK 1100
#define DEFAULT_MAX_CHECK 1900

extern "C" {
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    void configureAdjustment(uint8_t index, uint8_t auxSwitchChannelIndex, const adjustmentConfig_t *adjustmentConfig);

    extern uint8_t adjustmentStateMask;
    extern adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

    static const adjustmentConfig_t rateAdjustmentConfig = {
        .adjustmentFunction = ADJUSTMENT_RC_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { 1 }
    };
}
class RcControlsAdjustmentsTest : public ::testing::Test {
protected:
    controlRateConfig_t controlRateConfig = {
            .rcRates[FD_ROLL] = 90,
            .rcRates[FD_PITCH] = 90,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0, 0, 0},
            .dynThrPID = 0,
            .rcExpo[FD_YAW] = 0,
            .tpa_breakpoint = 0
    };

    virtual void SetUp() {
        adjustmentStateMask = 0;
        memset(&adjustmentStates, 0, sizeof(adjustmentStates));

        PG_RESET(rxConfig);
        rxConfigMutable()->mincheck = DEFAULT_MIN_CHECK;
        rxConfigMutable()->maxcheck = DEFAULT_MAX_CHECK;
        rxConfigMutable()->midrc = 1500;

        controlRateConfig.rcRates[FD_ROLL] = 90;
        controlRateConfig.rcRates[FD_PITCH] = 90;
        controlRateConfig.rcExpo[FD_ROLL] = 0;
        controlRateConfig.rcExpo[FD_PITCH] = 0;
        controlRateConfig.thrMid8 = 0;
        controlRateConfig.thrExpo8 = 0;
        controlRateConfig.rcExpo[FD_YAW] = 0;
        controlRateConfig.rates[0] = 0;
        controlRateConfig.rates[1] = 0;
        controlRateConfig.rates[2] = 0;
        controlRateConfig.dynThrPID = 0;
        controlRateConfig.tpa_breakpoint = 0;

    }
};

TEST_F(RcControlsAdjustmentsTest, processRcAdjustmentsSticksInMiddle)
{
    // given
    configureAdjustment(0, AUX3 - NON_AUX_CHANNEL_COUNT, &rateAdjustmentConfig);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 90);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 90);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 0);
    EXPECT_EQ(adjustmentStateMask, 0);
}

TEST_F(RcControlsAdjustmentsTest, processRcAdjustmentsWithRcRateFunctionSwitchUp)
{
    // given
    controlRateConfig_t controlRateConfig = {
            .rcRates[FD_ROLL] = 90,
            .rcRates[FD_PITCH] = 90,
            .rcExpo[FD_ROLL] = 0,
            .rcExpo[FD_PITCH] = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0,0,0},
            .dynThrPID = 0,
            .rcExpo[FD_YAW] = 0,
            .tpa_breakpoint = 0
    };

    // and
    PG_RESET(rxConfig);
    rxConfigMutable()->mincheck = DEFAULT_MIN_CHECK;
    rxConfigMutable()->maxcheck = DEFAULT_MAX_CHECK;
    rxConfigMutable()->midrc = 1500;

    // and
    adjustmentStateMask = 0;
    memset(&adjustmentStates, 0, sizeof(adjustmentStates));
    configureAdjustment(0, AUX3 - NON_AUX_CHANNEL_COUNT, &rateAdjustmentConfig);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    uint8_t expectedAdjustmentStateMask =
            (1 << 0);

    // and
    fixedMillis = 496;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 91);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 91);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 1);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // now pretend a short amount of time has passed, but not enough time to allow the value to have been increased
    //

    // given
    fixedMillis = 497;

    // when
    processRcAdjustments(&controlRateConfig);

    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 91);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 91);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);


    //
    // moving the switch back to the middle should immediately reset the state flag without increasing the value
    //


    // given
    rcData[AUX3] = PWM_RANGE_MIDDLE;

    // and
    fixedMillis = 498;

    // and
    expectedAdjustmentStateMask = adjustmentStateMask &
            ~(1 << 0);

    // when
    processRcAdjustments(&controlRateConfig);

    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 91);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 91);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);


    //
    // flipping the switch again, before the state reset would have occurred, allows the value to be increased again

    // given
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    expectedAdjustmentStateMask =
            (1 << 0);

    // and
    fixedMillis = 499;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 92);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 92);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 2);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // leaving the switch up, after the original timer would have reset the state should now NOT cause
    // the rate to increase, it should only increase after another 500ms from when the state was reset.
    //

    // given
    fixedMillis = 500;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 92);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 92);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // should still not be able to be increased
    //

    // given
    fixedMillis = 997;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 92);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 92);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // 500ms has now passed since the switch was returned to the middle, now that
    // switch is still in the UP position after the timer has elapses it should
    // be increased again.
    //

    // given
    fixedMillis = 998;

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRates[FD_ROLL], 93);
    EXPECT_EQ(controlRateConfig.rcRates[FD_PITCH], 93);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 3);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

static const adjustmentConfig_t rateProfileAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
    .mode = ADJUSTMENT_MODE_SELECT,
    .data = { 3 }
};

TEST_F(RcControlsAdjustmentsTest, processRcRateProfileAdjustments)
{
    // given
    int adjustmentIndex = 3;
    configureAdjustment(adjustmentIndex, AUX4 - NON_AUX_CHANNEL_COUNT, &rateProfileAdjustmentConfig);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX4] = PWM_RANGE_MAX;

    // and
    uint8_t expectedAdjustmentStateMask =
            (1 << adjustmentIndex);

    // when
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 1);
    EXPECT_EQ(CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE), 1);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

static const adjustmentConfig_t pidPitchAndRollPAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

static const adjustmentConfig_t pidPitchAndRollIAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

static const adjustmentConfig_t pidPitchAndRollDAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

static const adjustmentConfig_t pidYawPAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_P,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

static const adjustmentConfig_t pidYawIAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_I,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

static const adjustmentConfig_t pidYawDAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_D,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { 1 }
};

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController0)
{
    // given
    pidProfile_t pidProfile;
    memset(&pidProfile, 0, sizeof (pidProfile));
    pidProfile.pid[PID_PITCH].P = 0;
    pidProfile.pid[PID_PITCH].I = 10;
    pidProfile.pid[PID_PITCH].D = 20;
    pidProfile.pid[PID_ROLL].P = 5;
    pidProfile.pid[PID_ROLL].I = 15;
    pidProfile.pid[PID_ROLL].D = 25;
    pidProfile.pid[PID_YAW].P = 7;
    pidProfile.pid[PID_YAW].I = 17;
    pidProfile.pid[PID_YAW].D = 27;
    useAdjustmentConfig(&pidProfile);
    // and
    controlRateConfig_t controlRateConfig;
    memset(&controlRateConfig, 0, sizeof (controlRateConfig));

    configureAdjustment(0, AUX1 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollPAdjustmentConfig);
    configureAdjustment(1, AUX2 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollIAdjustmentConfig);
    configureAdjustment(2, AUX3 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollDAdjustmentConfig);
    configureAdjustment(3, AUX1 - NON_AUX_CHANNEL_COUNT, &pidYawPAdjustmentConfig);
    configureAdjustment(4, AUX2 - NON_AUX_CHANNEL_COUNT, &pidYawIAdjustmentConfig);
    configureAdjustment(5, AUX3 - NON_AUX_CHANNEL_COUNT, &pidYawDAdjustmentConfig);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MAX;
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    uint8_t expectedAdjustmentStateMask =
            (1 << 0) |
            (1 << 1) |
            (1 << 2) |
            (1 << 3) |
            (1 << 4) |
            (1 << 5);

    // when
    useRcControlsConfig(&pidProfile);
    processRcAdjustments(&controlRateConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
    EXPECT_EQ(1,  pidProfile.pid[PID_PITCH].P);
    EXPECT_EQ(11, pidProfile.pid[PID_PITCH].I);
    EXPECT_EQ(21, pidProfile.pid[PID_PITCH].D);
    EXPECT_EQ(6,  pidProfile.pid[PID_ROLL].P);
    EXPECT_EQ(16, pidProfile.pid[PID_ROLL].I);
    EXPECT_EQ(26, pidProfile.pid[PID_ROLL].D);
    EXPECT_EQ(8,  pidProfile.pid[PID_YAW].P);
    EXPECT_EQ(18, pidProfile.pid[PID_YAW].I);
    EXPECT_EQ(28, pidProfile.pid[PID_YAW].D);
}

#if 0 // only one PID controller

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController2)
{
    // given
    pidProfile_t pidProfile;
    memset(&pidProfile, 0, sizeof (pidProfile));
    pidProfile.pidController = 2;
    pidProfile.P_f[PIDPITCH] = 0.0f;
    pidProfile.P_f[PIDROLL] = 5.0f;
    pidProfile.P_f[PIDYAW] = 7.0f;
    pidProfile.I_f[PIDPITCH] = 10.0f;
    pidProfile.I_f[PIDROLL] = 15.0f;
    pidProfile.I_f[PIDYAW] = 17.0f;
    pidProfile.D_f[PIDPITCH] = 20.0f;
    pidProfile.D_f[PIDROLL] = 25.0f;
    pidProfile.D_f[PIDYAW] = 27.0f;
    useAdjustmentConfig(&pidProfile);

    // and
    controlRateConfig_t controlRateConfig;
    memset(&controlRateConfig, 0, sizeof (controlRateConfig));

    configureAdjustment(0, AUX1 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollPAdjustmentConfig);
    configureAdjustment(1, AUX2 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollIAdjustmentConfig);
    configureAdjustment(2, AUX3 - NON_AUX_CHANNEL_COUNT, &pidPitchAndRollDAdjustmentConfig);
    configureAdjustment(3, AUX1 - NON_AUX_CHANNEL_COUNT, &pidYawPAdjustmentConfig);
    configureAdjustment(4, AUX2 - NON_AUX_CHANNEL_COUNT, &pidYawIAdjustmentConfig);
    configureAdjustment(5, AUX3 - NON_AUX_CHANNEL_COUNT, &pidYawDAdjustmentConfig);

    // and
    for (int index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // and
    rcData[AUX1] = PWM_RANGE_MAX;
    rcData[AUX2] = PWM_RANGE_MAX;
    rcData[AUX3] = PWM_RANGE_MAX;

    // and
    uint8_t expectedAdjustmentStateMask =
            (1 << 0) |
            (1 << 1) |
            (1 << 2) |
            (1 << 3) |
            (1 << 4) |
            (1 << 5);

    // when
    useRcControlsConfig(&escAndServoConfig, &pidProfile);
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
    EXPECT_EQ(0.01f, pidProfile.P_f[PIDPITCH]);
    EXPECT_EQ(5.01f, pidProfile.P_f[PIDROLL]);
    EXPECT_EQ(7.01f, pidProfile.P_f[PIDYAW]);
    EXPECT_EQ(10.01f, pidProfile.I_f[PIDPITCH]);
    EXPECT_EQ(15.01f, pidProfile.I_f[PIDROLL]);
    EXPECT_EQ(17.01f, pidProfile.I_f[PIDYAW]);
    EXPECT_EQ(20.001f, pidProfile.D_f[PIDPITCH]);
    EXPECT_EQ(25.001f, pidProfile.D_f[PIDROLL]);
    EXPECT_EQ(27.001f, pidProfile.D_f[PIDYAW]);

}

#endif

extern "C" {
void saveConfigAndNotify(void) {}
void initRcProcessing(void) {}
void changePidProfile(uint8_t) {}
void pidInitConfig(const pidProfile_t *) {}
void accSetCalibrationCycles(uint16_t) {}
void gyroStartCalibration(void) {}
void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
bool feature(uint32_t) { return false;}
bool sensors(uint32_t) { return false;}
void tryArm(void) {}
void disarm(void) {}
void dashboardDisablePageCycling() {}
void dashboardEnablePageCycling() {}

bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }

uint8_t getCurrentControlRateProfileIndex(void) {
    return 0;
}
void GPS_reset_home_position(void) {}
void baroSetCalibrationCycles(uint16_t) {}

void blackboxLogEvent(FlightLogEvent, flightLogEventData_t *) {}

bool cmsInMenu = false;
uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxRuntimeConfig_t rxRuntimeConfig;
PG_REGISTER(blackboxConfig_t, blackboxConfig, PG_BLACKBOX_CONFIG, 0);
PG_REGISTER(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 2);
void resetArmingDisabled(void) {}
timeDelta_t getTaskDeltaTime(cfTaskId_e) { return 20000; }
}
armingDisableFlags_e getArmingDisableFlags(void) {
    return (armingDisableFlags_e) 0;
}