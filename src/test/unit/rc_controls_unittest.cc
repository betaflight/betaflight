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

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "io/beeper.h"
    #include "io/escservo.h"
    #include "io/rc_controls.h"

    #include "rx/rx.h"

    #include "flight/pid.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
extern void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfig, pidProfile_t *pidProfile);
}

class RcControlsModesTest : public ::testing::Test {
protected:
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];

    virtual void SetUp() {
        memset(&modeActivationConditions, 0, sizeof(modeActivationConditions));
    }
};

TEST_F(RcControlsModesTest, updateActivatedModesWithAllInputsAtMidde)
{
    // given
    rcModeActivationMask = 0;

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.auxChannelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // when
    updateActivatedModes(modeActivationConditions);

    // then
    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(false, IS_RC_MODE_ACTIVE(index));
    }
}

TEST_F(RcControlsModesTest, updateActivatedModesUsingValidAuxConfigurationAndRXValues)
{
    // given
    modeActivationConditions[0].modeId = (boxId_e)0;
    modeActivationConditions[0].auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[0].range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditions[0].range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditions[1].modeId = (boxId_e)1;
    modeActivationConditions[1].auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[1].range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditions[1].range.endStep = CHANNEL_VALUE_TO_STEP(1700);

    modeActivationConditions[2].modeId = (boxId_e)2;
    modeActivationConditions[2].auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[2].range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditions[2].range.endStep = CHANNEL_VALUE_TO_STEP(1200);

    modeActivationConditions[3].modeId = (boxId_e)3;
    modeActivationConditions[3].auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[3].range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditions[3].range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    modeActivationConditions[4].modeId = (boxId_e)4;
    modeActivationConditions[4].auxChannelIndex = AUX5 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[4].range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditions[4].range.endStep = CHANNEL_VALUE_TO_STEP(925);

    EXPECT_EQ(0, modeActivationConditions[4].range.startStep);
    EXPECT_EQ(1, modeActivationConditions[4].range.endStep);

    modeActivationConditions[5].modeId = (boxId_e)5;
    modeActivationConditions[5].auxChannelIndex = AUX6 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[5].range.startStep = CHANNEL_VALUE_TO_STEP(2075);
    modeActivationConditions[5].range.endStep = CHANNEL_VALUE_TO_STEP(2100);

    EXPECT_EQ(47, modeActivationConditions[5].range.startStep);
    EXPECT_EQ(48, modeActivationConditions[5].range.endStep);

    modeActivationConditions[6].modeId = (boxId_e)6;
    modeActivationConditions[6].auxChannelIndex = AUX7 - NON_AUX_CHANNEL_COUNT;
    modeActivationConditions[6].range.startStep = CHANNEL_VALUE_TO_STEP(925);
    modeActivationConditions[6].range.endStep = CHANNEL_VALUE_TO_STEP(950);

    EXPECT_EQ(1, modeActivationConditions[6].range.startStep);
    EXPECT_EQ(2, modeActivationConditions[6].range.endStep);

    // and
    rcModeActivationMask = 0;

    // and
    memset(&rxRuntimeConfig, 0, sizeof(rxRuntimeConfig_t));
    rxRuntimeConfig.auxChannelCount = MAX_SUPPORTED_RC_CHANNEL_COUNT - NON_AUX_CHANNEL_COUNT;

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
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
    uint32_t expectedMask = 0;
    expectedMask |= (1 << 0);
    expectedMask |= (1 << 1);
    expectedMask |= (1 << 2);
    expectedMask |= (1 << 3);
    expectedMask |= (1 << 4);
    expectedMask |= (1 << 5);
    expectedMask |= (0 << 6);

    // when
    updateActivatedModes(modeActivationConditions);

    // then
    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(expectedMask & (1 << index), rcModeActivationMask & (1 << index));
    }
}

enum {
    COUNTER_GENERATE_PITCH_ROLL_CURVE = 0,
    COUNTER_QUEUE_CONFIRMATION_BEEP,
    COUNTER_CHANGE_CONTROL_RATE_PROFILE
};
#define CALL_COUNT_ITEM_COUNT 3

static int callCounts[CALL_COUNT_ITEM_COUNT];

#define CALL_COUNTER(item) (callCounts[item])

extern "C" {
void generatePitchRollCurve(controlRateConfig_t *) {
    callCounts[COUNTER_GENERATE_PITCH_ROLL_CURVE]++;
}

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

rxConfig_t rxConfig;

extern uint8_t adjustmentStateMask;
extern adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

static const adjustmentConfig_t rateAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_RC_RATE,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

class RcControlsAdjustmentsTest : public ::testing::Test {
protected:
    controlRateConfig_t controlRateConfig = {
            .rcRate8 = 90,
            .rcExpo8 = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0, 0, 0},
            .dynThrPID = 0,
            .rcYawExpo8 = 0,
            .tpa_breakpoint = 0
    };

    virtual void SetUp() {
        adjustmentStateMask = 0;
        memset(&adjustmentStates, 0, sizeof(adjustmentStates));

        memset(&rxConfig, 0, sizeof (rxConfig));
        rxConfig.mincheck = DEFAULT_MIN_CHECK;
        rxConfig.maxcheck = DEFAULT_MAX_CHECK;
        rxConfig.midrc = 1500;

        controlRateConfig.rcRate8 = 90;
        controlRateConfig.rcExpo8 = 0;
        controlRateConfig.thrMid8 = 0;
        controlRateConfig.thrExpo8 = 0;
        controlRateConfig.rcYawExpo8 = 0;
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
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // when
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 90);
    EXPECT_EQ(CALL_COUNTER(COUNTER_GENERATE_PITCH_ROLL_CURVE), 0);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 0);
    EXPECT_EQ(adjustmentStateMask, 0);
}

TEST_F(RcControlsAdjustmentsTest, processRcAdjustmentsWithRcRateFunctionSwitchUp)
{
    // given
    controlRateConfig_t controlRateConfig = {
            .rcRate8 = 90,
            .rcExpo8 = 0,
            .thrMid8 = 0,
            .thrExpo8 = 0,
            .rates = {0,0,0},
            .dynThrPID = 0,
            .rcYawExpo8 = 0,
            .tpa_breakpoint = 0
    };

    // and
    memset(&rxConfig, 0, sizeof (rxConfig));
    rxConfig.mincheck = DEFAULT_MIN_CHECK;
    rxConfig.maxcheck = DEFAULT_MAX_CHECK;
    rxConfig.midrc = 1500;

    // and
    adjustmentStateMask = 0;
    memset(&adjustmentStates, 0, sizeof(adjustmentStates));
    configureAdjustment(0, AUX3 - NON_AUX_CHANNEL_COUNT, &rateAdjustmentConfig);

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
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
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 91);
    EXPECT_EQ(CALL_COUNTER(COUNTER_GENERATE_PITCH_ROLL_CURVE), 1);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 1);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);


    //
    // now pretend a short amount of time has passed, but not enough time to allow the value to have been increased
    //

    // given
    fixedMillis = 497;

    // when
    processRcAdjustments(&controlRateConfig, &rxConfig);

    EXPECT_EQ(controlRateConfig.rcRate8, 91);
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
    processRcAdjustments(&controlRateConfig, &rxConfig);

    EXPECT_EQ(controlRateConfig.rcRate8, 91);
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
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 92);
    EXPECT_EQ(CALL_COUNTER(COUNTER_GENERATE_PITCH_ROLL_CURVE), 2);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 2);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // leaving the switch up, after the original timer would have reset the state should now NOT cause
    // the rate to increase, it should only increase after another 500ms from when the state was reset.
    //

    // given
    fixedMillis = 500;

    // when
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 92);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // should still not be able to be increased
    //

    // given
    fixedMillis = 997;

    // when
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 92);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // 500ms has now passed since the switch was returned to the middle, now that
    // switch is still in the UP position after the timer has elapses it should
    // be increased again.
    //

    // given
    fixedMillis = 998;

    // when
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 93);
    EXPECT_EQ(CALL_COUNTER(COUNTER_GENERATE_PITCH_ROLL_CURVE), 3);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 3);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

static const adjustmentConfig_t rateProfileAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
    .mode = ADJUSTMENT_MODE_SELECT,
    .data = { { 3 } }
};

TEST_F(RcControlsAdjustmentsTest, processRcRateProfileAdjustments)
{
    // given
    int adjustmentIndex = 3;
    configureAdjustment(adjustmentIndex, AUX4 - NON_AUX_CHANNEL_COUNT, &rateProfileAdjustmentConfig);

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
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
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 1);
    EXPECT_EQ(CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE), 1);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

static const adjustmentConfig_t pidPitchAndRollPAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

static const adjustmentConfig_t pidPitchAndRollIAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

static const adjustmentConfig_t pidPitchAndRollDAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

static const adjustmentConfig_t pidYawPAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_P,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

static const adjustmentConfig_t pidYawIAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_I,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

static const adjustmentConfig_t pidYawDAdjustmentConfig = {
    .adjustmentFunction = ADJUSTMENT_YAW_D,
    .mode = ADJUSTMENT_MODE_STEP,
    .data = { { 1 } }
};

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController0)
{
    // given
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    memset(&modeActivationConditions, 0, sizeof (modeActivationConditions));

    escAndServoConfig_t escAndServoConfig;
    memset(&escAndServoConfig, 0, sizeof (escAndServoConfig));

    pidProfile_t pidProfile;
    memset(&pidProfile, 0, sizeof (pidProfile));
    pidProfile.pidController = 0;
    pidProfile.P8[PIDPITCH] = 0;
    pidProfile.P8[PIDROLL] = 5;
    pidProfile.P8[YAW] = 7;
    pidProfile.I8[PIDPITCH] = 10;
    pidProfile.I8[PIDROLL] = 15;
    pidProfile.I8[YAW] = 17;
    pidProfile.D8[PIDPITCH] = 20;
    pidProfile.D8[PIDROLL] = 25;
    pidProfile.D8[YAW] = 27;

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
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
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
    useRcControlsConfig(modeActivationConditions, &escAndServoConfig, &pidProfile);
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
    EXPECT_EQ(1, pidProfile.P8[PIDPITCH]);
    EXPECT_EQ(6, pidProfile.P8[PIDROLL]);
    EXPECT_EQ(8, pidProfile.P8[YAW]);
    EXPECT_EQ(11, pidProfile.I8[PIDPITCH]);
    EXPECT_EQ(16, pidProfile.I8[PIDROLL]);
    EXPECT_EQ(18, pidProfile.I8[YAW]);
    EXPECT_EQ(21, pidProfile.D8[PIDPITCH]);
    EXPECT_EQ(26, pidProfile.D8[PIDROLL]);
    EXPECT_EQ(28, pidProfile.D8[YAW]);
}

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController2)
{
    // given
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    memset(&modeActivationConditions, 0, sizeof (modeActivationConditions));

    escAndServoConfig_t escAndServoConfig;
    memset(&escAndServoConfig, 0, sizeof (escAndServoConfig));

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
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
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
    useRcControlsConfig(modeActivationConditions, &escAndServoConfig, &pidProfile);
    processRcAdjustments(&controlRateConfig, &rxConfig);

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
    EXPECT_EQ(0.1f, pidProfile.P_f[PIDPITCH]);
    EXPECT_EQ(5.1f, pidProfile.P_f[PIDROLL]);
    EXPECT_EQ(7.1f, pidProfile.P_f[PIDYAW]);
    EXPECT_EQ(10.01f, pidProfile.I_f[PIDPITCH]);
    EXPECT_EQ(15.01f, pidProfile.I_f[PIDROLL]);
    EXPECT_EQ(17.01f, pidProfile.I_f[PIDYAW]);
    EXPECT_EQ(20.001f, pidProfile.D_f[PIDPITCH]);
    EXPECT_EQ(25.001f, pidProfile.D_f[PIDROLL]);
    EXPECT_EQ(27.001f, pidProfile.D_f[PIDYAW]);

}

extern "C" {
void saveConfigAndNotify(void) {}
void generateThrottleCurve(controlRateConfig_t *, escAndServoConfig_t *) {}
void changeProfile(uint8_t) {}
void accSetCalibrationCycles(uint16_t) {}
void gyroSetCalibrationCycles(uint16_t) {}
void applyAndSaveAccelerometerTrimsDelta(rollAndPitchTrims_t*) {}
void handleInflightCalibrationStickPosition(void) {}
bool feature(uint32_t) { return false;}
bool sensors(uint32_t) { return false;}
void mwArm(void) {}
void mwDisarm(void) {}
void displayDisablePageCycling() {}
void displayEnablePageCycling() {}

bool failsafeIsActive() { return false; }
bool rxIsReceivingSignal() { return true; }

uint8_t getCurrentControlRateProfile(void) {
    return 0;
}
void GPS_reset_home_position(void) {}
void baroSetCalibrationCycles(uint16_t) {}

uint8_t armingFlags = 0;
int16_t heading;
uint8_t stateFlags = 0;
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];
rxRuntimeConfig_t rxRuntimeConfig;
}
