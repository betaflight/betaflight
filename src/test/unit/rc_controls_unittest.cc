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
    #include <platform.h>
    #include "build/build_config.h"

    #include "common/maths.h"
    #include "common/axis.h"

    #include "config/parameter_group.h"
    #include "config/parameter_group_ids.h"
    #include "config/profile.h"

    #include "drivers/sensor.h"
    #include "drivers/accgyro.h"

    #include "sensors/sensors.h"
    #include "sensors/acceleration.h"

    #include "io/beeper.h"
    #include "io/motor_and_servo.h"
    #include "fc/rc_controls.h"
    #include "fc/rate_profile.h"
    #include "fc/rc_adjustments.h"

    #include "rx/rx.h"


    #include "flight/pid.h"

    void useRcControlsConfig(modeActivationCondition_t *);

    PG_REGISTER_PROFILE(pidProfile_t, pidProfile, PG_PID_PROFILE, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);
    PG_REGISTER(motorAndServoConfig_t, motorAndServoConfig, PG_MOTOR_AND_SERVO_CONFIG, 0);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
extern void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions);
extern uint32_t rcModeActivationMask;
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

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // when
    rcModeUpdateActivated(modeActivationConditions);

    // then
    for (index = 0; index < CHECKBOX_ITEM_COUNT; index++) {
#ifdef DEBUG_RC_CONTROLS
        printf("iteration: %d\n", index);
#endif
        EXPECT_EQ(false, rcModeIsActive((boxId_e)index));
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
    rcModeUpdateActivated(modeActivationConditions);

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

extern uint8_t adjustmentStateMask;
extern adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];


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

    adjustmentRange_t adjustmentRange = {
            .auxChannelIndex = 0,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_NONE,
            .auxSwitchChannelIndex = 0,
            .adjustmentIndex = 0
    };

    virtual void SetUp() {
        adjustmentStateMask = 0;
        memset(&adjustmentStates, 0, sizeof(adjustmentStates));

        memset(motorAndServoConfig(), 0, sizeof(*motorAndServoConfig()));

        memset(rxConfig(), 0, sizeof(*rxConfig()));
        rxConfig()->mincheck = DEFAULT_MIN_CHECK;
        rxConfig()->maxcheck = DEFAULT_MAX_CHECK;
        rxConfig()->midrc = 1500;

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
    adjustmentRange.adjustmentFunction = ADJUSTMENT_RC_RATE;
    adjustmentRange.auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    adjustmentRange.auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    configureAdjustmentState(&adjustmentRange);

    // and
    uint8_t index;
    for (index = AUX1; index < MAX_SUPPORTED_RC_CHANNEL_COUNT; index++) {
        rcData[index] = PWM_RANGE_MIDDLE;
    }

    // and
    resetCallCounters();
    resetMillis();

    // when
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    memset(rxConfig(), 0, sizeof (*rxConfig()));
    rxConfig()->mincheck = DEFAULT_MIN_CHECK;
    rxConfig()->maxcheck = DEFAULT_MAX_CHECK;
    rxConfig()->midrc = 1500;

    // and
    adjustmentStateMask = 0;
    memset(&adjustmentStates, 0, sizeof(adjustmentStates));
    
    adjustmentRange.adjustmentFunction = ADJUSTMENT_RC_RATE;
    adjustmentRange.auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    adjustmentRange.auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT;
    configureAdjustmentState(&adjustmentRange);

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
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    processRcAdjustments(&controlRateConfig, rxConfig());

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 92);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    //
    // should still not be able to be increased
    //

    // given
    fixedMillis = 997;

    // when
    processRcAdjustments(&controlRateConfig, rxConfig());

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
    processRcAdjustments(&controlRateConfig, rxConfig());

    // then
    EXPECT_EQ(controlRateConfig.rcRate8, 93);
    EXPECT_EQ(CALL_COUNTER(COUNTER_GENERATE_PITCH_ROLL_CURVE), 3);
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 3);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

TEST_F(RcControlsAdjustmentsTest, processRcRateProfileAdjustments)
{
    // given
    int adjustmentIndex = 3;

    adjustmentRange.adjustmentFunction = ADJUSTMENT_RATE_PROFILE;
    adjustmentRange.auxChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    adjustmentRange.auxSwitchChannelIndex = AUX4 - NON_AUX_CHANNEL_COUNT;
    adjustmentRange.adjustmentIndex = adjustmentIndex;

    configureAdjustmentState(&adjustmentRange);

    adjustmentRange.adjustmentFunction = ADJUSTMENT_RC_RATE;
    configureAdjustmentState(&adjustmentRange);

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
    processRcAdjustments(&controlRateConfig, rxConfig());

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 1);
    EXPECT_EQ(CALL_COUNTER(COUNTER_CHANGE_CONTROL_RATE_PROFILE), 1);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);
}

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController0) // uses integers
{
    // given
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    memset(&modeActivationConditions, 0, sizeof (modeActivationConditions));

    memset(pidProfile(), 0, sizeof (*pidProfile()));
    pidProfile()->pidController = 0;
    pidProfile()->P8[PIDPITCH] = 0;
    pidProfile()->P8[PIDROLL] = 5;
    pidProfile()->P8[YAW] = 7;
    pidProfile()->I8[PIDPITCH] = 10;
    pidProfile()->I8[PIDROLL] = 15;
    pidProfile()->I8[YAW] = 17;
    pidProfile()->D8[PIDPITCH] = 20;
    pidProfile()->D8[PIDROLL] = 25;
    pidProfile()->D8[YAW] = 27;

    // and
    controlRateConfig_t controlRateConfig;
    memset(&controlRateConfig, 0, sizeof (controlRateConfig));

    adjustmentRange_t adjustmentRange1 = {
            .auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
            .auxSwitchChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 0
    };
    configureAdjustmentState(&adjustmentRange1);

    adjustmentRange_t adjustmentRange2 = {
            .auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
            .auxSwitchChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 1
    };
    configureAdjustmentState(&adjustmentRange2);

    adjustmentRange_t adjustmentRange3 = {
            .auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
            .auxSwitchChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 2
    };
    configureAdjustmentState(&adjustmentRange3);

    adjustmentRange_t adjustmentRange4 = {
            .auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_P,
            .auxSwitchChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 3
    };
    configureAdjustmentState(&adjustmentRange4);

    adjustmentRange_t adjustmentRange5 = {
            .auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_I,
            .auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 4
    };
    configureAdjustmentState(&adjustmentRange5);

    adjustmentRange_t adjustmentRange6 = {
            .auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_D,
            .auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 5
    };
    configureAdjustmentState(&adjustmentRange6);

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
    useRcControlsConfig(modeActivationConditions);
    processRcAdjustments(&controlRateConfig, rxConfig());

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
    EXPECT_EQ(1, pidProfile()->P8[PIDPITCH]);
    EXPECT_EQ(6, pidProfile()->P8[PIDROLL]);
    EXPECT_EQ(8, pidProfile()->P8[YAW]);
    EXPECT_EQ(11, pidProfile()->I8[PIDPITCH]);
    EXPECT_EQ(16, pidProfile()->I8[PIDROLL]);
    EXPECT_EQ(18, pidProfile()->I8[YAW]);
    EXPECT_EQ(21, pidProfile()->D8[PIDPITCH]);
    EXPECT_EQ(26, pidProfile()->D8[PIDROLL]);
    EXPECT_EQ(28, pidProfile()->D8[YAW]);
}

TEST_F(RcControlsAdjustmentsTest, processPIDIncreasePidController2) // uses floats
{
    // given
    modeActivationCondition_t modeActivationConditions[MAX_MODE_ACTIVATION_CONDITION_COUNT];
    memset(&modeActivationConditions, 0, sizeof (modeActivationConditions));

    memset(pidProfile(), 0, sizeof (*pidProfile()));
    pidProfile()->pidController = 2;
/* !!TODO - this is temporarily removed, asses permanent removal
    pidProfile()->P_f[PIDPITCH] = 0.0f;
    pidProfile()->P_f[PIDROLL] = 5.0f;
    pidProfile()->P_f[PIDYAW] = 7.0f;
    pidProfile()->I_f[PIDPITCH] = 10.0f;
    pidProfile()->I_f[PIDROLL] = 15.0f;
    pidProfile()->I_f[PIDYAW] = 17.0f;
    pidProfile()->D_f[PIDPITCH] = 20.0f;
    pidProfile()->D_f[PIDROLL] = 25.0f;
    pidProfile()->D_f[PIDYAW] = 27.0f;
*/
    // and
    controlRateConfig_t controlRateConfig;
    memset(&controlRateConfig, 0, sizeof (controlRateConfig));

    adjustmentRange_t adjustmentRange1 = {
            .auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
            .auxSwitchChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 0
    };
    configureAdjustmentState(&adjustmentRange1);

    adjustmentRange_t adjustmentRange2 = {
            .auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
            .auxSwitchChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 1
    };
    configureAdjustmentState(&adjustmentRange2);

    adjustmentRange_t adjustmentRange3 = {
            .auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
            .auxSwitchChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 2
    };
    configureAdjustmentState(&adjustmentRange3);

    adjustmentRange_t adjustmentRange4 = {
            .auxChannelIndex = AUX1 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_P,
            .auxSwitchChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 3
    };
    configureAdjustmentState(&adjustmentRange4);

    adjustmentRange_t adjustmentRange5 = {
            .auxChannelIndex = AUX2 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_I,
            .auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 4
    };
    configureAdjustmentState(&adjustmentRange5);

    adjustmentRange_t adjustmentRange6 = {
            .auxChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .range = { .startStep = 0, .endStep = 48 },
            .adjustmentFunction = ADJUSTMENT_YAW_D,
            .auxSwitchChannelIndex = AUX3 - NON_AUX_CHANNEL_COUNT,
            .adjustmentIndex = 5
    };
    configureAdjustmentState(&adjustmentRange6);

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
    useRcControlsConfig(modeActivationConditions);
    processRcAdjustments(&controlRateConfig, rxConfig());

    // then
    EXPECT_EQ(CALL_COUNTER(COUNTER_QUEUE_CONFIRMATION_BEEP), 6);
    EXPECT_EQ(adjustmentStateMask, expectedAdjustmentStateMask);

    // and
/* !!TODO - this is temporarily removed, asses permanent removal
    EXPECT_EQ(0.1f, pidProfile()->P_f[PIDPITCH]);
    EXPECT_EQ(5.1f, pidProfile()->P_f[PIDROLL]);
    EXPECT_EQ(7.1f, pidProfile()->P_f[PIDYAW]);
    EXPECT_EQ(10.01f, pidProfile()->I_f[PIDPITCH]);
    EXPECT_EQ(15.01f, pidProfile()->I_f[PIDROLL]);
    EXPECT_EQ(17.01f, pidProfile()->I_f[PIDYAW]);
    EXPECT_EQ(20.001f, pidProfile()->D_f[PIDPITCH]);
    EXPECT_EQ(25.001f, pidProfile()->D_f[PIDROLL]);
    EXPECT_EQ(27.001f, pidProfile()->D_f[PIDYAW]);
*/

}

extern "C" {
void saveConfigAndNotify(void) {}
void generateThrottleCurve(controlRateConfig_t *, motorAndServoConfig_t *) {}
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
