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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/system.h"

#include "flight/flight.h"

#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "mw.h"

#include "rx/rx.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"

static escAndServoConfig_t *escAndServoConfig;
static pidProfile_t *pidProfile;

static bool isUsingSticksToArm = true;

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e

bool areSticksInApModePosition(uint16_t ap_mode)
{
    return abs(rcCommand[ROLL]) < ap_mode && abs(rcCommand[PITCH]) < ap_mode;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    if (feature(FEATURE_3D) && (rcData[THROTTLE] > (rxConfig->midrc - deadband3d_throttle) && rcData[THROTTLE] < (rxConfig->midrc + deadband3d_throttle)))
        return THROTTLE_LOW;
    else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->mincheck))
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}



void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool retarded_arm, bool disarm_kill_switch)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;

    // ------------------ STICKS COMMAND HANDLER --------------------
    // checking sticks positions
    for (i = 0; i < 4; i++) {
        stTmp >>= 2;
        if (rcData[i] > rxConfig->mincheck)
            stTmp |= 0x80;  // check for MIN
        if (rcData[i] < rxConfig->maxcheck)
            stTmp |= 0x40;  // check for MAX
    }
    if (stTmp == rcSticks) {
        if (rcDelayCommand < 250)
            rcDelayCommand++;
    } else
        rcDelayCommand = 0;
    rcSticks = stTmp;

    // perform actions
    if (!isUsingSticksToArm) {

        if (IS_RC_MODE_ACTIVE(BOXARM)) {
            // Arming via ARM BOX
            if (throttleStatus == THROTTLE_LOW) {
                if (ARMING_FLAG(OK_TO_ARM)) {
                    mwArm();
                }
            }
        } else {
            // Disarming via ARM BOX
            if (ARMING_FLAG(ARMED)) {
                if (disarm_kill_switch) {
                    mwDisarm();
                } else if (throttleStatus == THROTTLE_LOW) {
                    mwDisarm();
                }
            }
        }
    }

    if (rcDelayCommand != 20) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed

        if (isUsingSticksToArm) {
            // Disarm on throttle down + yaw
            if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)
                mwDisarm();

            // Disarm on roll (only when retarded_arm is enabled)
            if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO))
                mwDisarm();
        }
        return;
    }

    // actions during not armed
    i = 0;

    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
        // GYRO calibration
        gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);

#ifdef GPS
        if (feature(FEATURE_GPS)) {
            GPS_reset_home_position();
        }
#endif

#ifdef BARO
        if (sensors(SENSOR_BARO))
            baroSetCalibrationCycles(10); // calibrate baro to new ground level (10 * 25 ms = ~250 ms non blocking)
#endif

        if (!sensors(SENSOR_MAG))
            heading = 0; // reset heading to zero after gyro calibration

        return;
    }

    if (feature(FEATURE_INFLIGHT_ACC_CAL) && (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_HI)) {
        // Inflight ACC Calibration
        handleInflightCalibrationStickPosition();
        return;
    }

    // Multiple configuration profiles
    if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_LO)          // ROLL left  -> Profile 1
        i = 1;
    else if (rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_CE)     // PITCH up   -> Profile 2
        i = 2;
    else if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)     // ROLL right -> Profile 3
        i = 3;
    if (i) {
        changeProfile(i - 1);
        return;
    }

    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_HI) {
        saveConfigAndNotify();
    }

    if (isUsingSticksToArm) {

        if (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) {
            // Arm via YAW
            mwArm();
            return;
        }

        if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_HI)) {
            // Arm via ROLL
            mwArm();
            return;
        }
    }

    if (rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_CE) {
        // Calibrating Acc
        accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
        return;
    }


    if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_CE) {
        // Calibrating Mag
        ENABLE_STATE(CALIBRATE_MAG);
        return;
    }


    // Accelerometer Trim

    rollAndPitchTrims_t accelerometerTrimsDelta;
    memset(&accelerometerTrimsDelta, 0, sizeof(accelerometerTrimsDelta));

    bool shouldApplyRollAndPitchTrimDelta = false;
    if (rcSticks == THR_HI + YAW_CE + PIT_HI + ROL_CE) {
        accelerometerTrimsDelta.values.pitch = 2;
        shouldApplyRollAndPitchTrimDelta = true;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_LO + ROL_CE) {
        accelerometerTrimsDelta.values.pitch = -2;
        shouldApplyRollAndPitchTrimDelta = true;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_HI) {
        accelerometerTrimsDelta.values.roll = 2;
        shouldApplyRollAndPitchTrimDelta = true;
    } else if (rcSticks == THR_HI + YAW_CE + PIT_CE + ROL_LO) {
        accelerometerTrimsDelta.values.roll = -2;
        shouldApplyRollAndPitchTrimDelta = true;
    }
    if (shouldApplyRollAndPitchTrimDelta) {
        applyAndSaveAccelerometerTrimsDelta(&accelerometerTrimsDelta);
        rcDelayCommand = 0; // allow autorepetition
        return;
    }
}

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range) {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= 900 + (range->startStep * 25) &&
            channelValue < 900 + (range->endStep * 25));
}

void updateActivatedModes(modeActivationCondition_t *modeActivationConditions)
{
    rcModeActivationMask = 0;

    uint8_t index;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];

        if (isRangeActive(modeActivationCondition->auxChannelIndex, &modeActivationCondition->range)) {
            ACTIVATE_RC_MODE(modeActivationCondition->modeId);
        }
    }
}

uint8_t adjustmentStateMask = 0;

#define MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex) adjustmentStateMask |= (1 << adjustmentIndex)
#define MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex) adjustmentStateMask &= ~(1 << adjustmentIndex)

#define IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex) (adjustmentStateMask & (1 << adjustmentIndex))

// sync with adjustmentFunction_e
static const adjustmentConfig_t defaultAdjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT - 1] = {
    {
        .adjustmentFunction = ADJUSTMENT_RC_RATE,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_RC_EXPO,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_THROTTLE_EXPO,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_RATE,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_RATE,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_P,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_I,
        .step = 1
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_D,
        .step = 1
    }
};

#define ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET 1


typedef struct adjustmentState_s {
    uint8_t auxChannelIndex;
    uint8_t adjustmentFunction;
    uint8_t step;
    uint32_t timeoutAt;
} adjustmentState_t;

static adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

void configureAdjustment(uint8_t index, uint8_t auxSwitchChannelIndex, const adjustmentConfig_t *adjustmentConfig) {
    adjustmentState_t *adjustmentState = &adjustmentStates[index];

    if (adjustmentState->adjustmentFunction == adjustmentConfig->adjustmentFunction) {
        // already configured
        return;
    }
    adjustmentState->auxChannelIndex = auxSwitchChannelIndex;
    adjustmentState->adjustmentFunction = adjustmentConfig->adjustmentFunction;
    adjustmentState->step = adjustmentConfig->step;
    adjustmentState->timeoutAt = 0;

    MARK_ADJUSTMENT_FUNCTION_AS_READY(index);
}

void applyAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta) {
    int newValue;

    if (delta > 0) {
        queueConfirmationBeep(2);
    } else {
        queueConfirmationBeep(1);
    }
    switch(adjustmentFunction) {
        case ADJUSTMENT_RC_RATE:
            newValue = (int)controlRateConfig->rcRate8 + delta;
            controlRateConfig->rcRate8 = constrain(newValue, 0, 250); // FIXME magic numbers repeated in serial_cli.c
            generatePitchRollCurve(controlRateConfig);
        break;
        case ADJUSTMENT_RC_EXPO:
            newValue = (int)controlRateConfig->rcExpo8 + delta;
            controlRateConfig->rcExpo8 = constrain(newValue, 0, 100); // FIXME magic numbers repeated in serial_cli.c
            generatePitchRollCurve(controlRateConfig);
            break;
        case ADJUSTMENT_THROTTLE_EXPO:
            newValue = (int)controlRateConfig->thrExpo8 + delta;
            controlRateConfig->thrExpo8 = constrain(newValue, 0, 100); // FIXME magic numbers repeated in serial_cli.c
            generateThrottleCurve(controlRateConfig, escAndServoConfig);
            break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
            newValue = (int)controlRateConfig->rollPitchRate + delta;
            controlRateConfig->rollPitchRate = constrain(newValue, 0, 100); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_YAW_RATE:
            newValue = (int)controlRateConfig->yawRate + delta;
            controlRateConfig->yawRate = constrain(newValue, 0, 100); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
            newValue = (int)pidProfile->P8[PIDPITCH] + delta;
            pidProfile->P8[PIDPITCH] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            newValue = (int)pidProfile->P8[PIDROLL] + delta;
            pidProfile->P8[PIDROLL] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
            newValue = (int)pidProfile->I8[PIDPITCH] + delta;
            pidProfile->I8[PIDPITCH] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            newValue = (int)pidProfile->I8[PIDROLL] + delta;
            pidProfile->I8[PIDROLL] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
            newValue = (int)pidProfile->D8[PIDPITCH] + delta;
            pidProfile->D8[PIDPITCH] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            newValue = (int)pidProfile->D8[PIDROLL] + delta;
            pidProfile->D8[PIDROLL] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_YAW_P:
            newValue = (int)pidProfile->P8[PIDYAW] + delta;
            pidProfile->P8[PIDYAW] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_YAW_I:
            newValue = (int)pidProfile->I8[PIDYAW] + delta;
            pidProfile->I8[PIDYAW] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        case ADJUSTMENT_YAW_D:
            newValue = (int)pidProfile->D8[PIDYAW] + delta;
            pidProfile->D8[PIDYAW] = constrain(newValue, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            break;
        default:
            break;
    };
}

#define RESET_FREQUENCY_2HZ (1000 / 2)

void processRcAdjustments(controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig)
{
    uint8_t adjustmentIndex;
    uint32_t now = millis();

    for (adjustmentIndex = 0; adjustmentIndex < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; adjustmentIndex++) {
        adjustmentState_t *adjustmentState = &adjustmentStates[adjustmentIndex];

        uint8_t adjustmentFunction = adjustmentState->adjustmentFunction;
        if (adjustmentFunction == ADJUSTMENT_NONE) {
            continue;
        }

        int32_t signedDiff = now - adjustmentState->timeoutAt;
        bool canResetReadyStates = signedDiff >= 0L;

        if (canResetReadyStates) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
        }


        uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentState->auxChannelIndex;

        int delta;
        if (rcData[channelIndex] > rxConfig->midrc + 200) {
            delta = adjustmentState->step;
        } else if (rcData[channelIndex] < rxConfig->midrc - 200) {
            delta = 0 - adjustmentState->step;
        } else {
            // returning the switch to the middle immediately resets the ready state
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            continue;
        }

        if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
            continue;
        }

        MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex);
        applyAdjustment(controlRateConfig, adjustmentFunction, delta);
    }
}

void updateAdjustmentStates(adjustmentRange_t *adjustmentRanges)
{
    uint8_t index;

    for (index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        adjustmentRange_t *adjustmentRange = &adjustmentRanges[index];

        if (isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range)) {

            const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentFunction - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];

            configureAdjustment(adjustmentRange->adjustmentIndex, adjustmentRange->auxSwitchChannelIndex, adjustmentConfig);
        }
    }
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse)
{
    uint8_t index;

    escAndServoConfig = escAndServoConfigToUse;
    pidProfile = pidProfileToUse;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            isUsingSticksToArm = false;
            break;
        }
    }
}
