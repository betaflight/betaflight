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

#include <platform.h>

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/runtime_config.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "io/rc_controls.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/escservo.h"
#include "io/rc_curves.h"

#include "io/display.h"

#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"

#include "blackbox/blackbox.h"

#include "mw.h"

#define AIRMODE_DEADBAND 12

static escAndServoConfig_t *escAndServoConfig;
static pidProfile_t *pidProfile;

// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e


void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue) {
#ifndef BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue) {
#ifndef BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newFloatValue);
#else
    if (feature(FEATURE_BLACKBOX)) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newFloatValue = newFloatValue;
        eventData.floatFlag = true;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

bool isUsingSticksForArming(void)
{
    return isUsingSticksToArm;
}


bool areSticksInApModePosition(uint16_t ap_mode)
{
    return ABS(rcCommand[ROLL]) < ap_mode && ABS(rcCommand[PITCH]) < ap_mode;
}

throttleStatus_e calculateThrottleStatus(rxConfig_t *rxConfig, uint16_t deadband3d_throttle)
{
    if (feature(FEATURE_3D) && (rcData[THROTTLE] > (rxConfig->midrc - deadband3d_throttle) && rcData[THROTTLE] < (rxConfig->midrc + deadband3d_throttle)))
        return THROTTLE_LOW;
    else if (!feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->mincheck))
        return THROTTLE_LOW;

    return THROTTLE_HIGH;
}

rollPitchStatus_e calculateRollPitchCenterStatus(rxConfig_t *rxConfig)
{
    if (((rcData[PITCH] < (rxConfig->midrc + AIRMODE_DEADBAND)) && (rcData[PITCH] > (rxConfig->midrc -AIRMODE_DEADBAND)))
            && ((rcData[ROLL] < (rxConfig->midrc + AIRMODE_DEADBAND)) && (rcData[ROLL] > (rxConfig->midrc -AIRMODE_DEADBAND))))
        return CENTERED;

    return NOT_CENTERED;
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

            if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()  ) {
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

    if (isUsingSticksToArm) {
        // Disarm on throttle down + yaw
        if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE) {
            if (ARMING_FLAG(ARMED))
                mwDisarm();
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
            }
        }
            // Disarm on roll (only when retarded_arm is enabled)
        if (retarded_arm && (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_LO)) {
            if (ARMING_FLAG(ARMED))
                mwDisarm();
            else {
                beeper(BEEPER_DISARM_REPEAT);    // sound tone while stick held
                rcDelayCommand = 0;              // reset so disarm tone will repeat
            }
        }
    }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed
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

#ifdef DISPLAY
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {
        displayDisablePageCycling();
    }

    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {
        displayEnablePageCycling();
    }
#endif

}

bool isModeActivationConditionPresent(modeActivationCondition_t *modeActivationConditions, boxId_e modeId)
{
    uint8_t index;

    for (index = 0; index < MAX_MODE_ACTIVATION_CONDITION_COUNT; index++) {
        modeActivationCondition_t *modeActivationCondition = &modeActivationConditions[index];
        
        if (modeActivationCondition->modeId == modeId && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            return true;
        }
    }
    
    return false;
}

bool isRangeActive(uint8_t auxChannelIndex, channelRange_t *range) {
    if (!IS_RANGE_USABLE(range)) {
        return false;
    }

    uint16_t channelValue = constrain(rcData[auxChannelIndex + NON_AUX_CHANNEL_COUNT], CHANNEL_RANGE_MIN, CHANNEL_RANGE_MAX - 1);
    return (channelValue >= MODE_STEP_TO_CHANNEL_VALUE(range->startStep)
            && channelValue < MODE_STEP_TO_CHANNEL_VALUE(range->endStep));
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

adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

void configureAdjustmentState(adjustmentRange_t *adjustmentRange) {
    uint8_t index = adjustmentRange->adjustmentIndex;

    adjustmentState_t *adjustmentState = &adjustmentStates[index];

    if (adjustmentState->range == adjustmentRange) {
        return;
    }
    adjustmentState->range = adjustmentRange;
    adjustmentState->auxChannelIndex = adjustmentRange->auxSwitchChannelIndex;
    adjustmentState->timeoutAt = 0;

    adjustmentState->config.adjustmentFunction = adjustmentRange->adjustmentFunction;

    if (adjustmentRange->adjustmentFunction == ADJUSTMENT_RATE_PROFILE) {
        adjustmentState->config.mode = ADJUSTMENT_MODE_SELECT;
        adjustmentState->config.data.selectConfig.switchPositions = 3;
    } else {
        adjustmentState->config.mode = ADJUSTMENT_MODE_STEP;
        adjustmentState->config.data.stepConfig.step = 1;
    }

    MARK_ADJUSTMENT_FUNCTION_AS_READY(index);
}
  
static void setAdjustment(uint8_t* ptr, float* ptrFP, uint8_t adjustment, int delta, float factor, uint8_t min, uint8_t max) {
  if(ptrFP != 0 && IS_PID_CONTROLLER_FP_BASED(pidProfile->pidController)) {
    *ptrFP = constrainf((*ptrFP) + (delta / factor), PID_F_MIN, PID_F_MAX);
    blackboxLogInflightAdjustmentEventFloat(adjustment, *ptrFP);
  } else {
    *ptr = constrain((int)(*ptr)+delta, min ,max);
    blackboxLogInflightAdjustmentEvent(adjustment, *ptr);
  }
}

void applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta) {

    if (delta > 0) {
        beeperConfirmationBeeps(2);
    } else {
        beeperConfirmationBeeps(1);
    }
    switch(adjustmentFunction) {
        case ADJUSTMENT_RC_RATE:
            setAdjustment(&controlRateConfig->rcRate8,0,ADJUSTMENT_RC_RATE,delta,0,RC_RATE_MIN,RC_RATE_MAX);
            generatePitchRollCurve(controlRateConfig);
        break;
        case ADJUSTMENT_RC_EXPO:
            setAdjustment(&controlRateConfig->rcExpo8,0,ADJUSTMENT_RC_EXPO,delta,0,EXPO_MIN,EXPO_MAX);
            generatePitchRollCurve(controlRateConfig);
        break;
        case ADJUSTMENT_THROTTLE_EXPO:
            setAdjustment(&controlRateConfig->thrExpo8,0,ADJUSTMENT_THROTTLE_EXPO,delta,0,EXPO_MIN,EXPO_MAX);
            generateThrottleCurve(controlRateConfig, escAndServoConfig);
        break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
        case ADJUSTMENT_PITCH_RATE:
            setAdjustment(&controlRateConfig->rates[FD_PITCH],0,ADJUSTMENT_PITCH_RATE,delta,0,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_RATE
        case ADJUSTMENT_ROLL_RATE:
            setAdjustment(&controlRateConfig->rates[FD_ROLL],0,ADJUSTMENT_ROLL_RATE,delta,0,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            break;
        case ADJUSTMENT_YAW_RATE:
            setAdjustment(&controlRateConfig->rates[FD_YAW],0,ADJUSTMENT_YAW_RATE,delta,0,0,CONTROL_RATE_CONFIG_YAW_RATE_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
        case ADJUSTMENT_PITCH_P:
            setAdjustment(&pidProfile->P8[PIDPITCH],&pidProfile->P_f[PIDPITCH],ADJUSTMENT_PITCH_P,delta,10.0f,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_P
        case ADJUSTMENT_ROLL_P:
            setAdjustment(&pidProfile->P8[PIDROLL],&pidProfile->P_f[PIDROLL],ADJUSTMENT_ROLL_P,delta,10.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
        case ADJUSTMENT_PITCH_I:
            setAdjustment(&pidProfile->I8[PIDPITCH],&pidProfile->I_f[PIDPITCH],ADJUSTMENT_PITCH_I,delta,100.0f,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_I
        case ADJUSTMENT_ROLL_I:
           setAdjustment(&pidProfile->I8[PIDROLL],&pidProfile->I_f[PIDROLL],ADJUSTMENT_ROLL_I,delta,100.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
        case ADJUSTMENT_PITCH_D:
            setAdjustment(&pidProfile->D8[PIDPITCH],&pidProfile->D_f[PIDPITCH],ADJUSTMENT_PITCH_D,delta,1000.0f,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_D
        case ADJUSTMENT_ROLL_D:
            setAdjustment(&pidProfile->D8[PIDROLL],&pidProfile->D_f[PIDROLL],ADJUSTMENT_ROLL_D,delta,1000.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_P:
                setAdjustment(&pidProfile->P8[PIDYAW],&pidProfile->P_f[PIDYAW],ADJUSTMENT_YAW_P,delta,10.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_I:
            setAdjustment(&pidProfile->I8[PIDYAW],&pidProfile->I_f[PIDYAW],ADJUSTMENT_YAW_I,delta,100.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_D:
                setAdjustment(&pidProfile->D8[PIDYAW],&pidProfile->D_f[PIDYAW],ADJUSTMENT_YAW_D,delta,1000.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_P:
            setAdjustment(&pidProfile->P8[PIDLEVEL],&pidProfile->A_level,ADJUSTMENT_LEVEL_P,delta,10.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_I:
            setAdjustment(&pidProfile->I8[PIDLEVEL],&pidProfile->H_level,ADJUSTMENT_LEVEL_I,delta,10.0f,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_D:
            if (IS_PID_CONTROLLER_FP_BASED(pidProfile->pidController)) {
              setAdjustment(&pidProfile->H_sensitivity,0,ADJUSTMENT_LEVEL_D,delta,0,PID_MIN,PID_MAX);
            } else {
              setAdjustment(&pidProfile->D8[PIDLEVEL],0,ADJUSTMENT_LEVEL_D,delta,0,PID_MIN,PID_MAX);
            }
            break;
        case ADJUSTMENT_ALT_P:
            setAdjustment(&pidProfile->P8[PIDALT],0,ADJUSTMENT_ALT_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_I:
            setAdjustment(&pidProfile->I8[PIDALT],0,ADJUSTMENT_ALT_I,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_D:
            setAdjustment(&pidProfile->D8[PIDALT],0,ADJUSTMENT_ALT_D,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_P:
            setAdjustment(&pidProfile->P8[PIDPOS],0,ADJUSTMENT_POS_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_I:
            setAdjustment(&pidProfile->I8[PIDPOS],0,ADJUSTMENT_POS_I,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_P:
            setAdjustment(&pidProfile->P8[PIDPOSR],0,ADJUSTMENT_POSR_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_I:
            setAdjustment(&pidProfile->I8[PIDPOSR],0,ADJUSTMENT_POSR_I,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_D:
            setAdjustment(&pidProfile->D8[PIDPOSR],0,ADJUSTMENT_POSR_D,delta,0,PID_MIN,PID_MAX);
            break;
       case ADJUSTMENT_NAVR_P:
            setAdjustment(&pidProfile->P8[PIDNAVR],0,ADJUSTMENT_NAVR_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_I:
            setAdjustment(&pidProfile->I8[PIDNAVR],0,ADJUSTMENT_NAVR_I,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_D:
            setAdjustment(&pidProfile->D8[PIDNAVR],0,ADJUSTMENT_NAVR_D,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_MAG_P:
            setAdjustment(&pidProfile->P8[PIDMAG],0,ADJUSTMENT_MAG_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_P:
            setAdjustment(&pidProfile->P8[PIDVEL],0,ADJUSTMENT_VEL_P,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_I:
            setAdjustment(&pidProfile->I8[PIDVEL],0,ADJUSTMENT_VEL_I,delta,0,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_D:
            setAdjustment(&pidProfile->D8[PIDVEL],0,ADJUSTMENT_VEL_D,delta,0,PID_MIN,PID_MAX);
            break;
        default:
            break;
    };
}

void changeControlRateProfile(uint8_t profileIndex);

void applySelectAdjustment(uint8_t adjustmentFunction, uint8_t position)
{
    bool applied = false;

    switch(adjustmentFunction) {
        case ADJUSTMENT_RATE_PROFILE:
            if (getCurrentControlRateProfile() != position) {
                changeControlRateProfile(position);
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);
                applied = true;
            }
            break;
    }

    if (applied) {
        beeperConfirmationBeeps(position + 1);
    }
}

#define RESET_FREQUENCY_2HZ (1000 / 2)

void processRcAdjustments(controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig)
{
    uint8_t adjustmentIndex;
    uint32_t now = millis();

    bool canUseRxData = rxIsReceivingSignal();


    for (adjustmentIndex = 0; adjustmentIndex < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; adjustmentIndex++) {
        adjustmentState_t *adjustmentState = &adjustmentStates[adjustmentIndex];

        uint8_t adjustmentFunction = adjustmentState->config.adjustmentFunction;
        if (adjustmentFunction == ADJUSTMENT_NONE) {
            continue;
        }

        int32_t signedDiff = now - adjustmentState->timeoutAt;
        bool canResetReadyStates = signedDiff >= 0L;

        if (canResetReadyStates) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
        }

        if (!canUseRxData) {
            continue;
        }

        uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentState->auxChannelIndex;

        if (adjustmentState->config.mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rcData[channelIndex] > rxConfig->midrc + 200) {
                delta = adjustmentState->config.data.stepConfig.step;
            } else if (rcData[channelIndex] < rxConfig->midrc - 200) {
                delta = 0 - adjustmentState->config.data.stepConfig.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
                continue;
            }

            applyStepAdjustment(controlRateConfig, adjustmentFunction, delta);
        } else if (adjustmentState->config.mode == ADJUSTMENT_MODE_SELECT) {
            uint16_t rangeWidth = ((2100 - 900) / adjustmentState->config.data.selectConfig.switchPositions);
            uint8_t position = (constrain(rcData[channelIndex], 900, 2100 - 1) - 900) / rangeWidth;

            applySelectAdjustment(adjustmentFunction, position);
        }
        MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex);
    }
}

void updateAdjustmentStates(adjustmentRange_t *adjustmentRanges)
{
    uint8_t index;

    for (index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        adjustmentRange_t *adjustmentRange = &adjustmentRanges[index];

        if (isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range)) {

            configureAdjustmentState(adjustmentRange);
        }
    }
}

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {
    return MIN(ABS(rcData[axis] - midrc), 500);
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, escAndServoConfig_t *escAndServoConfigToUse, pidProfile_t *pidProfileToUse)
{
    escAndServoConfig = escAndServoConfigToUse;
    pidProfile = pidProfileToUse;

    isUsingSticksToArm = !isModeActivationConditionPresent(modeActivationConditions, BOXARM);
}

void resetAdjustmentStates(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}

