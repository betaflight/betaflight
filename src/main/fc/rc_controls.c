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

#include "platform.h"

#include "blackbox/blackbox.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "drivers/system.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/runtime_config.h"

#include "io/gps.h"
#include "io/beeper.h"
#include "io/motors.h"
#include "io/vtx.h"
#include "io/dashboard.h"

#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "flight/pid.h"
#include "flight/navigation.h"
#include "flight/failsafe.h"


static motorConfig_t *motorConfig;
static pidProfile_t *pidProfile;

// true if arming is done via the sticks (as opposed to a switch)
static bool isUsingSticksToArm = true;

int16_t rcCommand[4];           // interval [1000;2000] for THROTTLE and [-500;+500] for ROLL/PITCH/YAW
int16_t rcCommandSmooth[4];

uint32_t rcModeActivationMask; // one bit per mode defined in boxId_e

bool isAirmodeActive(void) {
    return (IS_RC_MODE_ACTIVE(BOXAIRMODE) || feature(FEATURE_AIRMODE));
}

void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue) {
#ifndef BLACKBOX
#define UNUSED(x) (void)(x)
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
    if (feature(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3DDISABLESWITCH)) {
        if ((rcData[THROTTLE] > (rxConfig->midrc - deadband3d_throttle) && rcData[THROTTLE] < (rxConfig->midrc + deadband3d_throttle)))
            return THROTTLE_LOW;
    } else {
        if (rcData[THROTTLE] < rxConfig->mincheck)
            return THROTTLE_LOW;
    }

    return THROTTLE_HIGH;
}

void processRcStickPositions(rxConfig_t *rxConfig, throttleStatus_e throttleStatus, bool disarm_kill_switch)
{
    static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
    static uint8_t rcSticks;            // this hold sticks position for command combos
    static uint8_t rcDisarmTicks;       // this is an extra guard for disarming through switch to prevent that one frame can disarm it
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
            rcDisarmTicks = 0;
            // Arming via ARM BOX
            if (throttleStatus == THROTTLE_LOW) {
                if (ARMING_FLAG(OK_TO_ARM)) {
                    mwArm();
                }
            }
        } else {
            // Disarming via ARM BOX

            if (ARMING_FLAG(ARMED) && rxIsReceivingSignal() && !failsafeIsActive()  ) {
                rcDisarmTicks++;
                if (rcDisarmTicks > 3) {
                    if (disarm_kill_switch) {
                        mwDisarm();
                    } else if (throttleStatus == THROTTLE_LOW) {
                        mwDisarm();
                    }
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
    }

    if (ARMING_FLAG(ARMED)) {
        // actions during armed
        return;
    }

    // actions during not armed
    i = 0;

    if (rcSticks == THR_LO + YAW_LO + PIT_LO + ROL_CE) {
        // GYRO calibration
        gyroSetCalibrationCycles();

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

#ifdef USE_DASHBOARD
    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_LO) {
        dashboardDisablePageCycling();
    }

    if (rcSticks == THR_LO + YAW_CE + PIT_HI + ROL_HI) {
        dashboardEnablePageCycling();
    }
#endif

#ifdef VTX
    if (rcSticks ==  THR_HI + YAW_LO + PIT_CE + ROL_HI) {
        vtxIncrementBand();
    }
    if (rcSticks ==  THR_HI + YAW_LO + PIT_CE + ROL_LO) {
        vtxDecrementBand();
    }
    if (rcSticks ==  THR_HI + YAW_HI + PIT_CE + ROL_HI) {
        vtxIncrementChannel();
    }
    if (rcSticks ==  THR_HI + YAW_HI + PIT_CE + ROL_LO) {
        vtxDecrementChannel();
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
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_RC_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_THROTTLE_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_YAW_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .selectConfig = { .switchPositions = 3 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_PITCH_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_RC_RATE_YAW,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_D_SETPOINT,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    },
    {
        .adjustmentFunction = ADJUSTMENT_D_SETPOINT_TRANSITION,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .stepConfig = { .step = 1 }}
    }
};

#define ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET 1

adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

static void configureAdjustment(uint8_t index, uint8_t auxSwitchChannelIndex, const adjustmentConfig_t *adjustmentConfig) {
    adjustmentState_t *adjustmentState = &adjustmentStates[index];

    if (adjustmentState->config == adjustmentConfig) {
        // already configured
        return;
    }
    adjustmentState->auxChannelIndex = auxSwitchChannelIndex;
    adjustmentState->config = adjustmentConfig;
    adjustmentState->timeoutAt = 0;

    MARK_ADJUSTMENT_FUNCTION_AS_READY(index);
}

static void applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta) {
    int newValue;

    if (delta > 0) {
        beeperConfirmationBeeps(2);
    } else {
        beeperConfirmationBeeps(1);
    }
    switch(adjustmentFunction) {
        case ADJUSTMENT_RC_RATE:
            newValue = constrain((int)controlRateConfig->rcRate8 + delta, 0, 250); // FIXME magic numbers repeated in cli.c
            controlRateConfig->rcRate8 = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE, newValue);
        break;
        case ADJUSTMENT_RC_EXPO:
            newValue = constrain((int)controlRateConfig->rcExpo8 + delta, 0, 100); // FIXME magic numbers repeated in cli.c
            controlRateConfig->rcExpo8 = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_EXPO, newValue);
        break;
        case ADJUSTMENT_THROTTLE_EXPO:
            newValue = constrain((int)controlRateConfig->thrExpo8 + delta, 0, 100); // FIXME magic numbers repeated in cli.c
            controlRateConfig->thrExpo8 = newValue;
            generateThrottleCurve(controlRateConfig, motorConfig);
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO, newValue);
        break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
        case ADJUSTMENT_PITCH_RATE:
            newValue = constrain((int)controlRateConfig->rates[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            controlRateConfig->rates[FD_PITCH] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE, newValue);
            if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_RATE
        case ADJUSTMENT_ROLL_RATE:
            newValue = constrain((int)controlRateConfig->rates[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            controlRateConfig->rates[FD_ROLL] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE, newValue);
            break;
        case ADJUSTMENT_YAW_RATE:
            newValue = constrain((int)controlRateConfig->rates[FD_YAW] + delta, 0, CONTROL_RATE_CONFIG_YAW_RATE_MAX);
            controlRateConfig->rates[FD_YAW] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
        case ADJUSTMENT_PITCH_P:
            newValue = constrain((int)pidProfile->P8[PIDPITCH] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->P8[PIDPITCH] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);

            if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_P
        case ADJUSTMENT_ROLL_P:
            newValue = constrain((int)pidProfile->P8[PIDROLL] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->P8[PIDROLL] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
        case ADJUSTMENT_PITCH_I:
            newValue = constrain((int)pidProfile->I8[PIDPITCH] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->I8[PIDPITCH] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);

            if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_I
        case ADJUSTMENT_ROLL_I:
            newValue = constrain((int)pidProfile->I8[PIDROLL] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->I8[PIDROLL] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
        case ADJUSTMENT_PITCH_D:
            newValue = constrain((int)pidProfile->D8[PIDPITCH] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->D8[PIDPITCH] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);

            if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_D
        case ADJUSTMENT_ROLL_D:
            newValue = constrain((int)pidProfile->D8[PIDROLL] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->D8[PIDROLL] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
            break;
        case ADJUSTMENT_YAW_P:
            newValue = constrain((int)pidProfile->P8[PIDYAW] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->P8[PIDYAW] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
            break;
        case ADJUSTMENT_YAW_I:
            newValue = constrain((int)pidProfile->I8[PIDYAW] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->I8[PIDYAW] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
            break;
        case ADJUSTMENT_YAW_D:
            newValue = constrain((int)pidProfile->D8[PIDYAW] + delta, 0, 200); // FIXME magic numbers repeated in cli.c
            pidProfile->D8[PIDYAW] = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
            break;
        case ADJUSTMENT_RC_RATE_YAW:
            newValue = constrain((int)controlRateConfig->rcYawRate8 + delta, 0, 300); // FIXME magic numbers repeated in cli.c
            controlRateConfig->rcYawRate8 = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW, newValue);
            break;
        case ADJUSTMENT_D_SETPOINT:
            newValue = constrain((int)pidProfile->dtermSetpointWeight + delta, 0, 254); // FIXME magic numbers repeated in cli.c
            pidProfile->dtermSetpointWeight = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_D_SETPOINT, newValue);
            break;
        case ADJUSTMENT_D_SETPOINT_TRANSITION:
            newValue = constrain((int)pidProfile->setpointRelaxRatio + delta, 0, 100); // FIXME magic numbers repeated in cli.c
            pidProfile->setpointRelaxRatio = newValue;
            blackboxLogInflightAdjustmentEvent(ADJUSTMENT_D_SETPOINT_TRANSITION, newValue);
            break;
        default:
            break;
    };
}

static void applySelectAdjustment(uint8_t adjustmentFunction, uint8_t position)
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

        if (!adjustmentState->config) {
            continue;
        }
        uint8_t adjustmentFunction = adjustmentState->config->adjustmentFunction;
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

        if (adjustmentState->config->mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rcData[channelIndex] > rxConfig->midrc + 200) {
                delta = adjustmentState->config->data.stepConfig.step;
            } else if (rcData[channelIndex] < rxConfig->midrc - 200) {
                delta = 0 - adjustmentState->config->data.stepConfig.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
                continue;
            }

            applyStepAdjustment(controlRateConfig,adjustmentFunction,delta);
            pidInitConfig(pidProfile);
        } else if (adjustmentState->config->mode == ADJUSTMENT_MODE_SELECT) {
            uint16_t rangeWidth = ((2100 - 900) / adjustmentState->config->data.selectConfig.switchPositions);
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

            const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentFunction - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];

            configureAdjustment(adjustmentRange->adjustmentIndex, adjustmentRange->auxSwitchChannelIndex, adjustmentConfig);
        }
    }
}

int32_t getRcStickDeflection(int32_t axis, uint16_t midrc) {
    return MIN(ABS(rcData[axis] - midrc), 500);
}

void useRcControlsConfig(modeActivationCondition_t *modeActivationConditions, motorConfig_t *motorConfigToUse, pidProfile_t *pidProfileToUse)
{
    motorConfig = motorConfigToUse;
    pidProfile = pidProfileToUse;

    isUsingSticksToArm = !isModeActivationConditionPresent(modeActivationConditions, BOXARM);
}

void resetAdjustmentStates(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}

