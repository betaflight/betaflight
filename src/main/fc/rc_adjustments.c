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

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/feature.h"
#include "config/profile.h"

#include "flight/pid.h"

#include "blackbox/blackbox.h"

#include "io/beeper.h"

#include "fc/rate_profile.h"
#include "fc/rc_controls.h"
#include "fc/rc_curves.h"
#include "fc/rc_adjustments.h"
#include "fc/config.h"

PG_REGISTER_PROFILE(adjustmentProfile_t, adjustmentProfile, PG_ADJUSTMENT_PROFILE, 0);

uint8_t adjustmentStateMask = 0;

#define MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex) adjustmentStateMask |= (1 << adjustmentIndex)
#define MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex) adjustmentStateMask &= ~(1 << adjustmentIndex)

#define IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex) (adjustmentStateMask & (1 << adjustmentIndex))

void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue)
{
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

void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue)
{
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

adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

void configureAdjustmentState(adjustmentRange_t *adjustmentRange)
{
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

static void setAdjustment(uint8_t* ptr, uint8_t adjustment, int delta, uint8_t min, uint8_t max)
{
    *ptr = constrain((int)(*ptr)+delta, min ,max);
    blackboxLogInflightAdjustmentEvent(adjustment, *ptr);
}

void applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta)
{

    if (delta > 0) {
        beeperConfirmationBeeps(2);
    } else {
        beeperConfirmationBeeps(1);
    }
    switch(adjustmentFunction) {
        case ADJUSTMENT_RC_RATE:
            setAdjustment(&controlRateConfig->rcRate8,ADJUSTMENT_RC_RATE,delta,RC_RATE_MIN,RC_RATE_MAX);
            generatePitchRollCurve();
            break;
        case ADJUSTMENT_RC_EXPO:
            setAdjustment(&controlRateConfig->rcExpo8,ADJUSTMENT_RC_EXPO,delta,EXPO_MIN,EXPO_MAX);
            generatePitchRollCurve();
            break;
        case ADJUSTMENT_THROTTLE_EXPO:
            setAdjustment(&controlRateConfig->thrExpo8,ADJUSTMENT_THROTTLE_EXPO,delta,EXPO_MIN,EXPO_MAX);
            generateThrottleCurve();
            break;
        case ADJUSTMENT_PITCH_ROLL_RATE:
        case ADJUSTMENT_PITCH_RATE:
            setAdjustment(&controlRateConfig->rates[PITCH],ADJUSTMENT_PITCH_RATE,delta,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_RATE
        case ADJUSTMENT_ROLL_RATE:
            setAdjustment(&controlRateConfig->rates[ROLL],ADJUSTMENT_ROLL_RATE,delta,0,CONTROL_RATE_CONFIG_ROLL_PITCH_RATE_MAX);
            break;
        case ADJUSTMENT_YAW_RATE:
            setAdjustment(&controlRateConfig->rates[YAW],ADJUSTMENT_YAW_RATE,delta,0,CONTROL_RATE_CONFIG_YAW_RATE_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_P:
        case ADJUSTMENT_PITCH_P:
            setAdjustment(&pidProfile()->P8[PIDPITCH],ADJUSTMENT_PITCH_P,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_P
        case ADJUSTMENT_ROLL_P:
            setAdjustment(&pidProfile()->P8[PIDROLL],ADJUSTMENT_ROLL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_I:
        case ADJUSTMENT_PITCH_I:
            setAdjustment(&pidProfile()->I8[PIDPITCH],ADJUSTMENT_PITCH_I,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_I
        case ADJUSTMENT_ROLL_I:
           setAdjustment(&pidProfile()->I8[PIDROLL],ADJUSTMENT_ROLL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_PITCH_ROLL_D:
        case ADJUSTMENT_PITCH_D:
            setAdjustment(&pidProfile()->D8[PIDPITCH],ADJUSTMENT_PITCH_D,delta,PID_MIN,PID_MAX);
            if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
                break;
            }
            // follow though for combined ADJUSTMENT_PITCH_ROLL_D
        case ADJUSTMENT_ROLL_D:
            setAdjustment(&pidProfile()->D8[PIDROLL],ADJUSTMENT_ROLL_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_P:
                setAdjustment(&pidProfile()->P8[PIDYAW],ADJUSTMENT_YAW_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_I:
            setAdjustment(&pidProfile()->I8[PIDYAW],ADJUSTMENT_YAW_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_YAW_D:
                setAdjustment(&pidProfile()->D8[PIDYAW],ADJUSTMENT_YAW_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_P:
            setAdjustment(&pidProfile()->P8[PIDLEVEL],ADJUSTMENT_LEVEL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_I:
            setAdjustment(&pidProfile()->I8[PIDLEVEL],ADJUSTMENT_LEVEL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_LEVEL_D:
            setAdjustment(&pidProfile()->D8[PIDLEVEL],ADJUSTMENT_LEVEL_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_P:
            setAdjustment(&pidProfile()->P8[PIDALT],ADJUSTMENT_ALT_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_I:
            setAdjustment(&pidProfile()->I8[PIDALT],ADJUSTMENT_ALT_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_ALT_D:
            setAdjustment(&pidProfile()->D8[PIDALT],ADJUSTMENT_ALT_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_P:
            setAdjustment(&pidProfile()->P8[PIDPOS],ADJUSTMENT_POS_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POS_I:
            setAdjustment(&pidProfile()->I8[PIDPOS],ADJUSTMENT_POS_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_P:
            setAdjustment(&pidProfile()->P8[PIDPOSR],ADJUSTMENT_POSR_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_I:
            setAdjustment(&pidProfile()->I8[PIDPOSR],ADJUSTMENT_POSR_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_POSR_D:
            setAdjustment(&pidProfile()->D8[PIDPOSR],ADJUSTMENT_POSR_D,delta,PID_MIN,PID_MAX);
            break;
       case ADJUSTMENT_NAVR_P:
            setAdjustment(&pidProfile()->P8[PIDNAVR],ADJUSTMENT_NAVR_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_I:
            setAdjustment(&pidProfile()->I8[PIDNAVR],ADJUSTMENT_NAVR_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_NAVR_D:
            setAdjustment(&pidProfile()->D8[PIDNAVR],ADJUSTMENT_NAVR_D,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_MAG_P:
            setAdjustment(&pidProfile()->P8[PIDMAG],ADJUSTMENT_MAG_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_P:
            setAdjustment(&pidProfile()->P8[PIDVEL],ADJUSTMENT_VEL_P,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_I:
            setAdjustment(&pidProfile()->I8[PIDVEL],ADJUSTMENT_VEL_I,delta,PID_MIN,PID_MAX);
            break;
        case ADJUSTMENT_VEL_D:
            setAdjustment(&pidProfile()->D8[PIDVEL],ADJUSTMENT_VEL_D,delta,PID_MIN,PID_MAX);
            break;
        default:
            break;
    };
}

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

void resetAdjustmentStates(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}
