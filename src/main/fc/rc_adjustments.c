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
#include "blackbox/blackbox_fielddefs.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "config/feature.h"

#include "flight/pid.h"

#include "io/beeper.h"
#include "io/motors.h"
#include "io/pidaudio.h"

#include "fc/config.h"
#include "fc/controlrate_profile.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/fc_rc.h"

#include "rx/rx.h"

PG_REGISTER_ARRAY(adjustmentRange_t, MAX_ADJUSTMENT_RANGE_COUNT, adjustmentRanges, PG_ADJUSTMENT_RANGE_CONFIG, 0);

uint8_t pidAudioPositionToModeMap[7] = {
    // on a pot with a center detent, it's easy to have center area for off/default, then three positions to the left and three to the right.
    // current implementation yields RC values as below.

    PID_AUDIO_PIDSUM_X,     //   900 - ~1071 - Min
    PID_AUDIO_PIDSUM_Y,     // ~1071 - ~1242
    PID_AUDIO_PIDSUM_XY,    // ~1242 - ~1414
    PID_AUDIO_OFF,          // ~1414 - ~1585 - Center
    PID_AUDIO_OFF,          // ~1585 - ~1757
    PID_AUDIO_OFF,          // ~1757 - ~1928
    PID_AUDIO_OFF,          // ~1928 -  2100 - Max

    // Note: Last 3 positions are currently pending implementations and use PID_AUDIO_OFF for now.
};

static pidProfile_t *pidProfile;

static void blackboxLogInflightAdjustmentEvent(adjustmentFunction_e adjustmentFunction, int32_t newValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newValue);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newValue = newValue;
        eventData.floatFlag = false;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}

#if 0
static void blackboxLogInflightAdjustmentEventFloat(adjustmentFunction_e adjustmentFunction, float newFloatValue)
{
#ifndef USE_BLACKBOX
    UNUSED(adjustmentFunction);
    UNUSED(newFloatValue);
#else
    if (blackboxConfig()->device) {
        flightLogEvent_inflightAdjustment_t eventData;
        eventData.adjustmentFunction = adjustmentFunction;
        eventData.newFloatValue = newFloatValue;
        eventData.floatFlag = true;
        blackboxLogEvent(FLIGHT_LOG_EVENT_INFLIGHT_ADJUSTMENT, (flightLogEventData_t*)&eventData);
    }
#endif
}
#endif

STATIC_UNIT_TESTED uint8_t adjustmentStateMask = 0;

#define MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex) adjustmentStateMask |= (1 << adjustmentIndex)
#define MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex) adjustmentStateMask &= ~(1 << adjustmentIndex)

#define IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex) (adjustmentStateMask & (1 << adjustmentIndex))

// sync with adjustmentFunction_e
static const adjustmentConfig_t defaultAdjustmentConfigs[ADJUSTMENT_FUNCTION_COUNT - 1] = {
    {
        .adjustmentFunction = ADJUSTMENT_RC_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_THROTTLE_EXPO,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_YAW_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RATE_PROFILE,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 3 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_RATE,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PITCH_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_P,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_I,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_ROLL_D,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_RC_RATE_YAW,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_D_SETPOINT,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_D_SETPOINT_TRANSITION,
        .mode = ADJUSTMENT_MODE_STEP,
        .data = { .step = 1 }
    }, {
        .adjustmentFunction = ADJUSTMENT_HORIZON_STRENGTH,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = 255 }
    }, {
        .adjustmentFunction = ADJUSTMENT_PID_AUDIO,
        .mode = ADJUSTMENT_MODE_SELECT,
        .data = { .switchPositions = ARRAYLEN(pidAudioPositionToModeMap) }
    }
};

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
static const char * const adjustmentLabels[] = {
    "RC RATE",
    "RC EXPO",
    "THROTTLE EXPO",
    "ROLL RATE",
    "YAW RATE",
    "PITCH/ROLL P",
    "PITCH/ROLL I",
    "PITCH/ROLL D",
    "YAW P",
    "YAW I",
    "YAW D",
    "RATE PROFILE",
    "PITCH RATE",
    "ROLL RATE",
    "PITCH P",
    "PITCH I",
    "PITCH D",
    "ROLL P",
    "ROLL I",
    "ROLL D",
    "RC RATE YAW",
    "D SETPOINT",
    "D SETPOINT TRANSITION",
    "HORIZON STRENGTH",
    "PID AUDIO",
};

const char *adjustmentRangeName;
int adjustmentRangeValue = -1;
#endif

#define ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET 1

STATIC_UNIT_TESTED adjustmentState_t adjustmentStates[MAX_SIMULTANEOUS_ADJUSTMENT_COUNT];

STATIC_UNIT_TESTED void configureAdjustment(uint8_t index, uint8_t auxSwitchChannelIndex, const adjustmentConfig_t *adjustmentConfig)
{
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

static int applyStepAdjustment(controlRateConfig_t *controlRateConfig, uint8_t adjustmentFunction, int delta)
{

    beeperConfirmationBeeps(delta > 0 ? 2 : 1);
    int newValue;
    switch (adjustmentFunction) {
    case ADJUSTMENT_RC_RATE:
    case ADJUSTMENT_ROLL_RC_RATE:
        newValue = constrain((int)controlRateConfig->rcRates[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_RATE:
        newValue = constrain((int)controlRateConfig->rcRates[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_RATE, newValue);
        break;
    case ADJUSTMENT_RC_EXPO:
    case ADJUSTMENT_ROLL_RC_EXPO:
        newValue = constrain((int)controlRateConfig->rcExpo[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RC_EXPO, newValue);
        if (adjustmentFunction == ADJUSTMENT_ROLL_RC_EXPO) {
            break;
        }
        // fall through for combined ADJUSTMENT_RC_EXPO
        FALLTHROUGH;
    case ADJUSTMENT_PITCH_RC_EXPO:
        newValue = constrain((int)controlRateConfig->rcExpo[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_RC_EXPO_MAX);
        controlRateConfig->rcExpo[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RC_EXPO, newValue);
        break;
    case ADJUSTMENT_THROTTLE_EXPO:
        newValue = constrain((int)controlRateConfig->thrExpo8 + delta, 0, 100); // FIXME magic numbers repeated in cli.c
        controlRateConfig->thrExpo8 = newValue;
        initRcProcessing();
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_THROTTLE_EXPO, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_RATE:
    case ADJUSTMENT_PITCH_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_PITCH] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_PITCH] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_RATE, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_RATE) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_RATE
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_ROLL] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_ROLL] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_RATE, newValue);
        break;
    case ADJUSTMENT_YAW_RATE:
        newValue = constrain((int)controlRateConfig->rates[FD_YAW] + delta, 0, CONTROL_RATE_CONFIG_RATE_MAX);
        controlRateConfig->rates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_RATE, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_P:
    case ADJUSTMENT_PITCH_P:
        newValue = constrain((int)pidProfile->pid[PID_PITCH].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_PITCH].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_P, newValue);

        if (adjustmentFunction == ADJUSTMENT_PITCH_P) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_P
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_P:
        newValue = constrain((int)pidProfile->pid[PID_ROLL].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_ROLL].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_P, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_I:
    case ADJUSTMENT_PITCH_I:
        newValue = constrain((int)pidProfile->pid[PID_PITCH].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_PITCH].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_I, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_I) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_I
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_I:
        newValue = constrain((int)pidProfile->pid[PID_ROLL].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_ROLL].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_I, newValue);
        break;
    case ADJUSTMENT_PITCH_ROLL_D:
    case ADJUSTMENT_PITCH_D:
        newValue = constrain((int)pidProfile->pid[PID_PITCH].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_PITCH].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_PITCH_D, newValue);
        if (adjustmentFunction == ADJUSTMENT_PITCH_D) {
            break;
        }
        // fall through for combined ADJUSTMENT_PITCH_ROLL_D
        FALLTHROUGH;
    case ADJUSTMENT_ROLL_D:
        newValue = constrain((int)pidProfile->pid[PID_ROLL].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_ROLL].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_ROLL_D, newValue);
        break;
    case ADJUSTMENT_YAW_P:
        newValue = constrain((int)pidProfile->pid[PID_YAW].P + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_YAW].P = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_P, newValue);
        break;
    case ADJUSTMENT_YAW_I:
        newValue = constrain((int)pidProfile->pid[PID_YAW].I + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_YAW].I = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_I, newValue);
        break;
    case ADJUSTMENT_YAW_D:
        newValue = constrain((int)pidProfile->pid[PID_YAW].D + delta, 0, 200); // FIXME magic numbers repeated in cli.c
        pidProfile->pid[PID_YAW].D = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_YAW_D, newValue);
        break;
    case ADJUSTMENT_RC_RATE_YAW:
        newValue = constrain((int)controlRateConfig->rcRates[FD_YAW] + delta, 0, CONTROL_RATE_CONFIG_RC_RATES_MAX);
        controlRateConfig->rcRates[FD_YAW] = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RC_RATE_YAW, newValue);
        break;
    case ADJUSTMENT_D_SETPOINT:
        newValue = constrain((int)pidProfile->dtermSetpointWeight + delta, 0, 254); // FIXME magic numbers repeated in cli.c
        pidProfile->dtermSetpointWeight = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_D_SETPOINT, newValue);
        break;
    case ADJUSTMENT_D_SETPOINT_TRANSITION:
        newValue = constrain((int)pidProfile->setpointRelaxRatio + delta, 1, 100); // FIXME magic numbers repeated in cli.c
        pidProfile->setpointRelaxRatio = newValue;
        blackboxLogInflightAdjustmentEvent(ADJUSTMENT_D_SETPOINT_TRANSITION, newValue);
        break;
    default:
        newValue = -1;
        break;
    };

    return newValue;
}

static uint8_t applySelectAdjustment(uint8_t adjustmentFunction, uint8_t position)
{
    uint8_t beeps = 0;

    switch (adjustmentFunction) {
    case ADJUSTMENT_RATE_PROFILE:
        {
            if (getCurrentControlRateProfileIndex() != position) {
                changeControlRateProfile(position);
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_RATE_PROFILE, position);

                beeps = position + 1;
            }
            break;
        }
    case ADJUSTMENT_HORIZON_STRENGTH:
        {
            uint8_t newValue = constrain(position, 0, 200); // FIXME magic numbers repeated in serial_cli.c
            if (pidProfile->pid[PID_LEVEL].D != newValue) {
                beeps = ((newValue - pidProfile->pid[PID_LEVEL].D) / 8) + 1;
                pidProfile->pid[PID_LEVEL].D = newValue;
                blackboxLogInflightAdjustmentEvent(ADJUSTMENT_HORIZON_STRENGTH, position);
            }
            break;
        }
    case ADJUSTMENT_PID_AUDIO:
        {
#ifdef USE_PID_AUDIO
            uint8_t newMode = pidAudioPositionToModeMap[position];
            if (newMode != pidAudioGetMode()) {
                pidAudioSetMode(newMode);
            }
#endif
            break;
        }
    }

    if (beeps) {
        beeperConfirmationBeeps(beeps);
    }

    return position;
}

#define RESET_FREQUENCY_2HZ (1000 / 2)

void processRcAdjustments(controlRateConfig_t *controlRateConfig)
{
    const uint32_t now = millis();

    int newValue = -1;

    const bool canUseRxData = rxIsReceivingSignal();

    for (int adjustmentIndex = 0; adjustmentIndex < MAX_SIMULTANEOUS_ADJUSTMENT_COUNT; adjustmentIndex++) {
        adjustmentState_t *adjustmentState = &adjustmentStates[adjustmentIndex];

        if (!adjustmentState->config) {
            continue;
        }
        const uint8_t adjustmentFunction = adjustmentState->config->adjustmentFunction;
        if (adjustmentFunction == ADJUSTMENT_NONE) {
            continue;
        }

        if (cmp32(now, adjustmentState->timeoutAt) >= 0) {
            adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
            MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
            adjustmentRangeValue = -1;
#endif
        }

        if (!canUseRxData) {
            continue;
        }

        const uint8_t channelIndex = NON_AUX_CHANNEL_COUNT + adjustmentState->auxChannelIndex;

        if (adjustmentState->config->mode == ADJUSTMENT_MODE_STEP) {
            int delta;
            if (rcData[channelIndex] > rxConfig()->midrc + 200) {
                delta = adjustmentState->config->data.step;
            } else if (rcData[channelIndex] < rxConfig()->midrc - 200) {
                delta = -adjustmentState->config->data.step;
            } else {
                // returning the switch to the middle immediately resets the ready state
                MARK_ADJUSTMENT_FUNCTION_AS_READY(adjustmentIndex);
                adjustmentState->timeoutAt = now + RESET_FREQUENCY_2HZ;
                continue;
            }
            if (IS_ADJUSTMENT_FUNCTION_BUSY(adjustmentIndex)) {
                continue;
            }

            newValue = applyStepAdjustment(controlRateConfig, adjustmentFunction, delta);
            pidInitConfig(pidProfile);
        } else if (adjustmentState->config->mode == ADJUSTMENT_MODE_SELECT) {
            int switchPositions = adjustmentState->config->data.switchPositions;
            if (adjustmentFunction == ADJUSTMENT_RATE_PROFILE && systemConfig()->rateProfile6PosSwitch) {
                switchPositions =  6;
            }
            const uint16_t rangeWidth = (2100 - 900) / switchPositions;
            const uint8_t position = (constrain(rcData[channelIndex], 900, 2100 - 1) - 900) / rangeWidth;
            newValue = applySelectAdjustment(adjustmentFunction, position);
        }

#if defined(USE_OSD) && defined(USE_OSD_ADJUSTMENTS)
        if (newValue != -1 && adjustmentState->config->adjustmentFunction != ADJUSTMENT_RATE_PROFILE) { // Rate profile already has an OSD element
            adjustmentRangeName = &adjustmentLabels[adjustmentFunction - 1][0];
            adjustmentRangeValue = newValue;
        }
#else
        UNUSED(newValue);
#endif
        MARK_ADJUSTMENT_FUNCTION_AS_BUSY(adjustmentIndex);
    }
}

void resetAdjustmentStates(void)
{
    memset(adjustmentStates, 0, sizeof(adjustmentStates));
}

void updateAdjustmentStates(void)
{
    for (int index = 0; index < MAX_ADJUSTMENT_RANGE_COUNT; index++) {
        const adjustmentRange_t * const adjustmentRange = adjustmentRanges(index);
        if (isRangeActive(adjustmentRange->auxChannelIndex, &adjustmentRange->range)) {
            const adjustmentConfig_t *adjustmentConfig = &defaultAdjustmentConfigs[adjustmentRange->adjustmentFunction - ADJUSTMENT_FUNCTION_CONFIG_INDEX_OFFSET];
            configureAdjustment(adjustmentRange->adjustmentIndex, adjustmentRange->auxSwitchChannelIndex, adjustmentConfig);
        }
    }
}

void useAdjustmentConfig(pidProfile_t *pidProfileToUse)
{
    pidProfile = pidProfileToUse;
}
