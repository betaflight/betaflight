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
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/debug.h"
#include "build/build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/system.h"

#include "io/gimbal.h"
#include "io/servos.h"

#include "rx/rx.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation_rewrite.h"

#include "config/config.h"
#include "config/config_profile.h"
#include "config/feature.h"


extern mixerMode_e currentMixerMode;
extern const mixer_t mixers[];
extern mixerConfig_t *mixerConfig;

static rxConfig_t *rxConfig;

servoMixerConfig_t *servoMixerConfig;

static uint8_t servoRuleCount = 0;
static servoMixer_t currentServoMixer[MAX_SERVO_RULES];
static gimbalConfig_t *gimbalConfig;
int16_t servo[MAX_SUPPORTED_SERVOS];
static int servoOutputEnabled;

static uint8_t mixerUsesServos;
static uint8_t minServoIndex;
static uint8_t maxServoIndex;

static servoParam_t *servoConf;
static biquadFilter_t servoFitlerState[MAX_SUPPORTED_SERVOS];
static bool servoFilterIsSet;

#define COUNT_SERVO_RULES(rules) (sizeof(rules) / sizeof(servoMixer_t))
// mixer rule format servo, input, rate, speed, min, max, box
static const servoMixer_t servoMixerAirplane[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  100, 0, 0, 100 },
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100 },
    { SERVO_ELEVATOR, INPUT_STABILIZED_PITCH, 100, 0, 0, 100 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100 },
};

static const servoMixer_t servoMixerFlyingWing[] = {
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_ROLL,  100, 0, 0, 100 },
    { SERVO_FLAPPERON_1, INPUT_STABILIZED_PITCH, 100, 0, 0, 100 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_ROLL,  -100, 0, 0, 100 },
    { SERVO_FLAPPERON_2, INPUT_STABILIZED_PITCH, 100, 0, 0, 100 },
    { SERVO_THROTTLE, INPUT_STABILIZED_THROTTLE, 100, 0, 0, 100 },
};

static const servoMixer_t servoMixerTri[] = {
    { SERVO_RUDDER, INPUT_STABILIZED_YAW,   100, 0, 0, 100 },
};

const mixerRules_t servoMixers[] = {
    { 0, 0, 0, NULL },                // entry 0
    { COUNT_SERVO_RULES(servoMixerTri), SERVO_RUDDER, SERVO_RUDDER, servoMixerTri },       // MULTITYPE_TRI
    { 0, 0, 0, NULL },                // MULTITYPE_QUADP
    { 0, 0, 0, NULL },                // MULTITYPE_QUADX
    { 0, 0, 0, NULL },                // MULTITYPE_BI
    { 0, 0, 0, NULL },                // MULTITYPE_GIMBAL / MIXER_GIMBAL -> disabled
    { 0, 0, 0, NULL },                // MULTITYPE_Y6
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6
    { COUNT_SERVO_RULES(servoMixerFlyingWing), SERVO_FLAPPERONS_MIN, SERVO_FLAPPERONS_MAX, servoMixerFlyingWing },// * MULTITYPE_FLYING_WING
    { 0, 0, 0, NULL },                // MULTITYPE_Y4
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6X
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOX8
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOFLATP
    { 0, 0, 0, NULL },                // MULTITYPE_OCTOFLATX
    { COUNT_SERVO_RULES(servoMixerAirplane), SERVO_PLANE_INDEX_MIN, SERVO_PLANE_INDEX_MAX, servoMixerAirplane },  // * MULTITYPE_AIRPLANE
    { 0, 0, 0, NULL },                // * MIXER_HELI_120_CCPM -> disabled, never fully implemented in CF
    { 0, 0, 0, NULL },                // * MIXER_HELI_90_DEG -> disabled, never fully implemented in CF
    { 0, 0, 0, NULL },                // MULTITYPE_VTAIL4
    { 0, 0, 0, NULL },                // MULTITYPE_HEX6H
    { 0, 0, 0, NULL },                // * MULTITYPE_PPM_TO_SERVO
    { 0, 0, 0, NULL },                // MULTITYPE_DUALCOPTER
    { 0, 0, 0, NULL },                // MULTITYPE_SINGLECOPTER
    { 0, 0, 0, NULL },                // MULTITYPE_ATAIL4
    { 0, 2, 5, NULL },                // MULTITYPE_CUSTOM
    { 0, SERVO_PLANE_INDEX_MIN, SERVO_PLANE_INDEX_MAX, NULL },                // MULTITYPE_CUSTOM_PLANE
    { 0, SERVO_RUDDER, SERVO_RUDDER, NULL },                // MULTITYPE_CUSTOM_TRI
};

static servoMixer_t *customServoMixers;

void servosUseConfigs(servoMixerConfig_t *servoMixerConfigToUse, servoParam_t *servoParamsToUse, gimbalConfig_t *gimbalConfigToUse, rxConfig_t *rxConfigToUse)
{
    servoMixerConfig = servoMixerConfigToUse;
    servoConf = servoParamsToUse;
    gimbalConfig = gimbalConfigToUse;
    rxConfig = rxConfigToUse;
}

int16_t getFlaperonDirection(uint8_t servoPin) {
    if (servoPin == SERVO_FLAPPERON_2) {
        return -1;
    } else {
        return 1;
    }
}

int16_t determineServoMiddleOrForwardFromChannel(servoIndex_e servoIndex)
{
    uint8_t channelToForwardFrom = servoConf[servoIndex].forwardFromChannel;

    if (channelToForwardFrom != CHANNEL_FORWARDING_DISABLED && channelToForwardFrom < rxRuntimeConfig.channelCount) {
        return rcData[channelToForwardFrom];
    }

    return servoConf[servoIndex].middle;
}


int servoDirection(int servoIndex, int inputSource)
{
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoConf[servoIndex].reversedSources & (1 << inputSource))
        return -1;
    else
        return 1;
}

void servosInit(servoMixer_t *initialCustomServoMixers)
{
    int i;

    // set flag that we're on something with wings
    if (currentMixerMode == MIXER_FLYING_WING ||
        currentMixerMode == MIXER_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_AIRPLANE
    ) {
        ENABLE_STATE(FIXED_WING);
    } else {
        DISABLE_STATE(FIXED_WING);
    }

    if (currentMixerMode == MIXER_AIRPLANE || currentMixerMode == MIXER_CUSTOM_AIRPLANE) {
        ENABLE_STATE(FLAPERON_AVAILABLE);
    } else {
        DISABLE_STATE(FLAPERON_AVAILABLE);
    }

    customServoMixers = initialCustomServoMixers;

    minServoIndex = servoMixers[currentMixerMode].minServoIndex;
    maxServoIndex = servoMixers[currentMixerMode].maxServoIndex;

    // enable servos for mixes that require them. note, this shifts motor counts.
    mixerUsesServos = mixers[currentMixerMode].useServo || feature(FEATURE_SERVO_TILT);

    // if we want camstab/trig, that also enables servos, even if mixer doesn't
    servoOutputEnabled = mixerUsesServos || feature(FEATURE_CHANNEL_FORWARDING);

    // give all servos a default command
    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = DEFAULT_SERVO_MIDDLE;
    }

    /*
     * If mixer has predefined servo mixes, load them
     */
    if (mixerUsesServos) {
        servoRuleCount = servoMixers[currentMixerMode].servoRuleCount;
        if (servoMixers[currentMixerMode].rule) {
            for (i = 0; i < servoRuleCount; i++)
                currentServoMixer[i] = servoMixers[currentMixerMode].rule[i];
        }
    }

    /*
     * When we are dealing with CUSTOM mixers, load mixes defined with
     * smix and update internal variables
     */
    if (currentMixerMode == MIXER_CUSTOM_AIRPLANE ||
        currentMixerMode == MIXER_CUSTOM_TRI ||
        currentMixerMode == MIXER_CUSTOM)
    {
        loadCustomServoMixer();

        // If there are servo rules after all, update variables
        if (servoRuleCount > 0) {
            servoOutputEnabled = 1;
            mixerUsesServos = 1;
        }

    }
}
void loadCustomServoMixer(void)
{
    int i;

    // reset settings
    servoRuleCount = 0;
    minServoIndex = 255;
    maxServoIndex = 0;
    memset(currentServoMixer, 0, sizeof(currentServoMixer));

    // load custom mixer into currentServoMixer
    for (i = 0; i < MAX_SERVO_RULES; i++) {
        // check if done
        if (customServoMixers[i].rate == 0)
            break;

        if (customServoMixers[i].targetChannel < minServoIndex) {
            minServoIndex = customServoMixers[i].targetChannel;
        }

        if (customServoMixers[i].targetChannel > maxServoIndex) {
            maxServoIndex = customServoMixers[i].targetChannel;
        }

        currentServoMixer[i] = customServoMixers[i];
        servoRuleCount++;
    }
}

void servoMixerLoadMix(int index, servoMixer_t *customServoMixers)
{
    int i;

    // we're 1-based
    index++;
    // clear existing
    for (i = 0; i < MAX_SERVO_RULES; i++)
        customServoMixers[i].targetChannel = customServoMixers[i].inputSource = customServoMixers[i].rate = 0;

    for (i = 0; i < servoMixers[index].servoRuleCount; i++) {
        customServoMixers[i] = servoMixers[index].rule[i];
    }
}

STATIC_UNIT_TESTED void forwardAuxChannelsToServos(uint8_t firstServoIndex)
{
    // start forwarding from this channel
    uint8_t channelOffset = AUX1;

    int servoOffset;
    for (servoOffset = 0; servoOffset < MAX_AUX_CHANNEL_COUNT && channelOffset < MAX_SUPPORTED_RC_CHANNEL_COUNT; servoOffset++) {
        pwmWriteServo(firstServoIndex + servoOffset, rcData[channelOffset++]);
    }
}

void writeServos(void)
{
    int servoIndex = 0;

    bool zeroServoValue = false;

    /*
     * in case of tricopters, there might me a need to zero servo output when unarmed
     */
    if ((currentMixerMode == MIXER_TRI || currentMixerMode == MIXER_CUSTOM_TRI) && !ARMING_FLAG(ARMED) && !servoMixerConfig->tri_unarmed_servo) {
        zeroServoValue = true;
    }

    // Write mixer servo outputs
    //      mixerUsesServos might indicate SERVO_TILT, servoRuleCount indicate if mixer alone uses servos
    if (mixerUsesServos && servoRuleCount) {
        for (int i = minServoIndex; i <= maxServoIndex; i++) {
            if (zeroServoValue) {
                pwmWriteServo(servoIndex++, 0);
            } else {
                pwmWriteServo(servoIndex++, servo[i]);
            }
        }
    }

    if (feature(FEATURE_SERVO_TILT)) {
        pwmWriteServo(servoIndex++, servo[SERVO_GIMBAL_PITCH]);
        pwmWriteServo(servoIndex++, servo[SERVO_GIMBAL_ROLL]);
    }

    // forward AUX to remaining servo outputs (not constrained)
    if (feature(FEATURE_CHANNEL_FORWARDING)) {
        forwardAuxChannelsToServos(servoIndex);
        servoIndex += MAX_AUX_CHANNEL_COUNT;
    }
}

void servoMixer(uint16_t flaperon_throw_offset, uint8_t flaperon_throw_inverted)
{
    int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]
    static int16_t currentOutput[MAX_SERVO_RULES];
    int i;

    if (FLIGHT_MODE(PASSTHRU_MODE)) {
        // Direct passthru from RX
        input[INPUT_STABILIZED_ROLL] = rcCommand[ROLL];
        input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
        input[INPUT_STABILIZED_YAW] = rcCommand[YAW];
    } else {
        // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
        input[INPUT_STABILIZED_ROLL] = axisPID[ROLL];
        input[INPUT_STABILIZED_PITCH] = axisPID[PITCH];
        input[INPUT_STABILIZED_YAW] = axisPID[YAW];

        // Reverse yaw servo when inverted in 3D mode
        if (feature(FEATURE_3D) && (rcData[THROTTLE] < rxConfig->midrc)) {
            input[INPUT_STABILIZED_YAW] *= -1;
        }
    }

    input[INPUT_GIMBAL_PITCH] = scaleRange(attitude.values.pitch, -1800, 1800, -500, +500);
    input[INPUT_GIMBAL_ROLL] = scaleRange(attitude.values.roll, -1800, 1800, -500, +500);

    input[INPUT_STABILIZED_THROTTLE] = motor[0] - 1000 - 500;  // Since it derives from rcCommand or mincommand and must be [-500:+500]

    // center the RC input value around the RC middle value
    // by subtracting the RC middle value from the RC input value, we get:
    // data - middle = input
    // 2000 - 1500 = +500
    // 1500 - 1500 = 0
    // 1000 - 1500 = -500
    input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig->midrc;
    input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig->midrc;
    input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig->midrc;
    input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig->midrc;
    input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig->midrc;
    input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig->midrc;
    input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig->midrc;
    input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig->midrc;

    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++)
        servo[i] = 0;

    // mix servos according to rules
    for (i = 0; i < servoRuleCount; i++) {
        uint8_t target = currentServoMixer[i].targetChannel;
        uint8_t from = currentServoMixer[i].inputSource;
        uint16_t servo_width = servoConf[target].max - servoConf[target].min;
        int16_t min = currentServoMixer[i].min * servo_width / 100 - servo_width / 2;
        int16_t max = currentServoMixer[i].max * servo_width / 100 - servo_width / 2;

        if (currentServoMixer[i].speed == 0) {
            currentOutput[i] = input[from];
        } else {
            if (currentOutput[i] < input[from]) {
                currentOutput[i] = constrain(currentOutput[i] + currentServoMixer[i].speed, currentOutput[i], input[from]);
            } else if (currentOutput[i] > input[from]) {
                currentOutput[i] = constrain(currentOutput[i] - currentServoMixer[i].speed, input[from], currentOutput[i]);
            }
        }

        /*
        Flaperon fligh mode
        */
        if (FLIGHT_MODE(FLAPERON) && (target == SERVO_FLAPPERON_1 || target == SERVO_FLAPPERON_2)) {
            int8_t multiplier = 1;

            if (flaperon_throw_inverted == 1) {
                multiplier = -1;
            }
            currentOutput[i] += flaperon_throw_offset * getFlaperonDirection(target) * multiplier;
        }

        servo[target] += servoDirection(target, from) * constrain(((int32_t)currentOutput[i] * currentServoMixer[i].rate) / 100, min, max);
    }

    for (i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        servo[i] = ((int32_t)servoConf[i].rate * servo[i]) / 100L;
        servo[i] += determineServoMiddleOrForwardFromChannel(i);
    }
}

void processServoTilt(void)
{
    // center at fixed position, or vary either pitch or roll by RC channel
    servo[SERVO_GIMBAL_PITCH] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_PITCH);
    servo[SERVO_GIMBAL_ROLL] = determineServoMiddleOrForwardFromChannel(SERVO_GIMBAL_ROLL);

    if (IS_RC_MODE_ACTIVE(BOXCAMSTAB)) {
        if (gimbalConfig->mode == GIMBAL_MODE_MIXTILT) {
            servo[SERVO_GIMBAL_PITCH] -= (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 - (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
            servo[SERVO_GIMBAL_ROLL] += (-(int32_t)servoConf[SERVO_GIMBAL_PITCH].rate) * attitude.values.pitch / 50 + (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll / 50;
        } else {
            servo[SERVO_GIMBAL_PITCH] += (int32_t)servoConf[SERVO_GIMBAL_PITCH].rate * attitude.values.pitch / 50;
            servo[SERVO_GIMBAL_ROLL] += (int32_t)servoConf[SERVO_GIMBAL_ROLL].rate * attitude.values.roll  / 50;
        }
    }
}

#define SERVO_AUTOTRIM_TIMER_MS     1000

typedef enum {
    AUTOTRIM_IDLE,
    AUTOTRIM_COLLECTING,
    AUTOTRIM_SAVE_PENDING,
    AUTOTRIM_DONE,
} servoAutotrimState_e;

void processServoAutotrim(void)
{
    static servoAutotrimState_e trimState = AUTOTRIM_IDLE;
    static timeMs_t trimStartedAt;
    
    static int16_t servoMiddleBackup[MAX_SUPPORTED_SERVOS];
    static int32_t servoMiddleAccum[MAX_SUPPORTED_SERVOS];
    static int32_t servoMiddleAccumCount;
    
    if (IS_RC_MODE_ACTIVE(BOXAUTOTRIM)) {
        switch (trimState) {
            case AUTOTRIM_IDLE:
                if (ARMING_FLAG(ARMED)) {
                    // We are activating servo trim - backup current middles and prepare to average the data
                    for (int servoIndex = SERVO_ELEVATOR; servoIndex <= MIN(SERVO_RUDDER, MAX_SUPPORTED_SERVOS); servoIndex++) {
                        servoMiddleBackup[servoIndex] = servoConf[servoIndex].middle;
                        servoMiddleAccum[servoIndex] = 0;
                    }

                    trimStartedAt = millis();
                    servoMiddleAccumCount = 0;
                    trimState = AUTOTRIM_COLLECTING;
                }
                else {
                    break;
                }
                // Fallthru

            case AUTOTRIM_COLLECTING:
                if (ARMING_FLAG(ARMED)) {
                    servoMiddleAccumCount++;

                    for (int servoIndex = SERVO_ELEVATOR; servoIndex <= MIN(SERVO_RUDDER, MAX_SUPPORTED_SERVOS); servoIndex++) {
                        servoMiddleAccum[servoIndex] += servo[servoIndex];
                    }

                    if ((millis() - trimStartedAt) > SERVO_AUTOTRIM_TIMER_MS) {
                        for (int servoIndex = SERVO_ELEVATOR; servoIndex <= MIN(SERVO_RUDDER, MAX_SUPPORTED_SERVOS); servoIndex++) {
                            servoConf[servoIndex].middle = servoMiddleAccum[servoIndex] / servoMiddleAccumCount;
                        }
                        trimState = AUTOTRIM_SAVE_PENDING;
                    }
                }
                else {
                    trimState = AUTOTRIM_IDLE;
                }
                break;

            case AUTOTRIM_SAVE_PENDING:
                // Wait for disarm and save to EEPROM
                if (!ARMING_FLAG(ARMED)) {
                    saveConfigAndNotify();
                    trimState = AUTOTRIM_DONE;
                }
                break;

            case AUTOTRIM_DONE:
                break;
        }
    }
    else {
        // We are deactivating servo trim - restore servo midpoints
        if (trimState == AUTOTRIM_SAVE_PENDING) {
            for (int servoIndex = SERVO_ELEVATOR; servoIndex <= MIN(SERVO_RUDDER, MAX_SUPPORTED_SERVOS); servoIndex++) {
                servoConf[servoIndex].middle = servoMiddleBackup[servoIndex];
            }
        }

        trimState = AUTOTRIM_IDLE;
    }
}

bool isServoOutputEnabled(void)
{
    return servoOutputEnabled;
}

bool isMixerUsingServos(void)
{
    return mixerUsesServos;
}

void filterServos(void)
{
    int servoIdx;

    if (servoMixerConfig->servo_lowpass_enable) {
        // Initialize servo lowpass filter (servos are calculated at looptime rate)
        if (!servoFilterIsSet) {
            for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
                biquadFilterInitLPF(&servoFitlerState[servoIdx], servoMixerConfig->servo_lowpass_freq, gyro.targetLooptime);
            }

            servoFilterIsSet = true;
        }

        for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
            // Apply servo lowpass filter and do sanity cheching
            servo[servoIdx] = (int16_t) biquadFilterApply(&servoFitlerState[servoIdx], (float)servo[servoIdx]);
        }
    }

    for (servoIdx = 0; servoIdx < MAX_SUPPORTED_SERVOS; servoIdx++) {
        servo[servoIdx] = constrain(servo[servoIdx], servoConf[servoIdx].min, servoConf[servoIdx].max);
    }
}

#endif
